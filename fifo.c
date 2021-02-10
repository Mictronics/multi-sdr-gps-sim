/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>
#include "fifo.h"

static pthread_mutex_t fifo_mutex = PTHREAD_MUTEX_INITIALIZER; // mutex protecting the queues
static pthread_cond_t fifo_notempty_cond = PTHREAD_COND_INITIALIZER; // condition used to signal FIFO-not-empty
static pthread_cond_t fifo_empty_cond = PTHREAD_COND_INITIALIZER; // condition used to signal FIFO-empty
static pthread_cond_t fifo_free_cond = PTHREAD_COND_INITIALIZER; // condition used to signal freelist-not-empty
static pthread_cond_t fifo_full_cond = PTHREAD_COND_INITIALIZER; // condition used to signal FIFO-full
static struct iq_buf *fifo_head; // head of queued buffers awaiting demodulation
static struct iq_buf *fifo_tail; // tail of queued buffers awaiting demodulation
static struct iq_buf *fifo_freelist; // freelist of preallocated buffers
static bool fifo_halted; // true if queue has been halted

// Create the queue structures. Not threadsafe.

bool fifo_create(unsigned buffer_count, unsigned buffer_size, unsigned sample_size) {
    for (unsigned i = 0; i < buffer_count; ++i) {
        struct iq_buf *newbuf;
        if (!(newbuf = calloc(1, sizeof (*newbuf)))) {
            goto nomem;
        }

        if (sample_size == sizeof (signed short)) {
            if (!(newbuf->data16 = calloc(buffer_size, sizeof (newbuf->data16[0])))) {
                free(newbuf);
                goto nomem;
            }
            newbuf->data8 = NULL;
        } else {
            if (!(newbuf->data8 = calloc(buffer_size, sizeof (newbuf->data8[0])))) {
                free(newbuf);
                goto nomem;
            }
            newbuf->data16 = NULL;
        }

        newbuf->totalLength = buffer_size;
        newbuf->validLength = 0;
        newbuf->next = fifo_freelist;
        fifo_freelist = newbuf;
    }

    return true;

nomem:
    fifo_destroy();
    return false;
}

static void free_buffer_list(struct iq_buf *head) {
    while (head) {
        struct iq_buf *next = head->next;
        free(head->data8);
        free(head->data16);
        free(head);
        head = next;
    }
}

void fifo_destroy() {
    free_buffer_list(fifo_head);
    free_buffer_list(fifo_freelist);
    fifo_freelist = NULL;
    fifo_head = fifo_tail = NULL;
    pthread_cond_destroy(&fifo_notempty_cond);
    pthread_cond_destroy(&fifo_full_cond);
    pthread_cond_destroy(&fifo_free_cond);
    pthread_cond_destroy(&fifo_empty_cond);
    pthread_mutex_destroy(&fifo_mutex);
}

void fifo_wait_next() {
    pthread_mutex_lock(&fifo_mutex);
    while (fifo_head && !fifo_halted) {
        pthread_cond_wait(&fifo_empty_cond, &fifo_mutex);
    }
    pthread_mutex_unlock(&fifo_mutex);
}

void fifo_wait_full() {
    pthread_mutex_lock(&fifo_mutex);
    if (!fifo_halted) {
        pthread_cond_wait(&fifo_full_cond, &fifo_mutex);
    }
    pthread_mutex_unlock(&fifo_mutex);
}

void fifo_halt() {
    pthread_mutex_lock(&fifo_mutex);

    // Drain all enqueued buffers to the freelist
    while (fifo_head) {
        struct iq_buf *freebuf = fifo_head;
        fifo_head = freebuf->next;

        freebuf->next = fifo_freelist;
        fifo_freelist = freebuf;
    }

    fifo_tail = NULL;
    fifo_halted = true;

    // wake all waiters
    pthread_cond_broadcast(&fifo_notempty_cond);
    pthread_cond_broadcast(&fifo_empty_cond);
    pthread_cond_broadcast(&fifo_free_cond);
    pthread_cond_broadcast(&fifo_full_cond);
    pthread_mutex_unlock(&fifo_mutex);
}

struct iq_buf *fifo_acquire(void) {
    pthread_mutex_lock(&fifo_mutex);

    struct iq_buf *result = NULL;
    if (!fifo_halted && !fifo_freelist) {
        pthread_cond_signal(&fifo_full_cond);
        // No free buffers, wait for one
        pthread_cond_wait(&fifo_free_cond, &fifo_mutex);
    }

    if (!fifo_halted) {
        result = fifo_freelist;
        fifo_freelist = result->next;

        result->validLength = 0;
        result->next = NULL;
    }

    pthread_mutex_unlock(&fifo_mutex);
    return result;
}

void fifo_enqueue(struct iq_buf *buf) {
    assert(buf->validLength <= buf->totalLength);

    pthread_mutex_lock(&fifo_mutex);

    if (fifo_halted) {
        // Shutting down, just return the buffer to the freelist.
        buf->next = fifo_freelist;
        fifo_freelist = buf;
        goto done;
    }
    // enqueue and tell the main thread
    buf->next = NULL;
    if (!fifo_head) {
        fifo_head = fifo_tail = buf;
        pthread_cond_signal(&fifo_notempty_cond);
    } else {
        fifo_tail->next = buf;
    }

done:
    pthread_mutex_unlock(&fifo_mutex);
}

struct iq_buf *fifo_dequeue(void) {
    pthread_mutex_lock(&fifo_mutex);

    struct iq_buf *result = NULL;
    if (!fifo_head && !fifo_halted) {
        // No data pending, wait for some
        pthread_cond_wait(&fifo_notempty_cond, &fifo_mutex);
    }

    if (!fifo_halted) {
        result = fifo_head;
        fifo_head = result->next;
        result->next = NULL;
        if (!fifo_head) {
            fifo_tail = NULL;
            pthread_cond_broadcast(&fifo_empty_cond);
        }
    }

    pthread_mutex_unlock(&fifo_mutex);
    return result;
}

void fifo_release(struct iq_buf *buf) {
    pthread_mutex_lock(&fifo_mutex);
    if (!fifo_freelist) {
        pthread_cond_signal(&fifo_free_cond);
    }
    buf->next = fifo_freelist;
    fifo_freelist = buf;
    pthread_mutex_unlock(&fifo_mutex);
}

