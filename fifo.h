/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2026 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef FIFO_H
#define FIFO_H

#include <stdbool.h>
#include <stdint.h>

struct iq_buf
{
    signed char *data8;       // 8 bit IQ data
    signed short *data16;     // 16 bit IQ data
    unsigned int totalLength; // Maximum number of samples (allocated size of "data")
    unsigned int validLength; // Number of valid samples in "data"
    struct iq_buf *next;      // linked list forward link
    struct iq_buf *all_next;  // link in the list of all buffers, used by fifo_destroy
};

// Create the queue structures. Not threadsafe. Returns true on success.
//
//   buffer_count - the number of buffers to preallocate
//   buffer_size  - the size of each IQ buffer, in samples
bool fifo_create(unsigned buffer_count, unsigned buffer_size, unsigned sample_size);

// Free all buffers, including ones still held by users. Not threadsafe; halt the FIFO and
// stop every producer and consumer thread before calling.
void fifo_destroy();

// Block until the producer has used up all free buffers once, or the FIFO is halted.
void fifo_wait_full();

// Tell waiters the producer has stopped, so fifo_wait_full returns even if the FIFO never filled.
// Queued buffers stay available to the consumer.
void fifo_producer_done();

// Mark the FIFO as halted. Move any buffers in FIFO to the freelist immediately.
void fifo_halt();

// True when no filled buffer is waiting for the consumer.
bool fifo_is_empty(void);

// Number of filled buffers waiting for the SDR; 0 while streaming means the producer falls behind.
unsigned fifo_count(void);

// Get an unused buffer from the freelist and return it.
// Block waiting for a free buffer. Returns NULL if the FIFO is halted.
struct iq_buf *fifo_acquire(void);

// Put a filled buffer (previously obtained from fifo_acquire) onto the head of the FIFO.
// The caller should have filled:
//   buf->validLength
//   buf->data[0 .. buf->validLength-1]
void fifo_enqueue(struct iq_buf *buf);

// Get a buffer from the tail of the FIFO.
// If the FIFO is halted (or becomes halted), return NULL immediately.
// return NULL if no data arrives
struct iq_buf *fifo_dequeue(void);

// Release a buffer previously returned by fifo_acquire() back to the freelist.
void fifo_release(struct iq_buf *buf);

#endif