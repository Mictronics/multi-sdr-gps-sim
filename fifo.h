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

#ifndef FIFO_H
#define FIFO_H

#include <stdbool.h>
#include <stdint.h>

struct iq_buf {
    signed char *data8; // 8 bit IQ data
    signed short *data16; // 16 bit IQ data
    unsigned int totalLength; // Maximum number of samples (allocated size of "data")
    unsigned int validLength; // Number of valid samples in "data"
    struct iq_buf *next; // linked list forward link
};

// Create the queue structures. Not threadsafe. Returns true on success.
//
//   buffer_count - the number of buffers to preallocate
//   buffer_size  - the size of each IQ buffer, in samples
bool fifo_create(unsigned buffer_count, unsigned buffer_size, unsigned sample_size);

// Destroy the fifo structures allocated in fifo_create. Not threadsafe; ensure all FIFO users
// are done before calling.
void fifo_destroy();

// Block until the FIFO is empty.
void fifo_wait_next();

// Block until the FIFO is full.
void fifo_wait_full();

// Mark the FIFO as halted. Move any buffers in FIFO to the freelist immediately.
void fifo_halt();

// Get an unused buffer from the freelist and return it.
// Block waiting for a free buffer.
// free buffers available within the timeout, or if the FIFO is halted.
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