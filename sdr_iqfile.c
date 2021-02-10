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

#include "gui.h"
#include "sdr.h"
#include "sdr_iqfile.h"
#include "fifo.h"

static atomic_bool iqfile_thread_exit = false;
static pthread_t iqfile_thread;
static int sample_size = SC08;

static void *iqfile_thread_ep(void *arg) {
    (void) arg; // Not used
    FILE *fp = fopen("iqdata.bin", "wb");

    if (fp == NULL) {
        gui_status_wprintw(RED, "Error opening IQ data file.\n");
        pthread_exit(NULL);
    }

    /* On a multi-core CPU we run the main thread and reader thread on different cores.
     * Try sticking the main thread to core 3
     */
    thread_to_core(3);
    set_thread_name("iqfile-thread");

    while (!iqfile_thread_exit) {
        // Get a fifo block
        struct iq_buf *iq = fifo_dequeue();
        if (iq != NULL) {
            if (sample_size == sizeof (signed short)) {
                fwrite(iq->data16, sizeof (signed short), iq->validLength, fp);
            } else {
                fwrite(iq->data8, sizeof (signed char), iq->validLength, fp);
            }
            // Release and free up used block
            fifo_release(iq);
            if (ferror(fp)) {
                gui_status_wprintw(RED, "Error writing IQ data file.\n");
            }
        }
    }
    fclose(fp);
    pthread_exit(NULL);
}

int sdr_iqfile_init(simulator_t *simulator) {
    sample_size = simulator->sample_size;
    if (!fifo_create(NUM_FIFO_BUFFERS, IQ_BUFFER_SIZE, sample_size)) {
        gui_status_wprintw(RED, "Error creating IQ file fifo!");
        return -1;
    }
    return 0;
}

void sdr_iqfile_close(void) {
    iqfile_thread_exit = true;
    fifo_halt();
    fifo_destroy();
    pthread_join(iqfile_thread, NULL);
}

int sdr_iqfile_run(void) {
    fifo_wait_full();
    pthread_create(&iqfile_thread, NULL, iqfile_thread_ep, NULL);
    return 0;
}