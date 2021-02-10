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

#ifndef SDR_H
#define SDR_H

#include "gps-sim.h"

#define TX_FREQUENCY 1575420000 // 1575420000/1023 = 1540000
#define TX_SAMPLERATE 2618880 //chips-> 1023 * 2560 = 2618880
#define FREQ_ONE_MHZ (1000000ull)

#define NUM_FIFO_BUFFERS 8
// Number of samples for 0.1 second transmission
#define NUM_IQ_SAMPLES (TX_SAMPLERATE / 10)
// Size in number of IQ samples, one I and one Q per sample
// Size in IQ elements, not bytes!
#define IQ_BUFFER_SIZE (NUM_IQ_SAMPLES * 2)

// HackRF transfer buffer size
// Fixed to 4 * 8192 = 262144 bytes
// Defined in libhackrf, hackrf.c
#define HACKRF_TRANSFER_BUFFER_SIZE 262144

int sdr_init(simulator_t *simulator);
void sdr_close(void);
int sdr_run(void);

#endif /* SDR_H */

