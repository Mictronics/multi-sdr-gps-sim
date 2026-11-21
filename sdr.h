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

#ifndef SDR_H
#define SDR_H

#include <limits.h>
#include <stdint.h>
#include "gps-sim.h"

// Returned by set_gain for SDR types without gain control
#define SDR_GAIN_UNSUPPORTED INT_MIN

#define TX_FREQUENCY 1575420000

/* TX frequency corrected for an oscillator running ppb parts per billion fast */
static inline uint64_t sdr_corrected_frequency(int ppb)
{
    return (uint64_t)((double)TX_FREQUENCY * (1.0 - ppb * 1e-9) + 0.5);
}
#define FREQ_ONE_MHZ (1000000ull)
// 5 MS/s: about 2.4 samples per Galileo BOC(1,1) subchip; Pluto makes it with integer dividers (sdr_pluto.h),
// 6 MS/s (24 MB/s) underflows its USB 2 link
#define TX_SAMPLERATE 5000000
#define TX_BW (TX_SAMPLERATE * 2)

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
int sdr_set_gain(int gain);

#endif /* SDR_H */
