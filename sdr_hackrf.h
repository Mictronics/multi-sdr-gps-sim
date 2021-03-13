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

#ifndef SDR_HACKRF_H
#define SDR_HACKRF_H

#include "gps-sim.h"

#define TX_VGA1   0
#define TX_IF_GAIN_MIN 0
#define TX_IF_GAIN_MAX 47
#define BASEBAND_FILTER_BW_MIN (1750000)  /* 1.75 MHz min value */
#define BASEBAND_FILTER_BW_MAX (28000000) /* 28 MHz max value */

int sdr_hackrf_init(simulator_t *simulator);
void sdr_hackrf_close(void);
int sdr_hackrf_run(void);
int sdr_hackrf_set_gain(const int gain);

#endif /* SDR_HACKRF_H */

