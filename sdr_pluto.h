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

#ifndef SDR_PLUTO_H
#define SDR_PLUTO_H

#define PLUTO_TX_GAIN_MIN -80
#define PLUTO_TX_GAIN_MAX 0
#define PLUTO_TX_BW 2500000

int sdr_pluto_init(simulator_t *simulator);
void sdr_pluto_close(void);
int sdr_pluto_run(void);

#endif /* SDR_PLUTO_H */

