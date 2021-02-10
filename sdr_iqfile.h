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

#ifndef SDR_IQFILE_H
#define SDR_IQFILE_H

int sdr_iqfile_init(simulator_t *simulator);
void sdr_iqfile_close(void);
int sdr_iqfile_run(void);

#endif /* SDR_IQFILE_H */

