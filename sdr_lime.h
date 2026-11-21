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

#ifndef SDR_LIME_H
#define SDR_LIME_H

// LMS_SetGaindB range in LimeSuiteNG: TX PAD gain, 52 = full power, 2 dB steps below 42
#define LIME_TX_GAIN_MIN 0
#define LIME_TX_GAIN_MAX 52
#define LIME_OVERSAMPLE 2 // CGEN 8 x 5 MS/s = 40 MHz, read back exact
// TX gain [dB] during calibration, before the user's gain is set
#define LIME_CAL_GAIN 40

int sdr_lime_init(simulator_t *simulator);
void sdr_lime_close(void);
int sdr_lime_run(void);
int sdr_lime_set_gain(const int gain);

#endif /* SDR_LIME_H */
