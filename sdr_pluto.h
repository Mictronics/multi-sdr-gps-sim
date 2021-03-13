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

/*
 * TX path oscillator chain
 * Dividers with multiple of 2 and resulting sample rate.
 * BBPLL         DAC          T2           T1          TF          TXSAMP [Hz]
 * 1024000000     32000000     16000000     8000000     4000000    1000000
 *  768000000     48000000     24000000    12000000     6000000    1500000
 * 1024000000     64000000     32000000    16000000     8000000    2000000
 * 1280000000     80000000     40000000    20000000    10000000    2500000
 *  768000000     96000000     48000000    24000000    12000000    3000000
 *  896000000    112000000     56000000    28000000    14000000    3500000
 * 1024000000    128000000     64000000    32000000    16000000    4000000
 * 1152000000    144000000     72000000    36000000    18000000    4500000
 * 1280000000    160000000     80000000    40000000    20000000    5000000
 * 1408000000    176000000     88000000    44000000    22000000    5500000
 *  768000000    192000000     96000000    48000000    24000000    6000000
 *  832000000    208000000    104000000    52000000    26000000    6500000
 *  896000000    224000000    112000000    56000000    28000000    7000000
 *  960000000    240000000    120000000    60000000    30000000    7500000
 * 1024000000    256000000    128000000    64000000    32000000    8000000
 * etc. 
 * Any sample rate being a multiple of 250kHz can be generated with integer dividers.
 */

#ifndef SDR_PLUTO_H
#define SDR_PLUTO_H

#define PLUTO_TX_GAIN_MIN -80
#define PLUTO_TX_GAIN_MAX 0

int sdr_pluto_init(simulator_t *simulator);
void sdr_pluto_close(void);
int sdr_pluto_run(void);
int sdr_pluto_set_gain(const int gain);

#endif /* SDR_PLUTO_H */

