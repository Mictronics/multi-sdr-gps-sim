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

#ifndef NAVMSG_H
#define NAVMSG_H

#include "gps.h"
#include "almanac.h"

/* Navigation message (navmsg.c): subframes from ephemeris and almanac, words with parity for a channel */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, const char *msg, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]);
void generateNavMsg(gpstime_t g, channel_t *chan, int init);

#endif /* NAVMSG_H */
