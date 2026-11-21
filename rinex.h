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

#ifndef RINEX_H
#define RINEX_H

#include "gps.h"

// Last leap second event, 2016/12/31: used when a RINEX header gives no WN_LSF and DN.
#define LEAP_DEFAULT_WN 1929
#define LEAP_DEFAULT_DN 7
// Offset GPS time to UTC since that event; to be updated when a new leap second is introduced.
#define LEAP_DEFAULT_DTLS 18

/* PGM / RUN BY / DATE of the last RINEX header read */
extern char rinex_date[21];

/* Read the ephemerides of a RINEX 2 or 3 navigation file (see rinex.c).
 * Returns the number of records, negative on error. */
int readRinex(eph_table_t *tab, ionoutc_t *ionoutc, const char *fname, bool rinex3);

/* For every satellite pick the record closest in time to t (vflg false if none is near enough). */
void ephSelect(const eph_table_t *tab, gpstime_t t, ephem_t cur[MAX_SAT]);

/* Fill the ionosphere and UTC groups the file did not provide with neutral values for the start time g,
 * so subframe 4 page 18 is complete. Returns true if anything was filled. */
bool ionoDefaults(ionoutc_t *io, gpstime_t g);

#endif /* RINEX_H */
