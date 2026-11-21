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

#ifndef ORBIT_H
#define ORBIT_H

#include "gps.h"

/* Vectors, GPS time and coordinate conversions, satellite position from the ephemeris (orbit.c) */
void subVect(double *y, const double *x1, const double *x2);
double normVect(const double *x);
int resolveWeek(int week256, int ref_week);
void date2gps(const datetime_t *t, gpstime_t *g);
void gps2date(const gpstime_t *g, datetime_t *t);
void xyz2llh(const double *xyz, double *llh);
void llh2xyz(const double *llh, double *xyz);
void ltcmat(const double *llh, double t[3][3]);
void ecef2neu(const double *xyz, double t[3][3], double *neu);
void neu2azel(double *azel, const double *neu);
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk);
double subGpsTime(gpstime_t g1, gpstime_t g0);
gpstime_t incGpsTime(gpstime_t g0, double dt);

#endif /* ORBIT_H */
