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

#ifndef MODEL_H
#define MODEL_H

#include "gps.h"
#include "sbas.h"

/* Signal path models (model.c): Klobuchar ionosphere, troposphere, satellite to receiver range */
double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel);
void sbasIonoUpdate(sbas_iono_t *ion, const ionoutc_t *ionoutc, gpstime_t g, const double *llh);
double tropoDelay(const double *llh, const double *azel);
void rangeFromPos(range_t *rho, const double *pos0, const double vel[3], double clk0, ionoutc_t *ionoutc, gpstime_t g, double xyz[]);
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]);

#endif /* MODEL_H */
