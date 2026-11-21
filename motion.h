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

#ifndef MOTION_H
#define MOTION_H

/* Read the <trkpt> points of a GPX file and resample them linearly in ECEF onto 0.1 s steps.
 * Points without <ele> get default_height. Returns the number of steps written to xyz
 * (at most max), or -1 with *err set to the reason.
 */
int readGpx(double xyz[][3], int max, const char *filename, double default_height, const char **err);

#endif /* MOTION_H */
