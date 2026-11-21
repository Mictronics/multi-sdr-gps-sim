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

#ifndef DOWNLOAD_H
#define DOWNLOAD_H

#include <stdatomic.h>
#include <stdbool.h>
#include <curl/curl.h>

#define DOWNLOAD_USER_AGENT "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36"

/* Download url into filename within timeout seconds; HTTP 200 only, no partial file on failure,
 * aborted when *cancel becomes true (download.c). */
CURLcode download_file(const char *url, const char *filename, long timeout, const atomic_bool *cancel);

/* Download the newest broadcast ephemeris file (RINEX 2 from NOAA CORS, RINEX 3 from BKG). */
CURLcode download_nav_file(bool rinex3, const char *filename, const atomic_bool *cancel);

#endif /* DOWNLOAD_H */
