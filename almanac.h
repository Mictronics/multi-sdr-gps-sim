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

#ifndef ALMANAC_H
#define ALMANAC_H

#include <curl/curl.h>
#include "gps.h"

#define ALMANAC_DOWNLOAD_SEM_URL "https://www.celestrak.com/GPS/almanac/SEM/almanac.sem.txt"

typedef struct {
    unsigned char ura; // User Range Accuracy lookup code, [0-15], see p. 91 IS-GPS-200L, 0 <2.4m, 15 is >6144m
    unsigned char health; // 0=healthy, unhealthy otherwise [], subframe 4 and 5, page 25 six-bit health code
    unsigned char config_code; // configuration code [], if >=9 Anti-Spoofing is on
    unsigned short svid; // GPS SV id or prn number 1-32 []  
    unsigned short svn; // Satellite vehicle number []
    unsigned int valid; // Validity of this almanac
    double e; // eccentricity []
    double delta_i; // orbital inclination at reference time [rad]
    double omegadot; // rate of right ascension [rad/s]
    double sqrta; // square root of the semi-major axis [m^(1/2)]
    double omega0; // longitude of ascending node of orbit plane at weekly epoch [rad]
    double aop; // argument of perigee [rad]
    double m0; // mean anomaly at reference time [rad]
    double af0; // polynomial clock correction coefficient (clock bias) [s], Note: parameters from ephemeris preferred vs almanac (22 vs 11 bits)
    double af1; // polynomial clock correction coefficient (clock drift) [s/s], Note: parameters from ephemeris preferred vs almanac (16 vs 11 bits)
    gpstime_t toa; // almanac time of applicability (reference time) [s]
} almanac_prn_t;

typedef struct {
    unsigned int valid; // Almanac validity
    almanac_prn_t sv[MAX_SAT]; // 32 SV's almanac
} almanac_gps_t;

almanac_gps_t* almanac_init(void);
CURLcode almanac_read_file(void);
CURLcode almanac_download(void);

#endif /* ALMANAC_H */

