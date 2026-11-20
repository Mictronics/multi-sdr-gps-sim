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


#ifndef SBAS_H
#define SBAS_H

#define SBAS_MSG_BITS 250 // 1 s at 250 bit/s: preamble 8, type 6, data 212, CRC 24
#define SBAS_DATA_BITS 212

#define SBAS_PRN_MIN 120
#define SBAS_PRN_MAX 158
#define SBAS_GEO_RADIUS 42164170.0 // Geostationary orbit radius [m]

typedef struct {
    int prn;
    const char *system;
    double lon; // Orbital slot [deg], east positive
} sbas_geo_t;

// SBAS GEOs in service, from the gps.gov L1 C/A PRN code assignments; update it like LEAP_DEFAULT_DTLS
extern const sbas_geo_t sbas_geo[];
extern const int sbas_geo_count;

// Bits are one per byte (0/1), most significant bit of each field first.
unsigned long sbas_crc24q(const unsigned char *bits, int n);
void sbas_frame(int type, const unsigned char *data, int sec, unsigned char *msg);
void sbas_mt9(const double *pos, int sec, unsigned char *data);
void sbas_encode(unsigned *state, const unsigned char *bits, int n, unsigned char *sym);

#endif /* SBAS_H */
