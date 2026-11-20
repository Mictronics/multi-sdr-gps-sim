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


#include <math.h>
#include "sbas.h"

// gps.gov L1 C/A PRN code assignments, Jan 2026. ASECNA (120, 147) has no slot listed yet.
const sbas_geo_t sbas_geo[] = {
    {121, "EGNOS", -5.0}, {123, "EGNOS", 31.5}, {136, "EGNOS", 5.0}, {150, "EGNOS", 13.0},
    {131, "WAAS", -117.0}, {133, "WAAS", -129.0}, {135, "WAAS", -125.0}, {138, "WAAS", -117.0},
    {129, "MSAS", 90.5}, {137, "MSAS", 127.0}, {139, "MSAS", -175.0},
    {127, "GAGAN", 55.0}, {128, "GAGAN", 83.0}, {132, "GAGAN", 93.5},
    {125, "SDCM", -16.0}, {140, "SDCM", 95.0}, {141, "SDCM", 167.0},
    {130, "BDSBAS", 140.0}, {143, "BDSBAS", 110.5}, {144, "BDSBAS", 80.0},
    {134, "KASS", 91.5}, {142, "KASS", 116.0},
    {122, "SPAN", 143.5}, {124, "SPAN", 178.0},
    {145, "Pak-SBAS", 38.2}, {148, "A-SBAS", -24.8}, {158, "UK SBAS", -53.9},
};
const int sbas_geo_count = sizeof (sbas_geo) / sizeof (sbas_geo[0]);

// CRC-24Q, g(X) = 0x1864CFB, initial value 0, no reflection
unsigned long sbas_crc24q(const unsigned char *bits, int n) {
    unsigned long crc = 0;

    for (int i = 0; i < n; i++) {
        crc ^= (unsigned long) (bits[i] & 1) << 23;
        crc = (crc << 1) ^ ((crc & 0x800000UL) ? 0x1864CFBUL : 0);
    }
    return crc & 0xFFFFFFUL;
}

/* Build a 250 bit message. sec is the GPS second at which it starts: the 24 bit preamble
 * spreads over three messages and every other one starts at a 6 s subframe epoch. */
void sbas_frame(int type, const unsigned char *data, int sec, unsigned char *msg) {
    static const unsigned char preamble[3] = {0x53, 0x9A, 0xC6};
    int i, p = preamble[sec % 3];

    for (i = 0; i < 8; i++)
        msg[i] = (p >> (7 - i)) & 1;
    for (i = 0; i < 6; i++)
        msg[8 + i] = (type >> (5 - i)) & 1;
    for (i = 0; i < SBAS_DATA_BITS; i++)
        msg[14 + i] = data[i] & 1;

    unsigned long crc = sbas_crc24q(msg, 226);
    for (i = 0; i < 24; i++)
        msg[226 + i] = (crc >> (23 - i)) & 1;
}

static int parity(unsigned v) {
    return __builtin_parity(v);
}

/* Rate 1/2, K = 7 convolutional code, G1 = 171, G2 = 133 (octal), G1 symbol first, neither
 * inverted (gnss-sdr's SBAS decoder). state carries the register across calls, start with 0. */
void sbas_encode(unsigned *state, const unsigned char *bits, int n, unsigned char *sym) {
    unsigned s = *state;

    for (int i = 0; i < n; i++) {
        s = ((unsigned) (bits[i] & 1) << 6) | (s >> 1); // newest bit is tap 0 (MSB of the octal mask)
        sym[2 * i] = parity(s & 0171);
        sym[2 * i + 1] = parity(s & 0133);
    }
    *state = s; // All 7 bits: the oldest one only drops out with the next input bit
}

// Write v as a len bit two's complement field at data[*off], most significant bit first
static void putBits(unsigned char *data, int *off, long v, int len) {
    for (int i = len - 1; i >= 0; i--)
        data[(*off)++] = (v >> i) & 1;
}

/* Message type 9 data field for a GEO at rest at pos (ECEF, m), sent at second of week sec.
 * URA 15: not for ranging; ponytail: velocity, acceleration and clock are 0, a fixed slot. */
void sbas_mt9(const double *pos, int sec, unsigned char *data) {
    int off = 0;

    putBits(data, &off, 0, 8);                          // reserved (was IODN)
    putBits(data, &off, (sec % 86400) / 16, 13);        // t0, 16 s
    putBits(data, &off, 15, 4);                         // URA index
    putBits(data, &off, lround(pos[0] / 0.08), 30);     // XG, 0.08 m
    putBits(data, &off, lround(pos[1] / 0.08), 30);     // YG
    putBits(data, &off, lround(pos[2] / 0.4), 25);      // ZG, 0.4 m
    while (off < SBAS_DATA_BITS) // rates 17, 17, 18, accelerations 10, 10, 10, aGf0 12, aGf1 8
        data[off++] = 0;
}
