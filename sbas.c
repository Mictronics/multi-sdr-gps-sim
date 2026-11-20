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
#include <string.h>
#include "sbas.h"

#define SBAS_NMASK 32 // PRN mask: GPS PRN 1-32, so mask slot i is PRN i + 1
#define SBAS_IODP 0   // The mask never changes

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

// MT1: PRN mask
static void mt1(unsigned char *data) {
    int off = 0;
    for (int i = 0; i < 210; i++)
        data[off++] = i < SBAS_NMASK;
    putBits(data, &off, SBAS_IODP, 2);
}

// MT2-4: fast corrections of mask slots 13 * (type - 2) on; PRC 0, UDREI 5 (3.0 m) or 14 (not monitored)
static void mt2_4(int type, const int *iode, unsigned char *data) {
    int off = 0, first = 13 * (type - 2);

    putBits(data, &off, 0, 2); // IODF, constant as the corrections never change
    putBits(data, &off, SBAS_IODP, 2);
    for (int k = 0; k < 13; k++)
        putBits(data, &off, 0, 12); // PRC, 0.125 m
    for (int k = 0; k < 13; k++) {
        int slot = first + k;
        putBits(data, &off, slot >= SBAS_NMASK ? 0 : (iode[slot] >= 0 ? 5 : 14), 4); // unused slots 0, as EGNOS
    }
}

// MT7: fast correction degradation, the values EGNOS sends (t_lat 3 s, ai 15 for masked slots)
static void mt7(unsigned char *data) {
    int off = 0;
    putBits(data, &off, 3, 4);
    putBits(data, &off, SBAS_IODP, 2);
    putBits(data, &off, 0, 2);
    for (int k = 0; k < 51; k++)
        putBits(data, &off, k < SBAS_NMASK ? 15 : 0, 4);
}

// MT10: degradation parameters, copied from EGNOS (2026-11-20); the remaining 81 bits are spare
static void mt10(unsigned char *data) {
    static const int len[16] = {10, 10, 10, 9, 10, 9, 10, 10, 9, 6, 10, 9, 10, 1, 1, 7};
    static const int val[16] = {0, 179, 156, 120, 479, 120, 354, 400, 120, 0, 657, 300, 0, 0, 0, 0};
    int off = 0;
    for (int k = 0; k < 16; k++)
        putBits(data, &off, val[k], len[k]);
}

// MT25 number k (0-7): long term corrections of mask slots 4k to 4k+3, all deltas 0, IODE of the
// ephemeris being sent; a slot without ephemeris gets mask number 0 (no satellite)
static void mt25(int k, const int *iode, unsigned char *data) {
    int off = 0;
    for (int half = 0; half < 2; half++) {
        putBits(data, &off, 0, 1); // velocity code 0: two satellites, no rates
        for (int j = 0; j < 2; j++) {
            int slot = 4 * k + 2 * half + j;
            putBits(data, &off, iode[slot] >= 0 ? slot + 1 : 0, 6);
            putBits(data, &off, iode[slot] >= 0 ? iode[slot] : 0, 8);
            putBits(data, &off, 0, 9 + 9 + 9 + 10); // dx, dy, dz, da_f0
        }
        putBits(data, &off, SBAS_IODP, 2);
        putBits(data, &off, 0, 1);
    }
}

/* IGP at mask bit (0-200) of band 0-8, as RTKLIB's igpband1 (DO-229 appendix A); 0 if the bit
 * has no IGP (band 8 has 200). Columns are 5 deg apart; 10 deg columns reach +-75 deg, the others
 * +-55 deg, and one column per band also has the IGP at 85 N or 85 S. */
int sbas_igp(int band, int bit, double *lat, double *lon) {
    for (int c = 0; c < 8; c++) {
        int lo = -180 + 40 * band + 5 * c, n = 0;
        double lats[28];

        if (lo == -140 || lo == -50 || lo == 40 || lo == 130)
            lats[n++] = -85;
        if (c % 2 == 0) {
            lats[n++] = -75;
            lats[n++] = -65;
        }
        for (int la = -55; la <= 55; la += 5)
            lats[n++] = la;
        if (c % 2 == 0) {
            lats[n++] = 65;
            lats[n++] = 75;
        }
        if (lo == -180 || lo == -90 || lo == 0 || lo == 90)
            lats[n++] = 85;
        if (bit < n) {
            *lat = lats[bit];
            *lon = lo;
            return 1;
        }
        bit -= n;
    }
    return 0;
}

/* Mask the IGPs within 25 deg latitude and 30 deg great-circle longitude of the user, which
 * covers the pierce points down to 5 deg elevation. IODI changes when the mask does. */
void sbas_iono_mask(double lat, double lon, sbas_iono_t *ion) {
    sbas_iono_t old = *ion;

    ion->nband = 0;
    for (int b = 0; b <= 8 && ion->nband < SBAS_MAX_BANDS; b++) {
        int k = ion->nband, n = 0;
        for (int i = 0; i < SBAS_BAND_BITS; i++) {
            double la, lo;
            int in = 0;
            if (sbas_igp(b, i, &la, &lo)) {
                double dlon = fmod(lo - lon + 540.0, 360.0) - 180.0;
                in = fabs(la - lat) <= 25.0 && fabs(dlon) * cos(lat / 57.29577951308232) <= 30.0;
            }
            ion->mask[k][i] = in;
            if (in) {
                ion->lat[k][n] = la;
                ion->lon[k][n++] = lo;
            }
        }
        if (n > 0) {
            ion->band[k] = b;
            ion->nigp[k] = n;
            ion->nband++;
        }
    }
    int same = old.nband == ion->nband;
    for (int k = 0; same && k < ion->nband; k++)
        same = old.band[k] == ion->band[k] && memcmp(old.mask[k], ion->mask[k], SBAS_BAND_BITS) == 0;
    ion->iodi = same ? old.iodi : (old.iodi + 1) % 4;
}

// MT18: IGP mask of the k-th band
static void mt18(const sbas_iono_t *ion, int k, unsigned char *data) {
    int off = 0;
    putBits(data, &off, ion->nband, 4);
    putBits(data, &off, ion->band[k], 4);
    putBits(data, &off, ion->iodi, 2);
    for (int i = 0; i < SBAS_BAND_BITS; i++)
        data[off++] = ion->mask[k][i];
}

// MT26: vertical delays (0.125 m) of block blk (15 IGPs) of the k-th band, GIVEI 6 as EGNOS mostly sends
static void mt26(const sbas_iono_t *ion, int k, int blk, unsigned char *data) {
    int off = 0;
    putBits(data, &off, ion->band[k], 4);
    putBits(data, &off, blk, 4);
    for (int i = 15 * blk; i < 15 * blk + 15; i++) {
        if (i < ion->nigp[k]) {
            long givd = lround(ion->delay[k][i] / 0.125);
            putBits(data, &off, givd < 0 ? 0 : (givd > 510 ? 510 : givd), 9); // 511 means "do not use"
            putBits(data, &off, 6, 4);
        } else {
            putBits(data, &off, 0, 9);
            putBits(data, &off, 15, 4); // past the last IGP of the band: not monitored
        }
    }
    putBits(data, &off, ion->iodi, 2);
}

/* Data field of the message sent at second of week sec, returns its type. iode[sv] is the IODE
 * of GPS PRN sv + 1, -1 without ephemeris; pos is the GEO for MT9 (ECEF, m); ion the grid.
 * Every 6 s MT2, MT3, MT4, then three slots from a 60 s list: MT1 twice, all 8 MT25, MT9, MT7,
 * MT10, and 17 slots that cycle through MT18 and the MT26 blocks of each band (all within 300 s).
 * The ionosphere messages also give the stream enough symbol transitions: with only zero
 * corrections and null messages a u-blox M8T never synchronised (15 % vs 24 % transitions). */
int sbas_message(int sec, const int *iode, const double *pos, const sbas_iono_t *ion, unsigned char *data) {
    static const int rest[30] = {1, 25, 25, 25, 25, 9, 7, 10, 25, 25, 25, 25, 63, 63, 63,
                                 1, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63};
    int s = sec % 60, type, n25 = 0, n63 = 0, iono_k = 0, iono_blk = -1;

    if (s % 6 < 3) {
        type = 2 + s % 6;
    } else {
        int idx = (s / 6) * 3 + s % 6 - 3;
        type = rest[idx];
        for (int i = 0; i < idx; i++) { // which of the 8 MT25 / 17 free slots this is
            n25 += rest[i] == 25;
            n63 += rest[i] == 63;
        }
    }
    if (type == 63 && ion != NULL && ion->nband > 0) {
        int total = 0;
        for (int k = 0; k < ion->nband; k++)
            total += 1 + (ion->nigp[k] + 14) / 15;
        int item = ((sec / 60) * 17 + n63) % total; // position in the MT18/MT26 cycle
        for (iono_k = 0; item >= 1 + (ion->nigp[iono_k] + 14) / 15; iono_k++)
            item -= 1 + (ion->nigp[iono_k] + 14) / 15;
        iono_blk = item - 1; // -1: the band's MT18
        type = iono_blk < 0 ? 18 : 26;
    }
    memset(data, 0, SBAS_DATA_BITS);
    switch (type) {
        case 1: mt1(data); break;
        case 2: case 3: case 4: mt2_4(type, iode, data); break;
        case 7: mt7(data); break;
        case 9: sbas_mt9(pos, sec, data); break;
        case 10: mt10(data); break;
        case 25: mt25(n25, iode, data); break;
        case 18: mt18(ion, iono_k, data); break;
        case 26: mt26(ion, iono_k, iono_blk, data); break;
        default: break; // 63: null message
    }
    return type;
}
