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


#include "sbas.h"

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
 * inverted (gnss-sdr's SBAS decoder). state holds the last 6 bits across calls, start with 0. */
void sbas_encode(unsigned *state, const unsigned char *bits, int n, unsigned char *sym) {
    unsigned s = *state;

    for (int i = 0; i < n; i++) {
        s = ((unsigned) (bits[i] & 1) << 6) | (s >> 1); // newest bit is tap 0 (MSB of the octal mask)
        sym[2 * i] = parity(s & 0171);
        sym[2 * i + 1] = parity(s & 0133);
    }
    *state = s & 0x3F;
}
