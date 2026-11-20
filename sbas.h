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

// Bits are one per byte (0/1), most significant bit of each field first.
unsigned long sbas_crc24q(const unsigned char *bits, int n);
void sbas_frame(int type, const unsigned char *data, int sec, unsigned char *msg);
void sbas_encode(unsigned *state, const unsigned char *bits, int n, unsigned char *sym);

#endif /* SBAS_H */
