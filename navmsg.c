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

#include <string.h>
#include <math.h>
#include "gps.h"
#include "almanac.h"
#include "orbit.h"
#include "navmsg.h"

// Page number to SVID conversion for subframe 4&5
// See IS-GPS-200L, p.110, table 20-V
const unsigned long sbf4_svId[25] = {
    57UL, 0UL, 0UL, 0UL, 0UL, 57UL, 0UL, 0UL, 0UL, 0UL,
    57UL, 62UL, 52UL, 53UL, 54UL, 57UL, 55UL, 56UL, 58UL,
    59UL, 57UL, 60UL, 61UL, 62UL, 63UL
};

const unsigned long sbf5_svId[25] = {
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
    0UL, 0UL, 0UL, 0UL, 51UL
};

/* Place a 6-bit SV health value in slot 0-3 (first to last) of a 24 bit data word */
static unsigned long healthSlot(unsigned long health, int slot) {
    return (health & 0x3FUL) << (24 - 6 * slot);
}

/* Compute Subframe from Ephemeris
 * eph Ephemeris of given SV
 * sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, const char *msg, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]) {
    unsigned long wn;
    unsigned long toe;
    unsigned long toc;
    unsigned long iode;
    unsigned long iodc;
    long deltan;
    long cuc;
    long cus;
    long cic;
    long cis;
    long crc;
    long crs;
    unsigned long ecc;
    unsigned long sqrta;
    long m0;
    long omega0;
    long inc0;
    long aop;
    long omegadot;
    long idot;
    long af0;
    long af1;
    long af2;
    long tgd;

    unsigned long ura = 0UL;
    unsigned long svh = (unsigned long) eph.svh;
    unsigned long dataId = 1UL;

    unsigned long wna;
    unsigned long toa;

    signed long alpha0, alpha1, alpha2, alpha3;
    signed long beta0, beta1, beta2, beta3;
    signed long A0, A1;
    signed long dtls, dtlsf;
    unsigned long tot, wnt, wnlsf, dn;

    int sv, i;
    unsigned long svId;
    signed long delta_i; // Relative to i0 = 0.30 semicircles

    // FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
    //wn = (unsigned long)(eph.toe.week%1024);
    wn = 0UL;
    toe = (unsigned long) (eph.toe.sec / 16.0);
    toc = (unsigned long) (eph.toc.sec / 16.0);
    iode = (unsigned long) (eph.iode);
    iodc = (unsigned long) (eph.iodc);
    // Rounded: RINEX prints whole LSB multiples to ~13 digits, so a cast often lands one step low
    deltan = (long) round(eph.deltan / POW2_M43 / PI);
    cuc = (long) round(eph.cuc / POW2_M29);
    cus = (long) round(eph.cus / POW2_M29);
    cic = (long) round(eph.cic / POW2_M29);
    cis = (long) round(eph.cis / POW2_M29);
    crc = (long) round(eph.crc / POW2_M5);
    crs = (long) round(eph.crs / POW2_M5);
    ecc = (unsigned long) round(eph.ecc / POW2_M33);
    sqrta = (unsigned long) round(eph.sqrta / POW2_M19);
    m0 = (long) round(eph.m0 / POW2_M31 / PI);
    omega0 = (long) round(eph.omg0 / POW2_M31 / PI);
    inc0 = (long) round(eph.inc0 / POW2_M31 / PI);
    aop = (long) round(eph.aop / POW2_M31 / PI);
    omegadot = (long) round(eph.omgdot / POW2_M43 / PI);
    idot = (long) round(eph.idot / POW2_M43 / PI);
    af0 = (long) round(eph.af0 / POW2_M31);
    af1 = (long) round(eph.af1 / POW2_M43);
    af2 = (long) round(eph.af2 / POW2_M55);
    tgd = (long) round(eph.tgd / POW2_M31);

    alpha0 = (signed long) round(ionoutc.alpha0 / POW2_M30);
    alpha1 = (signed long) round(ionoutc.alpha1 / POW2_M27);
    alpha2 = (signed long) round(ionoutc.alpha2 / POW2_M24);
    alpha3 = (signed long) round(ionoutc.alpha3 / POW2_M24);
    beta0 = (signed long) round(ionoutc.beta0 / 2048.0);
    beta1 = (signed long) round(ionoutc.beta1 / 16384.0);
    beta2 = (signed long) round(ionoutc.beta2 / 65536.0);
    beta3 = (signed long) round(ionoutc.beta3 / 65536.0);
    A0 = (signed long) round(ionoutc.A0 / POW2_M30);
    A1 = (signed long) round(ionoutc.A1 / POW2_M50);
    dtls = (signed long) (ionoutc.dtls);
    tot = (unsigned long) (ionoutc.tot / 4096);
    wnt = (unsigned long) (ionoutc.wnt % 256);
    wnlsf = (unsigned long) (ionoutc.wnlsf % 256);
    dn = (unsigned long) ionoutc.dn;
    dtlsf = (signed long) (ionoutc.dtlsf);

    // Subframe 1
    sbf[0][0] = 0x8B0000UL << 6;
    sbf[0][1] = 0x1UL << 8;
    sbf[0][2] = ((wn & 0x3FFUL) << 20) | (ura << 14) | ((svh & 0x3FUL) << 8) | (((iodc >> 8)&0x3UL) << 6);
    sbf[0][3] = 0UL;
    sbf[0][4] = 0UL;
    sbf[0][5] = 0UL;
    sbf[0][6] = (tgd & 0xFFUL) << 6;
    sbf[0][7] = ((iodc & 0xFFUL) << 22) | ((toc & 0xFFFFUL) << 6);
    sbf[0][8] = ((af2 & 0xFFUL) << 22) | ((af1 & 0xFFFFUL) << 6);
    sbf[0][9] = (af0 & 0x3FFFFFUL) << 8;

    // Subframe 2
    sbf[1][0] = 0x8B0000UL << 6;
    sbf[1][1] = 0x2UL << 8;
    sbf[1][2] = ((iode & 0xFFUL) << 22) | ((crs & 0xFFFFUL) << 6);
    sbf[1][3] = ((deltan & 0xFFFFUL) << 14) | (((m0 >> 24)&0xFFUL) << 6);
    sbf[1][4] = (m0 & 0xFFFFFFUL) << 6;
    sbf[1][5] = ((cuc & 0xFFFFUL) << 14) | (((ecc >> 24)&0xFFUL) << 6);
    sbf[1][6] = (ecc & 0xFFFFFFUL) << 6;
    sbf[1][7] = ((cus & 0xFFFFUL) << 14) | (((sqrta >> 24)&0xFFUL) << 6);
    sbf[1][8] = (sqrta & 0xFFFFFFUL) << 6;
    sbf[1][9] = (toe & 0xFFFFUL) << 14;

    // Subframe 3
    sbf[2][0] = 0x8B0000UL << 6;
    sbf[2][1] = 0x3UL << 8;
    sbf[2][2] = ((cic & 0xFFFFUL) << 14) | (((omega0 >> 24)&0xFFUL) << 6);
    sbf[2][3] = (omega0 & 0xFFFFFFUL) << 6;
    sbf[2][4] = ((cis & 0xFFFFUL) << 14) | (((inc0 >> 24)&0xFFUL) << 6);
    sbf[2][5] = (inc0 & 0xFFFFFFUL) << 6;
    sbf[2][6] = ((crc & 0xFFFFUL) << 14) | (((aop >> 24)&0xFFUL) << 6);
    sbf[2][7] = (aop & 0xFFFFFFUL) << 6;
    sbf[2][8] = (omegadot & 0xFFFFFFUL) << 6;
    sbf[2][9] = ((iode & 0xFFUL) << 22) | ((idot & 0x3FFFUL) << 8);

    // Empty all the pages of subframes 4 and 5: alternating bits under the page's SV ID
    // (IS-GPS-200 Table 20-V, 0 = dummy SV for a missing almanac)
    for (i = 0; i < 25; i++) {
        for (int sf = 4; sf <= 5; sf++) {
            unsigned long *page = sbf[sf - 1 + i * 2];

            page[0] = 0x8B0000UL << 6; // Preamble for TLM
            page[1] = (unsigned long) sf << 8; // Subframe ID for HOW
            page[2] = (dataId << 28) | ((sf == 4 ? sbf4_svId[i] : sbf5_svId[i]) << 22) | ((EMPTY_WORD & 0xFFFFUL) << 6);
            for (int w = 3; w < 9; w++)
                page[w] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
            page[9] = (EMPTY_WORD & 0x3FFFFFUL) << 8;
        }
    }

    // Subframe 4, pages 2-5 and 7-10: almanac data for PRN 25 through 32
    for (sv = 24; sv < MAX_SAT; sv++) {
        if (sv >= 24 && sv <= 27) // PRN 25-28
            i = sv - 23; // Pages 2-5 (i = 1-4)
        else if (sv >= 28 && sv < MAX_SAT) // PRN 29-32
            i = sv - 22; // Page 7-10 (i = 6-9)

        if (alm->sv[sv].valid != 0) {
            svId = (unsigned long) (sv + 1);
            ecc = (unsigned long) round(alm->sv[sv].e / POW2_M21);
            toa = (unsigned long) (alm->sv[sv].toa.sec / POW2_12);
            delta_i = (signed long) round(alm->sv[sv].delta_i / POW2_M19);
            omegadot = (signed long) round(alm->sv[sv].omegadot / POW2_M38);
            sqrta = (unsigned long) round(alm->sv[sv].sqrta / POW2_M11);
            omega0 = (signed long) round(alm->sv[sv].omega0 / POW2_M23);
            aop = (signed long) round(alm->sv[sv].aop / POW2_M23);
            m0 = (signed long) round(alm->sv[sv].m0 / POW2_M23);
            af0 = (signed long) round(alm->sv[sv].af0 / POW2_M20);
            af1 = (signed long) round(alm->sv[sv].af1 / POW2_M38);

            sbf[3 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
            sbf[3 + i * 2][1] = 0x4UL << 8; // Subframe ID for HOW
            sbf[3 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
            sbf[3 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((delta_i & 0xFFFFUL) << 6);
            sbf[3 + i * 2][4] = ((omegadot & 0xFFFFUL) << 14) | ((unsigned long) alm->sv[sv].health << 6);
            sbf[3 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][6] = ((omega0 & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
            sbf[3 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
        }
    }

    // Subframe 4, page 18: ionospheric and UTC data
    if (ionoutc.vflg == true) {
        sbf[3 + 17 * 2][0] = 0x8B0000UL << 6;
        sbf[3 + 17 * 2][1] = 0x4UL << 8;
        sbf[3 + 17 * 2][2] = (dataId << 28) | (sbf4_svId[17] << 22) | ((alpha0 & 0xFFUL) << 14) | ((alpha1 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][3] = ((alpha2 & 0xFFUL) << 22) | ((alpha3 & 0xFFUL) << 14) | ((beta0 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][4] = ((beta1 & 0xFFUL) << 22) | ((beta2 & 0xFFUL) << 14) | ((beta3 & 0xFFUL) << 6);
        sbf[3 + 17 * 2][5] = (A1 & 0xFFFFFFUL) << 6;
        sbf[3 + 17 * 2][6] = ((A0 >> 8)&0xFFFFFFUL) << 6;
        sbf[3 + 17 * 2][7] = ((A0 & 0xFFUL) << 22) | ((tot & 0xFFUL) << 14) | ((wnt & 0xFFUL) << 6);
        sbf[3 + 17 * 2][8] = ((dtls & 0xFFUL) << 22) | ((wnlsf & 0xFFUL) << 14) | ((dn & 0xFFUL) << 6);
        sbf[3 + 17 * 2][9] = (dtlsf & 0xFFUL) << 22;
    }

    // Subframe 4, page 17: special message, 22 ASCII characters (IS-GPS-200 20.3.3.5.1.8), space padded.
    // Characters 1-2 in word 3, three per word in words 4-9, 21-22 in word 10 before 6 reserved bits.
    if (msg != NULL) {
        unsigned long c[SPECIAL_MSG_LEN];
        size_t n = strlen(msg);
        for (i = 0; i < SPECIAL_MSG_LEN; i++)
            c[i] = (unsigned long) (unsigned char) (i < (int) n ? msg[i] : ' ');
        unsigned long *page = sbf[3 + 16 * 2];
        page[2] = (dataId << 28) | (sbf4_svId[16] << 22) | (c[0] << 14) | (c[1] << 6);
        for (i = 0; i < 6; i++)
            page[3 + i] = (c[2 + 3 * i] << 22) | (c[3 + 3 * i] << 14) | (c[4 + 3 * i] << 6);
        page[9] = (c[20] << 22) | (c[21] << 14);
    }

    // Subframe 4, page 25: SV health data for PRN 25 through 32
    sbf[3 + 24 * 2][0] = 0x8B0000UL << 6;
    sbf[3 + 24 * 2][1] = 0x4UL << 8;
    sbf[3 + 24 * 2][2] = (dataId << 28) | (sbf4_svId[24] << 22);
    sbf[3 + 24 * 2][3] = 0UL;
    sbf[3 + 24 * 2][4] = 0UL;
    sbf[3 + 24 * 2][5] = 0UL;
    sbf[3 + 24 * 2][6] = 0UL;
    sbf[3 + 24 * 2][7] = 0UL;
    sbf[3 + 24 * 2][8] = 0UL;
    sbf[3 + 24 * 2][9] = 0UL;

    // Health of PRN 25 to 32: PRN 25 in word 8, PRN 26-29 in word 9, PRN 30-32 in word 10
    for (sv = 24; sv < MAX_SAT; sv++) {
        if (alm->sv[sv].valid != 0) {
            int k = sv - 24;
            int word = (k == 0) ? 7 : (k <= 4 ? 8 : 9);
            int slot = (k == 0) ? 3 : (k <= 4 ? k - 1 : k - 5);
            sbf[3 + 24 * 2][word] |= healthSlot(alm->sv[sv].health, slot);
        }
    }

    // SV configuration, 4 bits per PRN from page bit 69 on (PRN 1-4 in word 3, then 6 per word).
    // 0001: A-S off as in our HOW, Block II; 0000 is a reserved type receivers may reject.
    for (sv = 0; sv < MAX_SAT; sv++) {
        if (alm->sv[sv].valid != 0) {
            int b = sv < 4 ? 69 + 4 * sv : 91 + 30 * ((sv - 4) / 6) + 4 * ((sv - 4) % 6); // 1-based page bit
            sbf[3 + 24 * 2][(b - 1) / 30] |= 0x1UL << (26 - (b - 1) % 30);
        }
    }

    // Subframe 5, page 1-24: almanac data for PRN 1 through 24
    for (sv = 0; sv < 24; sv++) {
        i = sv;

        if (alm->sv[sv].svid != 0) {
            svId = (unsigned long) (sv + 1);
            ecc = (unsigned long) round(alm->sv[sv].e / POW2_M21);
            toa = (unsigned long) (alm->sv[sv].toa.sec / 4096.0);
            delta_i = (signed long) round(alm->sv[sv].delta_i / POW2_M19);
            omegadot = (signed long) round(alm->sv[sv].omegadot / POW2_M38);
            sqrta = (unsigned long) round(alm->sv[sv].sqrta / POW2_M11);
            omega0 = (signed long) round(alm->sv[sv].omega0 / POW2_M23);
            aop = (signed long) round(alm->sv[sv].aop / POW2_M23);
            m0 = (signed long) round(alm->sv[sv].m0 / POW2_M23);
            af0 = (signed long) round(alm->sv[sv].af0 / POW2_M20);
            af1 = (signed long) round(alm->sv[sv].af1 / POW2_M38);

            sbf[4 + i * 2][0] = 0x8B0000UL << 6; // Preamble
            sbf[4 + i * 2][1] = 0x5UL << 8; // Subframe ID
            sbf[4 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
            sbf[4 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((delta_i & 0xFFFFUL) << 6);
            sbf[4 + i * 2][4] = ((omegadot & 0xFFFFUL) << 14) | ((unsigned long) alm->sv[sv].health << 6);
            sbf[4 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][6] = ((omega0 & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
            sbf[4 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
        }
    }

    // Subframe 5, page 25: SV health data for PRN 1 through 24
    wna = (unsigned long) (eph.toe.week % 256);
    toa = (unsigned long) (eph.toe.sec / 4096.0);

    for (sv = 0; sv < MAX_SAT; sv++) {
        if (alm->sv[sv].svid != 0) // Valid almanac is available
        {
            wna = (unsigned long) (alm->sv[sv].toa.week % 256);
            toa = (unsigned long) (alm->sv[sv].toa.sec / 4096.0);
            break;
        }
    }

    sbf[4 + 24 * 2][0] = 0x8B0000UL << 6;
    sbf[4 + 24 * 2][1] = 0x5UL << 8;
    sbf[4 + 24 * 2][2] = (dataId << 28) | (sbf5_svId[24] << 22) | ((toa & 0xFFUL) << 14) | ((wna & 0xFFUL) << 6);
    sbf[4 + 24 * 2][3] = 0UL;
    sbf[4 + 24 * 2][4] = 0UL;
    sbf[4 + 24 * 2][5] = 0UL;
    sbf[4 + 24 * 2][6] = 0UL;
    sbf[4 + 24 * 2][7] = 0UL;
    sbf[4 + 24 * 2][8] = 0UL;
    sbf[4 + 24 * 2][9] = 0UL;

    // Health of PRN 1 to 24: four 6-bit values per word, words 4 to 9
    for (sv = 0; sv < 24; sv++) {
        if (alm->sv[sv].valid != 0) {
            sbf[4 + 24 * 2][3 + sv / 4] |= healthSlot(alm->sv[sv].health, sv % 4);
        }
    }
}

/* Count number of bits set to 1
 * v long word in which bits are counted
 * Count of bits set to 1
 */
static unsigned long countBits(unsigned long v) {
    unsigned long c;
    const int S[] = {1, 2, 4, 8, 16};
    const unsigned long B[] = {
        0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF
    };

    c = v;
    c = ((c >> S[0]) & B[0]) + (c & B[0]);
    c = ((c >> S[1]) & B[1]) + (c & B[1]);
    c = ((c >> S[2]) & B[2]) + (c & B[2]);
    c = ((c >> S[3]) & B[3]) + (c & B[3]);
    c = ((c >> S[4]) & B[4]) + (c & B[4]);

    return (c);
}

/* Compute the Checksum for one given word of a subframe
 * source The input data
 * nib Does this word contain non-information-bearing bits?
 * Computed Checksum
 */
static unsigned long computeChecksum(unsigned long source, int nib) {
    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Source data bits, d1, d2, ..., d24
    Bits  5 to  0 = Empty parity bits
     */

    /*
    Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
    Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
    Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
     */

    /*
                      1            2           3
    bit    12 3456 7890 1234 5678 9012 3456 7890
    ---    -------------------------------------
    D25    11 1011 0001 1111 0011 0100 1000 0000
    D26    01 1101 1000 1111 1001 1010 0100 0000
    D27    10 1110 1100 0111 1100 1101 0000 0000
    D28    01 0111 0110 0011 1110 0110 1000 0000
    D29    10 1011 1011 0001 1111 0011 0100 0000
    D30    00 1011 0111 1010 1000 1001 1100 0000
     */

    unsigned long bmask[6] = {
        0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
        0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL
    };

    unsigned long D;
    unsigned long d = source & 0x3FFFFFC0UL;
    unsigned long D29 = (source >> 31)&0x1UL;
    unsigned long D30 = (source >> 30)&0x1UL;

    if (nib) // Non-information bearing bits for word 2 and 10
    {
        /*
        Solve bits 23 and 24 to preserve parity check
        with zeros in bits 29 and 30.
         */

        if ((D30 + countBits(bmask[4] & d)) % 2)
            d ^= (0x1UL << 6);
        if ((D29 + countBits(bmask[5] & d)) % 2)
            d ^= (0x1UL << 7);
    }

    D = d;
    if (D30)
        D ^= 0x3FFFFFC0UL;

    D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
    D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
    D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
    D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
    D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
    D |= ((D29 + countBits(bmask[5] & d)) % 2);

    D &= 0x3FFFFFFFUL;
    D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits
    return (D);
}
void generateNavMsg(gpstime_t g, channel_t *chan, int init) {
    int iwrd, isbf;
    gpstime_t g0;
    unsigned long wn, tow;
    unsigned sbfwrd;
    unsigned long prevwrd;
    int nib;

    g0.week = g.week;
    g0.sec = (double) (((unsigned long) (g.sec + 0.5)) / 30UL) * 30.0; // Align with the full frame length = 30 sec
    chan->g0 = g0; // Data bit reference time

    wn = (unsigned long) (g0.week % 1024);
    tow = ((unsigned long) g0.sec) / 6UL;
    // All satellites send the same subframe 4/5 page: the frame count of the week modulo 25
    int ipage = (int) ((tow / 5UL) % 25UL);

    // Initialize the subframe 5
    if (init == 1) {
        prevwrd = 0UL;

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            sbfwrd = chan->sbf[4 + ((ipage + 24) % 25) * 2][iwrd]; // Previous frame's page

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[iwrd];
        }
    } else {
        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF * N_SBF + iwrd];

            prevwrd = chan->dwrd[iwrd];
        }
    }

    // Generate subframe words
    for (isbf = 0; isbf < N_SBF; isbf++) {
        tow = (tow + 1) % 100800; // TOW count is 0-100799; the last HOW of a week announces 0

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++) {
            if (isbf < 3) // Subframes 1-3
                sbfwrd = chan->sbf[isbf][iwrd];
            else if (isbf == 3) // Subframe 4
                sbfwrd = chan->sbf[3 + ipage * 2][iwrd];
            else // Subframe 5 
                sbfwrd = chan->sbf[4 + ipage * 2][iwrd];

            // Add transmission week number to Subframe 1
            if ((isbf == 0)&&(iwrd == 2))
                sbfwrd |= (wn & 0x3FFUL) << 20;

            // Add TOW-count message into HOW
            if (iwrd == 1)
                sbfwrd |= ((tow & 0x1FFFFUL) << 13);

            // Compute checksum
            sbfwrd |= (prevwrd << 30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            nib = ((iwrd == 1) || (iwrd == 9)) ? 1 : 0; // Non-information bearing bits for word 2 and 10
            chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd] = computeChecksum(sbfwrd, nib);

            prevwrd = chan->dwrd[(isbf + 1) * N_DWRD_SBF + iwrd];
        }
    }
}
