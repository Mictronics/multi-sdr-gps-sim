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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <curl/curl.h>
#include <zlib.h>
#include "sdr.h"
#include "gui.h"
#include "fifo.h"
#include "almanac.h"
#include "gps-sim.h"

// Last leap second event, 2016/12/31: used when a RINEX header gives no WN_LSF and DN.
#define LEAP_DEFAULT_WN 1929
#define LEAP_DEFAULT_DN 7
// Offset GPS time to UTC since that event; to be updated when a new leap second is introduced.
#define LEAP_DEFAULT_DTLS 18

static char rinex_date[21];

const int sinTable512[] = {
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
    50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 91, 94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100, 97,
    94, 91, 89, 86, 83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 53, 50,
    47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 2,
    -2, -5, -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97, -100, -103, -105, -108, -111, -114, -116, -119, -122, -125, -127, -130, -132, -135, -138,
    -140, -143, -145, -148, -150, -153, -155, -157, -160, -162, -164, -167, -169, -171, -173, -176,
    -178, -180, -182, -184, -186, -188, -190, -192, -194, -196, -198, -200, -202, -204, -205, -207,
    -209, -210, -212, -214, -215, -217, -218, -220, -221, -223, -224, -225, -227, -228, -229, -230,
    -232, -233, -234, -235, -236, -237, -238, -239, -240, -241, -241, -242, -243, -244, -244, -245,
    -245, -246, -247, -247, -248, -248, -248, -249, -249, -249, -249, -250, -250, -250, -250, -250,
    -250, -250, -250, -250, -250, -249, -249, -249, -249, -248, -248, -248, -247, -247, -246, -245,
    -245, -244, -244, -243, -242, -241, -241, -240, -239, -238, -237, -236, -235, -234, -233, -232,
    -230, -229, -228, -227, -225, -224, -223, -221, -220, -218, -217, -215, -214, -212, -210, -209,
    -207, -205, -204, -202, -200, -198, -196, -194, -192, -190, -188, -186, -184, -182, -180, -178,
    -176, -173, -171, -169, -167, -164, -162, -160, -157, -155, -153, -150, -148, -145, -143, -140,
    -138, -135, -132, -130, -127, -125, -122, -119, -116, -114, -111, -108, -105, -103, -100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11, -8, -5, -2
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
const double ant_pat_db[37] = {
    0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22,
    4.89, 5.56, 6.22, 6.89, 7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56
};
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

static int allocatedSat[MAX_SAT];

/* Subtract two vectors of double
 * y Result of subtraction
 * x1 Minuend of subtraction
 * x2 Subtrahend of subtraction
 */
static void subVect(double *y, const double *x1, const double *x2) {
    y[0] = x1[0] - x2[0];
    y[1] = x1[1] - x2[1];
    y[2] = x1[2] - x2[2];

    return;
}

/* Compute Norm of Vector
 * x Input vector
 * Length (Norm) of the input vector
 */
static double normVect(const double *x) {
    return (sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
}

/* Generate the C/A code sequence for a given Satellite Vehicle PRN
 * prn PRN number of the Satellite Vehicle
 * ca Caller-allocated integer array of 1023 bytes
 */
static void codegen(int *ca, int prn) {
    int delay[] = {
        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862
    };

    int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
    int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
    int c1, c2;
    int i, j;

    if (prn < 1 || prn > 32)
        return;

    for (i = 0; i < N_DWRD_SBF; i++)
        r1[i] = r2[i] = -1;

    for (i = 0; i < CA_SEQ_LEN; i++) {
        g1[i] = r1[9];
        g2[i] = r2[9];
        c1 = r1[2] * r1[9];
        c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];

        for (j = 9; j > 0; j--) {
            r1[j] = r1[j - 1];
            r2[j] = r2[j - 1];
        }
        r1[0] = c1;
        r2[0] = c2;
    }

    for (i = 0, j = CA_SEQ_LEN - delay[prn - 1]; i < CA_SEQ_LEN; i++, j++)
        ca[i] = (1 - g1[i] * g2[j % CA_SEQ_LEN]) / 2;

    return;
}

/* Expand a GPS week known modulo 256 to the full week closest to ref_week */
static int resolveWeek(int week256, int ref_week) {
    int week = ref_week - ref_week % 256 + week256;

    if (week - ref_week > 128)
        week -= 256;
    else if (ref_week - week > 128)
        week += 256;

    return week;
}

/* Convert a UTC date into a GPS date
 * t input date in UTC form
 * g output date in GPS form
 */
static void date2gps(const datetime_t *t, gpstime_t *g) {
    int doy[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int ye;
    int de;
    int lpdays;

    ye = t->y - 1980;

    // Compute the number of leap days since Jan 5/Jan 6, 1980.
    lpdays = ye / 4 + 1;
    if ((ye % 4) == 0 && t->m <= 2)
        lpdays--;

    // Compute the number of days elapsed since Jan 5/Jan 6, 1980.
    de = ye * 365 + doy[t->m - 1] + t->d + lpdays - 6;

    // Convert time to GPS weeks and seconds.
    g->week = de / 7;
    g->sec = (double) (de % 7) * SECONDS_IN_DAY + t->hh * SECONDS_IN_HOUR
            + t->mm * SECONDS_IN_MINUTE + t->sec;

    return;
}

static void gps2date(const gpstime_t *g, datetime_t *t) {
    // Convert Julian day number to calendar date
    int c = (int) (7 * g->week + floor(g->sec / 86400.0) + 2444245.0) + 1537;
    int d = (int) ((c - 122.1) / 365.25);
    int e = 365 * d + d / 4;
    int f = (int) ((c - e) / 30.6001);

    t->d = c - e - (int) (30.6001 * f);
    t->m = f - 1 - 12 * (f / 14);
    t->y = d - 4715 - ((7 + t->m) / 10);

    t->hh = ((int) (g->sec / 3600.0)) % 24;
    t->mm = ((int) (g->sec / 60.0)) % 60;
    t->sec = g->sec - 60.0 * floor(g->sec / 60.0);

    return;
}

/* Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Height
 * xyz Input Array of X, Y and Z ECEF coordinates
 * llh Output Array of Latitude, Longitude and Height
 */
static void xyz2llh(const double *xyz, double *llh) {
    double a, eps, e, e2;
    double x, y, z;
    double rho2, dz, zdz, nh, slat, n, dz_new;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;

    eps = 1.0e-3;
    e2 = e*e;

    if (!isfinite(normVect(xyz)) || normVect(xyz) < eps) {
        // Invalid ECEF vector
        llh[0] = 0.0;
        llh[1] = 0.0;
        llh[2] = -a;

        return;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    rho2 = x * x + y*y;
    dz = e2*z;

    for (int iter = 0; iter < 20; iter++) { // Converges in a few steps
        zdz = z + dz;
        nh = sqrt(rho2 + zdz * zdz);
        slat = zdz / nh;
        n = a / sqrt(1.0 - e2 * slat * slat);
        dz_new = n * e2*slat;

        if (fabs(dz - dz_new) < eps)
            break;

        dz = dz_new;
    }

    llh[0] = atan2(zdz, sqrt(rho2));
    llh[1] = atan2(y, x);
    llh[2] = nh - n;

    return;
}

/* Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 * llh Input Array of Latitude, Longitude and Height
 * xyz Output Array of X, Y and Z ECEF coordinates
 */
static void llh2xyz(const double *llh, double *xyz) {
    double n;
    double a;
    double e;
    double e2;
    double clat;
    double slat;
    double clon;
    double slon;
    double d, nph;
    double tmp;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;
    e2 = e*e;

    clat = cos(llh[0]);
    slat = sin(llh[0]);
    clon = cos(llh[1]);
    slon = sin(llh[1]);
    d = e*slat;

    n = a / sqrt(1.0 - d * d);
    nph = n + llh[2];

    tmp = nph*clat;
    xyz[0] = tmp*clon;
    xyz[1] = tmp*slon;
    xyz[2] = ((1.0 - e2) * n + llh[2]) * slat;

    return;
}

/* Compute the intermediate matrix for LLH to ECEF
 * llh Input position in Latitude-Longitude-Height format
 * t Three-by-Three output matrix
 */
static void ltcmat(const double *llh, double t[3][3]) {
    double slat, clat;
    double slon, clon;

    slat = sin(llh[0]);
    clat = cos(llh[0]);
    slon = sin(llh[1]);
    clon = cos(llh[1]);

    t[0][0] = -slat*clon;
    t[0][1] = -slat*slon;
    t[0][2] = clat;
    t[1][0] = -slon;
    t[1][1] = clon;
    t[1][2] = 0.0;
    t[2][0] = clat*clon;
    t[2][1] = clat*slon;
    t[2][2] = slat;

    return;
}

/* Convert Earth-centered Earth-Fixed to ?
 * xyz Input position as vector in ECEF format
 * t Intermediate matrix computed by \ref ltcmat
 * neu Output position as North-East-Up format
 */
static void ecef2neu(const double *xyz, double t[3][3], double *neu) {
    neu[0] = t[0][0] * xyz[0] + t[0][1] * xyz[1] + t[0][2] * xyz[2];
    neu[1] = t[1][0] * xyz[0] + t[1][1] * xyz[1] + t[1][2] * xyz[2];
    neu[2] = t[2][0] * xyz[0] + t[2][1] * xyz[1] + t[2][2] * xyz[2];

    return;
}

/* Convert North-East-Up to Azimuth + Elevation
 * neu Input position in North-East-Up format
 * azel Output array of azimuth + elevation as double
 */
static void neu2azel(double *azel, const double *neu) {
    double ne;

    azel[0] = atan2(neu[1], neu[0]);
    if (azel[0] < 0.0)
        azel[0] += (2.0 * PI);

    ne = sqrt(neu[0] * neu[0] + neu[1] * neu[1]);
    azel[1] = atan2(neu[2], ne);

    return;
}

/* Compute Satellite position, velocity and clock at given time
 * eph Ephemeris data of the satellite
 * g GPS time at which position is to be computed
 * pos Computed position (vector)
 * vel Computed velocity (vector)
 * clk Computed clock
 */
static void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk) {
    // Computing Satellite Velocity using the Broadcast Ephemeris
    // http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

    double tk;
    double mk;
    double ek;
    double ekold;
    double ekdot;
    double cek, sek;
    double pk;
    double pkdot;
    double c2pk, s2pk;
    double uk;
    double ukdot;
    double cuk, suk;
    double ok;
    double sok, cok;
    double ik;
    double ikdot;
    double sik, cik;
    double rk;
    double rkdot;
    double xpk, ypk;
    double xpkdot, ypkdot;

    double relativistic, OneMinusecosE, tmp;

    tk = g.sec - eph.toe.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    mk = eph.m0 + eph.n*tk;
    ek = mk;
    ekold = ek + 1.0;

    OneMinusecosE = 1.0 - eph.ecc * cos(ek);
    for (int iter = 0; iter < 50 && fabs(ek - ekold) > 1.0E-14; iter++) { // Bounded: bad eccentricity may not converge
        ekold = ek;
        OneMinusecosE = 1.0 - eph.ecc * cos(ekold);
        ek = ek + (mk - ekold + eph.ecc * sin(ekold)) / OneMinusecosE;
    }

    sek = sin(ek);
    cek = cos(ek);

    ekdot = eph.n / OneMinusecosE;

    relativistic = -4.442807633E-10 * eph.ecc * eph.sqrta*sek;

    pk = atan2(eph.sq1e2*sek, cek - eph.ecc) + eph.aop;
    pkdot = eph.sq1e2 * ekdot / OneMinusecosE;

    s2pk = sin(2.0 * pk);
    c2pk = cos(2.0 * pk);

    uk = pk + eph.cus * s2pk + eph.cuc*c2pk;
    suk = sin(uk);
    cuk = cos(uk);
    ukdot = pkdot * (1.0 + 2.0 * (eph.cus * c2pk - eph.cuc * s2pk));

    rk = eph.A * OneMinusecosE + eph.crc * c2pk + eph.crs*s2pk;
    rkdot = eph.A * eph.ecc * sek * ekdot + 2.0 * pkdot * (eph.crs * c2pk - eph.crc * s2pk);

    ik = eph.inc0 + eph.idot * tk + eph.cic * c2pk + eph.cis*s2pk;
    sik = sin(ik);
    cik = cos(ik);
    ikdot = eph.idot + 2.0 * pkdot * (eph.cis * c2pk - eph.cic * s2pk);

    xpk = rk*cuk;
    ypk = rk*suk;
    xpkdot = rkdot * cuk - ypk*ukdot;
    ypkdot = rkdot * suk + xpk*ukdot;

    ok = eph.omg0 + tk * eph.omgkdot - OMEGA_EARTH * eph.toe.sec;
    sok = sin(ok);
    cok = cos(ok);

    pos[0] = xpk * cok - ypk * cik*sok;
    pos[1] = xpk * sok + ypk * cik*cok;
    pos[2] = ypk*sik;

    tmp = ypkdot * cik - ypk * sik*ikdot;

    vel[0] = -eph.omgkdot * pos[1] + xpkdot * cok - tmp*sok;
    vel[1] = eph.omgkdot * pos[0] + xpkdot * sok + tmp*cok;
    vel[2] = ypk * cik * ikdot + ypkdot*sik;

    // Satellite clock correction
    tk = g.sec - eph.toc.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk<-SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    clk[0] = eph.af0 + tk * (eph.af1 + tk * eph.af2) + relativistic - eph.tgd;
    clk[1] = eph.af1 + 2.0 * tk * eph.af2;

    return;
}

/* Place a 6-bit SV health value in slot 0-3 (first to last) of a 24 bit data word */
static unsigned long healthSlot(unsigned long health, int slot) {
    return (health & 0x3FUL) << (24 - 6 * slot);
}

/* Compute Subframe from Ephemeris
 * eph Ephemeris of given SV
 * sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]) {
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

/* Replace all 'E' exponential designators to 'D'
 * str String in which all occurrences of 'E' are replaced with *  'D'
 * len Length of input string in bytes
 * Number of characters replaced
 */
static int replaceExpDesignator(char *str, int len) {
    int i, n = 0;

    for (i = 0; i < len; i++) {
        if (str[i] == 0) {
            break;
        }

        if (str[i] == 'D' || str[i] == 'd') {
            n++;
            str[i] = 'E';
        }
    }

    return (n);
}

static double subGpsTime(gpstime_t g1, gpstime_t g0) {
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double) (g1.week - g0.week) * SECONDS_IN_WEEK;

    return (dt);
}

static gpstime_t incGpsTime(gpstime_t g0, double dt) {
    gpstime_t g1;

    g1.week = g0.week;
    g1.sec = g0.sec + dt;

    g1.sec = round(g1.sec * 1000.0) / 1000.0; // Avoid rounding error

    while (g1.sec >= SECONDS_IN_WEEK) {
        g1.sec -= SECONDS_IN_WEEK;
        g1.week++;
    }

    while (g1.sec < 0.0) {
        g1.sec += SECONDS_IN_WEEK;
        g1.week--;
    }

    return (g1);
}

/* Parse a fixed-width RINEX field. Fortran 'D' exponents are accepted.
 * line RINEX line
 * col First column (0 based)
 * len Field width in characters
 * Parsed value, 0 if the line is too short
 */
static double rinexField(const char *line, int col, int len) {
    char tmp[24];

    if ((int) strlen(line) <= col || len >= (int) sizeof (tmp))
        return 0.0;

    strncpy(tmp, line + col, len);
    tmp[len] = 0;
    replaceExpDesignator(tmp, len);

    return atof(tmp);
}

static bool rinexLabel(const char *line, const char *label) {
    return strlen(line) >= 60 && strncmp(line + 60, label, strlen(label)) == 0;
}

/* Read the RINEX header: version check, iono/UTC parameters and leap seconds.
 * Returns 0 on success, -2 for an unsupported version, -3 for a wrong file type.
 */
static int readRinexHeader(gzFile fp, ionoutc_t *ionoutc, bool rinex3) {
    char str[MAX_CHAR];
    int flags = 0x0;

    while (gzgets(fp, str, MAX_CHAR) != NULL) {
        if (rinexLabel(str, "END OF HEADER")) {
            break;
        } else if (rinexLabel(str, "RINEX VERSION / TYPE")) {
            double ver = rinexField(str, 0, 9);
            if (rinex3 ? (ver < 3.0 || ver >= 4.0) : ver > 3.0)
                return -2;
            if (str[20] != 'N')
                return -3;
        } else if (rinexLabel(str, "PGM / RUN BY / DATE")) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (!rinex3 && rinexLabel(str, "ION ALPHA")) {
            ionoutc->alpha0 = rinexField(str, 2, 12);
            ionoutc->alpha1 = rinexField(str, 14, 12);
            ionoutc->alpha2 = rinexField(str, 26, 12);
            ionoutc->alpha3 = rinexField(str, 38, 12);
            flags |= 0x1;
        } else if (!rinex3 && rinexLabel(str, "ION BETA")) {
            ionoutc->beta0 = rinexField(str, 2, 12);
            ionoutc->beta1 = rinexField(str, 14, 12);
            ionoutc->beta2 = rinexField(str, 26, 12);
            ionoutc->beta3 = rinexField(str, 38, 12);
            flags |= 0x1 << 1;
        } else if (!rinex3 && rinexLabel(str, "DELTA-UTC")) {
            ionoutc->A0 = rinexField(str, 3, 19);
            ionoutc->A1 = rinexField(str, 22, 19);
            ionoutc->tot = (int) rinexField(str, 41, 9);
            ionoutc->wnt = (int) rinexField(str, 50, 9);
            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (rinex3 && rinexLabel(str, "IONOSPHERIC CORR") && strncmp(str, "GPSA", 4) == 0) {
            ionoutc->alpha0 = rinexField(str, 5, 12);
            ionoutc->alpha1 = rinexField(str, 17, 12);
            ionoutc->alpha2 = rinexField(str, 29, 12);
            ionoutc->alpha3 = rinexField(str, 41, 12);
            flags |= 0x1;
        } else if (rinex3 && rinexLabel(str, "IONOSPHERIC CORR") && strncmp(str, "GPSB", 4) == 0) {
            ionoutc->beta0 = rinexField(str, 5, 12);
            ionoutc->beta1 = rinexField(str, 17, 12);
            ionoutc->beta2 = rinexField(str, 29, 12);
            ionoutc->beta3 = rinexField(str, 41, 12);
            flags |= 0x1 << 1;
        } else if (rinex3 && rinexLabel(str, "TIME SYSTEM CORR") && strncmp(str, "GPUT", 4) == 0) {
            ionoutc->A0 = rinexField(str, 5, 17);
            ionoutc->A1 = rinexField(str, 22, 16);
            ionoutc->tot = (int) rinexField(str, 38, 7);
            ionoutc->wnt = (int) rinexField(str, 45, 6);
            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (rinexLabel(str, "LEAP SECONDS") && (strncmp(str + 24, "   ", 3) == 0 || strncmp(str + 24, "GPS", 3) == 0)) {
            // Lines for other systems carry a system id after the four I6 values
            ionoutc->dtls = (int) rinexField(str, 0, 6);
            // The last event is optional; without it assume the 2016/12/31 one, after which the offset is unchanged
            ionoutc->dtlsf = ionoutc->dtls;
            ionoutc->wnlsf = LEAP_DEFAULT_WN;
            ionoutc->dn = LEAP_DEFAULT_DN;
            if (rinexField(str, 12, 6) > 0) {
                ionoutc->dtlsf = (int) rinexField(str, 6, 6);
                ionoutc->wnlsf = (int) rinexField(str, 12, 6);
                ionoutc->dn = (int) rinexField(str, 18, 6);
            }
            flags |= 0x1 << 3;
        }
    }

    ionoutc->have = flags;
    ionoutc->vflg = (flags == 0xF); // All iono/UTC lines read
    return 0;
}

/* Parse one ephemeris record: the first line is already in str, the 7 broadcast
 * orbit lines are read from fp.
 * Returns false if the file ends inside the record or the record is invalid.
 */
static bool readRinexRecord(gzFile fp, char *str, bool rinex3, int *sv, ephem_t *e) {
    // RINEX 3 columns are those of RINEX 2 shifted by one
    const int o = rinex3 ? 1 : 0;
    datetime_t t;
    gpstime_t g;
    int y;

    if (rinex3) {
        *sv = (int) rinexField(str, 1, 2) - 1;
        t.y = (int) rinexField(str, 4, 4);
        t.m = (int) rinexField(str, 9, 2);
        t.d = (int) rinexField(str, 12, 2);
        t.hh = (int) rinexField(str, 15, 2);
        t.mm = (int) rinexField(str, 18, 2);
        t.sec = (int) rinexField(str, 21, 2);
    } else {
        *sv = (int) rinexField(str, 0, 2) - 1;
        y = (int) rinexField(str, 3, 2);
        t.y = y < 80 ? 2000 + y : 1900 + y;
        t.m = (int) rinexField(str, 6, 2);
        t.d = (int) rinexField(str, 9, 2);
        t.hh = (int) rinexField(str, 12, 2);
        t.mm = (int) rinexField(str, 15, 2);
        t.sec = rinexField(str, 17, 5);
    }

    // SV CLK
    e->t = t;
    e->af0 = rinexField(str, 22 + o, 19);
    e->af1 = rinexField(str, 41 + o, 19);
    e->af2 = rinexField(str, 60 + o, 19);

    for (int line = 1; line <= 7; line++) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            return false;

        const double a = rinexField(str, 3 + o, 19);
        const double b = rinexField(str, 22 + o, 19);
        const double c = rinexField(str, 41 + o, 19);
        const double d = rinexField(str, 60 + o, 19);

        switch (line) {
            case 1:
                e->iode = (int) a;
                e->crs = b;
                e->deltan = c;
                e->m0 = d;
                break;
            case 2:
                e->cuc = a;
                e->ecc = b;
                e->cus = c;
                e->sqrta = d;
                break;
            case 3:
                e->toe.sec = a;
                e->cic = b;
                e->omg0 = c;
                e->cis = d;
                break;
            case 4:
                e->inc0 = a;
                e->crc = b;
                e->aop = c;
                e->omgdot = d;
                break;
            case 5:
                e->idot = a;
                e->code = (int) b;
                e->toe.week = (int) c;
                e->flag = (int) d;
                break;
            case 6:
                e->sva = (int) a;
                e->svh = (int) b;
                if ((e->svh > 0) && (e->svh < 32))
                    e->svh += 32; // Set MSB to 1
                e->tgd = c;
                e->iodc = (int) d;
                break;
            case 7:
                e->fit = b;
                break;
        }
    }

    if (*sv < 0 || *sv >= MAX_SAT || t.m < 1 || t.m > 12)
        return false;

    date2gps(&t, &g);
    e->toc = g;
    e->vflg = true;

    // Update the working variables
    e->A = e->sqrta * e->sqrta;
    e->n = sqrt(GM_EARTH / (e->A * e->A * e->A)) + e->deltan;
    e->sq1e2 = sqrt(1.0 - e->ecc * e->ecc);
    e->omgkdot = e->omgdot - OMEGA_EARTH;

    return true;
}

struct eph_rec {
    int sv;
    int index; // Position in the file, keeps the sort stable
    ephem_t eph;
};

static int compareEphRec(const void *a, const void *b) {
    const struct eph_rec *ra = (const struct eph_rec *) a;
    const struct eph_rec *rb = (const struct eph_rec *) b;

    if (ra->sv != rb->sv)
        return ra->sv - rb->sv;

    double dt = subGpsTime(ra->eph.toc, rb->eph.toc);
    if (dt != 0.0)
        return dt < 0.0 ? -1 : 1;

    return ra->index - rb->index;
}

/* Read ephemeris data from a RINEX 2 or 3 navigation file.
 * Records may come in any order (the IGS merged files are sorted by satellite).
 * A satellite listed twice with the same clock time keeps the later record.
 * tab Output, all records by satellite and clock time; tab->rec is NULL without records
 * rinex3 true for RINEX 3, false for RINEX 2
 * Number of records, negative on error
 */
int readRinex(eph_table_t *tab, ionoutc_t *ionoutc, const char *fname, bool rinex3) {
    char str[MAX_CHAR];
    struct eph_rec *recs = NULL;
    int nrec = 0, capacity = 0;
    int result;
    gzFile fp;

    memset(tab, 0, sizeof (*tab));

    if (fname == NULL || NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    if ((result = readRinexHeader(fp, ionoutc, rinex3)) != 0) {
        gzclose(fp);
        return result;
    }

    while (NULL != gzgets(fp, str, MAX_CHAR)) {
        // RINEX 3 files can hold other constellations, whose lines are skipped
        if (rinex3 && str[0] != 'G')
            continue;

        if (nrec == capacity) {
            capacity = capacity ? capacity * 2 : 512;
            struct eph_rec *grown = realloc(recs, capacity * sizeof (*recs));
            if (grown == NULL) {
                free(recs);
                gzclose(fp);
                return (-1);
            }
            recs = grown;
        }

        // Records may be invalid; they are consumed and dropped
        if (readRinexRecord(fp, str, rinex3, &recs[nrec].sv, &recs[nrec].eph)) {
            recs[nrec].index = nrec;
            nrec++;
        }
    }

    gzclose(fp);

    if (nrec == 0) {
        free(recs);
        return 0;
    }

    qsort(recs, nrec, sizeof (*recs), compareEphRec);

    tab->rec = malloc(nrec * sizeof (*tab->rec));
    if (tab->rec == NULL) {
        free(recs);
        return (-1);
    }

    for (int i = 0; i < nrec; i++) {
        // Of equal records of one satellite only the last one (sorted last) is kept
        if (i + 1 < nrec && recs[i + 1].sv == recs[i].sv && subGpsTime(recs[i + 1].eph.toc, recs[i].eph.toc) == 0.0)
            continue;
        tab->rec[tab->n++] = recs[i].eph;
        tab->first[recs[i].sv + 1] = tab->n;
    }
    // Satellites without records inherit the end of the previous one
    for (int sv = 1; sv <= MAX_SAT; sv++)
        if (tab->first[sv] < tab->first[sv - 1])
            tab->first[sv] = tab->first[sv - 1];

    free(recs);
    return tab->n;
}

/* Zero Klobuchar and UTC terms change nothing: the delay equals that without iono data, and UTC
 * follows from the leap seconds alone. */
bool ionoDefaults(ionoutc_t *io, gpstime_t g) {
    if (io->have == 0xF)
        return false;

    if (!(io->have & 0x1))
        io->alpha0 = io->alpha1 = io->alpha2 = io->alpha3 = 0.0;
    if (!(io->have & 0x2))
        io->beta0 = io->beta1 = io->beta2 = io->beta3 = 0.0;
    if (!(io->have & 0x4)) {
        io->A0 = io->A1 = 0.0;
        io->tot = ((int) g.sec / 4096) * 4096;
        io->wnt = g.week;
    }
    if (!(io->have & 0x8)) {
        io->dtls = io->dtlsf = LEAP_DEFAULT_DTLS;
        io->wnlsf = LEAP_DEFAULT_WN;
        io->dn = LEAP_DEFAULT_DN;
    }
    io->have = 0xF;
    io->vflg = true;
    return true;
}

void ephSelect(const eph_table_t *tab, gpstime_t t, ephem_t cur[MAX_SAT]) {
    for (int sv = 0; sv < MAX_SAT; sv++) {
        double best = EPH_MAX_AGE;

        cur[sv].vflg = false;
        for (int i = tab->first[sv]; i < tab->first[sv + 1]; i++) {
            double age = fabs(subGpsTime(tab->rec[i].toc, t));
            if (age <= best) { // The later of two equally close records wins
                best = age;
                cur[sv] = tab->rec[i];
            }
        }
    }
}

static double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel) {
    double iono_delay = 0.0;
    double E, phi_u, lam_u, F;

    if (ionoutc->enable == false)
        return (0.0); // No ionospheric delay

    E = azel[1] / PI;
    phi_u = llh[0] / PI;
    lam_u = llh[1] / PI;

    // Obliquity factor
    F = 1.0 + 16.0 * pow((0.53 - E), 3.0);

    if (ionoutc->vflg == false)
        iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    else {
        double t, psi, phi_i, lam_i, phi_m, phi_m2, phi_m3;
        double AMP, PER, X, X2, X4;

        // Earth's central angle between the user position and the earth projection of
        // ionospheric intersection point (semi-circles)
        psi = 0.0137 / (E + 0.11) - 0.022;

        // Geodetic latitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        phi_i = phi_u + psi * cos(azel[0]);
        if (phi_i > 0.416)
            phi_i = 0.416;
        else if (phi_i<-0.416)
            phi_i = -0.416;

        // Geodetic longitude of the earth projection of the ionospheric intersection point
        // (semi-circles)
        lam_i = lam_u + psi * sin(azel[0]) / cos(phi_i * PI);

        // Geomagnetic latitude of the earth projection of the ionospheric intersection
        // point (mean ionospheric height assumed 350 km) (semi-circles)
        phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
        phi_m2 = phi_m*phi_m;
        phi_m3 = phi_m2*phi_m;

        AMP = ionoutc->alpha0 + ionoutc->alpha1 * phi_m
                + ionoutc->alpha2 * phi_m2 + ionoutc->alpha3*phi_m3;
        if (AMP < 0.0)
            AMP = 0.0;

        PER = ionoutc->beta0 + ionoutc->beta1 * phi_m
                + ionoutc->beta2 * phi_m2 + ionoutc->beta3*phi_m3;
        if (PER < 72000.0)
            PER = 72000.0;

        // Local time (sec)
        t = SECONDS_IN_DAY / 2.0 * lam_i + g.sec;
        while (t >= SECONDS_IN_DAY)
            t -= SECONDS_IN_DAY;
        while (t < 0)
            t += SECONDS_IN_DAY;

        // Phase (radians)
        X = 2.0 * PI * (t - 50400.0) / PER;

        if (fabs(X) < 1.57) {
            X2 = X*X;
            X4 = X2*X2;
            iono_delay = F * (5.0e-9 + AMP * (1.0 - X2 / 2.0 + X4 / 24.0)) * SPEED_OF_LIGHT;
        } else
            iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
    }

    return (iono_delay);
}

/* Troposphere delay in metres: Saastamoinen zenith delay with a standard atmosphere (15 C and
 * 1013.25 hPa at sea level, 70 % humidity), as in RTKLIB, and the Black-Eisner mapping of
 * RTCA DO-229. Saastamoinen's own 1/sin(el) grows without bound near the horizon; this mapping
 * stays below 22.4, so a setting satellite keeps a smooth range.
 * llh Receiver latitude, longitude (rad) and height (m); 0 above 10 km
 */
double tropoDelay(const double *llh, const double *azel) {
    if (llh[2] < -100.0 || llh[2] > 1.0e4)
        return 0.0;

    double h = llh[2] < 0.0 ? 0.0 : llh[2];
    double pres = 1013.25 * pow(1.0 - 2.2557e-5 * h, 5.2568); // hPa
    double temp = 15.0 - 6.5e-3 * h + 273.16; // K
    double e = 6.108 * 0.7 * exp((17.15 * temp - 4684.0) / (temp - 38.45)); // Water vapour pressure, hPa
    double sel = sin(azel[1]);

    double dry = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * llh[0]) - 0.00028 * h / 1e3);
    double wet = 0.002277 * (1255.0 / temp + 0.05) * e;
    return (dry + wet) * 1.001 / sqrt(0.002001 + sel * sel);
}

/* Compute range between a satellite and the receiver
 * rho The computed range
 * eph Ephemeris data of the satellite
 * g GPS time at time of receiving the signal
 * xyz position of the receiver
 */
static void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]) {
    double pos[3], vel[3], clk[2];
    double los[3];
    double tau;
    double range;
    double xrot, yrot;

    double llh[3], neu[3];
    double tmat[3][3];

    // SV position at time of the pseudorange observation.
    satpos(eph, g, pos, vel, clk);

    // Receiver to satellite vector and light-time.
    subVect(los, pos, xyz);
    tau = normVect(los) / SPEED_OF_LIGHT;

    // Extrapolate the satellite position backwards to the transmission time.
    pos[0] -= vel[0] * tau;
    pos[1] -= vel[1] * tau;
    pos[2] -= vel[2] * tau;

    // Earth rotation correction. The change in velocity can be neglected.
    xrot = pos[0] + pos[1] * OMEGA_EARTH*tau;
    yrot = pos[1] - pos[0] * OMEGA_EARTH*tau;
    pos[0] = xrot;
    pos[1] = yrot;

    // New observer to satellite vector and satellite range.
    subVect(los, pos, xyz);
    range = normVect(los);
    rho->d = range;

    // Pseudorange.
    rho->range = range - SPEED_OF_LIGHT * clk[0];

    // Time of application.
    rho->g = g;

    // Azimuth and elevation angles.
    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);
    ecef2neu(los, tmat, neu);
    neu2azel(rho->azel, neu);

    rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
    rho->range += rho->iono_delay;
    if (ionoutc->enable) // --disable-iono switches off both atmosphere delays
        rho->range += tropoDelay(llh, rho->azel);
}

/* Compute the code phase for a given channel (satellite)
 * chan Channel on which we operate (is updated)
 * rho1 Current range, after \a dt has expired
 * dt delta-t (time difference) in seconds
 */
static void computeCodePhase(channel_t *chan, range_t rho1, double dt) {
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range) / dt;

    // Carrier and code frequency.
    chan->f_carr = -rhorate / LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g, chan->g0) + 6.0) - chan->rho0.range / SPEED_OF_LIGHT)*1000.0;

    ims = (int) ms;
    chan->code_phase = (ms - (double) ims) * CA_SEQ_LEN; // in chip

    chan->iword = ims / 600; // 1 word = 30 bits = 600 ms
    ims -= chan->iword * 600;

    chan->ibit = ims / 20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int) chan->code_phase]*2 - 1;
    chan->dataBit = (int) ((chan->dwrd[chan->iword]>>(29 - chan->ibit)) & 0x1UL)*2 - 1;

    // Save current pseudorange
    chan->rho0 = rho1;
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

static int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel) {
    double llh[3], neu[3];
    double pos[3], vel[3], clk[3], los[3];
    double tmat[3][3];

    if (eph.vflg == false)
        return (-1); // Invalid

    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);

    if (azel[1] * R2D > elvMask)
        return (1); // Visible
    // else
    return (0); // Invisible
}

static int allocateChannel(channel_t *chan, almanac_gps_t *alm, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz) {
    int nsat = 0;
    int i, sv;
    double azel[2];
    range_t rho;

    for (sv = 0; sv < MAX_SAT; sv++) {
        if (checkSatVisibility(eph[sv], grx, xyz, 0.0, azel) == 1) {
            nsat++; // Number of visible satellites

            if (allocatedSat[sv] == -1) // Visible but not allocated
            {
                // Allocated new satellite
                for (i = 0; i < MAX_CHAN; i++) {
                    if (chan[i].prn == 0) {
                        // Initialize channel
                        chan[i].prn = sv + 1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, alm, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;
                        chan[i].carr_phase = 0.0; // Arbitrary: receivers track the phase from wherever it starts
                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < MAX_CHAN)
                    allocatedSat[sv] = i;
            }
        } else if (allocatedSat[sv] >= 0) // Not visible but allocated
        {
            // Clear channel
            chan[allocatedSat[sv]].prn = 0;

            // Clear satellite allocation flag
            allocatedSat[sv] = -1;
        }
    }

    return (nsat);
}

struct download_target {
    const char *filename;
    FILE *stream;
};

static size_t write_download(void *buffer, size_t size, size_t nmemb, void *userdata) {
    struct download_target *out = (struct download_target *) userdata;
    if (!out->stream && !(out->stream = fopen(out->filename, "wb"))) {
        return 0; // Short write makes curl abort the transfer
    }
    return fwrite(buffer, size, nmemb, out->stream);
}

static int download_progress(void *userdata, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow) {
    NOTUSED(dltotal);
    NOTUSED(dlnow);
    NOTUSED(ultotal);
    NOTUSED(ulnow);
    return *(const atomic_bool *) userdata ? 1 : 0; // Non-zero aborts the transfer
}

/* Download url into filename. A failed transfer leaves no partial file.
 * The transfer is aborted when *cancel becomes true. */
CURLcode download_file(const char *url, const char *filename, const atomic_bool *cancel) {
    struct download_target out = {filename, NULL};
    CURLcode res = CURLE_FAILED_INIT;
    CURL *curl = curl_easy_init();

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1000L); // Give up on stalled transfers
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 15L);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, download_progress);
        curl_easy_setopt(curl, CURLOPT_XFERINFODATA, cancel);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_download);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }

    if (out.stream) {
        fclose(out.stream);
        if (res != CURLE_OK) {
            remove(filename);
        }
    }
    return res;
}

static bool is_connect_error(CURLcode res) {
    return res == CURLE_COULDNT_CONNECT || res == CURLE_COULDNT_RESOLVE_HOST || res == CURLE_OPERATION_TIMEDOUT;
}

/* Download the newest available broadcast ephemeris file, trying each source in turn.
 * RINEX 2: NOAA CORS daily GPS file, refreshed about hourly, but its records can lag hours behind.
 * RINEX 3: BKG merged files; today's BRDC00WRD_S is built from real-time streams and reaches the
 * current hour, BRDC00IGS_R only appears the day after. Today's file is tried first for both.
 */
static CURLcode download_nav_file(bool rinex3, const char *filename, const atomic_bool *cancel) {
    static const char *const rinex3_names[] = {"BRDC00WRD_S", "BRDC00WRD_R", "BRDC00IGS_R"};
    CURLcode res = CURLE_FAILED_INIT;
    char url[256];
    time_t now = time(NULL);

    for (int back = 0; back <= (rinex3 ? 3 : 2); back++) {
        time_t t = now - back * 86400;
        struct tm tm;
        gmtime_r(&t, &tm);
        int year = tm.tm_year + 1900;
        int doy = tm.tm_yday + 1;

        for (int n = 0; n < (rinex3 ? 3 : 1); n++) {
            if (rinex3) {
                snprintf(url, sizeof (url), "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/%s_%04d%03d0000_01D_MN.rnx.gz",
                        year, doy, rinex3_names[n], year, doy);
            } else {
                snprintf(url, sizeof (url), "https://noaa-cors-pds.s3.amazonaws.com/rinex/%04d/%03d/brdc%03d0.%02dn.gz",
                        year, doy, doy, year % 100);
            }
            // Retry connection errors: a round-robin DNS name can hand out an unreachable server
            for (int attempt = 0; attempt < 3; attempt++) {
                res = download_file(url, filename, cancel);
                if (res == CURLE_OK) {
                    gui_status_wprintw(GREEN, "Downloaded %s\n", url);
                    return res;
                }
                if (res == CURLE_ABORTED_BY_CALLBACK || !is_connect_error(res)) {
                    break;
                }
            }
            if (res == CURLE_ABORTED_BY_CALLBACK || is_connect_error(res)) {
                return res; // Cancelled, or the host is down so the other candidates would fail too
            }
        }
    }
    return res;
}

/* Read the list of user motions from the input file
 * xyz Output array of ECEF vectors for user motion
 * filename File name of the text input file
 * Returns number of user data motion records read, -1 on error
 */
static int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename) {
    FILE *fp;
    int numd;
    char str[MAX_CHAR];
    double t, x, y, z;

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++) {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z) != 4) // Read CSV line
            break;

        xyz[numd][0] = x;
        xyz[numd][1] = y;
        xyz[numd][2] = z;
    }

    fclose(fp);

    return (numd);
}

// Layout of the LS_FIX window: the channel table starts at the left, the
// scenario information sits in the column on the right.
#define LS_COL_INFO 40
#define LS_ROW_NAV 3
#define LS_ROW_CHAN 4
#define LS_ROW_XYZ 5
#define LS_ROW_LLH 6
#define LS_ROW_DURATION 7
#define LS_ROW_RINEX 8
#define LS_ROW_ALMANAC 9
#define LS_ROW_START 10
#define LS_ROW_SIMTIME 11
#define LS_COL_SIMTIME 57
#define LS_ROW_ELAPSED 12
#define LS_ROW_IONO 14

#define SAMPLE_PERIOD (1.0 / (double) TX_SAMPLERATE)

// Satellites with a record near the start time that make -s now keep the ephemeris unshifted
#define COVER_MIN_SATS 4

// State of the signal generation, shared by the steps of gps_thread_ep.
typedef struct {
    simulator_t *sim;
    eph_table_t table; // Every ephemeris record of the file
    ephem_t eph[MAX_SAT]; // Record in use for each satellite: the closest in time
    ionoutc_t ionoutc;
    almanac_gps_t *alm;
    channel_t chan[MAX_CHAN];
    double gain[MAX_CHAN];
    double ant_pat[37]; // Receiver antenna gain per 5 degree of boresight angle
    double (*xyz)[3]; // User position every 0.1 s
    int numd; // Number of positions
    gpstime_t g0; // Scenario start
    gpstime_t grx; // Receiver time
    short *iq_buff; // 0.1 s of 16 bit IQ samples
} gps_ctx_t;

/* List the allocated channels in the LS_FIX window. */
static void showChannels(const gps_ctx_t *c) {
    int row = LS_ROW_CHAN;

    for (int i = 0; i < MAX_CHAN; i++) {
        if (c->chan[i].prn > 0) {
            gui_mvwprintw(LS_FIX, row++, 1, "%02d %6.1f %5.1f %11.1f %5.1f", c->chan[i].prn,
                    c->chan[i].azel[0] * R2D, c->chan[i].azel[1] * R2D, c->chan[i].rho0.d, c->chan[i].rho0.iono_delay);
        }
    }
    gui_mvwprintw(LS_FIX, LS_ROW_NAV, LS_COL_INFO, "Nav: %02d satellites", row - LS_ROW_CHAN);
}

/* Move the ECEF position xyz by neu (north, east, up in metres) in its own local frame.
 * A fixed frame would make straight travel leave the Earth's surface: 785 m up after 100 km. */
static void addNeu(double *xyz, const double *neu) {
    double llh[3], t[3][3];

    xyz2llh(xyz, llh);
    ltcmat(llh, t);
    for (int k = 0; k < 3; k++)
        xyz[k] += t[0][k] * neu[0] + t[1][k] * neu[1] + t[2][k] * neu[2];
}

/* Set the start position, or the target given with -t, as the user motion. */
static bool initUserPosition(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    double llh[3];
    double neu[3];

    c->xyz = malloc(sizeof (double[USER_MOTION_SIZE][3]));
    if (c->xyz == NULL) {
        gui_status_wprintw(RED, "Failed to allocate user motion memory.\n");
        return false;
    }

    c->numd = sim->duration;

    llh[0] = sim->location.lat / R2D;
    llh[1] = sim->location.lon / R2D;
    llh[2] = sim->location.height;
    llh2xyz(llh, c->xyz[0]);

    if (!sim->target.valid) {
        // No target position given, use location
        sim->target.lat = sim->location.lat;
        sim->target.lon = sim->location.lon;
        sim->target.height = sim->location.height;
    } else {
        // Target given as distance and bearing from the user location
        neu[0] = sim->target.distance * cos((sim->target.bearing / 1000) / R2D);
        neu[1] = sim->target.distance * sin((sim->target.bearing / 1000) / R2D);
        neu[2] = sim->target.height;
        addNeu(c->xyz[0], neu);
    }

    for (int i = 1; i < c->numd; i++) {
        c->xyz[i][0] = c->xyz[0][0];
        c->xyz[i][1] = c->xyz[0][1];
        c->xyz[i][2] = c->xyz[0][2];
    }

    gui_show_location(&sim->location);
    return true;
}

/* Download (-f) or read the ephemeris. */
static bool loadEphemeris(gps_ctx_t *c) {
    simulator_t *sim = c->sim;

    if ((sim->nav_file_name == NULL) && (sim->use_ftp == false)) {
        gui_status_wprintw(RED, "GPS ephemeris file is not specified.\n");
        return false;
    }

    // The online download takes precedence over -e
    const char *nav_file = sim->nav_file_name;
    if (sim->use_ftp) {
        nav_file = sim->use_rinex3 ? RINEX3_FILE_NAME : RINEX2_FILE_NAME;
        gui_status_wprintw(GREEN, "Downloading RINEX v%u ephemeris.\n", sim->use_rinex3 ? 3 : 2);
        CURLcode res = download_nav_file(sim->use_rinex3, nav_file, &sim->main_exit);
        if (res != CURLE_OK) {
            gui_status_wprintw(RED, "Ephemeris download failed: %s\n", curl_easy_strerror(res));
            return false;
        }
    }

    c->ionoutc.enable = sim->ionosphere_enable;
    int nrec = readRinex(&c->table, &c->ionoutc, nav_file, sim->use_rinex3);

    if (nrec <= 0) { // 0 = no records, negative = file could not be read or has wrong format
        gui_status_wprintw(RED, "No ephemeris available.\n");
        return false;
    }

    if (sim->show_verbose) {
        const ionoutc_t *io = &c->ionoutc;
        if (io->vflg && io->enable) {
            gui_mvwprintw(LS_FIX, LS_ROW_IONO, LS_COL_INFO, "ION ALPHA %12.3e %12.3e %12.3e %12.3e",
                    io->alpha0, io->alpha1, io->alpha2, io->alpha3);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 1, LS_COL_INFO, "ION BETA  %12.3e %12.3e %12.3e %12.3e",
                    io->beta0, io->beta1, io->beta2, io->beta3);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 2, LS_COL_INFO, "DELTA UTC %12.3e %12.3e %9d  %9d",
                    io->A0, io->A1, io->tot, io->wnt);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 3, LS_COL_INFO, "LEAP SECONDS %d, then %d (week %d, day %d)", io->dtls, io->dtlsf, io->wnlsf, io->dn);
        } else {
            gui_mvwprintw(LS_FIX, LS_ROW_IONO, LS_COL_INFO, "Ionospheric data incomplete or disabled!");
        }
    }
    return true;
}

/* Replace the user motion with the one from the -m file. */
static bool loadMotionFile(gps_ctx_t *c) {
    simulator_t *sim = c->sim;

    if (sim->motion_file_name == NULL)
        return true;

    c->numd = readUserMotion(c->xyz, sim->motion_file_name);
    if (c->numd <= 0) {
        gui_status_wprintw(RED, "Failed to read user motion file.\n");
        return false;
    }
    gui_status_wprintw(GREEN, "%u user motion points applied.\n", c->numd);
    if (c->numd > sim->duration) {
        c->numd = sim->duration;
    }
    return true;
}

/* Number of satellites with a record within EPH_MAX_AGE of g. */
static int countCovered(const eph_table_t *tab, gpstime_t g) {
    ephem_t eph[MAX_SAT];
    int n = 0;

    ephSelect(tab, g, eph);
    for (int sv = 0; sv < MAX_SAT; sv++)
        n += eph[sv].vflg ? 1 : 0;
    return n;
}

/* Set the scenario start time g0, shifting the ephemeris to it for -s now. */
static bool selectStartTime(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    datetime_t ttmp;
    datetime_t tmin = {0}, tmax = {0};
    gpstime_t gmin = {0, 0.0}, gmax = {0, 0.0};
    gpstime_t gtmp;

    if (sim->time_overwrite) { // -s now: read the clock here, after the download
        time_t now = time(NULL);
        struct tm tm;
        gmtime_r(&now, &tm);
        sim->start = (datetime_t) {tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, (double) tm.tm_sec};
    }

    c->g0.week = -1; // Invalid start time
    if (sim->start.m >= 1) { // Zero month means no -s given
        date2gps(&sim->start, &c->g0);
    }
    if (sim->time_overwrite) // The clock gives UTC; GPS time is ahead by the leap seconds
        c->g0 = incGpsTime(c->g0, (c->ionoutc.have & 0x8) ? c->ionoutc.dtls : LEAP_DEFAULT_DTLS);

    // The NOAA file can lag hours behind; BKG's file of the day reaches the current hour
    if (sim->time_overwrite && sim->use_ftp && !sim->use_rinex3 && countCovered(&c->table, c->g0) < COVER_MIN_SATS) {
        gui_status_wprintw(YELLOW, "RINEX v2 file does not reach the start time, trying RINEX v3.\n");
        eph_table_t t3;
        ionoutc_t io3 = c->ionoutc;
        if (download_nav_file(true, RINEX3_FILE_NAME, &sim->main_exit) == CURLE_OK &&
                readRinex(&t3, &io3, RINEX3_FILE_NAME, true) > 0) {
            if (countCovered(&t3, c->g0) > countCovered(&c->table, c->g0)) {
                free(c->table.rec);
                c->table = t3;
                c->ionoutc = io3;
            } else {
                free(t3.rec);
            }
        }
    }

    // Earliest and latest clock time of the file
    gmin = gmax = c->table.rec[0].toc;
    tmin = tmax = c->table.rec[0].t;
    for (int i = 1; i < c->table.n; i++) {
        const ephem_t *e = &c->table.rec[i];
        if (subGpsTime(e->toc, gmin) < 0.0) {
            gmin = e->toc;
            tmin = e->t;
        }
        if (subGpsTime(e->toc, gmax) > 0.0) {
            gmax = e->toc;
            tmax = e->t;
        }
    }

    if (c->g0.week >= 0) { // Scenario start time has been set.
        if (sim->time_overwrite == true) {
            // Shifting moves the satellites away from where the real almanac puts them, so it is
            // only done for data that does not reach the start time (e.g. an old nav file)
            if (countCovered(&c->table, c->g0) >= COVER_MIN_SATS) {
                gui_status_wprintw(GREEN, "Ephemeris covers the start time, used as it is.\n");
            } else {
                gui_status_wprintw(RED, "Ephemeris does not reach the start time, shifting it: it no longer matches the\n"
                        "almanac and receivers may distrust it (use --disable-almanac).\n");
                gtmp.week = c->g0.week;
                gtmp.sec = (double) (((int) (c->g0.sec)) / 7200)*7200.0;

                double dsec = subGpsTime(gtmp, gmin);

                // Overwrite the UTC reference week number
                c->ionoutc.wnt = gtmp.week;
                c->ionoutc.tot = (int) gtmp.sec;

                // Overwrite the TOC and TOE to the scenario start time
                for (int i = 0; i < c->table.n; i++) {
                    ephem_t *e = &c->table.rec[i];
                    e->toc = incGpsTime(e->toc, dsec);
                    gps2date(&e->toc, &ttmp);
                    e->t = ttmp;
                    e->toe = incGpsTime(e->toe, dsec);
                }
            }
        } else if (subGpsTime(c->g0, gmin) < 0.0 || subGpsTime(gmax, c->g0) < 0.0) {
            gui_status_wprintw(RED, "Invalid start time.\n");
            gui_status_wprintw(RED, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                    tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec, gmin.week, gmin.sec);
            gui_status_wprintw(RED, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                    tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec, gmax.week, gmax.sec);
            return false;
        }
    } else {
        c->g0 = gmin;
        sim->start = tmin;
    }

    if (ionoDefaults(&c->ionoutc, c->g0))
        gui_status_wprintw(YELLOW, "Iono/UTC data incomplete in the file, sending neutral values for it.\n");

    gui_mvwprintw(LS_FIX, LS_ROW_RINEX, LS_COL_INFO, "RINEX date:      %s", rinex_date);
    gui_mvwprintw(LS_FIX, LS_ROW_START, LS_COL_INFO, "Start time:      %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)",
            sim->start.y, sim->start.m, sim->start.d, sim->start.hh, sim->start.mm, sim->start.sec, c->g0.week, c->g0.sec);
    if (sim->show_verbose) {
        gui_mvwprintw(LS_FIX, LS_ROW_SIMTIME, LS_COL_INFO, "Simulation time: ");
    }
    gui_mvwprintw(LS_FIX, LS_ROW_DURATION, LS_COL_INFO, "Duration:        %.1fs", ((double) c->numd) / 10.0);

    // The ephemerides in use at the start
    ephSelect(&c->table, c->g0, c->eph);

    int usable = 0;
    for (int sv = 0; sv < MAX_SAT; sv++)
        usable += c->eph[sv].vflg ? 1 : 0;
    if (usable == 0) {
        gui_status_wprintw(RED, "No ephemeris found close to the start time.\n");
        return false;
    }
    return true;
}

/* Download (-f) or read the almanac and check it is close to the scenario time. */
static bool loadAlmanac(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    datetime_t ttmp;
    gpstime_t gtmp;

    c->alm = almanac_init();
    if (sim->almanac_enable) {
        CURLcode res = sim->use_ftp ? almanac_download(&sim->main_exit) : almanac_read_file();

        switch (res) {
            case CURLE_OK:
                break;
            case CURLE_REMOTE_FILE_NOT_FOUND:
                gui_status_wprintw(RED, "Almanac file not found!\n");
                break;
            case CURLE_READ_ERROR:
                gui_status_wprintw(RED, "Error reading almanac file!\n");
                break;
            default:
                gui_status_wprintw(RED, "Almanac error, code: %d\n", res);
                break;
        }
    }

    if (sim->almanac_enable && c->alm->valid) {
        gtmp.sec = 0.0;
        gtmp.week = 0;
        for (int sv = 0; sv < MAX_SAT; sv++) {
            almanac_prn_t *a = &c->alm->sv[sv];
            if (a->valid != 0) {
                a->toa.week = resolveWeek(a->toa.week, c->g0.week);
                gtmp = a->toa;
                double dt = subGpsTime(a->toa, c->g0);
                if (dt < (-4.0 * SECONDS_IN_WEEK) || dt > (4.0 * SECONDS_IN_WEEK)) {
                    gui_status_wprintw(RED, "Invalid time of almanac.\n");
                    return false;
                }
            }
        }
        gps2date(&gtmp, &ttmp);
        gui_mvwprintw(LS_FIX, LS_ROW_ALMANAC, LS_COL_INFO, "Almanac date:    %4d/%02d/%02d,%02d:%02d:%02.0f",
                ttmp.y, ttmp.m, ttmp.d, ttmp.hh, ttmp.mm, ttmp.sec);
    } else {
        gui_mvwprintw(LS_FIX, LS_ROW_ALMANAC, LS_COL_INFO, "Almanac date:    Disabled or invalid.");
    }
    return true;
}

/* Allocate the satellites visible at the start and set up the antenna pattern. */
static void initChannels(gps_ctx_t *c) {
    for (int i = 0; i < MAX_CHAN; i++)
        c->chan[i].prn = 0;

    for (int sv = 0; sv < MAX_SAT; sv++)
        allocatedSat[sv] = -1;

    c->grx = incGpsTime(c->g0, 0.0);
    allocateChannel(c->chan, c->alm, c->eph, c->ionoutc, c->grx, c->xyz[0]);
    showChannels(c);

    for (int i = 0; i < 37; i++)
        c->ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

    c->grx = incGpsTime(c->grx, 0.1);
}

/* Move the user by the speed and heading set in the interactive mode. */
static void moveUser(gps_ctx_t *c, int iumd) {
    const target_t *target = &c->sim->target;
    double dir = (target->bearing / 1000) / R2D;
    double neu[3];

    // Stay at the current location, then move by one 0.1 s step
    neu[0] = (target->velocity * cos(dir)) * 0.1;
    neu[1] = (target->velocity * sin(dir)) * 0.1;
    neu[2] = target->vertical_speed * 0.1;

    memcpy(c->xyz[iumd], c->xyz[iumd - 1], sizeof (c->xyz[iumd]));
    addNeu(c->xyz[iumd], neu);
}

/* Update range, code phase and signal gain of every channel for the next 0.1 s. */
static void updateChannels(gps_ctx_t *c, int iumd) {
    for (int i = 0; i < MAX_CHAN; i++) {
        channel_t *ch = &c->chan[i];
        if (ch->prn > 0) {
            range_t rho;

            computeRange(&rho, c->eph[ch->prn - 1], &c->ionoutc, c->grx, c->xyz[iumd]);

            ch->azel[0] = rho.azel[0];
            ch->azel[1] = rho.azel[1];

            computeCodePhase(ch, rho, 0.1);
            double path_loss = 20200000.0 / rho.d;

            // Receiver antenna gain by boresight angle
            int ibs = (int) ((90.0 - rho.azel[1] * R2D) / 5.0);
            c->gain[i] = path_loss * c->ant_pat[ibs];
        }
    }
}

/* Generate 0.1 s of IQ samples into c->iq_buff. */
static void generateSamples(gps_ctx_t *c) {
    channel_t *chan = c->chan;
    const double *gain = c->gain;
    const double delt = SAMPLE_PERIOD;

    for (int isamp = 0; isamp < NUM_IQ_SAMPLES; isamp++) {
        int i_acc = 0;
        int q_acc = 0;

        for (int i = 0; i < MAX_CHAN; i++) {
            if (chan[i].prn > 0) {
                int iTable = (int) floor(chan[i].carr_phase * 512.0) & 511; // Masked: a NaN phase must not index outside
                // dataBit and codeCA are -1 or 1; cosine is the sine 90 degrees (128 steps) ahead
                int ip = chan[i].dataBit * chan[i].codeCA * sinTable512[(iTable + 128) & 511] * gain[i];
                int qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

                // Accumulate for all visible satellites
                i_acc += ip;
                q_acc += qp;

                // Update code phase
                chan[i].code_phase += chan[i].f_code * delt;

                if (chan[i].code_phase >= CA_SEQ_LEN) {
                    chan[i].code_phase -= CA_SEQ_LEN;

                    chan[i].icode++;

                    if (chan[i].icode >= 20) { // 20 C/A codes = 1 navigation data bit
                        chan[i].icode = 0;
                        chan[i].ibit++;

                        if (chan[i].ibit >= 30) { // 30 navigation data bits = 1 word
                            chan[i].ibit = 0;
                            chan[i].iword++;
                        }

                        // Set new navigation data bit
                        chan[i].dataBit = (int) ((chan[i].dwrd[chan[i].iword]>>(29 - chan[i].ibit)) & 0x1UL)*2 - 1;
                    }
                }

                // Set current code chip
                chan[i].codeCA = chan[i].ca[(int) chan[i].code_phase]*2 - 1;

                // Update carrier phase
                chan[i].carr_phase += chan[i].f_carr * delt;

                chan[i].carr_phase -= floor(chan[i].carr_phase); // Back to [0, 1) for any step size
            }
        }

        c->iq_buff[isamp * 2] = (short) i_acc;
        c->iq_buff[isamp * 2 + 1] = (short) q_acc;
    }
}

/* Copy the samples into FIFO blocks for the SDR thread.
 * iq Block being filled
 * The block to fill next, NULL if the FIFO has been halted
 */
static struct iq_buf *queueSamples(const gps_ctx_t *c, struct iq_buf *iq) {
    const simulator_t *sim = c->sim;

    for (int isamp = 0; isamp < IQ_BUFFER_SIZE; isamp++) {
        // validLength starts with 0 on acquire
        if (sim->sample_size == SC16) {
            iq->data16[iq->validLength] = c->iq_buff[isamp];
        } else {
            iq->data8[iq->validLength] = c->iq_buff[isamp] >> 4;
        }
        iq->validLength += 1;
        if (sim->sdr_type == SDR_HACKRF) {
            // HackRF takes fixed size blocks, a partly filled one is kept for the next round
            if (iq->validLength == HACKRF_TRANSFER_BUFFER_SIZE) {
                fifo_enqueue(iq);
                iq = fifo_acquire();
                if (iq == NULL) {
                    return NULL;
                }
            }
        }
    }

    // File writer, Pluto and LimeSDR take the entire IQ buffer at once.
    if (sim->sdr_type == SDR_IQFILE || sim->sdr_type == SDR_PLUTOSDR || sim->sdr_type == SDR_LIMESDR) {
        fifo_enqueue(iq);
        iq = fifo_acquire();
    }
    return iq;
}

static bool sameEphemeris(const ephem_t *a, const ephem_t *b) {
    return a->vflg == b->vflg && a->iode == b->iode && a->toc.week == b->toc.week && a->toc.sec == b->toc.sec;
}

/* Every 30 s: next navigation message frame, newer ephemeris, channel allocation. */
static void updateNavigation(gps_ctx_t *c, int iumd, const double *llh) {
    simulator_t *sim = c->sim;

    // Use the record closest in time for every satellite. Its subframes go into the frame that
    // starts now, because the ranges follow the new record from now on too.
    ephem_t next[MAX_SAT];
    ephSelect(&c->table, c->grx, next);
    for (int i = 0; i < MAX_CHAN; i++) {
        int sv = c->chan[i].prn - 1;
        if (sv >= 0 && next[sv].vflg && !sameEphemeris(&next[sv], &c->eph[sv]))
            eph2sbf(next[sv], c->ionoutc, c->alm, c->chan[i].sbf);
    }
    memcpy(c->eph, next, sizeof (next));

    for (int i = 0; i < MAX_CHAN; i++) {
        if (c->chan[i].prn > 0) {
            generateNavMsg(c->grx, &c->chan[i], 0);
        }
    }

    allocateChannel(c->chan, c->alm, c->eph, c->ionoutc, c->grx, c->xyz[iumd]);

    if (sim->show_verbose) {
        gps2date(&c->grx, &sim->start);
        gui_mvwprintw(LS_FIX, LS_ROW_SIMTIME, LS_COL_SIMTIME, "%4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)",
                sim->start.y, sim->start.m, sim->start.d, sim->start.hh, sim->start.mm, sim->start.sec, c->grx.week, c->grx.sec);
        gui_mvwprintw(LS_FIX, LS_ROW_XYZ, LS_COL_INFO, "xyz = %11.1f, %11.1f, %11.1f", c->xyz[iumd][0], c->xyz[iumd][1], c->xyz[iumd][2]);
        gui_mvwprintw(LS_FIX, LS_ROW_LLH, LS_COL_INFO, "llh = %11.6f, %11.6f, %11.1f", llh[0] * R2D, llh[1] * R2D, llh[2]);
        showChannels(c);
    }
}

void *gps_thread_ep(void *arg) {
    simulator_t *simulator = (simulator_t *) (arg);
    gps_ctx_t *c = calloc(1, sizeof (*c));
    struct iq_buf *iq = NULL;

    thread_to_core(CORE_GPS);
    set_thread_name("gps-thread");

    if (c == NULL) {
        gui_status_wprintw(RED, "Failed to allocate simulation memory.\n");
        goto end_gps_thread;
    }
    c->sim = simulator;

    if (!initUserPosition(c) || !loadEphemeris(c) || !loadMotionFile(c) || !selectStartTime(c) || !loadAlmanac(c))
        goto end_gps_thread;

    initChannels(c);

    c->iq_buff = calloc(IQ_BUFFER_SIZE, sizeof (short));
    if (c->iq_buff == NULL) {
        gui_status_wprintw(RED, "Failed to allocate IQ buffer.\n");
        goto end_gps_thread;
    }

    // First fifo block for the transfer buffer
    iq = fifo_acquire();
    if (iq == NULL) { // FIFO halted
        goto end_gps_thread;
    }

    for (int iumd = 1; iumd < c->numd; iumd++) {
        if (simulator->gps_thread_exit) {
            break;
        }

        // Signal GPS init done and running
        if (simulator->gps_thread_running == false) {
            pthread_mutex_lock(&(simulator->gps_lock));
            simulator->gps_thread_running = true;
            pthread_cond_signal(&(simulator->gps_init_done));
            pthread_mutex_unlock(&(simulator->gps_lock));
        }

        if (simulator->interactive_mode) {
            moveUser(c, iumd);
        }

        updateChannels(c, iumd);
        generateSamples(c);

        iq = queueSamples(c, iq);
        if (iq == NULL) {
            goto end_gps_thread;
        }

        double llh[3];
        xyz2llh(c->xyz[iumd], llh);
        simulator->target.lat = llh[0] * R2D;
        simulator->target.lon = llh[1] * R2D;
        simulator->target.height = llh[2];
        gui_show_target(&simulator->target);

        if ((int) (c->grx.sec * 10.0 + 0.5) % 300 == 0) { // Every 30 seconds
            updateNavigation(c, iumd, llh);
        }

        c->grx = incGpsTime(c->grx, 0.1);
        gui_mvwprintw(LS_FIX, LS_ROW_ELAPSED, LS_COL_INFO, "Elapsed:         %5.1fs", subGpsTime(c->grx, c->g0));
    }

    gui_status_wprintw(GREEN, "Simulation complete\n");

end_gps_thread:
    fifo_producer_done();
    if (c != NULL) {
        free(c->iq_buff);
        free(c->xyz);
        free(c->table.rec);
        free(c);
    }
    gui_status_wprintw(RED, "Exit GPS thread\n");
    pthread_mutex_lock(&(simulator->gps_lock));
    simulator->gps_thread_exit = true;
    pthread_cond_signal(&(simulator->gps_init_done));
    pthread_mutex_unlock(&(simulator->gps_lock));
    pthread_exit(NULL);
}
