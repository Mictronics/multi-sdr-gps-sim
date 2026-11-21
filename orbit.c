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
#include "gps.h"
#include "orbit.h"

/* Subtract two vectors of double
 * y Result of subtraction
 * x1 Minuend of subtraction
 * x2 Subtrahend of subtraction
 */
void subVect(double *y, const double *x1, const double *x2) {
    y[0] = x1[0] - x2[0];
    y[1] = x1[1] - x2[1];
    y[2] = x1[2] - x2[2];

    return;
}

/* Compute Norm of Vector
 * x Input vector
 * Length (Norm) of the input vector
 */
double normVect(const double *x) {
    return (sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
}
/* Expand a GPS week known modulo 256 to the full week closest to ref_week */
int resolveWeek(int week256, int ref_week) {
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
void date2gps(const datetime_t *t, gpstime_t *g) {
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

void gps2date(const gpstime_t *g, datetime_t *t) {
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
void xyz2llh(const double *xyz, double *llh) {
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
void llh2xyz(const double *llh, double *xyz) {
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
void ltcmat(const double *llh, double t[3][3]) {
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
void ecef2neu(const double *xyz, double t[3][3], double *neu) {
    neu[0] = t[0][0] * xyz[0] + t[0][1] * xyz[1] + t[0][2] * xyz[2];
    neu[1] = t[1][0] * xyz[0] + t[1][1] * xyz[1] + t[1][2] * xyz[2];
    neu[2] = t[2][0] * xyz[0] + t[2][1] * xyz[1] + t[2][2] * xyz[2];

    return;
}

/* Convert North-East-Up to Azimuth + Elevation
 * neu Input position in North-East-Up format
 * azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu) {
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
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk) {
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

double subGpsTime(gpstime_t g1, gpstime_t g0) {
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double) (g1.week - g0.week) * SECONDS_IN_WEEK;

    return (dt);
}

gpstime_t incGpsTime(gpstime_t g0, double dt) {
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
