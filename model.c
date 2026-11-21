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
#include "sbas.h"
#include "orbit.h"
#include "model.h"

/* Klobuchar vertical delay [s] at a pierce point, phi_i and lam_i in semi-circles */
static double klobucharVertical(const ionoutc_t *ionoutc, gpstime_t g, double phi_i, double lam_i) {
    double t, phi_m, phi_m2, phi_m3;
    double AMP, PER, X, X2, X4;

    if (ionoutc->vflg == false)
        return 5.0e-9;

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
        return 5.0e-9 + AMP * (1.0 - X2 / 2.0 + X4 / 24.0);
    }
    return 5.0e-9;
}

double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel) {
    double E, phi_u, lam_u, F;
    double psi, phi_i, lam_i;

    if (ionoutc->enable == false)
        return (0.0); // No ionospheric delay

    E = azel[1] / PI;
    phi_u = llh[0] / PI;
    lam_u = llh[1] / PI;

    // Obliquity factor
    F = 1.0 + 16.0 * pow((0.53 - E), 3.0);

    if (ionoutc->vflg == false)
        return F * klobucharVertical(ionoutc, g, 0.0, 0.0) * SPEED_OF_LIGHT;

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

    return F * klobucharVertical(ionoutc, g, phi_i, lam_i) * SPEED_OF_LIGHT;
}

/* SBAS ionospheric grid around the user at llh (rad): mask, then the Klobuchar vertical delay the
 * signal carries at every IGP (0 with --disable-iono) */
void sbasIonoUpdate(sbas_iono_t *ion, const ionoutc_t *ionoutc, gpstime_t g, const double *llh) {
    sbas_iono_mask(llh[0] * R2D, llh[1] * R2D, ion);
    for (int k = 0; k < ion->nband; k++)
        for (int i = 0; i < ion->nigp[k]; i++)
            ion->delay[k][i] = ionoutc->enable ? klobucharVertical(ionoutc, g, ion->lat[k][i] / 180.0, ion->lon[k][i] / 180.0) * SPEED_OF_LIGHT : 0.0;
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

/* Range from a satellite at pos (m, ECEF) with velocity vel and clock offset clk0 (s), as computeRange */
void rangeFromPos(range_t *rho, const double *pos0, const double vel[3], double clk0, ionoutc_t *ionoutc, gpstime_t g, double xyz[]) {
    double pos[3] = {pos0[0], pos0[1], pos0[2]}; // Moved to the transmission time below
    double los[3];
    double tau;
    double range;
    double xrot, yrot;

    double llh[3], neu[3];
    double tmat[3][3];

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
    rho->range = range - SPEED_OF_LIGHT * clk0;

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

/* Compute range between a satellite and the receiver
 * rho The computed range
 * eph Ephemeris data of the satellite
 * g GPS time at time of receiving the signal
 * xyz position of the receiver
 */
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]) {
    double pos[3], vel[3], clk[2];

    satpos(eph, g, pos, vel, clk); // SV position at time of the pseudorange observation
    rangeFromPos(rho, pos, vel, clk[0], ionoutc, g, xyz);
}
