/* Checks the signal delay models against values worked out by hand and the C/A codes
 * against the published first 10 chips.
 * Run with "make test".
 */
#include <math.h>
#include <stdio.h>
#include "model.h"

void codegen(int *ca, int prn);

static int failures;

static void near(const char *what, double got, double want, double tol) {
    if (fabs(got - want) > tol) { printf("FAIL %-30s got %.4f want %.4f\n", what, got, want); failures++; }
}

// First 10 chips as octal: the leading digit is chip 1, the last three digits chips 2-10
// (IS-GPS-200 Table 3-Ia, gps.gov L1 C/A PRN code assignments Jan 2026)
static void firstChips(int prn, int want) {
    int ca[1023], got = 0;
    ca[0] = -1;
    codegen(ca, prn);
    for (int i = 0; i < 10; i++)
        got = got << 1 | ca[i];
    if (got != want) { printf("FAIL C/A PRN %3d first chips got %04o want %04o\n", prn, got, want); failures++; }
}

int main(void) {
    firstChips(1, 01440);
    firstChips(120, 00671);
    firstChips(121, 00536);
    firstChips(123, 01545);
    firstChips(131, 01226);
    firstChips(136, 01037);
    firstChips(155, 00003);
    firstChips(158, 00235);

    const double d2r = M_PI / 180.0;
    // Sea level at 45 deg: dry 0.0022768 * 1013.25 = 2.3070 m, wet 0.1205 m (e = 12.01 hPa at 288.16 K)
    double llh[3] = {45.0 * d2r, 0.0, 0.0}, zenith[2] = {0.0, 90.0 * d2r}, el30[2] = {0.0, 30.0 * d2r};
    near("tropo zenith, sea level", tropoDelay(llh, zenith), 2.4275, 0.001);
    near("tropo 30 deg, mapping 1.99404", tropoDelay(llh, el30), 1.99404 * 2.4275, 0.002);
    llh[2] = 2000.0;
    near("tropo zenith, 2000 m", tropoDelay(llh, zenith), 1.8628, 0.002); // 794.9 hPa, 275.16 K
    llh[2] = 10001.0;
    near("tropo above 10 km", tropoDelay(llh, zenith), 0.0, 0.0);
    // Bounded and smooth at the horizon: 1/sin(el) made the range of a setting satellite
    // jump by kilometres per 0.1 s (PRN 11 at 0.0003 deg crashed the sample loop)
    double horizon[2] = {0.0, 0.0}, setting[2] = {0.0, 0.0003 * d2r};
    llh[2] = 0.0;
    near("tropo at the horizon", tropoDelay(llh, horizon), 22.3775 * 2.4275, 0.02);
    near("tropo just above the horizon", tropoDelay(llh, setting), tropoDelay(llh, horizon), 0.001);

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
