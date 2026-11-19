/* Checks the signal delay models against values worked out by hand.
 * Run with "make test".
 */
#include <math.h>
#include <stdio.h>

double tropoDelay(const double *llh, const double *azel);

static int failures;

static void near(const char *what, double got, double want, double tol) {
    if (fabs(got - want) > tol) { printf("FAIL %-30s got %.4f want %.4f\n", what, got, want); failures++; }
}

int main(void) {
    const double d2r = M_PI / 180.0;
    // Sea level at 45 deg: dry 0.0022768 * 1013.25 = 2.3070 m, wet 0.1205 m (e = 12.01 hPa at 288.16 K)
    double llh[3] = {45.0 * d2r, 0.0, 0.0}, zenith[2] = {0.0, 90.0 * d2r}, el30[2] = {0.0, 30.0 * d2r};
    near("tropo zenith, sea level", tropoDelay(llh, zenith), 2.4275, 0.001);
    near("tropo 30 deg is twice zenith", tropoDelay(llh, el30), 2.0 * 2.4275, 0.002);
    llh[2] = 2000.0;
    near("tropo zenith, 2000 m", tropoDelay(llh, zenith), 1.8628, 0.002); // 794.9 hPa, 275.16 K
    llh[2] = 10001.0;
    near("tropo above 10 km", tropoDelay(llh, zenith), 0.0, 0.0);
    double below[2] = {0.0, -1.0 * d2r};
    llh[2] = 0.0;
    near("tropo below the horizon", tropoDelay(llh, below), 0.0, 0.0);

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
