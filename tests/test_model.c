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
