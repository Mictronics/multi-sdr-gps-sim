/* Checks readGpx: resampling onto 0.1 s steps, time zones, <ele> above the geoid, missing <ele>,
 * rejected files, and the EGM96 geoid table.
 * Run with "make test".
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "gps.h"
#include "orbit.h"
#include "motion.h"

static int failures;
static double xyz[100][3];

static void check(const char *what, int ok) {
    if (!ok) { printf("FAIL %s\n", what); failures++; }
}

static int run(const char *pts, int max, const char **err) {
    char name[] = "/tmp/test_motion_XXXXXX";
    int fd = mkstemp(name);
    FILE *fp = fdopen(fd, "w");
    fprintf(fp, "<?xml version=\"1.0\"?>\n<gpx version=\"1.1\"><trk><trkseg>\n%s</trkseg></trk></gpx>\n", pts);
    fclose(fp);
    int n = readGpx(xyz, max, name, 123.0, err);
    unlink(name);
    return n;
}

static double dist(const double *a, const double *b) {
    double d[3];
    subVect(d, a, b);
    return normVect(d);
}

int main(void) {
    const char *err = NULL;

    // Grid nodes against GeographicLib's egm96-15.pgm (decimetre table), between nodes within the 1 deg grid error
    check("geoid 48N 10E", fabs(geoidHeight(48.0, 10.0) - 46.635) < 0.06);
    check("geoid 33S 71W", fabs(geoidHeight(-33.0, -71.0) - 26.202) < 0.06);
    check("geoid north pole", fabs(geoidHeight(90.0, 0.0) - 13.605) < 0.06);
    check("geoid south pole", fabs(geoidHeight(-90.0, 0.0) - -29.535) < 0.06);
    check("geoid between nodes", fabs(geoidHeight(48.5, 10.5) - 47.049) < 0.5);
    check("geoid across 0E", fabs(geoidHeight(0.0, -0.5) - 17.157) < 0.5 && fabs(geoidHeight(0.0, 359.5) - geoidHeight(0.0, -0.5)) < 1e-9);
    double llh[3], p0[3], p1[3];

    // 1 Hz points, the second in another time zone, the first without <ele>
    int n = run("<trkpt lat=\"48.0\" lon=\"10.0\"><time>2026-01-01T10:00:00Z</time></trkpt>\n"
                "<trkpt lon='10.001' lat='48.0'><ele>123</ele><time>2026-01-01T11:00:01+01:00</time></trkpt>\n"
                "<trkpt lat=\"48.0\" lon=\"10.002\"><ele>223</ele><time>2026-01-01T10:00:02.000Z</time></trkpt>\n", 100, &err);
    check("21 steps for 2 s", n == 21);
    llh[0] = 48.0 / R2D; llh[1] = 10.0 / R2D; llh[2] = 123.0;
    llh2xyz(llh, p0);
    llh[1] = 10.001 / R2D;
    llh[2] = 123.0 + geoidHeight(48.0, 10.001);
    llh2xyz(llh, p1);
    check("first step at the first point, -l height", dist(xyz[0], p0) < 1e-6);
    check("step 10 at the second point", dist(xyz[10], p1) < 1e-6);
    check("step 5 halfway", fabs(dist(xyz[5], p0) - dist(p0, p1) / 2) < 1e-3);
    xyz2llh(xyz[15], llh);
    check("step 15 height interpolated, <ele> above the geoid", fabs(llh[2] - (173.0 + geoidHeight(48.0, 10.0015))) < 0.01);
    check("max caps the steps", run("<trkpt lat=\"0\" lon=\"0\"><time>2026-01-01T00:00:00Z</time></trkpt>"
                                    "<trkpt lat=\"0\" lon=\"1\"><time>2026-01-01T01:00:00Z</time></trkpt>", 100, &err) == 100);

    check("no time rejected", run("<trkpt lat=\"0\" lon=\"0\"><ele>1</ele></trkpt>", 100, &err) < 0);
    check("self-closing rejected", run("<trkpt lat=\"0\" lon=\"0\"/>", 100, &err) < 0);
    check("time going back rejected", run("<trkpt lat=\"0\" lon=\"0\"><time>2026-01-01T00:00:01Z</time></trkpt>"
                                          "<trkpt lat=\"0\" lon=\"0\"><time>2026-01-01T00:00:00Z</time></trkpt>", 100, &err) < 0);
    check("lat out of range rejected", run("<trkpt lat=\"95\" lon=\"0\"><time>2026-01-01T00:00:00Z</time></trkpt>", 100, &err) < 0);
    check("lon nan rejected", run("<trkpt lat=\"0\" lon=\"nan\"><time>2026-01-01T00:00:00Z</time></trkpt>", 100, &err) < 0);
    check("no points rejected", run("", 100, &err) < 0);
    check("missing file rejected", readGpx(xyz, 100, "/nonexistent.gpx", 0.0, &err) < 0);

    // The shipped sample
    static double big[3000][3];
    n = readGpx(big, 3000, "circle.gpx", 0.0, &err);
    check("circle.gpx reads", n > 2900);
    for (int k = 1; k < n; k++)
        if (dist(big[k], big[k - 1]) > 1.0) { check("circle.gpx steps under 1 m", 0); break; }

    if (failures == 0)
        printf("motion: all checks passed\n");
    return failures != 0;
}
