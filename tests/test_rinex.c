/* Tests readRinex on a trimmed IGS merged RINEX 3 file (tests/data/sample_rinex3.rnx).
 * The file is sorted by satellite as the real ones are (G01 at 00:00, 02:00, 04:00,
 * a Galileo E05 record, then G02 at the same epochs), which needs the records to be
 * sorted by time before they are split into ephemeris sets.
 * Run with "make test" from the repository root.
 */
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "gps-sim.h"

#define SAMPLE "tests/data/sample_rinex3.rnx"
#define MAX_LINES 200

static ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
static ionoutc_t io;
static int failures;

static void expect_int(const char *what, long got, long want) {
    if (got != want) {
        printf("FAIL %-40s got %ld want %ld\n", what, got, want);
        failures++;
    }
}

static void expect_near(const char *what, double got, double want) {
    if (fabs(got - want) > 1e-12 * fmax(fabs(want), 1e-20)) {
        printf("FAIL %-40s got %.15g want %.15g\n", what, got, want);
        failures++;
    }
}

static int read_file(const char *name, ephem_t out[][MAX_SAT], bool rinex3) {
    memset(out, 0, sizeof (ephem_t) * EPHEM_ARRAY_SIZE * MAX_SAT);
    memset(&io, 0, sizeof (io));
    return readRinex(out, &io, name, rinex3);
}

static int count_valid(int set) {
    int n = 0;
    for (int sv = 0; sv < MAX_SAT; sv++)
        n += eph[set][sv].vflg ? 1 : 0;
    return n;
}

/* The sample file split into lines (without line feeds). */
static char text[16384];
static char *lines[MAX_LINES];
static int nlines, first_record;

static void load_sample(void) {
    FILE *fp = fopen(SAMPLE, "r");
    size_t n = fp ? fread(text, 1, sizeof (text) - 1, fp) : 0;

    if (fp)
        fclose(fp);
    text[n] = 0;
    for (char *p = strtok(text, "\n"); p && nlines < MAX_LINES; p = strtok(NULL, "\n")) {
        lines[nlines++] = p;
        if (strstr(p, "END OF HEADER"))
            first_record = nlines;
    }
}

static char temp_path[] = "/tmp/test_rinexXXXXXX";
static int temp_made;

/* Write the header and the given lines to the temporary file, returns its name. */
static const char *write_temp(const char *const *body, int nbody) {
    int fd;
    FILE *fp;

    if (!temp_made) {
        fd = mkstemp(temp_path);
        temp_made = 1;
    } else {
        fd = open(temp_path, O_WRONLY | O_TRUNC);
    }
    fp = fdopen(fd, "w");
    for (int i = 0; i < first_record; i++)
        fprintf(fp, "%s\n", lines[i]);
    for (int i = 0; i < nbody; i++)
        fprintf(fp, "%s\n", body[i]);
    fclose(fp);
    return temp_path;
}

static void test_sorted_by_satellite(void) {
    int n = read_file(SAMPLE, eph, true);

    expect_int("sets", n, 3);

    // Header values: GPS only, the GAL and BDS iono lines are ignored
    expect_near("alpha0", io.alpha0, 2.0489e-08);
    expect_near("alpha1", io.alpha1, -1.4901e-08);
    expect_near("alpha2", io.alpha2, -1.1921e-07);
    expect_near("alpha3", io.alpha3, 1.7881e-07);
    expect_near("beta0", io.beta0, 1.2902e+05);
    expect_near("beta1", io.beta1, -9.8304e+04);
    expect_near("beta2", io.beta2, -1.3107e+05);
    expect_near("A0", io.A0, 1.8626451492e-09);
    expect_near("A1", io.A1, 5.329070518e-15);
    expect_int("tot", io.tot, 405504);
    expect_int("wnt", io.wnt, 2445);
    expect_int("dtls", io.dtls, 18);
    expect_int("dtlsf", io.dtlsf, 18);
    expect_int("wnlsf", io.wnlsf, 1929);
    expect_int("dn", io.dn, 7);
    expect_int("iono/UTC valid", io.vflg, 1);

    // Every set holds both GPS satellites and nothing else (E05 is skipped)
    for (int k = 0; k < 3; k++) {
        char what[48];
        snprintf(what, sizeof (what), "set %d satellites", k);
        expect_int(what, count_valid(k), 2);
        expect_int("G01 valid", eph[k][0].vflg, 1);
        expect_int("G02 valid", eph[k][1].vflg, 1);
        expect_int("E05 skipped", eph[k][4].vflg, 0);
        expect_int("toc week", eph[k][0].toc.week, 2445);
        expect_near("G01 toc", eph[k][0].toc.sec, 172800.0 + 7200.0 * k);
        expect_near("G02 toc", eph[k][1].toc.sec, 172800.0 + 7200.0 * k);
    }

    // G01 at 00:00, field by field
    const ephem_t *e = &eph[0][0];
    expect_near("G01 af0", e->af0, 0.000125890597701);
    expect_near("G01 af1", e->af1, -8.07176547823e-12);
    expect_near("G01 af2", e->af2, 0.0);
    expect_int("G01 iode", e->iode, 87);
    expect_near("G01 crs", e->crs, 47.09375);
    expect_near("G01 deltan", e->deltan, 4.60626329793e-09);
    expect_near("G01 m0", e->m0, 1.05555525387);
    expect_near("G01 cuc", e->cuc, 2.52202153206e-06);
    expect_near("G01 ecc", e->ecc, 0.00223058555275);
    expect_near("G01 cus", e->cus, 7.2717666626e-06);
    expect_near("G01 sqrta", e->sqrta, 5153.73573494);
    expect_near("G01 toe", e->toe.sec, 172800.0);
    expect_near("G01 cic", e->cic, 4.47034835815e-08);
    expect_near("G01 omg0", e->omg0, -1.50176116997);
    expect_near("G01 cis", e->cis, 1.86264514923e-09);
    expect_near("G01 inc0", e->inc0, 0.95568621219);
    expect_near("G01 crc", e->crc, 233.25);
    expect_near("G01 aop", e->aop, 0.143974234088);
    expect_near("G01 omgdot", e->omgdot, -8.17891211309e-09);
    expect_near("G01 idot", e->idot, -1.96436753808e-10);
    expect_int("G01 code", e->code, 1);
    expect_int("G01 week", e->toe.week, 2445);
    expect_int("G01 sva", e->sva, 2);
    expect_int("G01 svh", e->svh, 0);
    expect_near("G01 tgd", e->tgd, -8.84756445885e-09);
    expect_int("G01 iodc", e->iodc, 343);
    expect_near("G01 fit", e->fit, 4.0);

    // The last record of the last satellite lands in the last set
    expect_near("G02 04:00 sqrta", eph[2][1].sqrta, 5153.68704987);
    expect_near("G02 04:00 ecc", eph[2][1].ecc, 0.017336265184);
    expect_near("G02 04:00 toe", eph[2][1].toe.sec, 187200.0);
    expect_int("G02 04:00 iodc", eph[2][1].iodc, 89);
}

/* The same records in time order must give the same sets. */
static void test_order_independent(void) {
    static ephem_t other[EPHEM_ARRAY_SIZE][MAX_SAT];
    const char *body[MAX_LINES];
    int nbody = 0;
    int g01[3], g02[3], ng01 = 0, ng02 = 0;

    for (int i = first_record; i < nlines; i++) {
        if (strncmp(lines[i], "G01", 3) == 0 && ng01 < 3)
            g01[ng01++] = i;
        if (strncmp(lines[i], "G02", 3) == 0 && ng02 < 3)
            g02[ng02++] = i;
    }
    for (int k = 2; k >= 0; k--) { // Latest epoch first, G02 before G01
        for (int j = 0; j < 8; j++)
            body[nbody++] = lines[g02[k] + j];
        for (int j = 0; j < 8; j++)
            body[nbody++] = lines[g01[k] + j];
    }

    read_file(SAMPLE, eph, true);
    int n = read_file(write_temp(body, nbody), other, true);

    expect_int("reordered: sets", n, 3);
    for (int k = 0; k < 3; k++) {
        for (int sv = 0; sv < MAX_SAT; sv++) {
            expect_int("reordered: valid", other[k][sv].vflg, eph[k][sv].vflg);
            if (eph[k][sv].vflg) {
                expect_near("reordered: toc", other[k][sv].toc.sec, eph[k][sv].toc.sec);
                expect_near("reordered: sqrta", other[k][sv].sqrta, eph[k][sv].sqrta);
                expect_near("reordered: af0", other[k][sv].af0, eph[k][sv].af0);
            }
        }
    }
}

/* Records with a PRN or month out of range are dropped, a truncated last one too. */
static void test_bad_records(void) {
    char bad_prn0[200], bad_prn99[200], bad_month[200];
    const char *body[MAX_LINES];
    int nbody = 0;
    int g01_00 = -1, g01_02 = -1, g02_00 = -1, g02_02 = -1, g01_04 = -1;

    for (int i = first_record; i < nlines; i++) {
        if (!strncmp(lines[i], "G01 2026 11 17 00", 17)) g01_00 = i;
        if (!strncmp(lines[i], "G01 2026 11 17 02", 17)) g01_02 = i;
        if (!strncmp(lines[i], "G01 2026 11 17 04", 17)) g01_04 = i;
        if (!strncmp(lines[i], "G02 2026 11 17 00", 17)) g02_00 = i;
        if (!strncmp(lines[i], "G02 2026 11 17 02", 17)) g02_02 = i;
    }

    snprintf(bad_prn0, sizeof (bad_prn0), "G00%s", lines[g01_00] + 3);
    snprintf(bad_prn99, sizeof (bad_prn99), "G99%s", lines[g01_02] + 3);
    snprintf(bad_month, sizeof (bad_month), "%.9s13%s", lines[g02_02], lines[g02_02] + 11);

    body[nbody++] = bad_prn0;
    for (int j = 1; j < 8; j++) body[nbody++] = lines[g01_00 + j];
    body[nbody++] = bad_prn99;
    for (int j = 1; j < 8; j++) body[nbody++] = lines[g01_02 + j];
    for (int j = 0; j < 8; j++) body[nbody++] = lines[g02_00 + j]; // The only good one
    body[nbody++] = bad_month;
    for (int j = 1; j < 8; j++) body[nbody++] = lines[g02_02 + j];
    for (int j = 0; j < 3; j++) body[nbody++] = lines[g01_04 + j]; // Truncated

    int n = read_file(write_temp(body, nbody), eph, true);

    expect_int("bad records: sets", n, 1);
    expect_int("bad records: valid satellites", count_valid(0), 1);
    expect_int("bad records: G02 kept", eph[0][1].vflg, 1);
}

static void expect_header_result(const char *what, const char *version_line, bool rinex3, int want) {
    char line[128];
    char *original = lines[0];

    // Replace the sample's version line, keep the rest of its header
    snprintf(line, sizeof (line), "%-60s%s", version_line, "RINEX VERSION / TYPE");
    lines[0] = line;
    int n = read_file(write_temp(NULL, 0), eph, rinex3);
    lines[0] = original;
    expect_int(what, n, want);
}

static void test_rejected_files(void) {
    expect_header_result("RINEX 2 file read as RINEX 3", "     2.11           N: GPS NAV DATA", true, -2);
    expect_header_result("RINEX 4 file", "     4.00           N: GNSS NAV DATA    M: MIXED", true, -2);
    expect_header_result("RINEX 3 file read as RINEX 2", "     3.04           N: GNSS NAV DATA    M: MIXED", false, -2);
    expect_header_result("observation file", "     3.04           O: Observation data M: MIXED", true, -3);
    expect_header_result("header without records", "     3.04           N: GNSS NAV DATA    M: MIXED", true, 0);
    expect_int("missing file", read_file("/nonexistent.rnx", eph, true), -1);
    expect_int("no file name", read_file(NULL, eph, true), -1);
}

/* Read the header with its LEAP SECONDS line replaced by "text" (may hold several lines). */
static void read_with_leap_line(const char *text) {
    int leap = -1;

    for (int i = 0; i < first_record; i++)
        if (strstr(lines[i], "LEAP SECONDS"))
            leap = i;

    char *original = lines[leap];
    lines[leap] = (char *) text;
    read_file(write_temp(NULL, 0), eph, true);
    lines[leap] = original;
}

static void test_leap_seconds(void) {
    // All four values: the offset, the one after the event, its week and day of week
    read_with_leap_line("    19    20  2000     3                                    LEAP SECONDS");
    expect_int("leap: dtls", io.dtls, 19);
    expect_int("leap: dtlsf", io.dtlsf, 20);
    expect_int("leap: wnlsf", io.wnlsf, 2000);
    expect_int("leap: dn", io.dn, 3);
    expect_int("leap: valid", io.vflg, 1);

    // Only the offset, as in RINEX 2 files: the last known event is assumed
    read_with_leap_line("    18                                                      LEAP SECONDS");
    expect_int("offset only: dtls", io.dtls, 18);
    expect_int("offset only: dtlsf", io.dtlsf, 18);
    expect_int("offset only: wnlsf", io.wnlsf, 1929);
    expect_int("offset only: dn", io.dn, 7);

    // A line for another system must not replace the GPS values
    read_with_leap_line("    18    18  1929     7                                    LEAP SECONDS\n"
            "    14    14  1000     2BDS                                 LEAP SECONDS");
    expect_int("other system ignored: dtls", io.dtls, 18);
    expect_int("other system ignored: wnlsf", io.wnlsf, 1929);
    expect_int("other system ignored: dn", io.dn, 7);

    // No GPS line at all: the iono/UTC parameters are not complete
    read_with_leap_line("    14    14  1000     2BDS                                 LEAP SECONDS");
    expect_int("only another system: valid", io.vflg, 0);
}

int main(void) {
    load_sample();
    if (nlines < 20 || first_record == 0) {
        printf("cannot read %s (run from the repository root)\n", SAMPLE);
        return 1;
    }

    test_sorted_by_satellite();
    test_order_independent();
    test_bad_records();
    test_rejected_files();
    test_leap_seconds();

    if (temp_made)
        unlink(temp_path);
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
