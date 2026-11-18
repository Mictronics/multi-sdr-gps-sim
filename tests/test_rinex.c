/* Tests readRinex and ephSelect on a trimmed IGS merged RINEX 3 file (tests/data/sample_rinex3.rnx).
 * The file is sorted by satellite as the real ones are (G01 at 00:00, 02:00, 04:00,
 * a Galileo E05 record, then G02 at the same epochs). The other tests build files from its
 * records to reproduce layouts that broke the selection of the ephemerides in real files.
 * Run with "make test" from the repository root.
 */
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "gps-sim.h"

#define SAMPLE "tests/data/sample_rinex3.rnx"
#define MAX_LINES 200
#define HOUR 3600.0
#define DAY_START 172800.0 // 2026-11-17 00:00 as seconds of GPS week 2445

static eph_table_t tab;
static ephem_t cur[MAX_SAT];
static ionoutc_t io;
static int failures;

static void expect_int(const char *what, long got, long want) {
    if (got != want) {
        printf("FAIL %-44s got %ld want %ld\n", what, got, want);
        failures++;
    }
}

static void expect_near(const char *what, double got, double want) {
    if (fabs(got - want) > 1e-12 * fmax(fabs(want), 1e-20)) {
        printf("FAIL %-44s got %.15g want %.15g\n", what, got, want);
        failures++;
    }
}

static int read_file(const char *name, bool rinex3) {
    free(tab.rec);
    memset(&io, 0, sizeof (io));
    return readRinex(&tab, &io, name, rinex3);
}

static int records_of(int sv) {
    return tab.first[sv + 1] - tab.first[sv];
}

static const ephem_t *record(int sv, int k) {
    return &tab.rec[tab.first[sv] + k];
}

/* Satellites with a usable record at the time t (seconds of the week); cur holds them. */
static int select_at(double t) {
    int n = 0;
    gpstime_t g = {2445, t};

    ephSelect(&tab, g, cur);
    for (int sv = 0; sv < MAX_SAT; sv++)
        n += cur[sv].vflg ? 1 : 0;
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
    int n = read_file(SAMPLE, true);

    expect_int("records", n, 6);

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

    // Three records each for G01 and G02, in time order; E05 is skipped
    expect_int("G01 records", records_of(0), 3);
    expect_int("G02 records", records_of(1), 3);
    expect_int("E05 skipped", records_of(4), 0);
    expect_int("table size", tab.first[MAX_SAT], 6);
    for (int k = 0; k < 3; k++) {
        expect_int("toc week", record(0, k)->toc.week, 2445);
        expect_near("G01 toc", record(0, k)->toc.sec, DAY_START + 2 * HOUR * k);
        expect_near("G02 toc", record(1, k)->toc.sec, DAY_START + 2 * HOUR * k);
    }

    // G01 at 00:00, field by field
    const ephem_t *e = record(0, 0);
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

    // The last record of the last satellite
    expect_near("G02 04:00 sqrta", record(1, 2)->sqrta, 5153.68704987);
    expect_near("G02 04:00 ecc", record(1, 2)->ecc, 0.017336265184);
    expect_near("G02 04:00 toe", record(1, 2)->toe.sec, 187200.0);
    expect_int("G02 04:00 iodc", record(1, 2)->iodc, 89);
}

/* Every satellite gets the record closest in time, as long as it is within EPH_MAX_AGE. */
static void test_selection(void) {
    read_file(SAMPLE, true);

    expect_int("00:30 satellites", select_at(DAY_START + 0.5 * HOUR), 2);
    expect_near("00:30 G01 toc", cur[0].toc.sec, DAY_START);
    expect_near("00:30 G02 toc", cur[1].toc.sec, DAY_START);
    expect_int("00:30 E05 not used", cur[4].vflg, 0);

    select_at(DAY_START + 1 * HOUR + 600);
    expect_near("01:10 closer to 02:00", cur[0].toc.sec, DAY_START + 2 * HOUR);

    select_at(DAY_START + 3 * HOUR);
    expect_near("03:00 tie: the later record", cur[0].toc.sec, DAY_START + 4 * HOUR);

    expect_int("2 h after the last record", select_at(DAY_START + 6 * HOUR), 2);
    expect_int("just over 2 h after the last record", select_at(DAY_START + 6 * HOUR + 1), 0);
    expect_int("2 h before the first record", select_at(DAY_START - 2 * HOUR), 2);
    expect_int("just over 2 h before the first record", select_at(DAY_START - 2 * HOUR - 1), 0);
}

/* The same records in time order must give the same table. */
static void test_order_independent(void) {
    static ephem_t saved[6];
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

    read_file(SAMPLE, true);
    memcpy(saved, tab.rec, sizeof (saved));
    int firsts[MAX_SAT + 1];
    memcpy(firsts, tab.first, sizeof (firsts));

    expect_int("reordered: records", read_file(write_temp(body, nbody), true), 6);
    for (int sv = 0; sv <= MAX_SAT; sv++)
        expect_int("reordered: first", tab.first[sv], firsts[sv]);
    for (int i = 0; i < 6; i++) {
        expect_near("reordered: toc", tab.rec[i].toc.sec, saved[i].toc.sec);
        expect_near("reordered: sqrta", tab.rec[i].sqrta, saved[i].sqrta);
        expect_near("reordered: af0", tab.rec[i].af0, saved[i].af0);
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

    int n = read_file(write_temp(body, nbody), true);

    expect_int("bad records: kept", n, 1);
    expect_int("bad records: G02 kept", records_of(1), 1);
    expect_int("bad records: others", tab.first[MAX_SAT] - records_of(1), 0);
}

/* Builds a file of copies of the G01 record at 04:00 with other PRNs and epochs. */
static char built[64][100];
static const char *built_body[64 * 8];
static int nbuilt, nbuilt_body;
static int template_line;

static void add_record(int prn, int hh, int mm) {
    snprintf(built[nbuilt], sizeof (built[0]), "G%02d 2026 11 17 %02d %02d 00%s", prn, hh, mm, lines[template_line] + 23);
    built_body[nbuilt_body++] = built[nbuilt++];
    for (int j = 1; j < 8; j++)
        built_body[nbuilt_body++] = lines[template_line + j];
}

static void start_building(void) {
    nbuilt = nbuilt_body = 0;
    for (int i = first_record; i < nlines; i++)
        if (!strncmp(lines[i], "G01 2026 11 17 04", 17))
            template_line = i;
}

/* A lone upload shortly before and after a regular update of every satellite used to start
 * ephemeris sets that cut the update in two, leaving one satellite where four were usable. */
static void test_lone_uploads(void) {
    start_building();
    add_record(3, 5, 29); // Lone upload
    add_record(1, 6, 0);
    add_record(2, 6, 0);
    add_record(4, 6, 44); // Lone upload more than an hour after the first one
    add_record(1, 8, 0);
    add_record(2, 8, 0);
    read_file(write_temp(built_body, nbuilt_body), true);

    expect_int("lone uploads: records", tab.first[MAX_SAT], 6);
    expect_int("07:20 satellites", select_at(DAY_START + 7 * HOUR + 1200), 4);
    expect_near("07:20 G01 uses 08:00", cur[0].toc.sec, DAY_START + 8 * HOUR);
    expect_near("07:20 G02 uses 08:00", cur[1].toc.sec, DAY_START + 8 * HOUR);
    expect_near("07:20 G04 uses its 06:44", cur[3].toc.sec, DAY_START + 6 * HOUR + 44 * 60);

    select_at(DAY_START + 6 * HOUR + 1200);
    expect_near("06:20 G01 uses 06:00", cur[0].toc.sec, DAY_START + 6 * HOUR);
    expect_near("06:20 G02 uses 06:00", cur[1].toc.sec, DAY_START + 6 * HOUR);
}

/* A day has more than the 13 sets the old table held. */
static void test_many_epochs(void) {
    start_building();
    for (int hh = 0; hh < 24; hh++)
        add_record(1, hh, 0);
    read_file(write_temp(built_body, nbuilt_body), true);

    expect_int("24 hourly records kept", records_of(0), 24);
    expect_int("17:20 satellites", select_at(DAY_START + 17 * HOUR + 1200), 1);
    expect_near("17:20 uses 17:00", cur[0].toc.sec, DAY_START + 17 * HOUR);
    select_at(DAY_START + 23 * HOUR + 1500);
    expect_near("23:25 uses 23:00", cur[0].toc.sec, DAY_START + 23 * HOUR);
}

/* The same satellite and clock time twice: the later record wins. */
static void test_duplicate_record(void) {
    start_building();
    add_record(1, 0, 0);
    add_record(1, 0, 0);
    // Give the second copy another sqrt(A) (columns 61-79 of the third orbit line)
    snprintf(built[nbuilt++], sizeof (built[0]), "%.61s%s", built_body[nbuilt_body - 6], " 5.100000000000E+03");
    built_body[nbuilt_body - 6] = built[nbuilt - 1];
    read_file(write_temp(built_body, nbuilt_body), true);

    expect_int("duplicate: one record", records_of(0), 1);
    expect_near("duplicate: the later one", record(0, 0)->sqrta, 5100.0);
}

static void expect_header_result(const char *what, const char *version_line, bool rinex3, int want) {
    char line[128];
    char *original = lines[0];

    // Replace the sample's version line, keep the rest of its header
    snprintf(line, sizeof (line), "%-60s%s", version_line, "RINEX VERSION / TYPE");
    lines[0] = line;
    int n = read_file(write_temp(NULL, 0), rinex3);
    lines[0] = original;
    expect_int(what, n, want);
}

static void test_rejected_files(void) {
    expect_header_result("RINEX 2 file read as RINEX 3", "     2.11           N: GPS NAV DATA", true, -2);
    expect_header_result("RINEX 4 file", "     4.00           N: GNSS NAV DATA    M: MIXED", true, -2);
    expect_header_result("RINEX 3 file read as RINEX 2", "     3.04           N: GNSS NAV DATA    M: MIXED", false, -2);
    expect_header_result("observation file", "     3.04           O: Observation data M: MIXED", true, -3);
    expect_header_result("header without records", "     3.04           N: GNSS NAV DATA    M: MIXED", true, 0);
    expect_int("missing file", read_file("/nonexistent.rnx", true), -1);
    expect_int("no file name", read_file(NULL, true), -1);
}

/* Read the header with its LEAP SECONDS line replaced by "text" (may hold several lines). */
static void read_with_leap_line(const char *text) {
    int leap = -1;

    for (int i = 0; i < first_record; i++)
        if (strstr(lines[i], "LEAP SECONDS"))
            leap = i;

    char *original = lines[leap];
    lines[leap] = (char *) text;
    read_file(write_temp(NULL, 0), true);
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
    test_selection();
    test_order_independent();
    test_bad_records();
    test_lone_uploads();
    test_many_epochs();
    test_duplicate_record();
    test_rejected_files();
    test_leap_seconds();

    free(tab.rec);
    if (temp_made)
        unlink(temp_path);
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
