/* Checks the navigation message fields written by eph2sbf against the IS-GPS-200
 * bit positions (1-based bit numbers within the 300 bit subframe, the numbering
 * used by gnss-sdr): SV health, almanac health, SV id and leap seconds.
 * Run with "make test".
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "gps-sim.h"
#include "almanac.h"

void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]);

static unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF];
static int failures;
static unsigned long field(int page, int start, int len) {
    unsigned long v = 0;
    for (int n = start; n < start + len; n++) {
        int word = (n - 1) / 30, b = (n - 1) % 30 + 1;
        if (b > 24) { printf("field crosses parity bits at %d\n", n); failures++; }
        v = (v << 1) | ((sbf[page][word] >> (30 - b)) & 1UL);
    }
    return v;
}
static void expect(const char *what, unsigned long got, unsigned long want) {
    if (got != want) { printf("FAIL %-34s got %lu want %lu\n", what, got, want); failures++; }
}

static long sfield(int page, int start, int len) {
    long v = (long) field(page, start, len);
    return (v >> (len - 1)) ? v - (1L << len) : v;
}

static void near(const char *what, double got, double want, double lsb) {
    if (fabs(got - want) >= lsb) { printf("FAIL %-34s got %.12g want %.12g (1 LSB %.3g)\n", what, got, want, lsb); failures++; }
}

/* Almanac pages: every field read back with the IS-GPS-200 bit positions and scale factors must be
 * the value that was put in, within one LSB. words 3-10 start at bit 61, 91, 121, ... of the page. */
static void check_almanac_page(int page, int prn, const almanac_prn_t *a) {
    char w[48];
    #define CHK(name, got, want, lsb) do { snprintf(w, sizeof (w), "PRN%d %s", prn, name); near(w, got, want, lsb); } while (0)
    expect("almanac page data id", field(page, 61, 2), 1);
    expect("almanac page SV id", field(page, 63, 6), (unsigned long) prn);
    CHK("e", field(page, 69, 16) * pow(2, -21), a->e, pow(2, -21));
    CHK("toa", field(page, 91, 8) * 4096.0, a->toa.sec, 4096.0);
    CHK("delta i", sfield(page, 99, 16) * pow(2, -19), a->delta_i, pow(2, -19));
    CHK("omega dot", sfield(page, 121, 16) * pow(2, -38), a->omegadot, pow(2, -38));
    expect("almanac health", field(page, 137, 8), a->health);
    CHK("sqrt A", field(page, 151, 24) * pow(2, -11), a->sqrta, pow(2, -11));
    CHK("omega 0", sfield(page, 181, 24) * pow(2, -23), a->omega0, pow(2, -23));
    CHK("omega", sfield(page, 211, 24) * pow(2, -23), a->aop, pow(2, -23));
    CHK("M0", sfield(page, 241, 24) * pow(2, -23), a->m0, pow(2, -23));
    long af0 = (long) ((field(page, 271, 8) << 3) | field(page, 290, 3));
    if (af0 >> 10) af0 -= 1L << 11;
    CHK("af0", af0 * pow(2, -20), a->af0, pow(2, -20));
    CHK("af1", sfield(page, 279, 11) * pow(2, -38), a->af1, pow(2, -38));
    #undef CHK
}

int main(void) {
    static ephem_t eph; static ionoutc_t io; almanac_gps_t *alm = almanac_init();
    eph.vflg = 1; eph.svh = 0x2B; eph.iodc = 0x2AB; eph.sqrta = 5153.7; eph.ecc = 0.01; eph.toe.week = 2445;
    io.vflg = 1; io.dtls = 18; io.dtlsf = 19; io.wnlsf = 2000; io.dn = 3; // None of these equals the old constants
    unsigned health[MAX_SAT] = {0};
    for (int sv = 0; sv < MAX_SAT; sv++) {
        health[sv] = (sv * 7 + 3) % 64; // distinct, 6 bit
        if (sv == 12 || sv == 28) continue; // PRN13, PRN29 have no almanac: must stay 0
        alm->sv[sv].valid = 1; alm->sv[sv].svid = sv + 1; alm->sv[sv].health = health[sv];
        alm->sv[sv].sqrta = 5153.7; alm->sv[sv].e = 0.01;
    }
    alm->valid = 1;
    eph2sbf(eph, io, alm, sbf);

    expect("SF1 SV health", field(0, 77, 6), 0x2B);
    expect("SF1 IODC MSBs", field(0, 83, 2), (0x2AB >> 8) & 3);

    static const int sf5[24] = {91, 97, 103, 109, 121, 127, 133, 139, 151, 157, 163, 169, 181, 187, 193, 199, 211, 217, 223, 229, 241, 247, 253, 259};
    for (int sv = 0; sv < 24; sv++) {
        char w[40]; snprintf(w, sizeof w, "SF5 p25 health PRN%d", sv + 1);
        expect(w, field(4 + 24 * 2, sf5[sv], 6), (sv == 12) ? 0 : health[sv]);
    }
    static const int sf4[8] = {229, 241, 247, 253, 259, 271, 277, 283};
    for (int k = 0; k < 8; k++) {
        char w[40]; snprintf(w, sizeof w, "SF4 p25 health PRN%d", 25 + k);
        expect(w, field(3 + 24 * 2, sf4[k], 6), (24 + k == 28) ? 0 : health[24 + k]);
    }
    // Per page 8 bit almanac health: PRN5 is SF5 page 5, PRN26 is SF4 page 3 (index 2)
    expect("SF5 page PRN5 health", field(4 + 4 * 2, 137, 8), health[4]);
    expect("SF4 page PRN26 health", field(3 + 2 * 2, 137, 8), health[25]);
    expect("SF5 page PRN5 SV id", field(4 + 4 * 2, 63, 6), 5);
    expect("SF4 page PRN26 SV id", field(3 + 2 * 2, 63, 6), 26);
    // Leap seconds, SF4 page 18: dtls, WNlsf, DN in word 9, dtlsf in word 10
    expect("SF4 p18 dtls", field(3 + 17 * 2, 241, 8), 18);
    expect("SF4 p18 WNlsf", field(3 + 17 * 2, 249, 8), 2000 % 256);
    expect("SF4 p18 DN", field(3 + 17 * 2, 257, 8), 3);
    expect("SF4 p18 dtlsf", field(3 + 17 * 2, 271, 8), 19);

    // Neutral values for a file without iono/UTC data: page 18 is complete and says "no correction"
    ionoutc_t blank;
    memset(&blank, 0, sizeof (blank));
    gpstime_t start = {2445, 330237.0};
    ionoDefaults(&blank, start);
    eph2sbf(eph, blank, alm, sbf);
    expect("defaults: page 18 SV id", field(3 + 17 * 2, 63, 6), 56);
    expect("defaults: alpha0", field(3 + 17 * 2, 69, 8), 0);
    expect("defaults: beta0", field(3 + 17 * 2, 105, 8), 0);
    expect("defaults: WNt", field(3 + 17 * 2, 227, 8), 2445 % 256);
    expect("defaults: tot", field(3 + 17 * 2, 219, 8), 327680 / 4096);
    expect("defaults: dtls", field(3 + 17 * 2, 241, 8), 18);
    expect("defaults: WNlsf", field(3 + 17 * 2, 249, 8), 1929 % 256);
    expect("defaults: DN", field(3 + 17 * 2, 257, 8), 7);
    expect("defaults: dtlsf", field(3 + 17 * 2, 271, 8), 18);

    // Almanac pages for one satellite in subframe 5 (PRN 5) and one in subframe 4 (PRN 27)
    almanac_gps_t *al = almanac_init();
    almanac_prn_t *p5 = &al->sv[4], *p27 = &al->sv[26];
    *p5 = (almanac_prn_t) {.valid = 1, .svid = 5, .health = 21, .e = 0.0123, .delta_i = -0.0123, .omegadot = -2.6e-9, .sqrta = 5153.62,
        .omega0 = -0.7123, .aop = 0.3456, .m0 = -0.9871, .af0 = -1.2e-4, .af1 = -3.1e-12, .toa = {2445, 503808.0}};
    *p27 = (almanac_prn_t) {.valid = 1, .svid = 27, .health = 0, .e = 0.00321, .delta_i = 0.0087, .omegadot = -2.9e-9, .sqrta = 5153.9,
        .omega0 = 0.4567, .aop = -0.1234, .m0 = 0.8123, .af0 = 2.6e-5, .af1 = 1.1e-12, .toa = {2445, 503808.0}};
    al->valid = 1;
    eph2sbf(eph, io, al, sbf);
    check_almanac_page(4 + 4 * 2, 5, p5);   // subframe 5, page 5
    check_almanac_page(3 + 3 * 2, 27, p27); // subframe 4, page 4
    expect("SF5 page 25 SV id", field(4 + 24 * 2, 63, 6), 51);
    expect("SF5 page 25 toa", field(4 + 24 * 2, 69, 8), 503808 / 4096);
    expect("SF5 page 25 WNa", field(4 + 24 * 2, 77, 8), 2445 % 256);

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
