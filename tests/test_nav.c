/* Checks the navigation message fields written by eph2sbf against the IS-GPS-200
 * bit positions (1-based bit numbers within the 300 bit subframe, the numbering
 * used by gnss-sdr): SV health, almanac health, SV id and leap seconds.
 * Run with "make test".
 */
#include <stdio.h>
#include "gps-sim.h"
#include "gui.h"
#include "almanac.h"

void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_gps_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]);

/* gps.c reports to the GUI and sets thread affinity; stubs let the test link without them. */
void gui_status_wprintw(status_color_t c, const char *f, ...) { (void) c; (void) f; }
void gui_mvwprintw(window_panel_t w, int y, int x, const char *f, ...) { (void) w; (void) y; (void) x; (void) f; }
void gui_show_location(void *l) { (void) l; }
void gui_show_target(void *t) { (void) t; }
void set_thread_name(const char *n) { (void) n; }
int thread_to_core(int c) { return c; }

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

int main(void) {
    static ephem_t eph; static ionoutc_t io; almanac_gps_t *alm = almanac_init();
    eph.vflg = 1; eph.svh = 0x2B; eph.iodc = 0x2AB; eph.sqrta = 5153.7; eph.ecc = 0.01; eph.toe.week = 2445;
    io.vflg = 1; io.dtls = 19; // 19 differs from the old hard-coded 18
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
    expect("SF4 p18 dtls", field(3 + 17 * 2, 241, 8), 19);
    expect("SF4 p18 WNlsf", field(3 + 17 * 2, 249, 8), 1929 % 256);
    expect("SF4 p18 DN", field(3 + 17 * 2, 257, 8), 7);
    expect("SF4 p18 dtlsf", field(3 + 17 * 2, 271, 8), 19);
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
