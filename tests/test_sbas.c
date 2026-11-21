/* Checks the SBAS L1 message framing and convolutional code.
 * Run with "make test".
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "gps.h"
#include "sbas.h"
#include "model.h"

static int failures;
static void expect(const char *what, unsigned long got, unsigned long want) {
    if (got != want) { printf("FAIL %-34s got %#lx want %#lx\n", what, got, want); failures++; }
}

static unsigned long bitsToInt(const unsigned char *b, int n) {
    unsigned long v = 0;
    for (int i = 0; i < n; i++)
        v = v << 1 | b[i];
    return v;
}

int main(void) {
    unsigned char bits[256], msg[SBAS_MSG_BITS], data[SBAS_DATA_BITS], sym[2 * 512];

    // CRC catalogue check value of CRC-24/LTE-A (same polynomial, init 0, no reflection)
    const char *check = "123456789";
    for (int i = 0; i < 72; i++)
        bits[i] = (check[i / 8] >> (7 - i % 8)) & 1;
    expect("CRC-24Q of \"123456789\"", sbas_crc24q(bits, 72), 0xCDE703);

    for (int i = 0; i < SBAS_DATA_BITS; i++)
        data[i] = (i * 7 + 3) % 5 == 0;
    for (int sec = 0; sec < 6; sec++) {
        sbas_frame(9, data, sec, msg);
        expect("preamble", bitsToInt(msg, 8), (unsigned long[]){0x53, 0x9A, 0xC6}[sec % 3]);
        expect("message type", bitsToInt(msg + 8, 6), 9);
        expect("CRC over the whole message is 0", sbas_crc24q(msg, SBAS_MSG_BITS), 0);
    }
    expect("data field", memcmp(msg + 14, data, SBAS_DATA_BITS), 0);

    // Impulse response: G1 and G2 symbol streams are the generator taps, 171 and 133 octal
    unsigned state = 0;
    memset(bits, 0, sizeof(bits));
    bits[0] = 1;
    sbas_encode(&state, bits, 7, sym);
    unsigned long g1 = 0, g2 = 0;
    for (int i = 0; i < 7; i++) {
        g1 = g1 << 1 | sym[2 * i];
        g2 = g2 << 1 | sym[2 * i + 1];
    }
    expect("impulse response G1", g1, 0171);
    expect("impulse response G2", g2, 0133);

    // State carries across messages: two calls give the same symbols as one (bit 249 is a 1)
    unsigned char whole[2 * 500], split[2 * 500];
    unsigned char in[500];
    for (int i = 0; i < 500; i++)
        in[i] = i % 3 == 0 || i % 7 == 0;
    state = 0;
    sbas_encode(&state, in, 500, whole);
    state = 0;
    sbas_encode(&state, in, 250, split);
    sbas_encode(&state, in + 250, 250, split + 500);
    expect("test input bit 249", in[249], 1);
    expect("encoder state across messages", memcmp(whole, split, sizeof(whole)), 0);

    // MT9 field positions and scale factors: GEO at 5 deg E, sent at 01:00:00 of day 2
    double pos[3] = {SBAS_GEO_RADIUS * 0.99619469809, SBAS_GEO_RADIUS * 0.08715574275, 0.0};
    sbas_mt9(pos, 86400 + 3600, data);
    expect("MT9 t0 [16 s]", bitsToInt(data + 8, 13), 3600 / 16);
    expect("MT9 URA", bitsToInt(data + 21, 4), 15);
    expect("MT9 XG [0.08 m]", bitsToInt(data + 25, 30), 525046533);
    expect("MT9 YG [0.08 m]", bitsToInt(data + 55, 30), 45935619);
    expect("MT9 ZG", bitsToInt(data + 85, 25), 0);
    expect("MT9 rest is 0", bitsToInt(data + 110, 60) | bitsToInt(data + 170, 42), 0);

    // One minute of messages: MT2-4 every 6 s, MT1, MT9, MT7, MT10 and all 8 MT25, IODP 0 everywhere
    int iode[32], seen25[33] = {0}, count[64] = {0};
    for (int sv = 0; sv < 32; sv++)
        iode[sv] = (sv % 5 == 3) ? -1 : (sv * 7 + 1) % 256; // some PRNs without ephemeris
    for (int sec = 120; sec < 180; sec++) {
        int type = sbas_message(sec, iode, pos, NULL, NULL, 0, data);
        count[type]++;
        if (sec % 6 < 3)
            expect("MT2/3/4 at sec % 6 = 0/1/2", type, 2 + sec % 6);
        if (type >= 2 && type <= 4) {
            expect("MT2-4 IODP", bitsToInt(data + 2, 2), 0);
            expect("MT2-4 PRC all 0", bitsToInt(data + 4, 52) | bitsToInt(data + 56, 52) | bitsToInt(data + 108, 52), 0);
            for (int k = 0; k < 13; k++) {
                int slot = 13 * (type - 2) + k;
                unsigned long want = slot >= 32 ? 0 : (iode[slot] >= 0 ? 5 : 14);
                expect("MT2-4 UDREI", bitsToInt(data + 160 + 4 * k, 4), want);
            }
        } else if (type == 1) {
            expect("MT1 mask PRN 1-32", bitsToInt(data, 32), 0xFFFFFFFFUL);
            expect("MT1 mask rest 0", bitsToInt(data + 32, 60) | bitsToInt(data + 92, 60) | bitsToInt(data + 152, 58), 0);
            expect("MT1 IODP", bitsToInt(data + 210, 2), 0);
        } else if (type == 7) {
            expect("MT7 t_lat", bitsToInt(data, 4), 3);
            expect("MT7 ai slot 1", bitsToInt(data + 8, 4), 15);
            expect("MT7 ai slot 33", bitsToInt(data + 8 + 4 * 32, 4), 0);
        } else if (type == 10) {
            expect("MT10 Cltc_lsb", bitsToInt(data + 10, 10), 179);
            expect("MT10 iiono", bitsToInt(data + 103, 9), 300);
        } else if (type == 25) {
            for (int h = 0; h < 212; h += 106) {
                expect("MT25 velocity code", data[h], 0);
                expect("MT25 IODP", bitsToInt(data + h + 103, 2), 0);
                for (int j = 0; j < 2; j++) {
                    int o = h + 1 + 51 * j, slot = (int) bitsToInt(data + o, 6);
                    if (slot > 0) {
                        seen25[slot]++;
                        expect("MT25 IODE", bitsToInt(data + o + 6, 8), (unsigned long) iode[slot - 1]);
                        expect("MT25 deltas 0", bitsToInt(data + o + 14, 37), 0);
                    }
                }
            }
        }
    }
    expect("MT1 per minute", count[1], 2);
    expect("MT9 per minute", count[9], 1);
    expect("MT7 per minute", count[7], 1);
    expect("MT10 per minute", count[10], 1);
    expect("MT25 per minute", count[25], 8);
    for (int sv = 0; sv < 32; sv++)
        expect("MT25 once per PRN with ephemeris", seen25[sv + 1], iode[sv] >= 0 ? 1 : 0);

    // IGP geometry: spot checks against RTKLIB's igpband1 (mask bits counted from 0)
    double la, lo;
    expect("IGP band 4 bit 0", sbas_igp(4, 0, &la, &lo) && la == -75 && lo == -20, 1);
    expect("IGP band 4 bit 127 (85 N at 0 E)", sbas_igp(4, 127, &la, &lo) && la == 85 && lo == 0, 1);
    expect("IGP band 4 bit 128", sbas_igp(4, 128, &la, &lo) && la == -55 && lo == 5, 1);
    expect("IGP band 1 bit 0 (85 S at 140 W)", sbas_igp(1, 0, &la, &lo) && la == -85 && lo == -140, 1);
    expect("IGP band 0 bit 200", sbas_igp(0, 200, &la, &lo) && la == 55 && lo == -145, 1);
    expect("IGP band 8 bit 199", sbas_igp(8, 199, &la, &lo) && la == 55 && lo == 175, 1);
    expect("IGP band 8 has 200", sbas_igp(8, 200, &la, &lo), 0);

    // Grid at 48 N 10 E: bands 3-5, IODI kept while the mask is the same, changed after a move
    ionoutc_t io = {0};
    io.enable = 1;
    io.vflg = 1;
    io.alpha0 = 1.1176e-8; io.alpha1 = 7.4506e-9; io.alpha2 = -5.9605e-8; io.alpha3 = -5.9605e-8;
    io.beta0 = 90112; io.beta1 = 0; io.beta2 = -196608; io.beta3 = -65536;
    gpstime_t g = {2443, 4 * 86400 + 12 * 3600 - 10 / 15.0 * 3600}; // 14:00 local time at 10 E
    double llh[3] = {48.0 / 57.29577951308232, 10.0 / 57.29577951308232, 500.0};
    static sbas_iono_t ion;
    sbasIonoUpdate(&ion, &io, g, llh);
    expect("grid bands", ion.nband, 3);
    expect("grid band numbers", ion.band[0] * 100 + ion.band[1] * 10 + ion.band[2], 345);
    int iodi = ion.iodi;
    sbasIonoUpdate(&ion, &io, g, llh);
    expect("IODI kept for the same mask", ion.iodi, iodi);
    double far[3] = {40.0 / 57.29577951308232, -100.0 / 57.29577951308232, 0.0};
    sbasIonoUpdate(&ion, &io, g, far);
    expect("IODI changed after a far move", ion.iodi, (iodi + 1) % 4);
    sbasIonoUpdate(&ion, &io, g, llh);

    // MT18/MT26 in 5 minutes: every band's mask and every block, fields as the grid
    int seen18[SBAS_MAX_BANDS] = {0}, seen26[SBAS_MAX_BANDS][14] = {{0}};
    for (int sec = 600; sec < 900; sec++) {
        int type = sbas_message(sec, iode, pos, &ion, NULL, 0, data);
        expect("no null message with a grid", type == 63, 0);
        if (type == 18) {
            int k;
            for (k = 0; k < ion.nband && ion.band[k] != (int) bitsToInt(data + 4, 4); k++)
                ;
            seen18[k]++;
            expect("MT18 number of bands", bitsToInt(data, 4), 3);
            expect("MT18 IODI", bitsToInt(data + 8, 2), (unsigned long) ion.iodi);
            expect("MT18 mask", memcmp(data + 10, ion.mask[k], SBAS_BAND_BITS), 0);
        } else if (type == 26) {
            int k, blk = (int) bitsToInt(data + 4, 4);
            for (k = 0; k < ion.nband && ion.band[k] != (int) bitsToInt(data, 4); k++)
                ;
            seen26[k][blk]++;
            expect("MT26 IODI", bitsToInt(data + 203, 2), (unsigned long) ion.iodi);
            for (int i = 0; i < 15 && 15 * blk + i < ion.nigp[k]; i++) {
                expect("MT26 GIVD", bitsToInt(data + 8 + 13 * i, 9), (unsigned long) lround(ion.delay[k][15 * blk + i] / 0.125));
                expect("MT26 GIVEI", bitsToInt(data + 17 + 13 * i, 4), 6);
            }
        }
    }
    for (int k = 0; k < ion.nband; k++) {
        expect("MT18 of each band within 300 s", seen18[k] > 0, 1);
        for (int b = 0; b < (ion.nigp[k] + 14) / 15; b++)
            expect("MT26 of each block within 300 s", seen26[k][b] > 0, 1);
    }

    // MT17 once a minute: the GEOs being sent, provider ID and "no ranging" as EGNOS (0x11)
    const sbas_geo_t geos[2] = {{136, "EGNOS", 5.0}, {131, "WAAS", -117.0}};
    int n17 = 0;
    for (int sec = 600; sec < 660; sec++) {
        if (sbas_message(sec, iode, pos, &ion, geos, 2, data) != 17)
            continue;
        n17++;
        expect("MT17 PRN 1", bitsToInt(data + 2, 8), 136);
        expect("MT17 health 1 (EGNOS)", bitsToInt(data + 10, 8), 0x11);
        expect("MT17 XG 1 [2600 m]", bitsToInt(data + 18, 15), (unsigned long) lround(SBAS_GEO_RADIUS * cos(5.0 / 57.29577951308232) / 2600.0));
        expect("MT17 PRN 2", bitsToInt(data + 67 + 2, 8), 131);
        expect("MT17 health 2 (WAAS)", bitsToInt(data + 67 + 10, 8), 0x01);
        long yg = (long) bitsToInt(data + 67 + 33, 15);
        expect("MT17 YG 2 negative", yg >= 1 << 14, 1);
        expect("MT17 slot 3 empty", bitsToInt(data + 134, 67), 0);
        expect("MT17 t0 [64 s]", bitsToInt(data + 201, 11), (unsigned long) ((sec % 86400) / 64));
    }
    expect("MT17 per minute", n17, 1);

    // A receiver's MOPS model from our grid (pierce point at 350 km, bilinear in the 5 deg cell,
    // MOPS obliquity) must give the Klobuchar slant delay the signal carries, within 1 m above 15 deg
    const double d2r = 1 / 57.29577951308232, re = 6378.1363, hi = 350.0;
    double worst = 0.0;
    for (double az = 90; az <= 270; az += 15)
        for (double el = 15; el <= 90; el += 15) {
            double E = el * d2r, A = az * d2r;
            double psi = M_PI / 2 - E - asin(re / (re + hi) * cos(E));
            double plat = asin(sin(llh[0]) * cos(psi) + cos(llh[0]) * sin(psi) * cos(A));
            double plon = llh[1] + asin(sin(psi) * sin(A) / cos(plat));
            plat /= d2r; plon /= d2r;
            double la0 = floor(plat / 5) * 5, lo0 = floor(plon / 5) * 5, v[4] = {0};
            int found = 0;
            for (int k = 0; k < ion.nband; k++)
                for (int i = 0; i < ion.nigp[k]; i++)
                    for (int c = 0; c < 4; c++)
                        if (ion.lat[k][i] == la0 + 5 * (c / 2) && ion.lon[k][i] == lo0 + 5 * (c % 2)) {
                            v[c] = ion.delay[k][i];
                            found |= 1 << c;
                        }
            expect("pierce point cell has 4 IGPs", found, 15);
            double x = (plon - lo0) / 5, y = (plat - la0) / 5;
            double vert = (1 - x) * (1 - y) * v[0] + x * (1 - y) * v[1] + (1 - x) * y * v[2] + x * y * v[3];
            double F = 1 / sqrt(1 - pow(re * cos(E) / (re + hi), 2));
            double azel[2] = {A, E};
            double klob = ionosphericDelay(&io, g, llh, azel);
            if (fabs(F * vert - klob) > worst)
                worst = fabs(F * vert - klob);
        }
    printf("grid vs Klobuchar slant delay: worst %.2f m\n", worst);
    expect("grid vs Klobuchar within 1 m", worst < 1.0, 1);

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
