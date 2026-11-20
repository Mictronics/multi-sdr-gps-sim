/* Checks the SBAS L1 message framing and convolutional code.
 * Run with "make test".
 */
#include <stdio.h>
#include <string.h>
#include "sbas.h"

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
        int type = sbas_message(sec, iode, pos, data);
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

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
