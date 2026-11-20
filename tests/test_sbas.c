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

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
