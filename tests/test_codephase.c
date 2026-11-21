/* Checks that the code phase runs on continuously across the 0.1 s blocks: the phase generateSamples
 * reaches at a block end must equal the one updateChannels sets from the range for the next block.
 * Double rounding of the time of week leaves up to 1e-4 chip; a jump of 1e-3 chip (0.3 m) fails.
 * Includes gps.c for its static channel functions, so it links the modules without gps.o.
 * Run with "make test".
 */
#include "../gps.c"

#define SAMPLE "tests/data/sample_rinex3.rnx"
#define BLOCKS 100
#define MAX_JUMP 1e-3 // chip

// Code phase in chips since the start of the channel's data words
static double gpsPhase(const channel_t *ch) {
    return (ch->iword * 600.0 + ch->ibit * 20.0 + ch->icode) * CA_SEQ_LEN + ch->code_phase;
}

static double sbasPhase(const sbas_chan_t *sb) {
    return (sb->isym * 2.0 + sb->icode) * CA_SEQ_LEN + sb->code_phase;
}

int main(void) {
    static simulator_t sim;
    static gps_ctx_t c;
    static almanac_gps_t alm;
    int failures = 0;

    c.sim = &sim;
    c.alm = &alm;
    if (readRinex(&c.table, &c.ionoutc, SAMPLE, true) <= 0) {
        printf("FAIL cannot read %s\n", SAMPLE);
        return 1;
    }
    ionoDefaults(&c.ionoutc, c.table.rec[0].toc);
    c.ionoutc.enable = true;
    datetime_t t = {2026, 11, 17, 2, 0, 0};
    date2gps(&t, &c.g0);
    ephSelect(&c.table, c.g0, c.eph);

    // User below G01, with a forced GEO at the same longitude, so both kinds of channel run
    double pos[3], vel[3], clk[2], llh[3];
    satpos(c.eph[0], c.g0, pos, vel, clk);
    xyz2llh(pos, llh);
    llh[2] = 0.0;
    sim.sbas_enable = true;
    sim.sbas_prn = 136;
    sim.sbas_lon = llh[1] * R2D;

    c.numd = BLOCKS + 1;
    c.xyz = calloc(c.numd, sizeof (*c.xyz));
    c.iq_buff = calloc(IQ_BUFFER_SIZE, sizeof (short));
    for (int i = 0; i < c.numd; i++)
        llh2xyz(llh, c.xyz[i]);
    initChannels(&c);

    int ngps = 0, nsbas = 0;
    for (int i = 0; i < MAX_CHAN; i++)
        ngps += c.chan[i].prn > 0;
    for (int i = 0; i < SBAS_MAX_CHAN; i++)
        nsbas += c.sbas[i].prn > 0;
    if (ngps == 0 || nsbas == 0) {
        printf("FAIL setup: %d GPS and %d SBAS channels\n", ngps, nsbas);
        return 1;
    }

    double end[MAX_CHAN], send[SBAS_MAX_CHAN], worst = 0.0, sworst = 0.0;
    for (int k = 1; k <= BLOCKS; k++) {
        updateChannels(&c, k);
        for (int i = 0; k > 1 && i < MAX_CHAN; i++)
            if (c.chan[i].prn > 0)
                worst = fmax(worst, fabs(gpsPhase(&c.chan[i]) - end[i]));
        for (int i = 0; k > 1 && i < SBAS_MAX_CHAN; i++)
            if (c.sbas[i].prn > 0)
                sworst = fmax(sworst, fabs(sbasPhase(&c.sbas[i]) - send[i]));
        generateSamples(&c);
        for (int i = 0; i < MAX_CHAN; i++)
            end[i] = gpsPhase(&c.chan[i]);
        for (int i = 0; i < SBAS_MAX_CHAN; i++)
            send[i] = sbasPhase(&c.sbas[i]);
        c.grx = incGpsTime(c.grx, 0.1);
    }

    if (worst > MAX_JUMP) {
        printf("FAIL GPS code phase jumps %.2e chip at a block edge\n", worst);
        failures++;
    }
    if (sworst > MAX_JUMP) {
        printf("FAIL SBAS code phase jumps %.2e chip at a block edge\n", sworst);
        failures++;
    }
    free(c.xyz);
    free(c.iq_buff);
    free(c.table.rec);
    if (failures == 0)
        printf("all checks passed (%d GPS, %d SBAS channels, max jump %.1e / %.1e chip)\n", ngps, nsbas, worst, sworst);
    return failures ? 1 : 0;
}
