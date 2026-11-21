/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2026 Mictronics
 * Distributed under the MIT License.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <curl/curl.h>
#include <zlib.h>
#include "sdr.h"
#include "gui.h"
#include "fifo.h"
#include "almanac.h"
#include "sbas.h"
#include "gps-sim.h"
#include "orbit.h"
#include "rinex.h"
#include "model.h"
#include "navmsg.h"
#include "download.h"

const int sinTable512[] = {
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
    50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 91, 94,
    97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
    140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
    178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
    209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
    245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
    245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
    230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
    207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
    176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
    138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100, 97,
    94, 91, 89, 86, 83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 53, 50,
    47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 2,
    -2, -5, -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
    -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
    -97, -100, -103, -105, -108, -111, -114, -116, -119, -122, -125, -127, -130, -132, -135, -138,
    -140, -143, -145, -148, -150, -153, -155, -157, -160, -162, -164, -167, -169, -171, -173, -176,
    -178, -180, -182, -184, -186, -188, -190, -192, -194, -196, -198, -200, -202, -204, -205, -207,
    -209, -210, -212, -214, -215, -217, -218, -220, -221, -223, -224, -225, -227, -228, -229, -230,
    -232, -233, -234, -235, -236, -237, -238, -239, -240, -241, -241, -242, -243, -244, -244, -245,
    -245, -246, -247, -247, -248, -248, -248, -249, -249, -249, -249, -250, -250, -250, -250, -250,
    -250, -250, -250, -250, -250, -249, -249, -249, -249, -248, -248, -248, -247, -247, -246, -245,
    -245, -244, -244, -243, -242, -241, -241, -240, -239, -238, -237, -236, -235, -234, -233, -232,
    -230, -229, -228, -227, -225, -224, -223, -221, -220, -218, -217, -215, -214, -212, -210, -209,
    -207, -205, -204, -202, -200, -198, -196, -194, -192, -190, -188, -186, -184, -182, -180, -178,
    -176, -173, -171, -169, -167, -164, -162, -160, -157, -155, -153, -150, -148, -145, -143, -140,
    -138, -135, -132, -130, -127, -125, -122, -119, -116, -114, -111, -108, -105, -103, -100, -97,
    -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
    -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11, -8, -5, -2
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
const double ant_pat_db[37] = {
    0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22,
    4.89, 5.56, 6.22, 6.89, 7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56
};

static int allocatedSat[MAX_SAT];

/* Generate the C/A code sequence for a given Satellite Vehicle PRN
 * prn PRN number, 1-32 (GPS) or 120-158 (SBAS)
 * ca Caller-allocated integer array of 1023 bytes
 */
void codegen(int *ca, int prn) {
    int delay[] = {
        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862
    };
    // gps.gov L1 C/A PRN code assignments, Jan 2026
    static const int sbas_delay[] = {
        145, 175, 52, 21, 237, 235, 886, 657, 634, 762,
        355, 1012, 176, 603, 130, 359, 595, 68, 386, 797,
        456, 499, 883, 307, 127, 211, 121, 118, 163, 628,
        853, 484, 289, 811, 202, 1021, 463, 568, 904
    };
    int d;

    int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
    int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
    int c1, c2;
    int i, j;

    if (prn >= 1 && prn <= 32)
        d = delay[prn - 1];
    else if (prn >= 120 && prn <= 158)
        d = sbas_delay[prn - 120];
    else
        return;

    for (i = 0; i < N_DWRD_SBF; i++)
        r1[i] = r2[i] = -1;

    for (i = 0; i < CA_SEQ_LEN; i++) {
        g1[i] = r1[9];
        g2[i] = r2[9];
        c1 = r1[2] * r1[9];
        c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];

        for (j = 9; j > 0; j--) {
            r1[j] = r1[j - 1];
            r2[j] = r2[j - 1];
        }
        r1[0] = c1;
        r2[0] = c2;
    }

    for (i = 0, j = CA_SEQ_LEN - d; i < CA_SEQ_LEN; i++, j++)
        ca[i] = (1 - g1[i] * g2[j % CA_SEQ_LEN]) / 2;

    return;
}


/* Compute the code phase for a given channel (satellite)
 * chan Channel on which we operate (is updated)
 * rho1 Current range, after \a dt has expired
 * dt delta-t (time difference) in seconds
 */
static void computeCodePhase(channel_t *chan, range_t rho1, double dt) {
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range) / dt;

    // Carrier and code frequency.
    chan->f_carr = -rhorate / LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g, chan->g0) + 6.0) - chan->rho0.range / SPEED_OF_LIGHT)*1000.0;

    ims = (int) ms;
    chan->code_phase = (ms - (double) ims) * CA_SEQ_LEN; // in chip

    chan->iword = ims / 600; // 1 word = 30 bits = 600 ms
    ims -= chan->iword * 600;

    chan->ibit = ims / 20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int) chan->code_phase]*2 - 1;
    chan->dataBit = (int) ((chan->dwrd[chan->iword]>>(29 - chan->ibit)) & 0x1UL)*2 - 1;

    // Save current pseudorange
    chan->rho0 = rho1;
}

static int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel) {
    double llh[3], neu[3];
    double pos[3], vel[3], clk[3], los[3];
    double tmat[3][3];

    if (eph.vflg == false)
        return (-1); // Invalid

    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);

    if (azel[1] * R2D > elvMask)
        return (1); // Visible
    // else
    return (0); // Invisible
}

static int allocateChannel(channel_t *chan, almanac_gps_t *alm, const char *msg, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz) {
    int nsat = 0;
    int i, sv;
    double azel[2];
    range_t rho;

    for (sv = 0; sv < MAX_SAT; sv++) {
        if (checkSatVisibility(eph[sv], grx, xyz, 0.0, azel) == 1) {
            nsat++; // Number of visible satellites

            if (allocatedSat[sv] == -1) // Visible but not allocated
            {
                // Allocated new satellite
                for (i = 0; i < MAX_CHAN; i++) {
                    if (chan[i].prn == 0) {
                        // Initialize channel
                        chan[i].prn = sv + 1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, alm, msg, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;
                        chan[i].carr_phase = 0.0; // Arbitrary: receivers track the phase from wherever it starts
                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < MAX_CHAN)
                    allocatedSat[sv] = i;
            }
        } else if (allocatedSat[sv] >= 0) // Not visible but allocated
        {
            // Clear channel
            chan[allocatedSat[sv]].prn = 0;

            // Clear satellite allocation flag
            allocatedSat[sv] = -1;
        }
    }

    return (nsat);
}

/* Read the list of user motions from the input file
 * xyz Output array of ECEF vectors for user motion
 * filename File name of the text input file
 * Returns number of user data motion records read, -1 on error
 */
static int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename) {
    FILE *fp;
    int numd;
    char str[MAX_CHAR];
    double t, x, y, z;

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++) {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z) != 4) // Read CSV line
            break;

        xyz[numd][0] = x;
        xyz[numd][1] = y;
        xyz[numd][2] = z;
    }

    fclose(fp);

    return (numd);
}

#define SAMPLE_PERIOD (1.0 / (double) TX_SAMPLERATE)

// --cn0 noise floor: sigma per I/Q component in iq_buff units; all outputs clip at +-2047, 5 sigma
#define NOISE_SIGMA 400.0
#define NOISE_TABLE 4096 // Gaussian values, indexed by 12 random bits

// Satellites with a record near the start time that make -s now keep the ephemeris unshifted
#define COVER_MIN_SATS 4

#define SBAS_MAX_CHAN 3
#define SBAS_SYMS (60 * 500) // 60 s of 500 symbol/s, refilled every 30 s

// SBAS GEO channel: 1 ms C/A code, 2 code periods per symbol
typedef struct {
    int prn; // 0 = unused
    double pos[3]; // GEO at rest in ECEF
    int ca[CA_SEQ_LEN];
    double f_carr, f_code, carr_phase, code_phase;
    gpstime_t g0; // Transmit time of sym[0]
    int isym, icode, symVal, codeCA;
    unsigned enc; // Convolutional encoder state after the last symbol in sym[]
    unsigned char sym[SBAS_SYMS];
    double azel[2];
    range_t rho0;
} sbas_chan_t;

// State of the signal generation, shared by the steps of gps_thread_ep.
typedef struct {
    simulator_t *sim;
    eph_table_t table; // Every ephemeris record of the file
    ephem_t eph[MAX_SAT]; // Record in use for each satellite: the closest in time
    ionoutc_t ionoutc;
    almanac_gps_t *alm;
    channel_t chan[MAX_CHAN];
    double gain[MAX_CHAN];
    sbas_chan_t sbas[SBAS_MAX_CHAN];
    double sbas_gain[SBAS_MAX_CHAN];
    sbas_iono_t sbas_iono; // Ionospheric grid around the user, sent by all GEOs
    sbas_geo_t sbas_geos[SBAS_MAX_CHAN]; // GEOs being sent, for their almanac (MT17)
    int sbas_ngeo;
    double ant_pat[37]; // Receiver antenna gain per 5 degree of boresight angle
    double (*xyz)[3]; // User position every 0.1 s
    int numd; // Number of positions
    gpstime_t g0; // Scenario start
    gpstime_t grx; // Receiver time
    short *iq_buff; // 0.1 s of 16 bit IQ samples
    double level; // Signal scale: 1 without --cn0, else the --cn0 level at 20200 km and boresight
    bool noise; // --cn0: add noise[] to every sample
    short noise_tab[NOISE_TABLE];
    uint32_t rng; // xorshift32 state; fixed seed so runs stay reproducible
} gps_ctx_t;

/* List the allocated channels in the LS_FIX window. */
static void showChannels(const gps_ctx_t *c) {
    int row = LS_ROW_CHAN;

    for (int i = 0; i < MAX_CHAN; i++) {
        if (c->chan[i].prn > 0) {
            gui_mvwprintw(LS_FIX, row++, 1, "%02d %6.1f %5.1f %11.1f %5.1f", c->chan[i].prn,
                    c->chan[i].azel[0] * R2D, c->chan[i].azel[1] * R2D, c->chan[i].rho0.d, c->chan[i].rho0.iono_delay);
        }
    }
    gui_mvwprintw(LS_FIX, LS_ROW_NAV, LS_COL_INFO, "Nav: %02d satellites", row - LS_ROW_CHAN);

    // Fixed rows below the GPS rows, so a changing GPS count leaves no stale SBAS line
    for (int i = 0; i < SBAS_MAX_CHAN; i++) {
        const sbas_chan_t *sb = &c->sbas[i];
        if (sb->prn > 0)
            gui_mvwprintw(LS_FIX, LS_ROW_CHAN + MAX_CHAN + i, 1, "%03d %5.1f %5.1f %11.1f %5.1f", sb->prn,
                    sb->azel[0] * R2D, sb->azel[1] * R2D, sb->rho0.d, sb->rho0.iono_delay);
        else
            gui_mvwprintw(LS_FIX, LS_ROW_CHAN + MAX_CHAN + i, 1, "%35s", "");
    }
}

static int sbasSymbol(const sbas_chan_t *sb) {
    int k = sb->isym < 0 ? 0 : (sb->isym >= SBAS_SYMS ? SBAS_SYMS - 1 : sb->isym); // A bad range must not index outside
    return sb->sym[k] * 2 - 1;
}

/* Encode the messages of nsec seconds, starting at transmit time g, into sym[] from second off on.
 * eph gives the IODEs the long term corrections refer to. */
static void sbasFill(sbas_chan_t *sb, const gps_ctx_t *c, gpstime_t g, int off, int nsec) {
    unsigned char data[SBAS_DATA_BITS], msg[SBAS_MSG_BITS];
    int iode[MAX_SAT];

    for (int sv = 0; sv < MAX_SAT; sv++)
        iode[sv] = c->eph[sv].vflg ? c->eph[sv].iode : -1;
    for (int k = 0; k < nsec; k++) {
        int sec = (int) (incGpsTime(g, k).sec + 0.5);
        int type = sbas_message(sec, iode, sb->pos, &c->sbas_iono, c->sbas_geos, c->sbas_ngeo, data);

        sbas_frame(type, data, sec, msg);
        sbas_encode(&sb->enc, msg, SBAS_MSG_BITS, sb->sym + (off + k) * 500);
    }
}

/* Start an SBAS channel at receiver time grx (on a 30 s boundary, or the scenario start) */
static void sbasStart(sbas_chan_t *sb, const sbas_geo_t *geo, gps_ctx_t *c, double *xyz) {
    static const double zero[3] = {0.0, 0.0, 0.0};

    sb->prn = geo->prn;
    sb->pos[0] = SBAS_GEO_RADIUS * cos(geo->lon / R2D);
    sb->pos[1] = SBAS_GEO_RADIUS * sin(geo->lon / R2D);
    sb->pos[2] = 0.0;
    codegen(sb->ca, sb->prn);
    // Buffer covers [base - 30 s, base + 30 s); +0.05 s because grx sums 0.1 s steps
    sb->g0.week = c->grx.week;
    sb->g0.sec = floor((c->grx.sec + 0.05) / 30.0) * 30.0;
    sb->g0 = incGpsTime(sb->g0, -30.0);
    sb->enc = 0;
    sbasFill(sb, c, sb->g0, 0, 60);
    rangeFromPos(&sb->rho0, sb->pos, zero, 0.0, &c->ionoutc, c->grx, xyz);
    sb->azel[0] = sb->rho0.azel[0];
    sb->azel[1] = sb->rho0.azel[1];
    sb->carr_phase = 0.0;
}

/* Choose the GEOs: the system of the highest one, up to SBAS_MAX_CHAN of its GEOs above the horizon.
 * Channels keep running when their GEO stays chosen. */
static void allocateSbas(gps_ctx_t *c, double *xyz) {
    const simulator_t *sim = c->sim;
    static const double zero[3] = {0.0, 0.0, 0.0};
    sbas_geo_t forced = {sim->sbas_prn, "SBAS", sim->sbas_lon};
    for (int k = 0; k < sbas_geo_count; k++) // the system of a known PRN, for MT17
        if (sbas_geo[k].prn == sim->sbas_prn)
            forced.system = sbas_geo[k].system;
    const sbas_geo_t *list = sim->sbas_prn ? &forced : sbas_geo;
    int n = sim->sbas_prn ? 1 : sbas_geo_count;
    const sbas_geo_t *want[SBAS_MAX_CHAN] = {NULL};
    double el[64];
    int best = -1, nwant = 0;

    if (!sim->sbas_enable)
        return;

    for (int k = 0; k < n; k++) {
        double pos[3] = {SBAS_GEO_RADIUS * cos(list[k].lon / R2D), SBAS_GEO_RADIUS * sin(list[k].lon / R2D), 0.0};
        range_t rho;
        rangeFromPos(&rho, pos, zero, 0.0, &c->ionoutc, c->grx, xyz);
        el[k] = rho.azel[1];
        if (el[k] > 0.0 && (best < 0 || el[k] > el[best]))
            best = k;
    }
    // Highest GEOs of the best system first
    for (int pick = 0; best >= 0 && pick < SBAS_MAX_CHAN; pick++) {
        int top = -1;
        for (int k = 0; k < n; k++) {
            bool taken = false;
            for (int j = 0; j < nwant; j++)
                taken |= want[j] == &list[k];
            if (!taken && el[k] > 0.0 && strcmp(list[k].system, list[best].system) == 0 && (top < 0 || el[k] > el[top]))
                top = k;
        }
        if (top < 0)
            break;
        want[nwant++] = &list[top];
    }

    c->sbas_ngeo = nwant; // the almanac lists the GEOs chosen now
    for (int j = 0; j < nwant; j++)
        c->sbas_geos[j] = *want[j];
    for (int i = 0; i < SBAS_MAX_CHAN; i++) { // Drop GEOs no longer chosen
        bool keep = false;
        for (int j = 0; j < nwant; j++)
            keep |= c->sbas[i].prn == want[j]->prn;
        if (!keep)
            c->sbas[i].prn = 0;
    }
    for (int j = 0; j < nwant; j++) { // Start the new ones in free channels
        bool running = false;
        for (int i = 0; i < SBAS_MAX_CHAN; i++)
            running |= c->sbas[i].prn == want[j]->prn;
        for (int i = 0; !running && i < SBAS_MAX_CHAN; i++) {
            if (c->sbas[i].prn == 0) {
                sbasStart(&c->sbas[i], want[j], c, xyz);
                running = true;
            }
        }
    }
}

/* Move the ECEF position xyz by neu (north, east, up in metres) in its own local frame.
 * A fixed frame would make straight travel leave the Earth's surface: 785 m up after 100 km. */
static void addNeu(double *xyz, const double *neu) {
    double llh[3], t[3][3];

    xyz2llh(xyz, llh);
    ltcmat(llh, t);
    for (int k = 0; k < 3; k++)
        xyz[k] += t[0][k] * neu[0] + t[1][k] * neu[1] + t[2][k] * neu[2];
}

/* Set the start position, or the target given with -t, as the user motion. */
static bool initUserPosition(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    double llh[3];
    double neu[3];

    c->xyz = malloc(sizeof (double[USER_MOTION_SIZE][3]));
    if (c->xyz == NULL) {
        gui_status_wprintw(RED, "Failed to allocate user motion memory.\n");
        return false;
    }

    c->numd = sim->duration;

    llh[0] = sim->location.lat / R2D;
    llh[1] = sim->location.lon / R2D;
    llh[2] = sim->location.height;
    llh2xyz(llh, c->xyz[0]);

    if (!sim->target.valid) {
        // No target position given, use location
        sim->target.lat = sim->location.lat;
        sim->target.lon = sim->location.lon;
        sim->target.height = sim->location.height;
    } else {
        // Target given as distance and bearing from the user location
        neu[0] = sim->target.distance * cos((sim->target.bearing / 1000) / R2D);
        neu[1] = sim->target.distance * sin((sim->target.bearing / 1000) / R2D);
        neu[2] = sim->target.height;
        addNeu(c->xyz[0], neu);
    }

    for (int i = 1; i < c->numd; i++) {
        c->xyz[i][0] = c->xyz[0][0];
        c->xyz[i][1] = c->xyz[0][1];
        c->xyz[i][2] = c->xyz[0][2];
    }

    gui_show_location(&sim->location);
    return true;
}

/* Download (-f) or read the ephemeris. */
static bool loadEphemeris(gps_ctx_t *c) {
    simulator_t *sim = c->sim;

    if ((sim->nav_file_name == NULL) && (sim->use_ftp == false)) {
        gui_status_wprintw(RED, "GPS ephemeris file is not specified.\n");
        return false;
    }

    // The online download takes precedence over -e
    const char *nav_file = sim->nav_file_name;
    if (sim->use_ftp) {
        nav_file = sim->use_rinex3 ? RINEX3_FILE_NAME : RINEX2_FILE_NAME;
        gui_status_wprintw(GREEN, "Downloading RINEX v%u ephemeris.\n", sim->use_rinex3 ? 3 : 2);
        CURLcode res = download_nav_file(sim->use_rinex3, nav_file, &sim->main_exit);
        if (res != CURLE_OK) {
            gui_status_wprintw(RED, "Ephemeris download failed: %s\n", curl_easy_strerror(res));
            return false;
        }
    }

    c->ionoutc.enable = sim->ionosphere_enable;
    int nrec = readRinex(&c->table, &c->ionoutc, nav_file, sim->use_rinex3);

    if (nrec <= 0) { // 0 = no records, negative = file could not be read or has wrong format
        gui_status_wprintw(RED, "No ephemeris available.\n");
        return false;
    }

    if (sim->show_verbose) {
        const ionoutc_t *io = &c->ionoutc;
        if (io->vflg && io->enable) {
            gui_mvwprintw(LS_FIX, LS_ROW_IONO, LS_COL_INFO, "ION ALPHA %12.3e %12.3e %12.3e %12.3e",
                    io->alpha0, io->alpha1, io->alpha2, io->alpha3);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 1, LS_COL_INFO, "ION BETA  %12.3e %12.3e %12.3e %12.3e",
                    io->beta0, io->beta1, io->beta2, io->beta3);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 2, LS_COL_INFO, "DELTA UTC %12.3e %12.3e %9d  %9d",
                    io->A0, io->A1, io->tot, io->wnt);
            gui_mvwprintw(LS_FIX, LS_ROW_IONO + 3, LS_COL_INFO, "LEAP SECONDS %d, then %d (week %d, day %d)", io->dtls, io->dtlsf, io->wnlsf, io->dn);
        } else {
            gui_mvwprintw(LS_FIX, LS_ROW_IONO, LS_COL_INFO, "Ionospheric data incomplete or disabled!");
        }
    }
    return true;
}

/* Replace the user motion with the one from the -m file. */
static bool loadMotionFile(gps_ctx_t *c) {
    simulator_t *sim = c->sim;

    if (sim->motion_file_name == NULL)
        return true;

    c->numd = readUserMotion(c->xyz, sim->motion_file_name);
    if (c->numd <= 0) {
        gui_status_wprintw(RED, "Failed to read user motion file.\n");
        return false;
    }
    gui_status_wprintw(GREEN, "%u user motion points applied.\n", c->numd);
    if (c->numd > sim->duration) {
        c->numd = sim->duration;
    }
    return true;
}

/* Number of satellites with a record within EPH_MAX_AGE of g. */
static int countCovered(const eph_table_t *tab, gpstime_t g) {
    ephem_t eph[MAX_SAT];
    int n = 0;

    ephSelect(tab, g, eph);
    for (int sv = 0; sv < MAX_SAT; sv++)
        n += eph[sv].vflg ? 1 : 0;
    return n;
}

/* Set the scenario start time g0, shifting the ephemeris to it for -s now. */
static bool selectStartTime(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    datetime_t ttmp;
    datetime_t tmin = {0}, tmax = {0};
    gpstime_t gmin = {0, 0.0}, gmax = {0, 0.0};
    gpstime_t gtmp;

    if (sim->time_overwrite) { // -s now: read the clock here, after the download
        time_t now = time(NULL);
        struct tm tm;
        gmtime_r(&now, &tm);
        sim->start = (datetime_t) {tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, (double) tm.tm_sec};
    }

    c->g0.week = -1; // Invalid start time
    if (sim->start.m >= 1) { // Zero month means no -s given
        date2gps(&sim->start, &c->g0);
    }
    if (sim->time_overwrite) // The clock gives UTC; GPS time is ahead by the leap seconds
        c->g0 = incGpsTime(c->g0, (c->ionoutc.have & 0x8) ? c->ionoutc.dtls : LEAP_DEFAULT_DTLS);

    // The NOAA file can lag hours behind; BKG's file of the day reaches the current hour
    if (sim->time_overwrite && sim->use_ftp && !sim->use_rinex3 && countCovered(&c->table, c->g0) < COVER_MIN_SATS) {
        gui_status_wprintw(YELLOW, "RINEX v2 file does not reach the start time, trying RINEX v3.\n");
        eph_table_t t3;
        ionoutc_t io3 = c->ionoutc;
        if (download_nav_file(true, RINEX3_FILE_NAME, &sim->main_exit) == CURLE_OK &&
                readRinex(&t3, &io3, RINEX3_FILE_NAME, true) > 0) {
            if (countCovered(&t3, c->g0) > countCovered(&c->table, c->g0)) {
                free(c->table.rec);
                c->table = t3;
                c->ionoutc = io3;
            } else {
                free(t3.rec);
            }
        }
    }

    // Earliest and latest clock time of the file
    gmin = gmax = c->table.rec[0].toc;
    tmin = tmax = c->table.rec[0].t;
    for (int i = 1; i < c->table.n; i++) {
        const ephem_t *e = &c->table.rec[i];
        if (subGpsTime(e->toc, gmin) < 0.0) {
            gmin = e->toc;
            tmin = e->t;
        }
        if (subGpsTime(e->toc, gmax) > 0.0) {
            gmax = e->toc;
            tmax = e->t;
        }
    }

    if (c->g0.week >= 0) { // Scenario start time has been set.
        if (sim->time_overwrite == true) {
            // Shifting moves the satellites away from where the real almanac puts them, so it is
            // only done for data that does not reach the start time (e.g. an old nav file)
            if (countCovered(&c->table, c->g0) >= COVER_MIN_SATS) {
                gui_status_wprintw(GREEN, "Ephemeris covers the start time, used as it is.\n");
            } else {
                gui_status_wprintw(RED, "Ephemeris does not reach the start time, shifting it: it no longer matches the\n"
                        "almanac and receivers may distrust it (use --disable-almanac).\n");
                gtmp.week = c->g0.week;
                gtmp.sec = (double) (((int) (c->g0.sec)) / 7200)*7200.0;

                double dsec = subGpsTime(gtmp, gmin);

                // Overwrite the UTC reference week number
                c->ionoutc.wnt = gtmp.week;
                c->ionoutc.tot = (int) gtmp.sec;

                // Overwrite the TOC and TOE to the scenario start time
                for (int i = 0; i < c->table.n; i++) {
                    ephem_t *e = &c->table.rec[i];
                    e->toc = incGpsTime(e->toc, dsec);
                    gps2date(&e->toc, &ttmp);
                    e->t = ttmp;
                    e->toe = incGpsTime(e->toe, dsec);
                }
            }
        } else if (subGpsTime(c->g0, gmin) < 0.0 || subGpsTime(gmax, c->g0) < 0.0) {
            gui_status_wprintw(RED, "Invalid start time.\n");
            gui_status_wprintw(RED, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                    tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec, gmin.week, gmin.sec);
            gui_status_wprintw(RED, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                    tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec, gmax.week, gmax.sec);
            return false;
        }
    } else {
        c->g0 = gmin;
        sim->start = tmin;
    }

    if (ionoDefaults(&c->ionoutc, c->g0))
        gui_status_wprintw(YELLOW, "Iono/UTC data incomplete in the file, sending neutral values for it.\n");

    gui_mvwprintw(LS_FIX, LS_ROW_RINEX, LS_COL_INFO, "RINEX date:      %s", rinex_date);
    gui_mvwprintw(LS_FIX, LS_ROW_START, LS_COL_INFO, "Start time:      %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)",
            sim->start.y, sim->start.m, sim->start.d, sim->start.hh, sim->start.mm, sim->start.sec, c->g0.week, c->g0.sec);
    if (sim->show_verbose) {
        gui_mvwprintw(LS_FIX, LS_ROW_SIMTIME, LS_COL_INFO, "Simulation time: ");
    }
    gui_mvwprintw(LS_FIX, LS_ROW_DURATION, LS_COL_INFO, "Duration:        %.1fs", ((double) c->numd) / 10.0);

    // The ephemerides in use at the start
    ephSelect(&c->table, c->g0, c->eph);

    int usable = 0;
    for (int sv = 0; sv < MAX_SAT; sv++)
        usable += c->eph[sv].vflg ? 1 : 0;
    if (usable == 0) {
        gui_status_wprintw(RED, "No ephemeris found close to the start time.\n");
        return false;
    }
    return true;
}

/* Download (-f) or read the almanac and check it is close to the scenario time. */
static bool loadAlmanac(gps_ctx_t *c) {
    simulator_t *sim = c->sim;
    datetime_t ttmp;
    gpstime_t gtmp;

    c->alm = almanac_init();
    if (sim->almanac_enable) {
        CURLcode res = sim->use_ftp ? almanac_download(&sim->main_exit) : almanac_read_file();

        switch (res) {
            case CURLE_OK:
                break;
            case CURLE_REMOTE_FILE_NOT_FOUND:
                gui_status_wprintw(RED, "Almanac file not found!\n");
                break;
            case CURLE_READ_ERROR:
                gui_status_wprintw(RED, "Error reading almanac file!\n");
                break;
            default:
                gui_status_wprintw(RED, "Almanac error, code: %d\n", res);
                break;
        }
    }

    if (sim->almanac_enable && c->alm->valid) {
        gtmp.sec = 0.0;
        gtmp.week = 0;
        for (int sv = 0; sv < MAX_SAT; sv++) {
            almanac_prn_t *a = &c->alm->sv[sv];
            if (a->valid != 0) {
                a->toa.week = resolveWeek(a->toa.week, c->g0.week);
                gtmp = a->toa;
                double dt = subGpsTime(a->toa, c->g0);
                if (dt < (-4.0 * SECONDS_IN_WEEK) || dt > (4.0 * SECONDS_IN_WEEK)) {
                    gui_status_wprintw(RED, "Invalid time of almanac.\n");
                    return false;
                }
            }
        }
        gps2date(&gtmp, &ttmp);
        gui_mvwprintw(LS_FIX, LS_ROW_ALMANAC, LS_COL_INFO, "Almanac date:    %4d/%02d/%02d,%02d:%02d:%02.0f",
                ttmp.y, ttmp.m, ttmp.d, ttmp.hh, ttmp.mm, ttmp.sec);
    } else {
        gui_mvwprintw(LS_FIX, LS_ROW_ALMANAC, LS_COL_INFO, "Almanac date:    Disabled or invalid.");
    }
    return true;
}

/* Allocate the satellites visible at the start and set up the antenna pattern. */
static uint32_t xorshift32(uint32_t *x) {
    *x ^= *x << 13;
    *x ^= *x >> 17;
    *x ^= *x << 5;
    return *x;
}

/* Signal scale and noise table for --cn0. A satellite at the nominal zenith distance gets the
 * amplitude A = sigma * sqrt(2 * C/N0 / fs) (complex noise power 2 sigma^2 over fs); 250 is the
 * sine table's peak. Lower satellites keep the path loss and antenna pattern shape. */
static void initLevel(gps_ctx_t *c) {
    c->level = 1.0;
    c->noise = c->sim->cn0 > 0.0;
    if (!c->noise)
        return;
    c->level = NOISE_SIGMA * sqrt(2.0 * pow(10.0, c->sim->cn0 / 10.0) / TX_SAMPLERATE) / 250.0;
    c->rng = 0x9E3779B9u;
    for (int k = 0; k < NOISE_TABLE; k += 2) { // Box-Muller, two values per pair of uniforms
        double u1 = (xorshift32(&c->rng) + 1.0) / 4294967297.0;
        double u2 = xorshift32(&c->rng) / 4294967296.0;
        double r = NOISE_SIGMA * sqrt(-2.0 * log(u1));
        c->noise_tab[k] = (short) lround(r * cos(2.0 * PI * u2));
        c->noise_tab[k + 1] = (short) lround(r * sin(2.0 * PI * u2));
    }
    // Scale to exactly NOISE_SIGMA: 4096 random values miss it by about 1 %
    double var = 0.0;
    for (int k = 0; k < NOISE_TABLE; k++)
        var += (double) c->noise_tab[k] * c->noise_tab[k];
    double scale = NOISE_SIGMA / sqrt(var / NOISE_TABLE);
    for (int k = 0; k < NOISE_TABLE; k++)
        c->noise_tab[k] = (short) lround(c->noise_tab[k] * scale);
    gui_status_wprintw(GREEN, "Noise floor on, C/N0 %.1f dB-Hz for a satellite overhead\n", c->sim->cn0);
}

static void initChannels(gps_ctx_t *c) {
    for (int i = 0; i < MAX_CHAN; i++)
        c->chan[i].prn = 0;

    for (int sv = 0; sv < MAX_SAT; sv++)
        allocatedSat[sv] = -1;

    c->grx = incGpsTime(c->g0, 0.0);
    allocateChannel(c->chan, c->alm, c->sim->nav_message, c->eph, c->ionoutc, c->grx, c->xyz[0]);
    if (c->sim->sbas_enable) {
        double llh[3];
        xyz2llh(c->xyz[0], llh);
        sbasIonoUpdate(&c->sbas_iono, &c->ionoutc, c->grx, llh);
    }
    allocateSbas(c, c->xyz[0]);
    showChannels(c);

    initLevel(c);

    for (int i = 0; i < 37; i++)
        c->ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

    c->grx = incGpsTime(c->grx, 0.1);
}

/* Move the user by the speed and heading set in the interactive mode. */
static void moveUser(gps_ctx_t *c, int iumd) {
    const target_t *target = &c->sim->target;
    double dir = (target->bearing / 1000) / R2D;
    double neu[3];

    // Stay at the current location, then move by one 0.1 s step
    neu[0] = (target->velocity * cos(dir)) * 0.1;
    neu[1] = (target->velocity * sin(dir)) * 0.1;
    neu[2] = target->vertical_speed * 0.1;

    memcpy(c->xyz[iumd], c->xyz[iumd - 1], sizeof (c->xyz[iumd]));
    addNeu(c->xyz[iumd], neu);
}

/* Update range, code phase and signal gain of every channel for the next 0.1 s. */
static void updateChannels(gps_ctx_t *c, int iumd) {
    for (int i = 0; i < MAX_CHAN; i++) {
        channel_t *ch = &c->chan[i];
        if (ch->prn > 0) {
            range_t rho;

            computeRange(&rho, c->eph[ch->prn - 1], &c->ionoutc, c->grx, c->xyz[iumd]);

            ch->azel[0] = rho.azel[0];
            ch->azel[1] = rho.azel[1];

            computeCodePhase(ch, rho, 0.1);
            double path_loss = 20200000.0 / rho.d;

            // Receiver antenna gain by boresight angle
            int ibs = (int) ((90.0 - rho.azel[1] * R2D) / 5.0);
            c->gain[i] = path_loss * c->ant_pat[ibs] * c->level;
        }
    }

    static const double zero[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < SBAS_MAX_CHAN; i++) {
        sbas_chan_t *sb = &c->sbas[i];
        if (sb->prn == 0)
            continue;
        range_t rho;
        rangeFromPos(&rho, sb->pos, zero, 0.0, &c->ionoutc, c->grx, c->xyz[iumd]);
        sb->azel[0] = rho.azel[0];
        sb->azel[1] = rho.azel[1];

        // As computeCodePhase, with 2 ms symbols counted from the transmit time g0 of sym[0]
        sb->f_carr = -((rho.range - sb->rho0.range) / 0.1) / LAMBDA_L1;
        sb->f_code = CODE_FREQ + sb->f_carr * CARR_TO_CODE;
        double ms = (subGpsTime(sb->rho0.g, sb->g0) - sb->rho0.range / SPEED_OF_LIGHT) * 1000.0;
        int ims = (int) ms;
        sb->code_phase = (ms - (double) ims) * CA_SEQ_LEN;
        sb->isym = ims / 2;
        sb->icode = ims % 2;
        sb->codeCA = sb->ca[(int) sb->code_phase] * 2 - 1;
        sb->symVal = sbasSymbol(sb);
        sb->rho0 = rho;

        // Path loss relative to the GEO altitude, so an overhead GEO is as strong as an overhead GPS satellite
        int ibs = (int) ((90.0 - rho.azel[1] * R2D) / 5.0);
        c->sbas_gain[i] = 35786000.0 / rho.d * c->ant_pat[ibs] * c->level;
    }
}

/* With --cn0 round to nearest: a signal of a few units would lose power to truncation. Without it
 * v is returned as is, so the output stays as before. */
static inline double sampleRound(double v, bool on) {
    return on ? (double) (int) (v + copysign(0.5, v)) : v;
}

/* Generate 0.1 s of IQ samples into c->iq_buff. */
static void generateSamples(gps_ctx_t *c) {
    channel_t *chan = c->chan;
    const double *gain = c->gain;
    const double delt = SAMPLE_PERIOD;

    for (int isamp = 0; isamp < NUM_IQ_SAMPLES; isamp++) {
        int i_acc = 0;
        int q_acc = 0;

        for (int i = 0; i < MAX_CHAN; i++) {
            if (chan[i].prn > 0) {
                int iTable = (int) floor(chan[i].carr_phase * 512.0) & 511; // Masked: a NaN phase must not index outside
                // dataBit and codeCA are -1 or 1; cosine is the sine 90 degrees (128 steps) ahead
                int ip = sampleRound(chan[i].dataBit * chan[i].codeCA * sinTable512[(iTable + 128) & 511] * gain[i], c->noise);
                int qp = sampleRound(chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i], c->noise);

                // Accumulate for all visible satellites
                i_acc += ip;
                q_acc += qp;

                // Update code phase
                chan[i].code_phase += chan[i].f_code * delt;

                if (chan[i].code_phase >= CA_SEQ_LEN) {
                    chan[i].code_phase -= CA_SEQ_LEN;

                    chan[i].icode++;

                    if (chan[i].icode >= 20) { // 20 C/A codes = 1 navigation data bit
                        chan[i].icode = 0;
                        chan[i].ibit++;

                        if (chan[i].ibit >= 30) { // 30 navigation data bits = 1 word
                            chan[i].ibit = 0;
                            chan[i].iword++;
                        }

                        // Set new navigation data bit
                        chan[i].dataBit = (int) ((chan[i].dwrd[chan[i].iword]>>(29 - chan[i].ibit)) & 0x1UL)*2 - 1;
                    }
                }

                // Set current code chip
                chan[i].codeCA = chan[i].ca[(int) chan[i].code_phase]*2 - 1;

                // Update carrier phase
                chan[i].carr_phase += chan[i].f_carr * delt;

                chan[i].carr_phase -= floor(chan[i].carr_phase); // Back to [0, 1) for any step size
            }
        }

        for (int i = 0; i < SBAS_MAX_CHAN; i++) {
            sbas_chan_t *sb = &c->sbas[i];
            if (sb->prn == 0)
                continue;
            int iTable = (int) floor(sb->carr_phase * 512.0) & 511;
            i_acc += sampleRound(sb->symVal * sb->codeCA * sinTable512[(iTable + 128) & 511] * c->sbas_gain[i], c->noise);
            q_acc += sampleRound(sb->symVal * sb->codeCA * sinTable512[iTable] * c->sbas_gain[i], c->noise);

            sb->code_phase += sb->f_code * delt;
            if (sb->code_phase >= CA_SEQ_LEN) {
                sb->code_phase -= CA_SEQ_LEN;
                if (++sb->icode >= 2) { // 2 C/A codes = 1 symbol
                    sb->icode = 0;
                    sb->isym++;
                    sb->symVal = sbasSymbol(sb);
                }
            }
            sb->codeCA = sb->ca[(int) sb->code_phase] * 2 - 1;
            sb->carr_phase += sb->f_carr * delt;
            sb->carr_phase -= floor(sb->carr_phase);
        }

        if (c->noise) {
            uint32_t r = xorshift32(&c->rng);
            i_acc += c->noise_tab[r & (NOISE_TABLE - 1)];
            q_acc += c->noise_tab[(r >> 12) & (NOISE_TABLE - 1)];
        }

        c->iq_buff[isamp * 2] = (short) i_acc;
        c->iq_buff[isamp * 2 + 1] = (short) q_acc;
    }
}

/* Copy the samples into FIFO blocks for the SDR thread.
 * iq Block being filled
 * The block to fill next, NULL if the FIFO has been halted
 */
static struct iq_buf *queueSamples(const gps_ctx_t *c, struct iq_buf *iq) {
    const simulator_t *sim = c->sim;

    for (int isamp = 0; isamp < IQ_BUFFER_SIZE; isamp++) {
        // validLength starts with 0 on acquire
        if (sim->sample_size == SC16) {
            iq->data16[iq->validLength] = c->iq_buff[isamp];
        } else {
            int v = c->iq_buff[isamp] >> 4; // clip: a cast to signed char would wrap
            iq->data8[iq->validLength] = (signed char) (v > 127 ? 127 : (v < -128 ? -128 : v));
        }
        iq->validLength += 1;
        if (sim->sdr_type == SDR_HACKRF) {
            // HackRF takes fixed size blocks, a partly filled one is kept for the next round
            if (iq->validLength == HACKRF_TRANSFER_BUFFER_SIZE) {
                fifo_enqueue(iq);
                iq = fifo_acquire();
                if (iq == NULL) {
                    return NULL;
                }
            }
        }
    }

    // File writer, Pluto and LimeSDR take the entire IQ buffer at once.
    if (sim->sdr_type == SDR_IQFILE || sim->sdr_type == SDR_PLUTOSDR || sim->sdr_type == SDR_LIMESDR) {
        fifo_enqueue(iq);
        iq = fifo_acquire();
    }
    return iq;
}

static bool sameEphemeris(const ephem_t *a, const ephem_t *b) {
    return a->vflg == b->vflg && a->iode == b->iode && a->toc.week == b->toc.week && a->toc.sec == b->toc.sec;
}

/* Every 30 s: next navigation message frame, newer ephemeris, channel allocation. */
static void updateNavigation(gps_ctx_t *c, int iumd, const double *llh) {
    simulator_t *sim = c->sim;

    // Use the record closest in time for every satellite. Its subframes go into the frame that
    // starts now, because the ranges follow the new record from now on too.
    ephem_t next[MAX_SAT];
    ephSelect(&c->table, c->grx, next);
    for (int i = 0; i < MAX_CHAN; i++) {
        int sv = c->chan[i].prn - 1;
        if (sv >= 0 && next[sv].vflg && !sameEphemeris(&next[sv], &c->eph[sv]))
            eph2sbf(next[sv], c->ionoutc, c->alm, c->sim->nav_message, c->chan[i].sbf);
    }
    memcpy(c->eph, next, sizeof (next));

    for (int i = 0; i < MAX_CHAN; i++) {
        if (c->chan[i].prn > 0) {
            generateNavMsg(c->grx, &c->chan[i], 0);
        }
    }

    allocateChannel(c->chan, c->alm, c->sim->nav_message, c->eph, c->ionoutc, c->grx, c->xyz[iumd]);

    // Next 30 s of SBAS messages; channels started below fill their own buffer
    if (sim->sbas_enable)
        sbasIonoUpdate(&c->sbas_iono, &c->ionoutc, c->grx, llh);
    for (int i = 0; i < SBAS_MAX_CHAN; i++) {
        sbas_chan_t *sb = &c->sbas[i];
        if (sb->prn > 0) {
            memmove(sb->sym, sb->sym + SBAS_SYMS / 2, SBAS_SYMS / 2);
            sb->g0 = incGpsTime(sb->g0, 30.0);
            sbasFill(sb, c, incGpsTime(sb->g0, 30.0), 30, 30);
        }
    }
    allocateSbas(c, c->xyz[iumd]);

    if (sim->show_verbose) {
        gps2date(&c->grx, &sim->start);
        gui_mvwprintw(LS_FIX, LS_ROW_SIMTIME, LS_COL_SIMTIME, "%4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)",
                sim->start.y, sim->start.m, sim->start.d, sim->start.hh, sim->start.mm, sim->start.sec, c->grx.week, c->grx.sec);
        gui_mvwprintw(LS_FIX, LS_ROW_XYZ, LS_COL_INFO, "xyz = %11.1f, %11.1f, %11.1f", c->xyz[iumd][0], c->xyz[iumd][1], c->xyz[iumd][2]);
        gui_mvwprintw(LS_FIX, LS_ROW_LLH, LS_COL_INFO, "llh = %11.6f, %11.6f, %11.1f", llh[0] * R2D, llh[1] * R2D, llh[2]);
        showChannels(c);
    }
}

void *gps_thread_ep(void *arg) {
    simulator_t *simulator = (simulator_t *) (arg);
    gps_ctx_t *c = calloc(1, sizeof (*c));
    struct iq_buf *iq = NULL;

    thread_to_core(CORE_GPS);
    set_thread_name("gps-thread");

    if (c == NULL) {
        gui_status_wprintw(RED, "Failed to allocate simulation memory.\n");
        goto end_gps_thread;
    }
    c->sim = simulator;

    if (!initUserPosition(c) || !loadEphemeris(c) || !loadMotionFile(c) || !selectStartTime(c) || !loadAlmanac(c))
        goto end_gps_thread;

    initChannels(c);

    c->iq_buff = calloc(IQ_BUFFER_SIZE, sizeof (short));
    if (c->iq_buff == NULL) {
        gui_status_wprintw(RED, "Failed to allocate IQ buffer.\n");
        goto end_gps_thread;
    }

    // First fifo block for the transfer buffer
    iq = fifo_acquire();
    if (iq == NULL) { // FIFO halted
        goto end_gps_thread;
    }

    for (int iumd = 1; iumd < c->numd; iumd++) {
        if (simulator->gps_thread_exit) {
            break;
        }

        // Signal GPS init done and running
        if (simulator->gps_thread_running == false) {
            pthread_mutex_lock(&(simulator->gps_lock));
            simulator->gps_thread_running = true;
            pthread_cond_signal(&(simulator->gps_init_done));
            pthread_mutex_unlock(&(simulator->gps_lock));
        }

        if (simulator->interactive_mode) {
            moveUser(c, iumd);
        }

        updateChannels(c, iumd);
        generateSamples(c);

        iq = queueSamples(c, iq);
        if (iq == NULL) {
            goto end_gps_thread;
        }

        double llh[3];
        xyz2llh(c->xyz[iumd], llh);
        simulator->target.lat = llh[0] * R2D;
        simulator->target.lon = llh[1] * R2D;
        simulator->target.height = llh[2];
        gui_show_target(&simulator->target);

        if ((int) (c->grx.sec * 10.0 + 0.5) % 300 == 0) { // Every 30 seconds
            updateNavigation(c, iumd, llh);
        }

        c->grx = incGpsTime(c->grx, 0.1);
        gui_mvwprintw(LS_FIX, LS_ROW_ELAPSED, LS_COL_INFO, "Elapsed:         %5.1fs", subGpsTime(c->grx, c->g0));
    }

    gui_status_wprintw(GREEN, "Simulation complete\n");

end_gps_thread:
    fifo_producer_done();
    if (c != NULL) {
        free(c->iq_buff);
        free(c->xyz);
        free(c->table.rec);
        free(c);
    }
    gui_status_wprintw(RED, "Exit GPS thread\n");
    pthread_mutex_lock(&(simulator->gps_lock));
    simulator->gps_thread_exit = true;
    pthread_cond_signal(&(simulator->gps_init_done));
    pthread_mutex_unlock(&(simulator->gps_lock));
    pthread_exit(NULL);
}
