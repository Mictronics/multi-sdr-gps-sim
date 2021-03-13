/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#include "sdr_hackrf.h"
#include "sdr_iqfile.h"
#include "sdr_pluto.h"
#include "sdr.h"

static int no_init(void);
static void no_close(void);
static int no_run(void);
static int no_set_gain(const int);

typedef struct {
    int (*init)();
    void (*close)();
    int (*run)();
    int (*set_gain)(const int);
    const char *name;
    sdr_type_t sdr_type;
} sdr_handler;

static sdr_type_t current_type = SDR_NONE;

static sdr_handler sdr_handlers[] = {
    { no_init, no_close, no_run, no_set_gain, "none", SDR_NONE},
    { sdr_iqfile_init, sdr_iqfile_close, sdr_iqfile_run, no_set_gain, "iqfile", SDR_IQFILE},
#ifdef ENABLE_HACKRFSDR
    { sdr_hackrf_init, sdr_hackrf_close, sdr_hackrf_run, sdr_hackrf_set_gain, "hackrf", SDR_HACKRF},
#endif

#ifdef ENABLE_PLUTOSDR
    { sdr_pluto_init, sdr_pluto_close, sdr_pluto_run, sdr_pluto_set_gain, "plutosdr", SDR_PLUTOSDR},
#endif
    { NULL, NULL, NULL, NULL, NULL, SDR_NONE} /* must come last */
};

static int no_init() {
    fprintf(stderr, "SDR type not recognized; supported SDR types are:\n");
    for (int i = 0; sdr_handlers[i].name; ++i) {
        fprintf(stderr, "  %s\n", sdr_handlers[i].name);
    }
    return -1;
}

static void no_close() {
}

static int no_run() {
    return -1;
}

static int no_set_gain(const int gain) {
    NOTUSED(gain);
    return -100;
}

static sdr_handler *current_handler(void) {
    for (int i = 0; sdr_handlers[i].name; ++i) {
        if (current_type == sdr_handlers[i].sdr_type) {
            return &sdr_handlers[i];
        }
    }

    return &sdr_handlers[0];
}

int sdr_init(simulator_t *simulator) {
    for (int i = 0; sdr_handlers[i].name; ++i) {
        if (!strcasecmp(sdr_handlers[i].name, simulator->sdr_name)) {
            current_type = sdr_handlers[i].sdr_type;
            simulator->sdr_type = sdr_handlers[i].sdr_type;
        }
    }

    return current_handler()->init(simulator);
}

void sdr_close(void) {
    current_handler()->close();
}

int sdr_run(void) {
    return current_handler()->run();
}

int sdr_set_gain(const int gain) {
    return current_handler()->set_gain(gain);
}