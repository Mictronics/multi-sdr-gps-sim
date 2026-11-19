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

#include "gui.h"
#include "sdr_hackrf.h"
#include "sdr_iqfile.h"
#include "sdr_pluto.h"
#include "sdr_lime.h"
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

static sdr_handler sdr_handlers[] = {
    { no_init, no_close, no_run, no_set_gain, "none", SDR_NONE},
    { sdr_iqfile_init, sdr_iqfile_close, sdr_iqfile_run, no_set_gain, "iqfile", SDR_IQFILE},
#ifdef ENABLE_HACKRFSDR
    { sdr_hackrf_init, sdr_hackrf_close, sdr_hackrf_run, sdr_hackrf_set_gain, "hackrf", SDR_HACKRF},
#endif

#ifdef ENABLE_PLUTOSDR
    { sdr_pluto_init, sdr_pluto_close, sdr_pluto_run, sdr_pluto_set_gain, "plutosdr", SDR_PLUTOSDR},
#endif

#ifdef ENABLE_LIMESDR
    { sdr_lime_init, sdr_lime_close, sdr_lime_run, sdr_lime_set_gain, "limesdr", SDR_LIMESDR},
#endif
    { NULL, NULL, NULL, NULL, NULL, SDR_NONE} /* must come last */
};

static int no_init() {
    gui_status_wprintw(RED, "SDR type not recognized; supported SDR types are:\n");
    for (int i = 0; sdr_handlers[i].name; ++i) {
        gui_status_wprintw(RED, "  %s\n", sdr_handlers[i].name);
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
    return SDR_GAIN_UNSUPPORTED;
}

// The "none" handler until sdr_init selects one by name.
static sdr_handler *current = &sdr_handlers[0];

int sdr_init(simulator_t *simulator) {
    const char *name = simulator->sdr_name ? simulator->sdr_name : "none";
    for (int i = 0; sdr_handlers[i].name; ++i) {
        if (!strcasecmp(sdr_handlers[i].name, name)) {
            current = &sdr_handlers[i];
            simulator->sdr_type = current->sdr_type;
            break;
        }
    }

    return current->init(simulator);
}

void sdr_close(void) {
    current->close();
}

int sdr_run(void) {
    return current->run();
}

int sdr_set_gain(const int gain) {
    return current->set_gain(gain);
}