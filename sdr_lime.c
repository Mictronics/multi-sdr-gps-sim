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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <inttypes.h>
#include <lime/LimeSuite.h>
#include "fifo.h"
#include "gui.h"
#include "sdr.h"
#include "sdr_lime.h"

static atomic_bool lime_tx_thread_exit = false;
static lms_device_t *device = NULL;
static lms_stream_t tx_stream;
static bool stream_setup = false;
static pthread_t lime_tx_thread;
static bool thread_started = false;
static const int gui_y_offset = 4;
static const int gui_x_offset = 2;

#define LIME_CHECK(call) do { \
        if ((call) != 0) { \
            gui_status_wprintw(RED, #call " failed: %s\n", LMS_GetLastErrorMessage()); \
            return -1; \
        } \
    } while (0)

static int clamp_gain(int gain) {
    return gain < LIME_TX_GAIN_MIN ? LIME_TX_GAIN_MIN : (gain > LIME_TX_GAIN_MAX ? LIME_TX_GAIN_MAX : gain);
}

static void *lime_tx_thread_ep(void *arg) {
    (void) arg;

    thread_to_core(CORE_SDR);
    set_thread_name("limesdr-thread");

    while (!lime_tx_thread_exit) {
        struct iq_buf *iq = fifo_dequeue();
        if (iq == NULL || iq->data16 == NULL) {
            break;
        }
        // I12 goes to the FPGA as 12 bit words, so a larger value would wrap instead of clip
        for (unsigned k = 0; k < iq->validLength; k++) {
            int v = iq->data16[k];
            iq->data16[k] = (int16_t) (v > 2047 ? 2047 : (v < -2048 ? -2048 : v));
        }
        size_t total = iq->validLength / 2; // IQ pairs
        size_t sent = 0;
        while (sent < total && !lime_tx_thread_exit) {
            int n = LMS_SendStream(&tx_stream, iq->data16 + 2 * sent, total - sent, NULL, 1000);
            if (n < 0) {
                gui_status_wprintw(RED, "LMS_SendStream failed: %s\n", LMS_GetLastErrorMessage());
                lime_tx_thread_exit = true;
                break;
            }
            sent += (size_t) n;
        }
        fifo_release(iq);
    }

    pthread_exit(NULL);
}

// LimeSuite writes to stderr by default, which garbles the TUI
static void lime_log(int lvl, const char *msg) {
    if (lvl <= LMS_LOG_WARNING) {
        gui_status_wprintw(lvl == LMS_LOG_WARNING ? YELLOW : RED, "LimeSDR: %s\n", msg);
    }
}

// The TX port whose range covers the frequency, -1 if none does.
static int find_tx_antenna(uint64_t freq_hz, lms_name_t *name) {
    lms_name_t list[16];
    int n = LMS_GetAntennaList(device, LMS_CH_TX, 0, list);
    // Index 0 is "None"; LimeSuiteNG's wrapper throws a C++ exception for its range
    for (int i = 1; i < n && i < 16; i++) {
        lms_range_t r;
        if (LMS_GetAntennaBW(device, LMS_CH_TX, 0, i, &r) == 0 && r.min <= freq_hz && freq_hz <= r.max) {
            snprintf(*name, sizeof (*name), "%s", list[i]);
            return i;
        }
    }
    return -1;
}

int sdr_lime_init(simulator_t *simulator) {
    lms_info_str_t list[8];
    int y = gui_y_offset;

    simulator->sample_size = SC16; // LMS_FMT_I12: 12 bit samples in 16 bit words
    LMS_RegisterLogHandler(lime_log);

    int count = LMS_GetDeviceList(list);
    if (count < 1) {
        gui_status_wprintw(RED, "No LimeSDR boards found.\n");
        return -1;
    }
    gui_mvwprintw(TRACK, y++, gui_x_offset, count > 1 ? "Found %d LimeSDR devices. Using index 0." : "Found %d LimeSDR device.", count);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "%.100s", list[0]);

    LIME_CHECK(LMS_Open(&device, list[0], NULL));
    LIME_CHECK(LMS_Init(device));
    LIME_CHECK(LMS_EnableChannel(device, LMS_CH_TX, 0, true));
    // Oversampling 4 puts CGEN at exactly 48 MHz; the default (32) misses 3 MS/s by -333 ppb
    LIME_CHECK(LMS_SetSampleRate(device, TX_SAMPLERATE, LIME_OVERSAMPLE));

    uint64_t freq_gps_hz = sdr_corrected_frequency(simulator->ppb);
    LIME_CHECK(LMS_SetLOFrequency(device, LMS_CH_TX, 0, (double) freq_gps_hz));

    lms_name_t antenna;
    int ant = find_tx_antenna(freq_gps_hz, &antenna);
    if (ant < 0) {
        gui_status_wprintw(RED, "No LimeSDR TX port covers %" PRIu64 " Hz.\n", freq_gps_hz);
        return -1;
    }
    LIME_CHECK(LMS_SetAntenna(device, LMS_CH_TX, 0, ant));

    LIME_CHECK(LMS_SetLPFBW(device, LMS_CH_TX, 0, TX_BW));

    // At low gains the calibration test signal is too weak and leaves LO leakage at L1
    LIME_CHECK(LMS_SetGaindB(device, LMS_CH_TX, 0, LIME_CAL_GAIN));
    LIME_CHECK(LMS_Calibrate(device, LMS_CH_TX, 0, TX_BW, 0));

    simulator->tx_gain = clamp_gain(simulator->tx_gain);
    LIME_CHECK(LMS_SetGaindB(device, LMS_CH_TX, 0, (unsigned) simulator->tx_gain));

    double host_hz = 0, rf_hz = 0, lo_hz = 0;
    LMS_GetSampleRate(device, LMS_CH_TX, 0, &host_hz, &rf_hz);
    LMS_GetLOFrequency(device, LMS_CH_TX, 0, &lo_hz);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Freq (%.0f Hz/%.03f MHz)", lo_hz, lo_hz / FREQ_ONE_MHZ);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Sample rate (%.0f Hz, RF %.0f Hz)", host_hz, rf_hz);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "TX port: %s", antenna);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "TX gain: %idB", simulator->tx_gain);

    tx_stream.channel = 0;
    tx_stream.isTx = true;
    tx_stream.fifoSize = NUM_IQ_SAMPLES * 4; // 0.4 s
    tx_stream.throughputVsLatency = 0.5;
    tx_stream.dataFmt = LMS_FMT_I12;
    LIME_CHECK(LMS_SetupStream(device, &tx_stream));
    stream_setup = true;

    if (!fifo_create(NUM_FIFO_BUFFERS, IQ_BUFFER_SIZE, SC16)) {
        gui_status_wprintw(RED, "Error creating TX fifo!\n");
        return -1;
    }

    return 0;
}

void sdr_lime_close(void) {
    lime_tx_thread_exit = true;
    fifo_halt();
    if (thread_started) {
        pthread_join(lime_tx_thread, NULL);
    }
    if (stream_setup) {
        LMS_StopStream(&tx_stream);
        LMS_DestroyStream(device, &tx_stream);
    }
    if (device != NULL) {
        LMS_SetGaindB(device, LMS_CH_TX, 0, 0);
        LMS_EnableChannel(device, LMS_CH_TX, 0, false);
        LMS_Close(device);
        device = NULL;
    }
    fifo_destroy();
}

int sdr_lime_run(void) {
    if (device == NULL) {
        gui_status_wprintw(RED, "LimeSDR device is NULL\n");
        return -1;
    }

    fifo_wait_full();
    LIME_CHECK(LMS_StartStream(&tx_stream));
    thread_started = pthread_create(&lime_tx_thread, NULL, lime_tx_thread_ep, NULL) == 0;
    return thread_started ? 0 : -1;
}

// No LMS_GetGaindB readback: LimeSuiteNG's wrapper adds 12 dB there but not in LMS_SetGaindB
int sdr_lime_set_gain(const int gain) {
    int g = clamp_gain(gain);

    if (LMS_SetGaindB(device, LMS_CH_TX, 0, (unsigned) g) != 0) {
        gui_status_wprintw(RED, "LMS_SetGaindB failed: %s\n", LMS_GetLastErrorMessage());
    }
    return g;
}
