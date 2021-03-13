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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sched.h>
#include <unistd.h>
/* for PRIX64 */
#include <inttypes.h>
#include <iio.h>
#include <ad9361.h>
#include "fifo.h"
#include "gui.h"
#include "sdr.h"
#include "sdr_pluto.h"

#include <sys/time.h>

static atomic_bool pluto_tx_thread_exit = false;
static struct iio_context *ctx = NULL;
static struct iio_device *tx = NULL;
static struct iio_device *phydev = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer *tx_buffer = NULL;
static pthread_t pluto_tx_thread;
static const int gui_y_offset = 4;
static const int gui_x_offset = 2;

static void *pluto_tx_thread_ep(void *arg) {
    (void) arg; // Not used

    // Try sticking this thread to core 2
    thread_to_core(2);
    set_thread_name("plutosdr-thread");

    int32_t ntx = 0;
    char *ptx_buffer = (char *) iio_buffer_start(tx_buffer);

    while (!pluto_tx_thread_exit) {
        // Get a fifo block
        struct iq_buf *iq = fifo_dequeue();
        if (iq != NULL && iq->data16 != NULL) {
            // Fifo has transfer block size
            memcpy(ptx_buffer, iq->data16, iq->validLength * sizeof (signed short));
            // Schedule TX buffer, iio_buffer_push will block if there is no room to push to.
            ntx = iio_buffer_push(tx_buffer);
            if (ntx < 0) {
                gui_status_wprintw(RED, "Error pushing TX buffer: %d\n", (int) ntx);
                break;
            }
            // Release and free up used block
            fifo_release(iq);
        } else {
            break;
        }
    }

    if (ctx) {
        iio_channel_attr_write_bool(
                iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
                , "powerdown", true); // Turn OFF TX LO
    }

    if (tx_buffer) {
        iio_buffer_destroy(tx_buffer);
    }
    if (tx0_i) {
        iio_channel_disable(tx0_i);
    }
    if (tx0_q) {
        iio_channel_disable(tx0_q);
    }
    if (ctx) {
        iio_context_destroy(ctx);
    }

    pthread_exit(NULL);
}

int sdr_pluto_init(simulator_t *simulator) {
    char buf[1024];
    struct iio_scan_context *scan_ctx;
    struct iio_context_info **info;
    int ret;
    int y = gui_y_offset;
    unsigned int irates[6];
    long long lo_hz = 0;
    unsigned long xo_correction = 0;

    // ADLAM-Pluto wants 16 bit signed samples
    if (simulator->sample_size == SC08) {
        gui_status_wprintw(YELLOW, "8 bit sample size requested. Reset to 16 bit with ADLAM-Pluto.\n");
    }
    simulator->sample_size = SC16;

    scan_ctx = iio_create_scan_context(NULL, 0);
    if (!scan_ctx) {
        gui_status_wprintw(RED, "Unable to create IIO scan context.\n");
    } else {

        ret = iio_scan_context_get_info_list(scan_ctx, &info);
        if (ret < 0) {
            iio_strerror(errno, buf, sizeof (buf));
            gui_status_wprintw(RED, "Scanning for IIO contexts failed: %s\n", buf);
        }

        if (ret == 0) {
            gui_status_wprintw(RED, "No IIO context found.\n");
        } else {
            gui_mvwprintw(TRACK, y++, gui_x_offset, "IIO contexts:");
            for (int i = 0; i < ret; i++) {
                gui_mvwprintw(TRACK, y++, gui_x_offset, "%u: %s",
                        i, iio_context_info_get_description(info[i]));
                gui_mvwprintw(TRACK, y++, gui_x_offset, "   %s",
                        iio_context_info_get_uri(info[i]));
            }
        }

        iio_context_info_list_free(info);
        iio_scan_context_destroy(scan_ctx);
    }

    // Create IIO context to access ADALM-Pluto
    ctx = iio_create_default_context();
    if (ctx == NULL) {
        if (simulator->pluto_hostname != NULL) {
            ctx = iio_create_network_context(simulator->pluto_hostname);
        } else if (simulator->pluto_uri != NULL) {
            ctx = iio_create_context_from_uri(simulator->pluto_uri);
        } else {
            ctx = iio_create_network_context("pluto.local");
        }
    }

    if (ctx == NULL) {
        iio_strerror(errno, buf, sizeof (buf));
        gui_status_wprintw(RED, "Failed creating IIO context: %s\n", buf);
        return -1;
    }

    int device_count = iio_context_get_devices_count(ctx);
    if (!device_count) {
        gui_status_wprintw(RED, "No supported PLUTOSDR devices found.\n", buf);
        return -1;
    }

    tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (tx == NULL) {
        iio_strerror(errno, buf, sizeof (buf));
        gui_status_wprintw(RED, "Error opening PLUTOSDR TX device: %s\n", buf);
        return -1;
    }

    // Additional IQ kernel buffers, default is 4
    iio_device_set_kernel_buffers_count(tx, 8);

    // Limit user gain to Pluto constrains
    if (simulator->tx_gain > PLUTO_TX_GAIN_MAX) simulator->tx_gain = PLUTO_TX_GAIN_MAX;
    if (simulator->tx_gain < PLUTO_TX_GAIN_MIN) simulator->tx_gain = PLUTO_TX_GAIN_MIN;
    // Change the freq and sample rate to correct the crystal clock error.
    uint64_t freq_gps_hz = TX_FREQUENCY;
    freq_gps_hz = freq_gps_hz * (10000000 - simulator->ppb) / 10000000;

    phydev = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", true);
    iio_channel_attr_write(phy_chn, "rf_port_select", "A");
    iio_channel_attr_write_longlong(phy_chn, "rf_bandwidth", TX_BW);
    iio_channel_attr_write_longlong(phy_chn, "sampling_frequency", TX_SAMPLERATE);
    iio_channel_attr_write_double(phy_chn, "hardwaregain", simulator->tx_gain);

    iio_channel_attr_write_bool(
            iio_device_find_channel(phydev, "altvoltage0", true)
            , "powerdown", true); // Turn OFF RX LO

    iio_channel_attr_write_longlong(
            iio_device_find_channel(phydev, "altvoltage1", true)
            , "frequency", freq_gps_hz); // Set TX LO frequency

    tx0_i = iio_device_find_channel(tx, "voltage0", true);
    if (!tx0_i)
        tx0_i = iio_device_find_channel(tx, "altvoltage0", true);

    tx0_q = iio_device_find_channel(tx, "voltage1", true);
    if (!tx0_q)
        tx0_q = iio_device_find_channel(tx, "altvoltage1", true);

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    ad9361_set_bb_rate(iio_context_find_device(ctx, "ad9361-phy"), TX_SAMPLERATE);

    // Read back TX path oscillator chain settings
    ret = iio_device_attr_read(phydev, "tx_path_rates", buf, sizeof (buf));
    if (ret > 0) {
        sscanf(buf, "BBPLL:%u DAC:%u T2:%u T1:%u TF:%u TXSAMP:%u",
                &irates[0], &irates[1], &irates[2], &irates[3], &irates[4], &irates[5]);
    }

    // Read external reference oscillator calibration value
    ret = iio_device_attr_read(phydev, "xo_correction", buf, sizeof (buf));
    if (ret > 0) {
        sscanf(buf, "%lu", &xo_correction);
    }

    // Read back real TX frequency
    ret = iio_channel_attr_read_longlong(iio_device_find_channel(phydev, "altvoltage1", true), "frequency", &lo_hz);
    if (ret == 0) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Freq (%llu Hz/%.03f MHz)", lo_hz, ((double) lo_hz / (double) FREQ_ONE_MHZ));
    }

    gui_mvwprintw(TRACK, y++, gui_x_offset, "Baseband filter bandwidth (%d Hz/%.03f MHz)", TX_BW, ((float) TX_BW / (float) FREQ_ONE_MHZ));
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Sample rate (%u Hz/%.03f MHz)", irates[5], ((float) irates[5] / (float) FREQ_ONE_MHZ));
    gui_mvwprintw(TRACK, y++, gui_x_offset, "TX gain: %idB", simulator->tx_gain);

    if (simulator->show_verbose) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "XO Correction: %lu Hz", xo_correction);
        gui_mvwprintw(TRACK, y++, gui_x_offset, "TX path rates");
        gui_mvwprintw(TRACK, y++, gui_x_offset, "   BBPLL: %4.6f", irates[0] / 1e6);
        gui_mvwprintw(TRACK, y++, gui_x_offset, "   DAC: %4.6f", irates[1] / 1e6);
        gui_mvwprintw(TRACK, y++, gui_x_offset, "   T1: %4.6f", irates[3] / 1e6);
        gui_mvwprintw(TRACK, y++, gui_x_offset, "   T2: %4.6f", irates[2] / 1e6);
        gui_mvwprintw(TRACK, y++, gui_x_offset, "   TF: %4.6f", irates[4] / 1e6);
    }

    tx_buffer = iio_device_create_buffer(tx, NUM_IQ_SAMPLES, false);
    if (!tx_buffer) {
        gui_status_wprintw(RED, "Could not create TX buffer.\n");
        return -1;
    }
    iio_buffer_set_blocking_mode(tx_buffer, true);

    if (!fifo_create(NUM_FIFO_BUFFERS, IQ_BUFFER_SIZE, SC16)) {
        gui_status_wprintw(RED, "Error creating IQ file fifo!\n");
        return -1;
    }

    return 0;
}

void sdr_pluto_close(void) {
    pluto_tx_thread_exit = true;
    fifo_halt();
    fifo_destroy();
    pthread_join(pluto_tx_thread, NULL);
}

int sdr_pluto_run(void) {
    iio_channel_attr_write_bool(
            iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
            , "powerdown", false); // Turn ON TX LO

    fifo_wait_full();
    pthread_create(&pluto_tx_thread, NULL, pluto_tx_thread_ep, NULL);
    return 0;
}

int sdr_pluto_set_gain(const int gain) {
    char buf[1024];
    double g = gain;
    int ret;

    if (g > PLUTO_TX_GAIN_MAX) g = PLUTO_TX_GAIN_MAX;
    if (g < PLUTO_TX_GAIN_MIN) g = PLUTO_TX_GAIN_MIN;

    struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", true);
    iio_channel_attr_write_double(phy_chn, "hardwaregain", g);

    // Read back TX gain value
    ret = iio_channel_attr_read(phy_chn, "hardwaregain", buf, sizeof (buf));
    if (ret > 0) {
        sscanf(buf, "%lf", &g);
    }

    return (int) (g);
}