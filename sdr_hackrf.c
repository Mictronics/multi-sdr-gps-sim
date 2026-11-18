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
#include <unistd.h>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
/* for PRIX64 */
#include <inttypes.h>
#include <hackrf.h>
#include "gui.h"
#include "fifo.h"
#include "sdr.h"
#include "sdr_hackrf.h"

static hackrf_device_list_t *list;
static hackrf_device* device;
static const int gui_y_offset = 4;
static const int gui_x_offset = 2;

int sdr_hackrf_init(simulator_t *simulator) {
    int result = HACKRF_SUCCESS;
    double sample_rate_gps_hz;
    uint32_t baseband_filter_bw_hackrf_hz = 0;
    uint64_t freq_gps_hz;
    int y = gui_y_offset;

    // HackRF wants 8 bit signed samples
    if (simulator->sample_size == SC16) {
        gui_status_wprintw(YELLOW, "16 bit sample size requested. Reset to 8 bit with HackRF.\n");
    }
    simulator->sample_size = SC08;

    result = hackrf_init();
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_init() failed: %s (%d)\n", hackrf_error_name(result), result);
        return -1;
    }

    list = hackrf_device_list();
    if (list == NULL || list->devicecount < 1) {
        gui_status_wprintw(RED, "No HackRF boards found.\n");
        return -1;
    }

    if (list->devicecount > 1) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Found %d HackRF devices. Using index 0.", list->devicecount);
    } else {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Found HackRF device.");
    }

    if (list->serial_numbers[0]) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Serial number: %s", list->serial_numbers[0]);
    }

    device = NULL;
    result = hackrf_device_list_open(list, 0, &device);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_open() failed: %s (%d)\n", hackrf_error_name(result), result);
        return -1;
    }

    sample_rate_gps_hz = TX_SAMPLERATE;
    // Change the freq to correct the crystal clock error.
    freq_gps_hz = sdr_corrected_frequency(simulator->ppb);

    /* Compute default value depending on sample rate */
    baseband_filter_bw_hackrf_hz = hackrf_compute_baseband_filter_bw(TX_BW);

    if (baseband_filter_bw_hackrf_hz > BASEBAND_FILTER_BW_MAX) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Baseband filter BW must be less or equal to %u Hz/%.03f MHz",
                BASEBAND_FILTER_BW_MAX, (float) (BASEBAND_FILTER_BW_MAX / FREQ_ONE_MHZ));
        return -1;
    }

    if (baseband_filter_bw_hackrf_hz < BASEBAND_FILTER_BW_MIN) {
        gui_mvwprintw(TRACK, y++, 60, "Baseband filter BW must be greater or equal to %u Hz/%.03f MHz",
                BASEBAND_FILTER_BW_MIN, (float) (BASEBAND_FILTER_BW_MIN / FREQ_ONE_MHZ));
        return -1;
    }

    // Disable antenna bias tee.
    result = hackrf_set_antenna_enable(device, 0);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_antenna_enable() failed: %s (%d)\n", hackrf_error_name(result), result);
        return -1;
    }

    gui_mvwprintw(TRACK, y++, gui_x_offset, "Sample rate (%.0lf Hz/%.03f MHz)", sample_rate_gps_hz, ((float) sample_rate_gps_hz / (float) FREQ_ONE_MHZ));
    result = hackrf_set_sample_rate(device, sample_rate_gps_hz);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_sample_rate_set() failed: %s (%d)\n", hackrf_error_name(result), result);
        return -1;
    }

    gui_mvwprintw(TRACK, y++, gui_x_offset, "Baseband filter bandwidth (%d Hz/%.03f MHz)",
            baseband_filter_bw_hackrf_hz, ((float) baseband_filter_bw_hackrf_hz / (float) FREQ_ONE_MHZ));
    result = hackrf_set_baseband_filter_bandwidth(device, baseband_filter_bw_hackrf_hz);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_baseband_filter_bandwidth_set() failed: %s (%d)\n", hackrf_error_name(result), result);
        return -1;
    }

    gui_mvwprintw(TRACK, y++, gui_x_offset, "Freq (%" PRIu64 " Hz/%.03f MHz)", freq_gps_hz, ((double) freq_gps_hz / (double) FREQ_ONE_MHZ));
    result = hackrf_set_freq(device, freq_gps_hz);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_freq() failed: %s (%d)", hackrf_error_name(result), result);
        return -1;
    }

    if (simulator->enable_tx_amp) {
        gui_mvwprintw(TRACK, y++, gui_x_offset, "Amplifier enabled");
        result = hackrf_set_amp_enable(device, 1);
    } else {
        result = hackrf_set_amp_enable(device, 0);
    }

    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_amp_enable() failed: %s (%d)", hackrf_error_name(result), result);
        return -1;
    }

    if (simulator->tx_gain < TX_IF_GAIN_MIN) {
        simulator->tx_gain = TX_IF_GAIN_MIN;
    } else if (simulator->tx_gain > TX_IF_GAIN_MAX) {
        simulator->tx_gain = TX_IF_GAIN_MAX;
    }

    gui_mvwprintw(TRACK, y++, gui_x_offset, "TX IF gain: %idB", simulator->tx_gain);
    result = hackrf_set_txvga_gain(device, simulator->tx_gain);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_txvga_gain() failed: %s (%d)", hackrf_error_name(result), result);
        return -1;
    }

    result = hackrf_set_hw_sync_mode(device, 0);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_hw_sync_mode() failed: %s (%d)", hackrf_error_name(result), result);
        return -1;
    }

    if (!fifo_create(NUM_FIFO_BUFFERS, HACKRF_TRANSFER_BUFFER_SIZE, sizeof (signed char))) {
        gui_status_wprintw(RED, "Error creating TX fifo!");
        return -1;
    }

    return 0;
}

void sdr_hackrf_close(void) {
    fifo_halt(); // Unblocks the TX callback so hackrf_stop_tx can return
    if (device != NULL) {
        hackrf_stop_tx(device);
        hackrf_set_amp_enable(device, 0);
        hackrf_set_txvga_gain(device, 0);
        hackrf_close(device);
    }
    fifo_destroy();
    if (list != NULL) {
        hackrf_device_list_free(list);
    }
    hackrf_exit();
}

static int sdr_tx_callback(hackrf_transfer *transfer) {
    // Get a fifo block
    struct iq_buf *iq = fifo_dequeue();
    if (iq != NULL && iq->data8 != NULL) {
        // Fifo has transfer block size
        memcpy(transfer->buffer, iq->data8, transfer->valid_length);
        // Release and free up used block
        fifo_release(iq);
        return 0;
    }

    return -1;
}

int sdr_hackrf_run(void) {
    if (device == NULL) {
        gui_status_wprintw(RED, "HackRF device is NULL\n");
        return -1;
    }

    fifo_wait_full();

    int result = hackrf_start_tx(device, sdr_tx_callback, NULL);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_start_tx() failed: %s (%d)", hackrf_error_name(result), result);
        return -1;
    }

    return 0;
}

int sdr_hackrf_set_gain(const int gain) {
    int g = gain;
    if (g < TX_IF_GAIN_MIN) {
        g = TX_IF_GAIN_MIN;
    } else if (g > TX_IF_GAIN_MAX) {
        g = TX_IF_GAIN_MAX;
    }

    int result = hackrf_set_txvga_gain(device, g);
    if (result != HACKRF_SUCCESS) {
        gui_status_wprintw(RED, "hackrf_set_txvga_gain() failed: %s (%d)", hackrf_error_name(result), result);
    }

    return g;
}