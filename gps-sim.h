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

#ifndef GPS_SIM_H
#define GPS_SIM_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdatomic.h>
#include "gps.h"

#define NOTUSED(V) ((void) V)

// Sampling data format
#define SC08 sizeof(signed char)
#define SC16 sizeof(signed short)

/* SDR device types */
typedef enum {
    SDR_NONE = 0, SDR_IQFILE, SDR_HACKRF, SDR_PLUTOSDR
} sdr_type_t;

/* Target information. */
typedef struct {
    double bearing;
    double distance;
    double lat;
    double lon;
    double height;
    double velocity;
    double speed;
    double vertical_speed;
    bool valid;
} target_t;

/* Simulator location. */
typedef struct {
    double lat; // Latitude
    double lon; // Longitude
    double height; // Height/Elevation
} location_t;

/* All the GPS simulators variables. */
typedef struct {
    atomic_bool main_exit;
    atomic_bool gps_thread_exit;
    atomic_bool gps_thread_running;
    bool show_verbose;
    bool ionosphere_enable;
    bool interactive_mode;
    bool use_ftp;
    bool enable_tx_amp;
    bool use_rinex3;
    bool time_overwrite;
    bool almanac_enable;
    int duration;
    int tx_gain;
    int ppb;
    int sample_size;
    sdr_type_t sdr_type;
    char *nav_file_name;
    char *motion_file_name;
    char *sdr_name;
    char *pluto_uri;
    char *pluto_hostname;
    char *station_id;
    pthread_mutex_t gps_lock;
    pthread_t gps_thread;
    pthread_cond_t gps_init_done; // Condition signals GPS thread is running
    location_t location; // Simulator geo location
    target_t target; // Target information
    datetime_t start; // Simulation start time
} simulator_t;

void set_thread_name(const char *name);
int thread_to_core(int core_id);

#endif /* GPS_SIM_H */

