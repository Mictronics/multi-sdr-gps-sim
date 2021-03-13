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
#include <signal.h>
#include <sched.h>
#include <fcntl.h> 
#include <unistd.h>
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include "help.h"
#include "gui.h"
#include "sdr.h"
#include "gps-sim.h"

simulator_t simulator;

static error_t parse_opt(int key, char *arg, struct argp_state *state);
const char *argp_program_version = "v1.0";
const char args_doc[] = "";
const char doc[] = "gps-sim generates a GPS L1 baseband signal IQ data stream, which is then transmitted by a software-defined radio (SDR) platform.";
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL};

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
    double d = 0.0;

    switch (key) {
        case 'e':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.nav_file_name = strdup(arg);
            break;
        case 'r':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.sdr_name = strdup(arg);
            break;
        case 'U':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.pluto_uri = strdup(arg);
            break;
        case 'N':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.pluto_hostname = strdup(arg);
            break;
        case 'm':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.motion_file_name = strdup(arg);
            simulator.interactive_mode = false;
            break;
        case 701: // --station
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.station_id = strndup(arg, 9);
            break;
        case 'f':
            simulator.use_ftp = true;
            break;
        case 'l':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            sscanf(arg, "%lf,%lf,%lf", &simulator.location.lat, &simulator.location.lon, &simulator.location.height);
            break;
        case 's':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            if (strncmp(arg, "now", 3) == 0) {
                simulator.time_overwrite = true;
                time_t timer;
                struct tm *gmt;

                time(&timer);
                gmt = gmtime(&timer);

                simulator.start.y = gmt->tm_year + 1900;
                simulator.start.m = gmt->tm_mon + 1;
                simulator.start.d = gmt->tm_mday;
                simulator.start.hh = gmt->tm_hour;
                simulator.start.mm = gmt->tm_min;
                simulator.start.sec = (double) gmt->tm_sec;
            } else {
                sscanf(arg, "%d/%d/%d,%d:%d:%lf", &simulator.start.y, &simulator.start.m, &simulator.start.d, &simulator.start.hh, &simulator.start.mm, &simulator.start.sec);
            }
            if (simulator.start.y <= 1980 ||
                    simulator.start.m < 1 || simulator.start.m > 12 ||
                    simulator.start.d < 1 || simulator.start.d > 31 ||
                    simulator.start.hh < 0 || simulator.start.hh > 23 ||
                    simulator.start.mm < 0 || simulator.start.mm > 59 ||
                    simulator.start.sec < 0.0 || simulator.start.sec >= 60.0) {
                fprintf(stderr, "ERROR: Invalid date and time.\n");
                return ARGP_ERR_UNKNOWN;
            }
            break;
        case 'I':
            simulator.ionosphere_enable = false;
            break;
        case 'v':
            simulator.show_verbose = true;
            break;
        case 'a':
            simulator.enable_tx_amp = true;
            break;
        case 'g':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.tx_gain = (int) strtoul(arg, NULL, 10);
            break;
        case 'd':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            d = atof(arg);
            if (d < 0.0 || d > ((double) USER_MOTION_SIZE) / 10.0) {
                gui_status_wprintw(RED, "Error: Invalid duration.\n");
                return -1;
            }
            simulator.duration = (int) (d * 10.0 + 0.5);
            break;
        case 't':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.target.valid = true;
            sscanf(arg, "%lf,%lf,%lf", &simulator.target.distance, &simulator.target.bearing, &simulator.target.height);
            simulator.target.bearing *= 1000;
            break;
        case 'i':
            simulator.interactive_mode = true;
            break;
        case '3':
            simulator.use_rinex3 = true;
            break;
        case 'p':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.ppb = atoi(arg);
            break;
        case 700: // --iq16
            simulator.sample_size = SC16;
            break;
        case 702: // --disable-almanac
            simulator.almanac_enable = false;
            break;
        case ARGP_KEY_END:
            if (state->arg_num > 0)
                /* We use only options but no arguments */
                argp_usage(state);
            break;
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static void simulator_init(void) {
    simulator.main_exit = false;
    simulator.show_verbose = false;
    simulator.ionosphere_enable = true;
    simulator.gps_thread_running = false;
    simulator.interactive_mode = false;
    simulator.use_ftp = false;
    simulator.enable_tx_amp = false;
    simulator.use_rinex3 = false;
    simulator.time_overwrite = false;
    simulator.almanac_enable = true;
    simulator.duration = USER_MOTION_SIZE;
    simulator.tx_gain = 0;
    simulator.ppb = 0;
    simulator.location.lat = 0;
    simulator.location.lon = 0;
    simulator.location.height = 0;
    simulator.target.bearing = 0;
    simulator.target.distance = 0;
    simulator.target.lat = 0;
    simulator.target.lon = 0;
    simulator.target.height = 0;
    simulator.target.valid = false;
    simulator.nav_file_name = NULL;
    simulator.sdr_name = NULL;
    simulator.pluto_hostname = NULL;
    simulator.motion_file_name = NULL;
    simulator.pluto_uri = NULL;
    simulator.station_id = NULL;
    simulator.sdr_type = SDR_NONE;
    simulator.sample_size = SC08;
    pthread_cond_init(&simulator.gps_init_done, NULL);
    pthread_mutex_init(&simulator.gps_lock, NULL);
}

static void signal_handler(int sig) {
    signal(sig, SIG_DFL); // Reset signal handler
    simulator.main_exit = true;
    gui_status_wprintw(RED, "Caught signal %s, shutting down\n", strsignal(sig));
}

static void cleanup_and_exit(int code) {
    simulator.gps_thread_exit = true;
    pthread_join(simulator.gps_thread, NULL); /* Wait on GPS read thread exit */

    pthread_cond_destroy(&simulator.gps_init_done);
    pthread_mutex_destroy(&simulator.gps_lock);

    /* Free when pointing to string in heap (strdup allocated when given as run option) */
    free(simulator.nav_file_name);
    free(simulator.sdr_name);
    free(simulator.pluto_hostname);
    free(simulator.pluto_uri);
    free(simulator.motion_file_name);
    free(simulator.station_id);
    sdr_close();
    gui_destroy();
    fflush(stdout);
    exit(code);
}

/* Set trhead name if supported. */
void set_thread_name(const char *name) {
#if (__GLIBC__ > 2) || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 12)
    pthread_setname_np(pthread_self(), name);
#else
    NOTUSED(name);
#endif
}

// Set affinity of calling thread to specific core on a multi-core CPU

int thread_to_core(int core_id) {
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= num_cores)
        return EINVAL;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    return pthread_setaffinity_np(current_thread, sizeof (cpu_set_t), &cpuset);
}

/*
 * 
 */
int main(int argc, char** argv) {
    int ch = 0;
    bool is_info_shown = false;
    bool is_help_shown = false;

    // signal handlers:
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGABRT, signal_handler);

    // Initialize all simulator variables
    simulator_init();
    /* On a multi-core CPU we run the main thread and reader thread on different cores.
     * Try sticking the main thread to core 1
     */
    thread_to_core(1);
    set_thread_name("simulator-thread");

    // Parse the command line options
    if (argp_parse(&argp, argc, argv, 0, 0, 0)) {
        return (EXIT_FAILURE);
    }

    if (simulator.nav_file_name == NULL && simulator.use_ftp == false) {
        fprintf(stderr, "Error: GPS ephemeris file is not specified\n");
        return (EXIT_FAILURE);
    }
    gui_init();
    // No access to GUI until this point

    if (simulator.interactive_mode && simulator.motion_file_name != NULL) {
        simulator.interactive_mode = false;
        simulator.target.valid = false;
        gui_status_wprintw(YELLOW, "User motion file supplied. Interactive mode disabled!\n");
    }

    // Wait maximum 30 seconds for GPS thread to become ready
    struct timespec timeout;
    struct timeval now;
    gettimeofday(&now, NULL);
    timeout.tv_sec = now.tv_sec + 30;
    timeout.tv_nsec = 0;

    // Init prior GPS thread, creates FIFO.
    if (sdr_init(&simulator) == 0) {
        // Start GPS baseband signal generation
        gui_top_panel(LS_FIX);
        pthread_create(&simulator.gps_thread, NULL, gps_thread_ep, &simulator);

        pthread_mutex_lock(&(simulator.gps_lock));
        int ret = pthread_cond_timedwait(&(simulator.gps_init_done), &(simulator.gps_lock), &timeout);
        pthread_mutex_unlock(&(simulator.gps_lock));

        if (ret == ETIMEDOUT) {
            gui_status_wprintw(RED, "Time out waiting for GPS thread. Running?\n");
        }

        if (!simulator.gps_thread_exit) {
            if (sdr_run() != 0) {
                gui_status_wprintw(RED, "Starting SDR streaming failed.\n");
                simulator.gps_thread_exit = true;
            }
        }
    }
    // Run this until we get a termination signal.
    while (!simulator.main_exit) {
        ch = gui_getch();
        if (ch != -1) {
            switch (ch) {
                case 'x':
                case 'X':
                    simulator.main_exit = true;
                    break;
                case 'i':
                case 'I':
                    gui_show_panel(INFO, ON);
                    is_info_shown = true;
                    break;
                case '?':
                case 'h':
                case 'H':
                    gui_show_panel(HELP, ON);
                    is_help_shown = true;
                    break;
                case 9: // TAB
                    gui_toggle_current_panel();
                    break;
                case 265: // F1
                    gui_top_panel(TRACK);
                    break;
                case 266: // F2
                    gui_top_panel(LS_FIX);
                    break;
                case 267: // F3
                    gui_top_panel(KF_FIX);
                    break;
                case LEFT_KEY:
                    simulator.target.bearing -= 127.0;
                    if (simulator.target.bearing < 0) simulator.target.bearing = 360000.0;
                    if (simulator.target.bearing > 360000) simulator.target.bearing = 0;
                    gui_show_heading((float) (simulator.target.bearing / 1000));
                    break;
                case RIGHT_KEY:
                    simulator.target.bearing += 127.0;
                    if (simulator.target.bearing < 0) simulator.target.bearing = 360000.0;
                    if (simulator.target.bearing > 360000) simulator.target.bearing = 0;
                    gui_show_heading((float) (simulator.target.bearing / 1000));
                    break;
                case UP_KEY:
                    simulator.target.vertical_speed += 1;
                    gui_show_vertical_speed((float) simulator.target.vertical_speed);
                    break;
                case DOWN_KEY:
                    simulator.target.vertical_speed -= 1;
                    gui_show_vertical_speed((float) simulator.target.vertical_speed);
                    break;
                case UPSPEED_KEY:
                    simulator.target.speed += 1.0;
                    simulator.target.velocity = simulator.target.speed / 100.0;
                    gui_show_speed((float) (simulator.target.velocity * 3.6));
                    break;
                case DOWNSPEED_KEY:
                    simulator.target.speed -= 1.0;
                    if (simulator.target.speed < 0) simulator.target.speed = 0;
                    simulator.target.velocity = simulator.target.speed / 100.0;
                    gui_show_speed((float) (simulator.target.velocity * 3.6));
                    break;
                case GAIN_INC_KEY:
                    simulator.tx_gain = sdr_set_gain(simulator.tx_gain + 1);
                    gui_status_wprintw(GREEN, "Gain: %ddB.\r", simulator.tx_gain);
                    break;
                case GAIN_DEC_KEY:
                    simulator.tx_gain = sdr_set_gain(simulator.tx_gain - 1);
                    gui_status_wprintw(GREEN, "Gain: %ddB.\r", simulator.tx_gain);
                    break;
                default:
                    if (is_info_shown) {
                        gui_show_panel(INFO, OFF);
                        is_info_shown = false;
                    }
                    if (is_help_shown) {
                        gui_show_panel(HELP, OFF);
                        is_help_shown = false;
                    }
                    break;
            }
        }
    }

    cleanup_and_exit(EXIT_SUCCESS);
    return (EXIT_SUCCESS);
}

