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
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <curl/curl.h>
#include <ncurses.h>
#include <sched.h>
#include <fcntl.h> 
#include <unistd.h>
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include "help.h"
#include "gui.h"
#include "sdr.h"
#include "fifo.h"
#include "gps-sim.h"

simulator_t simulator;
static bool gps_thread_started = false;

static error_t parse_opt(int key, char *arg, struct argp_state *state);
const char *argp_program_version = "v1.0";
const char args_doc[] = "";
const char doc[] = "gps-sim generates a GPS L1 baseband signal IQ data stream, which is then transmitted by a software-defined radio (SDR) platform.";
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL};

/* Store a copy of an option argument; a repeated option replaces the earlier one. */
static void set_string(char **dst, const char *src) {
    char *copy = strdup(src);
    if (copy == NULL) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    free(*dst);
    *dst = copy;
}

/* Parse a whole-string integer option or exit with a usage error. */
static int parse_int(const char *arg, struct argp_state *state, const char *what) {
    char *end;
    errno = 0;
    long v = strtol(arg, &end, 10);
    if (end == arg || *end != '\0' || errno != 0 || v < INT_MIN || v > INT_MAX) {
        argp_error(state, "Invalid %s '%s'.", what, arg);
    }
    return (int) v;
}

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
    double d = 0.0;

    switch (key) {
        case 'e':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            set_string(&simulator.nav_file_name, arg);
            break;
        case 'r':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            set_string(&simulator.sdr_name, arg);
            break;
        case 'U':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            set_string(&simulator.pluto_uri, arg);
            break;
        case 'N':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            set_string(&simulator.pluto_hostname, arg);
            break;
        case 'm':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            set_string(&simulator.motion_file_name, arg);
            simulator.interactive_mode = false;
            break;
        case 'f':
            simulator.use_ftp = true;
            break;
        case 'l':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.location.height = 0.0; // Height is optional
            if (sscanf(arg, "%lf,%lf,%lf", &simulator.location.lat, &simulator.location.lon, &simulator.location.height) < 2 ||
                    fabs(simulator.location.lat) > 90.0 || fabs(simulator.location.lon) > 180.0) {
                argp_error(state, "Invalid location '%s', expected latitude,longitude[,height].", arg);
            }
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
                if (sscanf(arg, "%d/%d/%d,%d:%d:%lf", &simulator.start.y, &simulator.start.m, &simulator.start.d, &simulator.start.hh, &simulator.start.mm, &simulator.start.sec) != 6) {
                    argp_error(state, "Invalid start time '%s', expected YYYY/MM/DD,hh:mm:ss.", arg);
                }
            }
            if (simulator.start.y <= 1980 ||
                    simulator.start.m < 1 || simulator.start.m > 12 ||
                    simulator.start.d < 1 || simulator.start.d > 31 ||
                    simulator.start.hh < 0 || simulator.start.hh > 23 ||
                    simulator.start.mm < 0 || simulator.start.mm > 59 ||
                    simulator.start.sec < 0.0 || simulator.start.sec >= 60.0) {
                argp_error(state, "Invalid date and time '%s'.", arg);
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
            simulator.tx_gain = parse_int(arg, state, "gain");
            break;
        case 'd':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            d = atof(arg);
            if (d < 0.0 || d > ((double) USER_MOTION_SIZE) / 10.0) {
                argp_error(state, "Invalid duration, maximum is %.1f seconds.", ((double) USER_MOTION_SIZE) / 10.0);
            }
            simulator.duration = (int) (d * 10.0 + 0.5);
            break;
        case 't':
            if (arg == NULL) {
                return ARGP_ERR_UNKNOWN;
            }
            simulator.target.valid = true;
            simulator.target.height = 0.0; // Height is optional
            if (sscanf(arg, "%lf,%lf,%lf", &simulator.target.distance, &simulator.target.bearing, &simulator.target.height) < 2) {
                argp_error(state, "Invalid target '%s', expected distance,bearing[,height].", arg);
            }
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
            simulator.ppb = parse_int(arg, state, "ppb");
            if (abs(simulator.ppb) > 1000000) { // 1000 ppm, far beyond any crystal error
                argp_error(state, "Oscillator error %d ppb is out of range (+-1000000).", simulator.ppb);
            }
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
    simulator.sdr_type = SDR_NONE;
    simulator.sample_size = SC08;
    pthread_cond_init(&simulator.gps_init_done, NULL);
    pthread_mutex_init(&simulator.gps_lock, NULL);
}

// Only async-signal-safe work here: the GUI takes a mutex and calls ncurses
static void signal_handler(int sig) {
    NOTUSED(sig);
    simulator.main_exit = true;
}

/* Turn the target heading; the bearing is kept in thousandths of a degree. */
static void change_bearing(double delta) {
    double bearing = fmod(simulator.target.bearing + delta, 360000.0);
    if (bearing < 0.0)
        bearing += 360000.0;
    simulator.target.bearing = bearing;
    gui_show_heading((float) (bearing / 1000));
}

static void change_gain(int delta) {
    int gain = sdr_set_gain(simulator.tx_gain + delta);
    if (gain == SDR_GAIN_UNSUPPORTED) {
        gui_status_wprintw(YELLOW, "This SDR has no gain control.\n");
        return;
    }
    simulator.tx_gain = gain;
    gui_status_wprintw(GREEN, "Gain: %ddB.\r", gain);
}

static void cleanup_and_exit(int code) {
    simulator.gps_thread_exit = true;
    fifo_halt(); // Unblock the GPS thread if it waits for a free buffer
    if (gps_thread_started) {
        pthread_join(simulator.gps_thread, NULL); /* Wait on GPS read thread exit */
    }

    pthread_cond_destroy(&simulator.gps_init_done);
    pthread_mutex_destroy(&simulator.gps_lock);

    /* Free when pointing to string in heap (strdup allocated when given as run option) */
    if (simulator.use_ftp) {
        curl_global_cleanup();
    }
    free(simulator.nav_file_name);
    free(simulator.sdr_name);
    free(simulator.pluto_hostname);
    free(simulator.pluto_uri);
    free(simulator.motion_file_name);
    sdr_close();
    gui_destroy();
    fflush(stdout);
    exit(code);
}

/* Set thread name if supported. */
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

    // signal handlers, reset after the first signal so a second one terminates
    struct sigaction sa = {.sa_handler = signal_handler, .sa_flags = SA_RESETHAND};
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);

    // Initialize all simulator variables
    simulator_init();
    thread_to_core(CORE_MAIN);
    set_thread_name("simulator-thread");

    // Parse the command line options
    if (argp_parse(&argp, argc, argv, 0, 0, 0)) {
        return (EXIT_FAILURE);
    }

    if (simulator.nav_file_name == NULL && simulator.use_ftp == false) {
        fprintf(stderr, "Error: GPS ephemeris file is not specified\n");
        return (EXIT_FAILURE);
    }
    if (simulator.use_ftp) {
        curl_global_init(CURL_GLOBAL_DEFAULT); // Once, before any thread uses curl
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
        if (pthread_create(&simulator.gps_thread, NULL, gps_thread_ep, &simulator) == 0) {
            gps_thread_started = true;
        }

        // Wait until the GPS thread is running or has exited
        int ret = 0;
        pthread_mutex_lock(&(simulator.gps_lock));
        while (gps_thread_started && ret != ETIMEDOUT && !simulator.gps_thread_running && !simulator.gps_thread_exit) {
            ret = pthread_cond_timedwait(&(simulator.gps_init_done), &(simulator.gps_lock), &timeout);
        }
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
                case '\t':
                    gui_toggle_current_panel();
                    break;
                case KEY_F(1):
                    gui_top_panel(TRACK);
                    break;
                case KEY_F(2):
                    gui_top_panel(LS_FIX);
                    break;
                case KEY_F(3):
                    gui_top_panel(KF_FIX);
                    break;
                case LEFT_KEY:
                    change_bearing(-127.0);
                    break;
                case RIGHT_KEY:
                    change_bearing(127.0);
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
                    change_gain(1);
                    break;
                case GAIN_DEC_KEY:
                    change_gain(-1);
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

