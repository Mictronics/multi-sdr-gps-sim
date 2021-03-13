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

#ifndef HELP_H
#define HELP_H

#include <argp.h>
const char *argp_program_bug_address = "https://github.com/Mictronics/multi-sdr-gps-sim";
static error_t parse_opt(int key, char *arg, struct argp_state *state);

static struct argp_option options[] = {
    {0, 0, 0, 0, "Options:", 1},
    {"nav-file", 'e', "filename", 0, "RINEX navigation file for GPS ephemeris (required)", 1},
    {"use-ftp", 'f', 0, 0, "Pull actual RINEX navigation file and almanac from online source", 1},
    {"geo-loc", 'l', "location", 0, "Latitude, Longitude, Height (static mode) e.g. 35.681298,139.766247,10.0", 1},
    {"start", 's', "date,time", 0, "Scenario start time YYYY/MM/DD,hh:mm:ss (use 'now' for actual time)", 1},
    {"disable-iono", 'I', 0, 0, "Disable ionospheric delay for spacecraft scenario", 1},
    {"verbose", 'v', 0, 0, "Show verbose output and details about simulated channels", 1},
    {"interactive", 'i', 0, 0, "Use interactive mode", 1},
    {"amplifier", 'a', 0, 0, "Enable TX amplifier (default OFF)", 1},
    {"gain", 'g', "gain", 0, "Set initial TX gain, HackRF: 0-47dB, Pluto: -80-0dB (default 0)", 1},
    {"duration", 'd', "seconds", 0, "Duration in seconds", 1},
    {"target", 't', "distance,bearing,height", 0, "Target distance [m], bearing [°] and height [m]", 1},
    {"ppb", 'p', "ppb", 0, "Set oscillator error in ppb (default 0)", 1},
    {"rinex3", '3', 0, 0, "Use RINEX v3 navigation data format", 1},
    {"radio", 'r', "name", 0, "Set the SDR device type name (default none)", 1},
    {"iq16", 700, 0, 0, "Set IQ sample size to 16 bit (default 8 bit)", 1},
    {"uri", 'U', "uri", 0, "ADLAM-Pluto URI", 1},
    {"network", 'N', "network", 0, "ADLAM-Pluto network IP or hostname (default pluto.local)", 1},
    {"motion", 'm', "name", 0, "User motion file (dynamic mode)", 1},
    {"disable-almanac", 702, 0, 0, "Disable transmission of almanac information", 1},
    {"station", 701, "id", 0, "Use station with given ID for RINEX FTP download (4 or 9 character ID)", 2},
    {0, 0, 0, OPTION_DOC, "Station is a GPS ground station around the world which provides RINEX hourly updated data. See gps.c for station details. A random station is picked if no ID is given", 2},
    {0, 0, 0, 0, "SDR device types (use with --radio or -r option):", 3},
    {0, 0, 0, OPTION_DOC, "   none", 3},
    {0, 0, 0, OPTION_DOC, "   iqfile", 3},
#ifdef ENABLE_HACKRFSDR    
    {0, 0, 0, OPTION_DOC, "   hackrf", 3},
#endif
#ifdef ENABLE_PLUTOSDR
    {0, 0, 0, OPTION_DOC, "   plutosdr", 3},
#endif    
    { 0}
};

#endif /* HELP_H */

