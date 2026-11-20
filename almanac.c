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
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "gps-sim.h"
#include "gui.h"
#include "almanac.h"

static almanac_gps_t almanac_gps;

/* Almanac sources, tried in this order. A url with a date (strftime format) is a daily archive,
 * tried for today and the given number of days before. All serve the same SEM format. */
static const struct {
    const char *url;
    int days_back;
} almanac_sources[] = {
    {"https://www.navcen.uscg.gov/sites/default/files/gps/almanac/current_sem.al3", 0},
    {"https://celestrak.org/GPS/almanac/SEM/almanac.sem.txt", 0},
    {"https://www.navcen.uscg.gov/sites/default/files/gps/almanac/%Y/Sem/%j.al3", 3},
};
#define ALMANAC_TIMEOUT 20L // Per attempt [s]
static const int almanac_retry_delay[] = {1, 3}; // Before each retry [s]
#define ALMANAC_TMP_FILE ALMANAC_FILE ".tmp"

static CURLcode almanac_parse(const char *path);

/**
 * Initialize empty almanac.
 */
almanac_gps_t* almanac_init(void) {
    memset(&almanac_gps, 0, sizeof (almanac_gps));
    return &almanac_gps;
}

/**
 * Read almanac from local file.
 * sem format expected.
 */
CURLcode almanac_read_file(void) {
    return almanac_parse(ALMANAC_FILE);
}

static CURLcode almanac_parse(const char *path) {
    char buf[100];
    char title[25]; // 24 chars for %24s plus NUL
    char *pbuf;
    unsigned int n, week, sec, id;
    unsigned short ignored;
    FILE *fp = fopen(path, "rt");
    almanac_prn_t* a = NULL;

    // Start with empty almanac
    almanac_init();

    if (!fp) {
        return CURLE_READ_ERROR;
    }

    pbuf = fgets(buf, sizeof (buf), fp);
    if (pbuf == NULL || sscanf(buf, "%u %24s", &n, title) != 2) goto error;

    pbuf = fgets(buf, sizeof (buf), fp);
    if (pbuf == NULL || sscanf(buf, "%u %u", &week, &sec) != 2) goto error;

    n -= 1; // PRN in file counts 1-32, array counts 0-31
    if (n > 31) n = 31; // Max 32 PRN's to read (0-31)

    for (unsigned int j = 0; j <= n; j++) {
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        // Check and skip blank line
        if (buf[0] == '\n' || buf[0] == '\r') {
            pbuf = fgets(buf, sizeof (buf), fp);
            if (pbuf == NULL) goto error;
        }

        if (sscanf(buf, "%u", &id) != 1) goto error;
        if (id == 0) id = 1;
        if (id > 32) id = 32;

        a = &almanac_gps.sv[id - 1];

        a->svid = (unsigned short) id;

        // SVN is optional (could be a blank line) and not used
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (buf[0] != '\n' && buf[0] != '\r' && sscanf(buf, "%hu", &ignored) != 1) goto error;

        // URA, not used
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hu", &ignored) != 1) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->e, &a->delta_i, &a->omegadot) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->sqrta, &a->omega0, &a->aop) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%lf %lf %lf", &a->m0, &a->af0, &a->af1) != 3) goto error;

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->health) != 1) goto error;
        if (a->health > 63) a->health = 63;

        // Configuration code, not used
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hu", &ignored) != 1) goto error;

        /**
         * ublox u-center software provides the full GPS week number whereas Celestrak follows
         * IC-GPS-200L, p. 116, 20.3.3.5.1.5 Almanac Reference Week:
         * The WNa term consists of eight bits which shall be a modulo 256 binary
         * representation of the GPS week number...
         * Celestrak almanac files use a modulo 256 week number in file name.
         * See https://celestrak.com/GPS/almanac/SEM/2021/
         *
         * Current Celestrak files hold the week modulo 1024. Reducing it modulo 256 loses nothing
         * for that, for 8 bit or for full week numbers; gps.c resolves the full week against the
         * scenario time.
         */
        a->toa.week = (int) (week % 256); // Full week is resolved against the scenario time
        a->toa.sec = (double) sec;
        a->valid = 1;
        almanac_gps.valid = 1; // We have at least one valid record
    }
    fclose(fp);
    return CURLE_OK;

error:
    if (!feof(fp)) {
        // Not end of file, something wrong
        // Drop all parsed almanac data on error
        almanac_init();
    }
    /**
     * If this is the end of file we may read less records than what 
     * field "number of records" announced.
     * Found this happening when saving almanac.sem in ublox u-center software.
     */
    fclose(fp);
    return CURLE_READ_ERROR;
}

// Wait seconds, false if cancelled meanwhile
static bool almanac_wait(int seconds, const atomic_bool *cancel) {
    const struct timespec step = {0, 100000000L};
    for (int i = 0; i < seconds * 10 && !*cancel; i++)
        nanosleep(&step, NULL);
    return !*cancel;
}

static bool is_connect_error(CURLcode res) {
    return res == CURLE_OPERATION_TIMEDOUT || res == CURLE_COULDNT_CONNECT || res == CURLE_COULDNT_RESOLVE_HOST;
}

// Download one url, retrying connection errors and timeouts (a server can stall a request)
static CURLcode almanac_fetch(const char *url, const atomic_bool *cancel) {
    const int retries = sizeof (almanac_retry_delay) / sizeof (almanac_retry_delay[0]);
    CURLcode res = CURLE_FAILED_INIT;
    for (int attempt = 0; attempt <= retries; attempt++) {
        if (attempt > 0 && !almanac_wait(almanac_retry_delay[attempt - 1], cancel))
            return CURLE_ABORTED_BY_CALLBACK;
        res = download_file(url, ALMANAC_TMP_FILE, ALMANAC_TIMEOUT, cancel);
        if (!is_connect_error(res))
            break;
    }
    return res;
}

/* Check a downloaded file before it replaces the cached one; NULL if it is a usable SEM almanac,
 * otherwise the reason. It is parsed into the almanac, so a valid one is loaded right away. */
static const char *almanac_check(const char *path) {
    static char reason[64];
    char head[256] = {0};
    FILE *fp = fopen(path, "rb");
    if (!fp)
        return "no file";
    size_t n = fread(head, 1, sizeof (head) - 1, fp);
    fclose(fp);
    for (size_t i = 0; i < n; i++)
        head[i] = (char) tolower((unsigned char) head[i]);
    if (strstr(head, "<html") || strstr(head, "<!doctype"))
        return "HTML page instead of a SEM almanac"; // Error pages often come with HTTP 200
    if (almanac_parse(path) != CURLE_OK)
        return "not a SEM almanac";
    int nsat = 0;
    for (int sv = 0; sv < MAX_SAT; sv++)
        nsat += almanac_gps.sv[sv].valid != 0;
    if (nsat < ALMANAC_MIN_SATS) {
        snprintf(reason, sizeof (reason), "only %d satellites", nsat);
        return reason;
    }
    return NULL;
}

/**
 * Download the almanac, trying each source in almanac_sources until one gives a valid SEM file,
 * which then replaces almanac.sem. If all fail, the almanac.sem of an earlier download is used.
 */
CURLcode almanac_download(const atomic_bool *cancel) {
    CURLcode res = CURLE_FAILED_INIT;
    time_t now = time(NULL);

    for (size_t s = 0; s < sizeof (almanac_sources) / sizeof (almanac_sources[0]); s++) {
        for (int d = 0; d <= almanac_sources[s].days_back; d++) {
            char url[256];
            time_t day = now - (time_t) d * 86400;
            struct tm tm;
            gmtime_r(&day, &tm);
            strftime(url, sizeof (url), almanac_sources[s].url, &tm);

            res = almanac_fetch(url, cancel);
            if (res == CURLE_ABORTED_BY_CALLBACK)
                return res;
            const char *reason = res == CURLE_OK ? almanac_check(ALMANAC_TMP_FILE) : curl_easy_strerror(res);
            if (reason == NULL && rename(ALMANAC_TMP_FILE, ALMANAC_FILE) == 0) {
                gui_status_wprintw(GREEN, "Almanac from %s\n", url);
                return CURLE_OK;
            }
            remove(ALMANAC_TMP_FILE);
            gui_status_wprintw(RED, "Almanac %s: %s\n", url, reason ? reason : "cannot replace " ALMANAC_FILE);
            if (is_connect_error(res))
                break; // Host down: the archive's older days would fail the same way
            if (res == CURLE_OK)
                res = CURLE_READ_ERROR;
        }
    }

    if (almanac_read_file() == CURLE_OK) {
        gui_status_wprintw(RED, "All almanac sources failed, using the cached %s\n", ALMANAC_FILE);
        return CURLE_OK;
    }
    return res;
}
