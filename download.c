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
#include <time.h>
#include "gps-sim.h"
#include "gui.h"
#include "download.h"

struct download_target {
    const char *filename;
    FILE *stream;
};

static size_t write_download(void *buffer, size_t size, size_t nmemb, void *userdata) {
    struct download_target *out = (struct download_target *) userdata;
    if (!out->stream && !(out->stream = fopen(out->filename, "wb"))) {
        return 0; // Short write makes curl abort the transfer
    }
    return fwrite(buffer, size, nmemb, out->stream);
}

static int download_progress(void *userdata, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow) {
    NOTUSED(dltotal);
    NOTUSED(dlnow);
    NOTUSED(ultotal);
    NOTUSED(ulnow);
    return *(const atomic_bool *) userdata ? 1 : 0; // Non-zero aborts the transfer
}

/* Download url into filename within timeout seconds. Anything but HTTP 200 fails, and a failed
 * transfer leaves no partial file. The transfer is aborted when *cancel becomes true. */
CURLcode download_file(const char *url, const char *filename, long timeout, const atomic_bool *cancel) {
    struct download_target out = {filename, NULL};
    CURLcode res = CURLE_FAILED_INIT;
    CURL *curl = curl_easy_init();

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, DOWNLOAD_USER_AGENT);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1000L); // Give up on stalled transfers
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, 15L);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, download_progress);
        curl_easy_setopt(curl, CURLOPT_XFERINFODATA, cancel);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_download);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
        res = curl_easy_perform(curl);
        long code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
        if (res == CURLE_OK && code != 200) {
            res = CURLE_HTTP_RETURNED_ERROR; // e.g. 204: FAILONERROR only rejects 400 and up
        }
        curl_easy_cleanup(curl);
    }

    if (out.stream) {
        fclose(out.stream);
        if (res != CURLE_OK) {
            remove(filename);
        }
    }
    return res;
}

static bool is_connect_error(CURLcode res) {
    return res == CURLE_COULDNT_CONNECT || res == CURLE_COULDNT_RESOLVE_HOST || res == CURLE_OPERATION_TIMEDOUT;
}

/* Download the newest available broadcast ephemeris file, trying each source in turn.
 * RINEX 2: NOAA CORS daily GPS file, refreshed about hourly, but its records can lag hours behind.
 * RINEX 3: BKG merged files; today's BRDC00WRD_S is built from real-time streams and reaches the
 * current hour, BRDC00IGS_R only appears the day after. Today's file is tried first for both.
 */
CURLcode download_nav_file(bool rinex3, const char *filename, const atomic_bool *cancel) {
    static const char *const rinex3_names[] = {"BRDC00WRD_S", "BRDC00WRD_R", "BRDC00IGS_R"};
    CURLcode res = CURLE_FAILED_INIT;
    char url[256];
    time_t now = time(NULL);

    for (int back = 0; back <= (rinex3 ? 3 : 2); back++) {
        time_t t = now - back * 86400;
        struct tm tm;
        gmtime_r(&t, &tm);
        int year = tm.tm_year + 1900;
        int doy = tm.tm_yday + 1;

        for (int n = 0; n < (rinex3 ? 3 : 1); n++) {
            if (rinex3) {
                snprintf(url, sizeof (url), "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/%04d/%03d/%s_%04d%03d0000_01D_MN.rnx.gz",
                        year, doy, rinex3_names[n], year, doy);
            } else {
                snprintf(url, sizeof (url), "https://noaa-cors-pds.s3.amazonaws.com/rinex/%04d/%03d/brdc%03d0.%02dn.gz",
                        year, doy, doy, year % 100);
            }
            // Retry connection errors: a round-robin DNS name can hand out an unreachable server
            for (int attempt = 0; attempt < 3; attempt++) {
                res = download_file(url, filename, 60L, cancel);
                if (res == CURLE_OK) {
                    gui_status_wprintw(GREEN, "Downloaded %s\n", url);
                    return res;
                }
                if (res == CURLE_ABORTED_BY_CALLBACK || !is_connect_error(res)) {
                    break;
                }
            }
            if (res == CURLE_ABORTED_BY_CALLBACK || is_connect_error(res)) {
                return res; // Cancelled, or the host is down so the other candidates would fail too
            }
        }
    }
    return res;
}
