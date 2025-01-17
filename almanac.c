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
#include "gps-sim.h"
#include "almanac.h"

static almanac_gps_t almanac_gps;

struct sem_file {
    const char *filename;
    FILE *stream;
};

/**
 * Initialize empty almanac.
 */
almanac_gps_t* almanac_init(void) {
    almanac_gps.valid = 0;
    almanac_prn_t* a = NULL;
    for (int prn = 0; prn < MAX_SAT; prn++) {
        a = &almanac_gps.sv[prn];
        a->ura = 0;
        a->health = 0;
        a->config_code = 0;
        a->svid = 0;
        a->svn = 0;
        a->valid = 0;
        a->toa.sec = 0.0;
        a->toa.week = 0;
        a->e = 0.0;
        a->delta_i = 0.0;
        a->omegadot = 0.0;
        a->sqrta = 0.0;
        a->omega0 = 0.0;
        a->aop = 0.0;
        a->m0 = 0.0;
        a->af0 = 0.0;
        a->af1 = 0.0;
    }
    return &almanac_gps;
}

/**
 * Curl file writer callback.
 */
static size_t fwrite_sem(void *buffer, size_t size, size_t nmemb, void *stream) {
    struct sem_file *out = (struct sem_file *) stream;
    if (out && !out->stream) {
        /* open file for writing */
        out->stream = fopen(out->filename, "wb");
        if (!out->stream)
            return -1; /* failure, can't open file to write */
    }
    return fwrite(buffer, size, nmemb, out->stream);
}

/**
 * Read almanac from local file.
 * sem format expected.
 */
CURLcode almanac_read_file(void) {
    char buf[100];
    char title[24];
    char *pbuf;
    unsigned int n, week, sec, id;
    FILE *fp = fopen("almanac.sem", "rt");
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

        // SVN is optional, could be a blank line
        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (buf[0] == '\n' || buf[0] == '\r') {
            a->svn = 0;
        } else {
            if (sscanf(buf, "%hu", &a->svn) != 1) goto error;
        }

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->ura) != 1) goto error;
        if (a->ura > 15) a->ura = 15;

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

        pbuf = fgets(buf, sizeof (buf), fp);
        if (pbuf == NULL) goto error;
        if (sscanf(buf, "%hhu", &a->config_code) != 1) goto error;
        if (a->config_code > 15) a->config_code = 15;

        /**
         * ublox u-center software provides the full GPS week number whereas Celestrak follows
         * IC-GPS-200L, p. 116, 20.3.3.5.1.5 Almanac Reference Week:
         * The WNa term consists of eight bits which shall be a modulo 256 binary
         * representation of the GPS week number...
         * Celestrak almanac files use a modulo 256 week number in file name.
         * See https://celestrak.com/GPS/almanac/SEM/2021/
	 *
	 * NOTE: It seems like sometime in the last four years this changed.
	 * a->toa.week = (int) week % 256; results in incorrect date parsing.
         */
        a->toa.week = (int) week;
        a->toa.sec = (double) sec;
        // GPS week rollover
        a->toa.week += 2048;
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

/**
 * Read almanac from online source.
 * sem format expected.
 *  
 */
CURLcode almanac_download(void) {
    CURL *curl;
    CURLcode res = CURLE_GOT_NOTHING;
    struct sem_file sem = {
        "almanac.sem",
        NULL
    };

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, ALMANAC_DOWNLOAD_SEM_URL);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fwrite_sem);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sem);
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }

    if (sem.stream)
        fclose(sem.stream);

    curl_global_cleanup();

    if (res != CURLE_OK) {
        return res;
    }

    return (almanac_read_file());
}
