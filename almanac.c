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
#include <curl/curl.h>
#include "gui.h"
#include "gps-sim.h"
#include "gps.h"
#include "almanac.h"

almanac_gps_t almanac_gps;

struct sem_file {
    const char *filename;
    FILE *stream;
};

static void almanac_init(void) {
    almanac_gps.valid = 0;
    almanac_prn_t* a = NULL;
    for (int prn = 0; prn < 32; prn++) {
        a = &almanac_gps.prn[prn];
        a->ura = 0;
        a->health = 0;
        a->config_code = 0;
        a->week = 0;
        a->prn = 0;
        a->svn = 0;
        a->valid = 0;
        a->toa = 0;
        a->e = 0.0;
        a->delta_i = 0.0;
        a->omegadot = 0.0;
        a->sqrta = 0.0;
        a->omega0 = 0.0;
        a->w = 0.0;
        a->m0 = 0.0;
        a->af0 = 0.0;
        a->af1 = 0.0;
    }
}

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

almanac_gps_t* almanac_read_file(void) {
    int l;
    char buf[64];
    unsigned j, n, week, toa, ui;
    double dbl;
    FILE *fp = fopen("almanac.sem", "r");
    almanac_prn_t* a = NULL;

    almanac_init();

    if (!fp) {
        return &almanac_gps;
    }
    l = fscanf(fp, "%u", &n);
    if (l != 1) goto error;
    l = fscanf(fp, "%s", buf);
    if (l != 1) goto error;
    l = fscanf(fp, "%u", &week);
    if (l != 1) goto error;
    l = fscanf(fp, "%u", &toa);
    if (l != 1) goto error;

    n -= 1; // PRN in file counts 1-32, array counts 0-31
    if (n > 31) n = 31; // Max 32 PRN's to read (0-31)

    for (j = 1; j <= n; j++) {
        int prn;
        l = fscanf(fp, "%u", &ui);
        if (l != 1) goto error;
        prn = (unsigned short) ui;

        a = &almanac_gps.prn[prn];

        a->prn = (unsigned short) prn;
        a->week = (unsigned short) week;
        a->toa = (unsigned int) toa;

        l = fscanf(fp, "%u", &ui);
        if (l != 1) goto error;
        a->svn = (unsigned short) ui;
        l = fscanf(fp, "%u", &ui);
        if (l != 1) goto error;
        a->ura = (unsigned char) ui;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->e = dbl;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->delta_i = dbl;
        a->delta_i = (0.30 + a->delta_i) * PI;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->omegadot = dbl * PI;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->sqrta = dbl;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->omega0 = dbl * PI;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->w = dbl * PI;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->m0 = dbl * PI;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->af0 = dbl;
        l = fscanf(fp, "%lf", &dbl);
        if (l != 1) goto error;
        a->af1 = dbl;

        l = fscanf(fp, "%u", &ui);
        if (l != 1) goto error;
        a->health = (unsigned char) ui;
        l = fscanf(fp, "%u", &ui);
        if (l != 1) goto error;
        a->config_code = (unsigned char) ui;
        a->week &= 0xFF;
        a->valid = 1;
    }
    fclose(fp);
    almanac_gps.valid = 1;
    return &almanac_gps;

error:
    fclose(fp);
    gui_status_wprintw(RED, "Failed to read file almanac.sem!\n");
    // Drop all parsed almanac data on error
    almanac_init();
    return &almanac_gps;
}

almanac_gps_t* almanac_download(void) {
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
        if (0) {
            curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        } else {
            curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);
        }
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }

    if (sem.stream)
        fclose(sem.stream);

    curl_global_cleanup();

    if (res != CURLE_OK) {
        switch (res) {
            case CURLE_REMOTE_FILE_NOT_FOUND:
                gui_status_wprintw(RED, "Curl error: Almanac file not found!\n");
                break;
            default:
                gui_status_wprintw(RED, "Curl error: %d\n", res);
                break;
        }
        return NULL;
    }

    return almanac_read_file();
}

bool almanac_get_live(void) {
    return false;
}