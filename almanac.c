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
    char buf[100];
    char title[25]; // 24 chars for %24s plus NUL
    char *pbuf;
    unsigned int n, week, sec, id;
    unsigned short ignored;
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

/**
 * Read almanac from online source.
 * sem format expected.
 *  
 */
CURLcode almanac_download(const atomic_bool *cancel) {
    CURLcode res = download_file(ALMANAC_DOWNLOAD_SEM_URL, "almanac.sem", cancel);
    if (res != CURLE_OK) {
        return res;
    }

    return (almanac_read_file());
}
