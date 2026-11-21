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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"
#include "orbit.h"
#include "motion.h"
#include "geoid_egm96.h"

typedef struct {
    double t; // seconds since the first point
    double xyz[3];
} trkpt_t;

double geoidHeight(double lat, double lon) {
    double y = (90.0 - lat) / GEOID_STEP, x = fmod(lon + 360.0, 360.0) / GEOID_STEP;
    int r = (int) y, c = (int) x;
    if (r >= GEOID_ROWS - 1)
        r = GEOID_ROWS - 2;
    double fy = y - r, fx = x - c;
    int c1 = (c + 1) % GEOID_COLS;
    double top = (1 - fx) * geoid_egm96[r][c] + fx * geoid_egm96[r][c1];
    double bot = (1 - fx) * geoid_egm96[r + 1][c] + fx * geoid_egm96[r + 1][c1];
    return ((1 - fy) * top + fy * bot) / GEOID_SCALE;
}

static char *readFile(const char *filename) {
    FILE *fp = fopen(filename, "rb");
    if (fp == NULL)
        return NULL;
    fseek(fp, 0, SEEK_END);
    long n = ftell(fp);
    rewind(fp);
    char *buf = n >= 0 ? malloc(n + 1) : NULL;
    if (buf != NULL) {
        buf[fread(buf, 1, n, fp)] = '\0';
    }
    fclose(fp);
    return buf;
}

/* Value of attribute name="..." (or '...') inside the tag [tag, end). */
static bool attr(const char *tag, const char *end, const char *name, double *v) {
    size_t len = strlen(name);
    for (const char *p = tag; (p = strstr(p, name)) != NULL && p < end; p += len) {
        if ((p[-1] == ' ' || p[-1] == '\t' || p[-1] == '\n' || p[-1] == '\r') && p[len] == '=' && (p[len + 1] == '"' || p[len + 1] == '\'')) {
            *v = strtod(p + len + 2, NULL);
            return true;
        }
    }
    return false;
}

/* Start of the text of child element <name> inside [body, end), or NULL. */
static const char *child(const char *body, const char *end, const char *name) {
    const char *p = strstr(body, name);
    return p != NULL && p < end ? p + strlen(name) : NULL;
}

/* ISO 8601 "YYYY-MM-DDThh:mm:ss[.s][Z|+hh:mm|-hh:mm]" as seconds on the GPS time scale. */
static bool isoTime(const char *s, double *sec) {
    datetime_t d;
    gpstime_t g;
    int n = 0;

    if (sscanf(s, "%d-%d-%dT%d:%d:%lf%n", &d.y, &d.m, &d.d, &d.hh, &d.mm, &d.sec, &n) != 6)
        return false;
    date2gps(&d, &g);
    *sec = g.week * 604800.0 + g.sec;
    int oh, om;
    if ((s[n] == '+' || s[n] == '-') && sscanf(s + n + 1, "%d:%d", &oh, &om) == 2)
        *sec -= (s[n] == '+' ? 1 : -1) * (oh * 3600.0 + om * 60.0);
    return true;
}

static int parsePoints(const char *gpx, double default_height, trkpt_t **out, const char **err) {
    trkpt_t *pts = NULL;
    int n = 0, cap = 0;
    double t0 = 0.0;

    for (const char *p = gpx; (p = strstr(p, "<trkpt")) != NULL; p++) {
        const char *tag_end = strchr(p, '>');
        const char *body_end = tag_end ? strstr(tag_end, "</trkpt>") : NULL;
        double llh[3] = {0.0, 0.0, default_height}, t;
        const char *v;

        if (tag_end == NULL || tag_end[-1] == '/' || body_end == NULL || (v = child(tag_end, body_end, "<time>")) == NULL || !isoTime(v, &t)) {
            *err = "every track point needs a <time>";
            goto fail;
        }
        // Negated so NaN fails too; geoidHeight indexes its table with these
        if (!attr(p, tag_end, "lat", &llh[0]) || !attr(p, tag_end, "lon", &llh[1]) || !(fabs(llh[0]) <= 90.0) || !(fabs(llh[1]) <= 180.0)) {
            *err = "track point without valid lat/lon";
            goto fail;
        }
        // <ele> is above mean sea level in practice, the simulation needs the ellipsoid height
        if ((v = child(tag_end, body_end, "<ele>")) != NULL)
            llh[2] = strtod(v, NULL) + geoidHeight(llh[0], llh[1]);
        if (n == 0)
            t0 = t;
        t -= t0;
        if (n > 0 && t <= pts[n - 1].t) {
            *err = "track point times must increase";
            goto fail;
        }
        if (n == cap) {
            cap = cap ? 2 * cap : 1024;
            trkpt_t *q = realloc(pts, cap * sizeof (*pts));
            if (q == NULL) {
                *err = "out of memory";
                goto fail;
            }
            pts = q;
        }
        llh[0] /= R2D;
        llh[1] /= R2D;
        pts[n].t = t;
        llh2xyz(llh, pts[n].xyz);
        n++;
        p = body_end;
    }
    if (n == 0) {
        *err = "no <trkpt> found";
        goto fail;
    }
    *out = pts;
    return n;
fail:
    free(pts);
    return -1;
}

int readGpx(double xyz[][3], int max, const char *filename, double default_height, const char **err) {
    char *gpx = readFile(filename);
    trkpt_t *pts;

    if (gpx == NULL) {
        *err = "cannot read file";
        return -1;
    }
    int n = parsePoints(gpx, default_height, &pts, err);
    free(gpx);
    if (n < 0)
        return -1;

    // 1e-6 so a span of whole 0.1 s steps keeps its last step despite rounding
    int steps = (int) floor(pts[n - 1].t / 0.1 + 1e-6) + 1;
    if (steps > max)
        steps = max;
    for (int k = 0, i = 0; k < steps; k++) {
        double t = k * 0.1;
        while (i < n - 2 && pts[i + 1].t < t)
            i++;
        if (n == 1) {
            memcpy(xyz[k], pts[0].xyz, sizeof (xyz[k]));
            continue;
        }
        double f = (t - pts[i].t) / (pts[i + 1].t - pts[i].t);
        if (f > 1.0)
            f = 1.0;
        for (int j = 0; j < 3; j++)
            xyz[k][j] = pts[i].xyz[j] + f * (pts[i + 1].xyz[j] - pts[i].xyz[j]);
    }
    free(pts);
    return steps;
}
