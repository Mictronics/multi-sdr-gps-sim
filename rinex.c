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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zlib.h>
#include "gps.h"
#include "orbit.h"
#include "rinex.h"

char rinex_date[21];

/* Replace all 'E' exponential designators to 'D'
 * str String in which all occurrences of 'E' are replaced with *  'D'
 * len Length of input string in bytes
 * Number of characters replaced
 */
static int replaceExpDesignator(char *str, int len) {
    int i, n = 0;

    for (i = 0; i < len; i++) {
        if (str[i] == 0) {
            break;
        }

        if (str[i] == 'D' || str[i] == 'd') {
            n++;
            str[i] = 'E';
        }
    }

    return (n);
}

/* Parse a fixed-width RINEX field. Fortran 'D' exponents are accepted.
 * line RINEX line
 * col First column (0 based)
 * len Field width in characters
 * Parsed value, 0 if the line is too short
 */
static double rinexField(const char *line, int col, int len) {
    char tmp[24];

    if ((int) strlen(line) <= col || len >= (int) sizeof (tmp))
        return 0.0;

    strncpy(tmp, line + col, len);
    tmp[len] = 0;
    replaceExpDesignator(tmp, len);

    return atof(tmp);
}

static bool rinexLabel(const char *line, const char *label) {
    return strlen(line) >= 60 && strncmp(line + 60, label, strlen(label)) == 0;
}

/* Read the RINEX header: version check, iono/UTC parameters and leap seconds.
 * Returns 0 on success, -2 for an unsupported version, -3 for a wrong file type.
 */
static int readRinexHeader(gzFile fp, ionoutc_t *ionoutc, bool rinex3) {
    char str[MAX_CHAR];
    int flags = 0x0;

    while (gzgets(fp, str, MAX_CHAR) != NULL) {
        if (rinexLabel(str, "END OF HEADER")) {
            break;
        } else if (rinexLabel(str, "RINEX VERSION / TYPE")) {
            double ver = rinexField(str, 0, 9);
            if (rinex3 ? (ver < 3.0 || ver >= 4.0) : ver > 3.0)
                return -2;
            if (str[20] != 'N')
                return -3;
        } else if (rinexLabel(str, "PGM / RUN BY / DATE")) {
            strncpy(rinex_date, str + 40, 20);
            rinex_date[20] = 0;
        } else if (!rinex3 && rinexLabel(str, "ION ALPHA")) {
            ionoutc->alpha0 = rinexField(str, 2, 12);
            ionoutc->alpha1 = rinexField(str, 14, 12);
            ionoutc->alpha2 = rinexField(str, 26, 12);
            ionoutc->alpha3 = rinexField(str, 38, 12);
            flags |= 0x1;
        } else if (!rinex3 && rinexLabel(str, "ION BETA")) {
            ionoutc->beta0 = rinexField(str, 2, 12);
            ionoutc->beta1 = rinexField(str, 14, 12);
            ionoutc->beta2 = rinexField(str, 26, 12);
            ionoutc->beta3 = rinexField(str, 38, 12);
            flags |= 0x1 << 1;
        } else if (!rinex3 && rinexLabel(str, "DELTA-UTC")) {
            ionoutc->A0 = rinexField(str, 3, 19);
            ionoutc->A1 = rinexField(str, 22, 19);
            ionoutc->tot = (int) rinexField(str, 41, 9);
            ionoutc->wnt = (int) rinexField(str, 50, 9);
            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (rinex3 && rinexLabel(str, "IONOSPHERIC CORR") && strncmp(str, "GPSA", 4) == 0) {
            ionoutc->alpha0 = rinexField(str, 5, 12);
            ionoutc->alpha1 = rinexField(str, 17, 12);
            ionoutc->alpha2 = rinexField(str, 29, 12);
            ionoutc->alpha3 = rinexField(str, 41, 12);
            flags |= 0x1;
        } else if (rinex3 && rinexLabel(str, "IONOSPHERIC CORR") && strncmp(str, "GPSB", 4) == 0) {
            ionoutc->beta0 = rinexField(str, 5, 12);
            ionoutc->beta1 = rinexField(str, 17, 12);
            ionoutc->beta2 = rinexField(str, 29, 12);
            ionoutc->beta3 = rinexField(str, 41, 12);
            flags |= 0x1 << 1;
        } else if (rinex3 && rinexLabel(str, "TIME SYSTEM CORR") && strncmp(str, "GPUT", 4) == 0) {
            ionoutc->A0 = rinexField(str, 5, 17);
            ionoutc->A1 = rinexField(str, 22, 16);
            ionoutc->tot = (int) rinexField(str, 38, 7);
            ionoutc->wnt = (int) rinexField(str, 45, 6);
            if (ionoutc->tot % 4096 == 0)
                flags |= 0x1 << 2;
        } else if (rinexLabel(str, "LEAP SECONDS") && (strncmp(str + 24, "   ", 3) == 0 || strncmp(str + 24, "GPS", 3) == 0)) {
            // Lines for other systems carry a system id after the four I6 values
            ionoutc->dtls = (int) rinexField(str, 0, 6);
            // The last event is optional; without it assume the 2016/12/31 one, after which the offset is unchanged
            ionoutc->dtlsf = ionoutc->dtls;
            ionoutc->wnlsf = LEAP_DEFAULT_WN;
            ionoutc->dn = LEAP_DEFAULT_DN;
            if (rinexField(str, 12, 6) > 0) {
                ionoutc->dtlsf = (int) rinexField(str, 6, 6);
                ionoutc->wnlsf = (int) rinexField(str, 12, 6);
                ionoutc->dn = (int) rinexField(str, 18, 6);
            }
            flags |= 0x1 << 3;
        }
    }

    ionoutc->have = flags;
    ionoutc->vflg = (flags == 0xF); // All iono/UTC lines read
    return 0;
}

/* Parse one ephemeris record: the first line is already in str, the 7 broadcast
 * orbit lines are read from fp.
 * Returns false if the file ends inside the record or the record is invalid.
 */
static bool readRinexRecord(gzFile fp, char *str, bool rinex3, int *sv, ephem_t *e) {
    // RINEX 3 columns are those of RINEX 2 shifted by one
    const int o = rinex3 ? 1 : 0;
    datetime_t t;
    gpstime_t g;
    int y;

    if (rinex3) {
        *sv = (int) rinexField(str, 1, 2) - 1;
        t.y = (int) rinexField(str, 4, 4);
        t.m = (int) rinexField(str, 9, 2);
        t.d = (int) rinexField(str, 12, 2);
        t.hh = (int) rinexField(str, 15, 2);
        t.mm = (int) rinexField(str, 18, 2);
        t.sec = (int) rinexField(str, 21, 2);
    } else {
        *sv = (int) rinexField(str, 0, 2) - 1;
        y = (int) rinexField(str, 3, 2);
        t.y = y < 80 ? 2000 + y : 1900 + y;
        t.m = (int) rinexField(str, 6, 2);
        t.d = (int) rinexField(str, 9, 2);
        t.hh = (int) rinexField(str, 12, 2);
        t.mm = (int) rinexField(str, 15, 2);
        t.sec = rinexField(str, 17, 5);
    }

    // SV CLK
    e->t = t;
    e->af0 = rinexField(str, 22 + o, 19);
    e->af1 = rinexField(str, 41 + o, 19);
    e->af2 = rinexField(str, 60 + o, 19);

    for (int line = 1; line <= 7; line++) {
        if (NULL == gzgets(fp, str, MAX_CHAR))
            return false;

        const double a = rinexField(str, 3 + o, 19);
        const double b = rinexField(str, 22 + o, 19);
        const double c = rinexField(str, 41 + o, 19);
        const double d = rinexField(str, 60 + o, 19);

        switch (line) {
            case 1:
                e->iode = (int) a;
                e->crs = b;
                e->deltan = c;
                e->m0 = d;
                break;
            case 2:
                e->cuc = a;
                e->ecc = b;
                e->cus = c;
                e->sqrta = d;
                break;
            case 3:
                e->toe.sec = a;
                e->cic = b;
                e->omg0 = c;
                e->cis = d;
                break;
            case 4:
                e->inc0 = a;
                e->crc = b;
                e->aop = c;
                e->omgdot = d;
                break;
            case 5:
                e->idot = a;
                e->code = (int) b;
                e->toe.week = (int) c;
                e->flag = (int) d;
                break;
            case 6:
                e->sva = (int) a;
                e->svh = (int) b;
                if ((e->svh > 0) && (e->svh < 32))
                    e->svh += 32; // Set MSB to 1
                e->tgd = c;
                e->iodc = (int) d;
                break;
            case 7:
                e->fit = b;
                break;
        }
    }

    if (*sv < 0 || *sv >= MAX_SAT || t.m < 1 || t.m > 12)
        return false;

    date2gps(&t, &g);
    e->toc = g;
    e->vflg = true;

    // Update the working variables
    e->A = e->sqrta * e->sqrta;
    e->n = sqrt(GM_EARTH / (e->A * e->A * e->A)) + e->deltan;
    e->sq1e2 = sqrt(1.0 - e->ecc * e->ecc);
    e->omgkdot = e->omgdot - OMEGA_EARTH;

    return true;
}

struct eph_rec {
    int sv;
    int index; // Position in the file, keeps the sort stable
    ephem_t eph;
};

static int compareEphRec(const void *a, const void *b) {
    const struct eph_rec *ra = (const struct eph_rec *) a;
    const struct eph_rec *rb = (const struct eph_rec *) b;

    if (ra->sv != rb->sv)
        return ra->sv - rb->sv;

    double dt = subGpsTime(ra->eph.toc, rb->eph.toc);
    if (dt != 0.0)
        return dt < 0.0 ? -1 : 1;

    return ra->index - rb->index;
}

/* Read ephemeris data from a RINEX 2 or 3 navigation file.
 * Records may come in any order (the IGS merged files are sorted by satellite).
 * A satellite listed twice with the same clock time keeps the later record.
 * tab Output, all records by satellite and clock time; tab->rec is NULL without records
 * rinex3 true for RINEX 3, false for RINEX 2
 * Number of records, negative on error
 */
int readRinex(eph_table_t *tab, ionoutc_t *ionoutc, const char *fname, bool rinex3) {
    char str[MAX_CHAR];
    struct eph_rec *recs = NULL;
    int nrec = 0, capacity = 0;
    int result;
    gzFile fp;

    memset(tab, 0, sizeof (*tab));

    if (fname == NULL || NULL == (fp = gzopen(fname, "rt")))
        return (-1);

    if ((result = readRinexHeader(fp, ionoutc, rinex3)) != 0) {
        gzclose(fp);
        return result;
    }

    while (NULL != gzgets(fp, str, MAX_CHAR)) {
        // RINEX 3 files can hold other constellations, whose lines are skipped
        if (rinex3 && str[0] != 'G')
            continue;

        if (nrec == capacity) {
            capacity = capacity ? capacity * 2 : 512;
            struct eph_rec *grown = realloc(recs, capacity * sizeof (*recs));
            if (grown == NULL) {
                free(recs);
                gzclose(fp);
                return (-1);
            }
            recs = grown;
        }

        // Records may be invalid; they are consumed and dropped
        if (readRinexRecord(fp, str, rinex3, &recs[nrec].sv, &recs[nrec].eph)) {
            recs[nrec].index = nrec;
            nrec++;
        }
    }

    gzclose(fp);

    if (nrec == 0) {
        free(recs);
        return 0;
    }

    qsort(recs, nrec, sizeof (*recs), compareEphRec);

    tab->rec = malloc(nrec * sizeof (*tab->rec));
    if (tab->rec == NULL) {
        free(recs);
        return (-1);
    }

    for (int i = 0; i < nrec; i++) {
        // Of equal records of one satellite only the last one (sorted last) is kept
        if (i + 1 < nrec && recs[i + 1].sv == recs[i].sv && subGpsTime(recs[i + 1].eph.toc, recs[i].eph.toc) == 0.0)
            continue;
        tab->rec[tab->n++] = recs[i].eph;
        tab->first[recs[i].sv + 1] = tab->n;
    }
    // Satellites without records inherit the end of the previous one
    for (int sv = 1; sv <= MAX_SAT; sv++)
        if (tab->first[sv] < tab->first[sv - 1])
            tab->first[sv] = tab->first[sv - 1];

    free(recs);
    return tab->n;
}

/* Zero Klobuchar and UTC terms change nothing: the delay equals that without iono data, and UTC
 * follows from the leap seconds alone. */
bool ionoDefaults(ionoutc_t *io, gpstime_t g) {
    if (io->have == 0xF)
        return false;

    if (!(io->have & 0x1))
        io->alpha0 = io->alpha1 = io->alpha2 = io->alpha3 = 0.0;
    if (!(io->have & 0x2))
        io->beta0 = io->beta1 = io->beta2 = io->beta3 = 0.0;
    if (!(io->have & 0x4)) {
        io->A0 = io->A1 = 0.0;
        io->tot = ((int) g.sec / 4096) * 4096;
        io->wnt = g.week;
    }
    if (!(io->have & 0x8)) {
        io->dtls = io->dtlsf = LEAP_DEFAULT_DTLS;
        io->wnlsf = LEAP_DEFAULT_WN;
        io->dn = LEAP_DEFAULT_DN;
    }
    io->have = 0xF;
    io->vflg = true;
    return true;
}

void ephSelect(const eph_table_t *tab, gpstime_t t, ephem_t cur[MAX_SAT]) {
    for (int sv = 0; sv < MAX_SAT; sv++) {
        double best = EPH_MAX_AGE;

        cur[sv].vflg = false;
        for (int i = tab->first[sv]; i < tab->first[sv + 1]; i++) {
            double age = fabs(subGpsTime(tab->rec[i].toc, t));
            if (age <= best) { // The later of two equally close records wins
                best = age;
                cur[sv] = tab->rec[i];
            }
        }
    }
}
