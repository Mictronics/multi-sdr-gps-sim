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

#ifndef GPS_H
#define GPS_H

#include <stdatomic.h>
#include <stdbool.h>
#include <curl/curl.h>

/* Real-time signal generation */
#define REAL_TIME_GPS

#define RINEX2_FILE_NAME "rinex2.gz"
#define RINEX3_FILE_NAME "rinex3.gz"

/* Maximum length of a line in a text file (RINEX, motion) */
#define MAX_CHAR (100)

/* Maximum number of satellites in RINEX file */
#define MAX_SAT (32)

/* Maximum number of channels we simulate */
#define MAX_CHAN (12)

/* Maximum number of user motion points */
#ifndef REAL_TIME_GPS
#define USER_MOTION_SIZE (3000) // max duration at 10Hz
#else
#define USER_MOTION_SIZE (864000) // for 24 hours at 10Hz
#endif

/* Number of subframes */
#define N_SBF (5) // 5 subframes per frame

/* Number of words per subframe */
#define N_DWRD_SBF (10) // 10 word per subframe

/* Number of words */
#define N_DWRD ((N_SBF + 1) * N_DWRD_SBF) // Subframe word buffer size

#define N_SBF_PAGE (3 + 2 * 25) // Subframes 1 to 3 and 25 pages of subframes 4 and 5

/* C/A code sequence length */
#define CA_SEQ_LEN (1023)

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5 0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

#define POW2_M21 4.76837158203125e-007
#define POW2_12 4096
#define POW2_M38 3.63797880709171e-012
#define POW2_M11 0.00048828125
#define POW2_M23 1.19209289550781e-007
#define POW2_M20 9.5367431640625e-007

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5

#ifndef PI
#define PI 3.1415926535898
#endif

#define WGS84_RADIUS 6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426

#ifndef R2D
#define R2D 57.2957795131 // *180/pi
#endif

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

/* C/A code frequency */
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0 / 1540.0)

/*
 * The almanac message for any dummy SVs shall contain alternating ones and zeros
 * with valid parity. (IS-GPS-200L, p.111, 20.3.3.5.1.2)
 */
#define EMPTY_WORD 0xaaaaaaaaUL
#define SPECIAL_MSG_LEN 22 // Characters in subframe 4 page 17

/* Structure representing GPS time */
typedef struct
{
    int week;   /* GPS week number (since January 1980) */
    double sec; /* second inside the GPS \a week */
} gpstime_t;

/* Structure representing UTC time */
typedef struct
{
    int y;      /* Calendar year */
    int m;      /* Calendar month */
    int d;      /* Calendar day */
    int hh;     /* Calendar hour */
    int mm;     /* Calendar minutes */
    double sec; /* Calendar seconds */
} datetime_t;

/* Structure representing ephemeris of a single satellite */
typedef struct
{
    int vflg; /* Valid Flag */
    int sva;  /* SV accuracy (URA index) */
    int svh;  /* SV health */
    int code; /* 0 or 1 code L2 (Codes on L2 channel) */
    int flag; /* L2 P data flag data indicates
               * whether navigation data is being modulated onto the L2 P(Y) code.
               */
    double fit;
    datetime_t t;
    gpstime_t toc; /* Time of Clock */
    gpstime_t toe; /* Time of Ephemeris */
    int iodc;      /* Issue of Data, Clock */
    int iode;      /* Issue of Data, Ephemeris */
    double deltan; /* Delta-N (radians/sec) */
    double cuc;    /* Cuc (radians) */
    double cus;    /* Cus (radians) */
    double cic;    /* Correction to inclination cos (radians) */
    double cis;    /* Correction to inclination sin (radians) */
    double crc;    /* Correction to radius cos (meters) */
    double crs;    /* Correction to radius sin (meters) */
    double ecc;    /* e Eccentricity */
    double sqrta;  /* sqrt(A) (sqrt(m)) */
    double m0;     /* Mean anomaly (radians) */
    double omg0;   /* Longitude of the ascending node (radians) */
    double inc0;   /* Inclination (radians) */
    double aop;
    double omgdot; /* Omega dot (radians/s) */
    double idot;   /* IDOT (radians/s) */
    double af0;    /* a_f0	Clock offset (seconds) */
    double af1;    /* a_f1	rate (sec/sec) */
    double af2;    /* a_f2	acceleration (sec/sec^2) */
    double tgd;    /* Group delay L2 bias */
    // Working variables follow
    double n;       /* Mean motion (Average angular velocity) */
    double sq1e2;   /* sqrt(1-e^2) */
    double A;       /* Semi-major axis */
    double omgkdot; /* OmegaDot-OmegaEdot */
} ephem_t;

typedef struct
{
    int enable;
    int vflg;
    double alpha0, alpha1, alpha2, alpha3;
    double beta0, beta1, beta2, beta3;
    double A0, A1;
    int dtls, tot, wnt;
    int dtlsf, wnlsf, dn; /* Leap second offset after the last or next event, its GPS week and day of week (1 = Sunday) */
    int have;             /* Groups read from the file: 1 alpha, 2 beta, 4 UTC parameters, 8 leap seconds */
} ionoutc_t;

typedef struct
{
    gpstime_t g;
    double range; // pseudorange
    double d;     // geometric distance
    double azel[2];
    double iono_delay;
} range_t;

/* Structure representing a Channel */
typedef struct
{
    int prn;                                   /* PRN Number */
    int ca[CA_SEQ_LEN];                        /* C/A Sequence */
    double f_carr;                             /* Carrier frequency */
    double f_code;                             /* Code frequency */
    double carr_phase;                         /* Carrier phase, 0.0 - 1.0 */
    double code_phase;                         /* Code phase */
    gpstime_t g0;                              /* GPS time at start */
    unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]; /*!< current subframe */
    unsigned long dwrd[N_DWRD];                /*!< Data words of sub-frame */
    int iword;                                 /* initial word */
    int ibit;                                  /* initial bit */
    int icode;                                 /* initial code */
    int dataBit;                               /* current data bit */
    int codeCA;                                /* current C/A code */
    double azel[2];
    range_t rho0;
} channel_t;

#define DOWNLOAD_USER_AGENT "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/152.0.0.0 Safari/537.36"
CURLcode download_file(const char *url, const char *filename, long timeout, const atomic_bool *cancel);

/* All ephemeris records of a RINEX file, grouped by satellite and sorted by clock time. */
typedef struct
{
    ephem_t *rec;           /* Free with free() */
    int n;                  /* Number of records */
    int first[MAX_SAT + 1]; /* Satellite sv + 1 owns rec[first[sv]] up to, not including, rec[first[sv + 1]] */
} eph_table_t;

/* Records further than this from the wanted time are not used: half of the 4 hour fit interval */
#define EPH_MAX_AGE 7200.0

/* Read the ephemerides of a RINEX 2 or 3 navigation file (see gps.c).
 * Returns the number of records, negative on error. */
int readRinex(eph_table_t *tab, ionoutc_t *ionoutc, const char *fname, bool rinex3);

/* For every satellite pick the record closest in time to t (vflg false if none is near enough). */
void ephSelect(const eph_table_t *tab, gpstime_t t, ephem_t cur[MAX_SAT]);

/* Fill the ionosphere and UTC groups the file did not provide with neutral values for the start time g,
 * so subframe 4 page 18 is complete. Returns true if anything was filled. */
bool ionoDefaults(ionoutc_t *io, gpstime_t g);

void *gps_thread_ep(void *arg);

#endif /* GPS_H */
