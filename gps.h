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

#ifndef GPS_H
#define GPS_H

/* For RKT simulation. Higher computational load, but smoother carrier phase.*/
#define FLOAT_CARR_PHASE

/* Real-time signal generation */
#define REAL_TIME_GPS

#define RINEX2_FILE_NAME "rinex2.gz"
#define RINEX3_FILE_NAME "rinex3.gz"
#define RINEX_FTP_URL "ftp://igs.bkg.bund.de/IGS/"
#define RINEX2_SUBFOLDER "nrt"
#define RINEX3_SUBFOLDER "nrt_v3"
#define RINEX_FTP_FILE "%s/%03i/%02i/%4s%03i%c.%02in.gz"

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
#define N_DWRD ((N_SBF+1)*N_DWRD_SBF) // Subframe word buffer size

#define N_SBF_PAGE (3+2*25) // Subframes 1 to 3 and 25 pages of subframes 4 and 5
#define MAX_PAGE (25)

/* C/A code sequence length */
#define CA_SEQ_LEN (1023)

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5  0.03125
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
#define POW2_12  4096
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
#define CARR_TO_CODE (1.0/1540.0)

#define EPHEM_ARRAY_SIZE (13) // for daily GPS broadcast ephemers file (brdc)

/* GPS parity bit-vectors
 * The last 6 bits of a 30bit GPS word are parity check bits.
 * Each parity bit is computed from the XOR of a selection of bits from the
 * 1st 24 bits of the current GPS word, and the last 2 bits of the _previous_
 * GPS word.
 * These parity bit-vectors are used to select which message bits will be used
 * for computing each of the 6 parity check bits.
 * We assume the two bits from the previous message (b29, b30) and the 24 bits
 * from the current message, are packed into a 32bit word in this order:
 * < b29, b30, b1, b2, b3, ... b23, b24, X, X, X, X, X, X > (X = don't care)
 * Example: if PBn = 0x40000080,
 * The parity check bit "n" would be computed from the expression (b30 XOR b23).
 */
#define PB1 0xbb1f3480
#define PB2 0x5d8f9a40
#define PB3 0xaec7cd00
#define PB4 0x5763e680
#define PB5 0x6bb1f340
#define PB6 0x8b7a89c0

/*
 * The almanac message for any dummy SVs shall contain alternating ones and zeros
 * with valid parity. (IS-GPS-200L, p.111, 20.3.3.5.1.2)
 */
#define EMPTY_WORD 0xaaaaaaaaUL 

/* Structure representing GPS time */
typedef struct {
    int week; /* GPS week number (since January 1980) */
    double sec; /* second inside the GPS \a week */
} gpstime_t;

/* Structure repreenting UTC time */
typedef struct {
    int y; /* Calendar year */
    int m; /* Calendar month */
    int d; /* Calendar day */
    int hh; /* Calendar hour */
    int mm; /* Calendar minutes */
    double sec; /* Calendar seconds */
} datetime_t;

/* Structure representing ephemeris of a single satellite */
typedef struct {
    int vflg; /* Valid Flag */
    int sva; /* SV accuracy (URA index) */
    int svh; /* SV health */
    int code; /* 0 or 1 code L2 (Codes on L2 channel) */
    int flag; /* L2 P data flag data indicates 
               * whether navigation data is being modulated onto the L2 P(Y) code.
               */
    double fit;
    datetime_t t;
    gpstime_t toc; /* Time of Clock */
    gpstime_t toe; /* Time of Ephemeris */
    int iodc; /* Issue of Data, Clock */
    int iode; /* Isuse of Data, Ephemeris */
    double deltan; /* Delta-N (radians/sec) */
    double cuc; /* Cuc (radians) */
    double cus; /* Cus (radians) */
    double cic; /* Correction to inclination cos (radians) */
    double cis; /* Correction to inclination sin (radians) */
    double crc; /* Correction to radius cos (meters) */
    double crs; /* Correction to radius sin (meters) */
    double ecc; /* e Eccentricity */
    double sqrta; /* sqrt(A) (sqrt(m)) */
    double m0; /* Mean anamoly (radians) */
    double omg0; /* Longitude of the ascending node (radians) */
    double inc0; /* Inclination (radians) */
    double aop;
    double omgdot; /* Omega dot (radians/s) */
    double idot; /* IDOT (radians/s) */
    double af0; /* a_f0	Clock offset (seconds) */
    double af1; /* a_f1	rate (sec/sec) */
    double af2; /* a_f2	acceleration (sec/sec^2) */
    double tgd; /* Group delay L2 bias */
    // Working variables follow
    double n; /* Mean motion (Average angular velocity) */
    double sq1e2; /* sqrt(1-e^2) */
    double A; /* Semi-major axis */
    double omgkdot; /* OmegaDot-OmegaEdot */
} ephem_t;

typedef struct {
    int enable;
    int vflg;
    double alpha0, alpha1, alpha2, alpha3;
    double beta0, beta1, beta2, beta3;
    double A0, A1;
    int dtls, tot, wnt;
    int dtlsf, dn, wnlsf;
} ionoutc_t;

typedef struct {
    gpstime_t g;
    double range; // pseudorange
    double rate;
    double d; // geometric distance
    double azel[2];
    double iono_delay;
} range_t;

/* Structure representing a Channel */
typedef struct {
    int prn; /* PRN Number */
    int ca[CA_SEQ_LEN]; /* C/A Sequence */
    double f_carr; /* Carrier frequency */
    double f_code; /* Code frequency */
#ifdef FLOAT_CARR_PHASE
    double carr_phase;
#else
    unsigned int carr_phase; /* Carrier phase */
    int carr_phasestep; /* Carrier phasestep */
#endif
    double code_phase; /* Code phase */
    gpstime_t g0; /* GPS time at start */
	unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]; /*!< current subframe */
	unsigned long dwrd[N_DWRD]; /*!< Data words of sub-frame */
	int ipage;
    int iword; /* initial word */
    int ibit; /* initial bit */
    int icode; /* initial code */
    int dataBit; /* current data bit */
    int codeCA; /* current C/A code */
    double azel[2];
    range_t rho0;
} channel_t;

/* Structure represending a single GPS monitoring station. */
typedef struct {
    const char *id_v2;
    const char *id_v3;
    const char *name;
} stations_t;

void *gps_thread_ep(void *arg);

#endif /* GPS_H */

