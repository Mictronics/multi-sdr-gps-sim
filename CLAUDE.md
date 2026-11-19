# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```
make all HACKRFSDR=yes PLUTOSDR=yes   # either flag optional; iqfile backend is always built
make test
make clean
```

Flat Makefile, C11, no linter. `make test` runs two unit tests, linked against `gps.c` with the GUI stubbed (`tests/stubs.c`) and run from the repository root: `tests/test_nav.c` checks the navigation message fields (SV health, almanac health, leap seconds) that `eph2sbf` writes against the IS-GPS-200 bit positions, and `tests/test_rinex.c` checks `readRinex` and `ephSelect` on `tests/data/sample_rinex3.rnx`, a trimmed real IGS merged file sorted by satellite, plus files built from its records that reproduce layouts which broke the ephemeris selection (lone uploads next to a regular update, more than a dozen epochs, duplicates). Deps: libm, pthread, libcurl, libz, ncurses/panel, plus libhackrf and/or libiio + libad9361 when the matching SDR flag is set. Backends are compiled in via `-DENABLE_HACKRFSDR` / `-DENABLE_PLUTOSDR`.

Objects track header dependencies (`-MMD`) and are rebuilt when `HACKRFSDR`, `PLUTOSDR` or `OPT` change (`.build-flags`). The default is `-O2`; use `make OPT=-O0` for debugging.

Run example: `./gps-sim -f -s now -l 48.0,10.0,500 -r iqfile` (downloads current data; with your own files use `-e nav.gz` instead of `-f`) (see README for all options; `help.h` holds the usage text). `-r` selects the SDR backend (`none|iqfile|hackrf|plutosdr`). Nav files may be gzipped RINEX 2 or 3 (`-3`). `-f` downloads the nav file (NOAA CORS for RINEX 2, BKG for RINEX 3) and the Celestrak almanac over HTTPS instead.

The program needs a terminal of at least 120x26 and writes `iqdata.bin` in the working directory with `-r iqfile`. Without `-f` the almanac is read from `almanac.sem` in the working directory; one more than 4 weeks away from the scenario time is rejected, so use `--disable-almanac` with old nav files.

## Architecture

Derived from Takuji Ebinuma's gps-sdr-sim. The program synthesizes GPS L1 C/A baseband IQ on the fly and streams it to an SDR, with an ncurses TUI.

- `gps-sim.c` — `main`, option parsing, and the global `simulator` struct (`gps-sim.h`) shared across threads. Starts the GPS thread, then the selected SDR backend and the TUI.
- `gps.c` (largest file) — the simulation core: downloads (`download_file`), RINEX parsing (`readRinex` handles v2 and v3 into an `eph_table_t` holding every record per satellite, so satellite-ordered merged files work; `ephSelect` gives each satellite the record closest in time within `EPH_MAX_AGE`, used at the start and at every 30 s update), ephemeris/almanac handling, satellite channel allocation, pseudorange/Doppler computation, navigation message (subframe) generation, and IQ sample generation. `gps_thread_ep` is the producer thread: a short orchestrator over the steps `initUserPosition`, `loadEphemeris`, `loadMotionFile`, `selectStartTime`, `loadAlmanac` and `initChannels`, then per 0.1 s block `moveUser`, `updateChannels`, `generateSamples`, `queueSamples` (`NUM_IQ_SAMPLES` at 3 MS/s, see `sdr.h`), plus `updateNavigation` every 30 s. They share a `gps_ctx_t`.
- `fifo.c` — ring buffer of `NUM_FIFO_BUFFERS` IQ buffers connecting the GPS producer thread to the SDR consumer thread.
- `sdr.c` / `sdr.h` — backend-neutral SDR interface; `sdr_hackrf.c`, `sdr_pluto.c`, `sdr_iqfile.c` implement it, each with its own TX thread that drains the FIFO.
- `almanac.c` — reads SEM-format almanac (`almanac.sem`) and scales values into the subframe 4/5 pages (recent bug fixes were in this scaling).
- `gui.c` — curses/panel TUI: status windows and interactive position changes in real time.

Things that are easy to get wrong when changing the stream (found on real phones, not by unit tests):
- Ephemeris and almanac in the stream must agree (they differ by 1-2 km for fresh data). `-s now` therefore only shifts a nav file's times to the start time when fewer than `COVER_MIN_SATS` satellites have a record within `EPH_MAX_AGE` of it (an old file); shifting fresh data put the satellites ~26,000 km away from the almanac and phones distrusted them.
- Each channel sends one almanac page (subframe 4/5, 25 pages) per 30 s frame, so a receiver gets a given PRN's almanac about `30 * (page index)` s after the start and everything after 12.5 minutes. Subframe 4 page 18 (iono/UTC/leap seconds) first airs after about 8.5 minutes.
- Missing iono/UTC/leap data in a RINEX header is filled by `ionoDefaults` with neutral values (leap seconds `LEAP_DEFAULT_DTLS`, 18 s; update it when a new leap second is introduced). Groups the file provides are kept and applied to the simulated delays too.
- The simulator models a simple ionosphere delay and no troposphere; phones subtract a troposphere delay the signal does not have, so their height is some metres too high.
- `-s now` is the system clock (UTC) plus the leap seconds, read after the download; a time given with `-s` is taken as GPS time, like the RINEX epochs.

Data flow: RINEX/almanac → `gps.c` channel + nav-message state → IQ buffers → FIFO → SDR TX thread. Position comes from `--geo-loc`, a motion file (`-m`, e.g. `circle.csv`), `--target` or TUI input.

The working directory also collects untracked build artifacts (`*.o`, `gps-sim`) and data files (`*.gz` RINEX files, `almanac.sem`, `iqdata.bin`); downloads with `-f` overwrite `rinex2.gz`, `rinex3.gz` and `almanac.sem` there. `circle.csv` is a tracked sample motion file for `-m`.
