# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```
make all HACKRFSDR=yes PLUTOSDR=yes   # either flag optional; iqfile backend is always built
make clean
```

Flat Makefile, C11, no tests or linter. Deps: libm, pthread, libcurl, libz, ncurses/panel, plus libhackrf and/or libiio + libad9361 when the matching SDR flag is set. Backends are compiled in via `-DENABLE_HACKRFSDR` / `-DENABLE_PLUTOSDR`.

The Makefile has no header dependency tracking and does not rebuild when `HACKRFSDR`/`PLUTOSDR` change; run `make clean` when switching flags.

Run example: `./gps-sim -e brdc1320.21n.gz -l 35.68,139.76,10 -r iqfile` (see README for all options; `help.h` holds the usage text). `-r` selects the SDR backend (`none|iqfile|hackrf|plutosdr`). Nav files may be gzipped RINEX 2 or 3 (`-3`). `-f` downloads the nav file (NOAA CORS for RINEX 2, BKG for RINEX 3) and the Celestrak almanac over HTTPS instead.

The program needs a terminal of at least 120x26 and writes `iqdata.bin` in the working directory with `-r iqfile`. The local sample `brdc1320.21n.gz` (2021-05-12) matches the local `almanac.sem` (GPS week 2157); a current almanac is rejected as too far from the scenario time, so use `--disable-almanac` with old nav files. Neither file is tracked by git.

## Architecture

Derived from Takuji Ebinuma's gps-sdr-sim. The program synthesizes GPS L1 C/A baseband IQ on the fly and streams it to an SDR, with an ncurses TUI.

- `gps-sim.c` — `main`, option parsing, and the global `simulator` struct (`gps-sim.h`) shared across threads. Starts the GPS thread, then the selected SDR backend and the TUI.
- `gps.c` (largest file) — the simulation core: downloads (`download_file`), RINEX parsing (`readRinex` handles v2 and v3, sorts records by time so satellite-ordered merged files work), ephemeris/almanac handling, satellite channel allocation, pseudorange/Doppler computation, navigation message (subframe) generation, and IQ sample generation. `gps_thread_ep` is the producer thread; each iteration produces 0.1 s of samples (`NUM_IQ_SAMPLES` at 3 MS/s, see `sdr.h`).
- `fifo.c` — ring buffer of `NUM_FIFO_BUFFERS` IQ buffers connecting the GPS producer thread to the SDR consumer thread.
- `sdr.c` / `sdr.h` — backend-neutral SDR interface; `sdr_hackrf.c`, `sdr_pluto.c`, `sdr_iqfile.c` implement it, each with its own TX thread that drains the FIFO.
- `almanac.c` — reads SEM-format almanac (`almanac.sem`) and scales values into the subframe 4/5 pages (recent bug fixes were in this scaling).
- `gui.c` — curses/panel TUI: status windows and interactive position changes in real time.

Data flow: RINEX/almanac → `gps.c` channel + nav-message state → IQ buffers → FIFO → SDR TX thread. Position comes from `--geo-loc`, a motion file (`-m`, e.g. `circle.csv`), `--target` or TUI input.

The working directory also collects untracked build artifacts (`*.o`, `gps-sim`) and data files (`*.gz` RINEX files, `almanac.sem`, `circle.csv`, `iqdata.bin`); downloads with `-f` overwrite `rinex2.gz`, `rinex3.gz` and `almanac.sem`.
