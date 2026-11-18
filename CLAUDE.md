# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```
make all HACKRFSDR=yes PLUTOSDR=yes   # either flag optional; iqfile backend is always built
make clean
```

Flat Makefile, C11, no tests or linter. Deps: libm, pthread, libcurl, libz, ncurses/panel, plus libhackrf and/or libiio + libad9361 when the matching SDR flag is set. Backends are compiled in via `-DENABLE_HACKRFSDR` / `-DENABLE_PLUTOSDR`.

Note: the Makefile's link line lists `fifo.o almanac.o gps.o gui.o sdr.o gps-sim.o` and does not include `subframe.o` (new, untracked `subframe.c/h`, declares `gps_decode_subframe_raw`). Add it to the `gps-sim:` prerequisites if it gets used.

Run example: `./gps-sim -e brdc1320.21n.gz -l 35.68,139.76,10 -r iqfile` (see README for all options; `help.h` holds the usage text). `-r` selects the SDR backend (`none|iqfile|hackrf|plutosdr`). Nav files may be gzipped RINEX 2 or 3 (`-3`).

## Architecture

Derived from Takuji Ebinuma's gps-sdr-sim. The program synthesizes GPS L1 C/A baseband IQ on the fly and streams it to an SDR, with an ncurses TUI.

- `gps-sim.c` — `main`, option parsing, and the global `simulator` struct (`gps-sim.h`) shared across threads. Starts the GPS thread, then the selected SDR backend and the TUI.
- `gps.c` (largest file) — the simulation core: RINEX parsing, ephemeris/almanac handling, satellite channel allocation, pseudorange/Doppler computation, navigation message (subframe) generation, and IQ sample generation. `gps_thread_ep` is the producer thread; each iteration produces 0.1 s of samples (`NUM_IQ_SAMPLES` at 3 MS/s, see `sdr.h`).
- `fifo.c` — ring buffer of `NUM_FIFO_BUFFERS` IQ buffers connecting the GPS producer thread to the SDR consumer thread.
- `sdr.c` / `sdr.h` — backend-neutral SDR interface; `sdr_hackrf.c`, `sdr_pluto.c`, `sdr_iqfile.c` implement it, each with its own TX thread that drains the FIFO.
- `almanac.c` — reads SEM-format almanac (`almanac.sem`) and scales values into the subframe 4/5 pages (recent bug fixes were in this scaling).
- `gui.c` — curses/panel TUI: status windows and interactive position changes in real time.
- `subframe.c` — standalone decoder for raw subframe words (untracked, not yet wired in).

Data flow: RINEX/almanac → `gps.c` channel + nav-message state → IQ buffers → FIFO → SDR TX thread. Position comes from `--geo-loc`, a motion file (`-m`, e.g. `circle.csv`), `--target` or TUI input.

The repo root contains build artifacts (`*.o`, `gps-sim`) and sample data (`*.gz` RINEX files, `almanac.sem`, `circle.csv`).
