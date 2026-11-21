# Development

[Back to the README](../README.md)

## Code layout

| File | Content |
|---|---|
| `gps-sim.c` | `main`, options (`help.h` holds the usage text), key handling |
| `gps.c` | channels, code phase, sample generation, the GPS thread (`gps_thread_ep`) |
| `orbit.c` | vectors, GPS time, coordinate conversions, satellite position |
| `rinex.c` | RINEX 2/3 reader, ephemeris selection, missing iono/UTC defaults |
| `model.c` | ionosphere, troposphere, pseudorange |
| `navmsg.c` | navigation message subframes and parity |
| `download.c` | HTTPS downloads for `-f` |
| `almanac.c` | SEM almanac reader and download |
| `sbas.c` | SBAS messages, CRC-24Q, convolutional encoder |
| `fifo.c` | buffer between the GPS thread and the SDR thread |
| `sdr.c`, `sdr_*.c` | SDR interface and one backend per SDR |
| `gui.c` | terminal UI |

## Unit tests

```
$ make test
```

The tests need no SDR and no terminal (the TUI is stubbed in `tests/stubs.c`) and run from the repository root:

| Test | Checks |
|---|---|
| `test_nav` | navigation message bits against IS-GPS-200: health, almanac pages, page 25, special message, leap seconds, ephemeris rounding, TOW at the week end |
| `test_rinex` | `readRinex` and the ephemeris selection on a trimmed real IGS file (`tests/data/sample_rinex3.rnx`) and on layouts that broke it before |
| `test_fifo` | the FIFO loses no block |
| `test_model` | troposphere values and the first C/A chips |
| `test_sbas` | CRC-24Q, convolutional code, message fields and schedule, the ionospheric grid against RTKLIB's IGP bands |
| `test_almanac` | almanac download order, retries, rejected files, the cached `almanac.sem` (no network) |
| `test_signal` | no code phase jump between the 0.1 s blocks; noise sigma and C/N0 with `--cn0` |

## Regression check

The output for a given nav file and `-s` time is deterministic, so a change that is not meant to alter the signal must leave `iqdata.bin` byte-identical. Run headless before and after the change and compare checksums:

```
$ ./gps-sim --no-tui -e rinex2.gz -s 2026/11/21,06:00:00 -l 48.0,10.0,500 -d 70 -r iqfile
$ sha256sum iqdata.bin
```

`--no-tui` writes the status to stderr and exits once all samples are written; the file holds `-d`×10 − 1 blocks of 0.1 s. Check with and without `--disable-sbas` (which must match a build without SBAS) and with `--cn0` if the change touches the signal level. `-s` must lie within the nav file's records.

To run the TUI without a terminal, e.g. in a script: `script -qec "stty cols 130 rows 31; exec ./gps-sim ..." /dev/null`. The TUI needs at least 120x31 characters.
