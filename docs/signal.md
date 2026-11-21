# The simulated signal

[Back to the README](../README.md)

## Overview

The simulator computes the GPS L1 C/A signal (1575.42 MHz) as complex baseband IQ at 3 MS/s and streams it to the SDR while it runs. Every 0.1 s it computes, for each GPS satellite above the horizon (0° elevation, up to 12 channels), the pseudorange and Doppler from the broadcast ephemeris and the user position, and generates the code, the navigation message and the carrier from them. Up to three SBAS GEO signals are added the same way (see [SBAS](#sbas)).

The user position is static (`-l`, optionally moved by `-t distance,bearing,height`), follows a motion file (`-m`), or is steered from the keyboard in interactive mode (`-i`, see the key table in the README).

## Navigation data and start time

The satellite orbits come from a RINEX 2 or 3 navigation file (`-e`, may be gzipped; `-3` for RINEX 3), the almanac (subframes 4 and 5) from a SEM file `almanac.sem` in the working directory. An almanac more than 4 weeks away from the scenario time is rejected; use `--disable-almanac` with old nav files.

- `-s YYYY/MM/DD,hh:mm:ss` is taken as GPS time, like the RINEX epochs, and must lie within the file's records. The ephemeris record closest in time (within 2 hours) is used for each satellite, at the start and again every 30 s.
- `-s now` is the system clock (UTC) plus the leap seconds. A nav file that has records for fewer than 4 satellites within 2 hours of that time is shifted to it; the satellites then no longer match a real almanac, so use fresh data for receiver tests.
- Without `-s` the first epoch of the file is used.

### Online data sources

With `-f` the program downloads the ephemeris and almanac over HTTPS, no login needed:

- RINEX 2 (default): daily GPS broadcast file from the NOAA CORS public bucket (`noaa-cors-pds`), updated about hourly, though its records can lag several hours behind. With `-s now`, a file that does not reach the start time is replaced by the RINEX 3 file below.
- RINEX 3 (`-3`): merged broadcast file of the day from BKG (`igs.bkg.bund.de`): `BRDC00WRD_S`, built from real-time streams and reaching the current hour, then `BRDC00WRD_R`, then the previous days' files.
- Almanac: current SEM almanac from the US Coast Guard (`navcen.uscg.gov`), then Celestrak (`celestrak.org`), then NAVCEN's daily archive for today and up to three days back. Each source gets 20 s and two retries; a reply that is no SEM almanac (an HTML error page, fewer than 24 satellites) is skipped. If every source fails, the `almanac.sem` of an earlier download is used. The status window shows which source was used.

The files are saved as `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.

The NOAA RINEX 2 files have no leap second line in their header. Whatever ionosphere, UTC or leap second data a file lacks is replaced by neutral values (no ionosphere correction, UTC following from the leap seconds, currently 18 s), so subframe 4 page 18 is always complete. Data that the file does provide is used as it is, and the same Klobuchar parameters that are transmitted are applied to the simulated delays.

## Propagation

The signal carries the delays a receiver corrects for: the broadcast (Klobuchar) ionosphere with the parameters that are also transmitted, and a troposphere delay (Saastamoinen zenith delay with a standard atmosphere, Black-Eisner mapping). `--disable-iono` turns off both, e.g. for a spacecraft scenario.

## Signal level (--cn0)

Without `--cn0` the signal carries no noise, so the C/N0 a receiver reports depends on the TX gain, the cable and the receiver. With `--cn0 45` the simulator adds its own noise floor and sets every satellite relative to it: 45 dB-Hz for a satellite overhead, less for lower ones (path loss and antenna pattern, about 34 dB-Hz at the horizon). SBAS GEOs follow the same model.

The receiver then reports the requested values once the simulated noise is well above its own noise. To set it up, start at a low gain and raise it step by step (`t` in the program) until the receiver's C/N0 stops rising; from there on a higher gain changes nothing but the RF level, so stay at the lowest gain that reaches it. The output is reproducible: the same `-e`, `-s` and `--cn0` give the same `iqdata.bin`.

Measured results per SDR are in [hardware](hardware.md#signal-level---cn0).

## SBAS

Besides the GPS satellites the simulator sends SBAS GEO signals on L1 (250 bit/s, rate 1/2 convolutional code, PRN 120-158). By default it picks the system whose GEO stands highest at the user position and sends up to three of its GEOs from the table in `sbas.c` (at 48° N 10° E EGNOS 150, 136 and 121), each at rest in its orbital slot. `--sbas 136,5.0` sends only the given PRN at the given longitude, `--disable-sbas` sends none (the output is then the same as without SBAS support).

The messages carry no real corrections, but everything a receiver needs to use the service:

- PRN mask for GPS 1-32 (MT1), fast corrections of zero with UDRE 3 m for satellites with ephemeris and "not monitored" for the others (MT2-4), long-term corrections of zero with the current IODEs (MT25), degradation parameters (MT7, MT10).
- GEO navigation message (MT9, not for ranging) and an almanac of the GEOs being sent (MT17).
- Ionospheric grid around the user (MT18, MT26), computed from the same Klobuchar model that delays the simulated signal, so a receiver's SBAS iono correction matches the delay it sees.

Results with a u-blox NEO-M8T are in [hardware](hardware.md#sbas).

## Files

### Motion file (`-m`)

A CSV text file with one line per 0.1 s: `time, x, y, z`, the position in ECEF metres (the time column is not used, the lines are taken as consecutive 0.1 s steps). The run ends at the last line or after `-d` seconds, whichever comes first; up to 864000 lines (24 hours). `circle.csv` in the repository is an example:

```
  0.0,-3813477.954, 3554276.552, 3662785.237
  0.1,-3813477.599, 3554276.226, 3662785.918
```

A motion file switches off interactive mode.

### IQ file (`-r iqfile`)

`iqdata.bin` in the working directory: interleaved I and Q samples at 3 MS/s, signed 8 bit by default or signed 16 bit in the machine's byte order (little-endian on x86 and ARM) with `--iq16`. One second of signal is 6 MB (8 bit) or 12 MB (16 bit). The output for a given nav file, `-s` time and options is reproducible, which the [regression check](development.md) relies on.
