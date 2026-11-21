# multi-sdr-gps-sim

multi-sdr-gps-sim generates a GPS L1 C/A signal in real time and transmits it with a software-defined radio (SDR): HackRF One, ADALM-Pluto or LimeSDR (Mini), or writes it to an IQ file. A GPS receiver or phone connected to it gets a fix at the position you choose, static, along a motion file or steered from the keyboard. The signal includes the navigation message with almanac, ionosphere and troposphere delays, SBAS GEO signals and, optionally, a noise floor with a set C/N0. The program runs in a terminal with a text user interface (TUI).

Based on the work of [Takuji Ebinuma](https://github.com/osqzss/) and [IvanKor](https://github.com/IvanKor).

> **Transmitting on the GPS L1 frequency is regulated in most countries and disturbs every receiver within reach.** Keep the output conducted: a cable with attenuation (60 dB in all tests here), a DC block if the receiver powers an active antenna, no antenna on the SDR. Leave the amplifier (`-a`) off, use the lowest gain that works and keep runs short.

## Quick start

1. **Build** for your SDR (dependencies in [docs/building.md](docs/building.md)):
   ```
   $ make all HACKRFSDR=yes      # or PLUTOSDR=yes, LIMESDR=yes, or several
   ```
2. **Connect** the SDR's TX port through the attenuated cable to the receiver. A phone has no antenna connector: place it next to the cable.
3. **Run** from an empty directory (it downloads the current satellite data there) at your position:
   ```
   $ mkdir -p ~/gps-tx && cd ~/gps-tx
   $ ~/multi-sdr-gps-sim/gps-sim -f -s now -l 48.0,10.0,500 -r hackrf -g 14
   ```
   Start with the gain of the [working points](docs/hardware.md#working-points) (HackRF `-g 14`, Pluto `-g -50`, LimeSDR `-g 10` on a 60 dB cable), raise it with `t` until the receiver tracks the satellites, keep it constant for the first minutes and press `x` to quit. A cold receiver needs 30-60 s for a fix and about 12.5 minutes for the whole almanac.

The SDR's oscillator must be precise and stable enough for GPS: a TCXO or OCXO gives the best results (see the [SDR comparison](docs/hardware.md#sdr-comparison-on-a-u-blox-neo-m8t)).

## Usage

````
gps-sim [options]
Options:
--nav-file          -e  <filename> RINEX navigation file for GPS ephemeris (required)
--geo-loc           -l  <location> Latitude, Longitude, Height (static mode) e.g. 35.681298,139.766247,10.0
--start             -s  <date,time> Scenario start time YYYY/MM/DD,hh:mm:ss (use 'now' for actual time, an old nav file is shifted to it)
--gain              -g  <gain> Set TX gain, HackRF: 0-47dB, Pluto: -80-0dB, LimeSDR: 0-52dB (default 0)
--duration          -d  <seconds> Duration in seconds
--target            -t  <distance,bearing,height> Target distance [m], bearing [°] and height [m]
--ppb               -p  <ppb> Set oscillator error in ppb (default 0)
--radio             -r  <name> SDR device type, required (see the list below)
--uri               -U  <uri> ADLAM-Pluto URI
--network           -N  <network> ADLAM-Pluto network IP or hostname (default pluto.local)
--motion            -m  <name> User motion GPX track (dynamic mode)
--iq16                  Set IQ sample size to 16 bit (default 8 bit)
--disable-iono      -I  Disable ionospheric and tropospheric delay for spacecraft scenario
--verbose           -v  Show verbose output and details about simulated channels
--interactive       -i  Use interactive mode
--amplifier         -a  Enable TX amplifier (default OFF)
--use-ftp           -f  Pull actual RINEX navigation file and almanac from online sources
--rinex3            -3  Use RINEX v3 navigation data format
--disable-almanac       Disable transmission of almanac information
--disable-sbas          Disable the SBAS GEO signals
--no-tui                No terminal UI: print status to stderr and exit when the simulation ends
--message               <text> Send up to 22 characters (A-Z 0-9 space + - . ' : / ") as the GPS special message (subframe 4 page 17)
--cn0                   <dB-Hz> Add a noise floor and set the C/N0 of a satellite overhead, 20-55; lower ones are weaker (default: no noise). Raise the TX gain until the receiver's C/N0 stops rising
--sbas                  <prn,longitude> Use only this SBAS GEO, e.g. 136,5.0 (default: the GEOs of the highest system in view)
--help              -?  Give this help list
--usage                 Give a short usage message
--version           -V  Print program version

SDR device types (use with --radio or -r option):
    iqfile
    hackrf
    plutosdr
    limesdr
````

`-r iqfile` writes `iqdata.bin` instead of transmitting, `-m` takes a GPX track; both formats are described in [docs/signal.md](docs/signal.md#files). The terminal needs at least 120x31 characters, or use `--no-tui`.

### Keys

| Key | Action | Key | Action |
|---|---|---|---|
| `t` / `g` | TX gain +1 / -1 dB | `i` | info |
| `w` / `s` | vertical speed +1 / -1 m/s (interactive mode) | `h`, `?` | help |
| `d` / `a` | heading 5° right / left (interactive mode) | `F1`, `F2`, `F3`, `TAB` | switch display: setup, status, dynamic position, next |
| `e` / `q` | speed +1 / -1 km/h (interactive mode) | `x` | exit |

Interactive mode (`-i`) starts at the `-l` position (or the `-t` target) and moves it by the speed, heading and vertical speed set with the keys, every 0.1 s. A motion file switches it off.

## Documentation

- [docs/building.md](docs/building.md): dependencies, libiio/libad9361 and LimeSuiteNG from source
- [docs/signal.md](docs/signal.md): navigation data and start time, online data sources, delays, signal level (`--cn0`), SBAS, file formats
- [docs/hardware.md](docs/hardware.md): tested setups, working gains, SDR comparison, C/N0 and SBAS results, practical notes
- [docs/development.md](docs/development.md): code layout, unit tests, regression check

## License

Copyright &copy; 2026 Mictronics
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
