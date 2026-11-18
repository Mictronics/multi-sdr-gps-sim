# multi-sdr-gps-sim

multi-sdr-gps-sim generates a GPS L1 baseband signal IQ data stream, which is then transmitted by a
software-defined radio (SDR) platform. Supported at the moment are HackRF, ADLAM-Pluto and binary IQ file output.
The software interacts with the user through a curses based text user interface (TUI) in terminal.

Project based on work of [Takuji Ebinuma](https://github.com/osqzss/) and [IvanKor](https://github.com/IvanKor).

### Generating the GPS signal

The user is able to assign a static location directly through the command line or sends motion data through a predefined input file.
In addition the simulated position can be modified in realtime by user inputs.

The user also specifies the GPS satellite constellation through a GPS broadcast ephemeris file. The daily GPS broadcast ephemeris file
is a merge of the individual site navigation files into one.

These files are then used to generate the simulated pseudorange and Doppler for the GPS satellites in view. This simulated range data is
then used to generate the digitized I/Q samples for the GPS signal which are then feed into ADALM-Pluto SDR to transmit the GPS L1 frequency.

The simulation start time can be specified if the corresponding set of ephemerides is available. Otherwise the first time of ephemeris in the RINEX navigation file
is selected. RINEX input format 2 and 3 are supported.

__Note__

The SDR internal oscillator must be precise enough (frequency offset) and stable (temperature drift) for GPS signal generation.
Ideally the SDR shall be clocked from either by TCXO or more expensive (but highest precision) OCXO.

Best experience with HackRF.

### Build instructions

#### Dependencies

Build depends on libm, libpthread, libcurl4-openssl-dev, libncurses, libz and libhackrf.

You will also need the latest build and install of libhackrf, libad9361-dev and libiio-dev. The Debian packages
libad9361-dev that is available up to Debian 9 (stretch) is outdated and missing a required function.

So you have to build packages from source:

The first step is to fetch the dependencies, which as of now is only libxml2. On a Debian-flavoured GNU/Linux distribution, like Ubuntu for instance:

```
$ sudo apt-get install libxml2 libxml2-dev bison flex libcdk5-dev cmake
```

Depending on the backend (how you want to attach the IIO device), you may need at least one of:

```
$ sudo apt-get install libaio-dev libusb-1.0-0-dev libserialport-dev libxml2-dev libavahi-client-dev doxygen graphviz
```

Then build in this order:

```
$ git clone https://github.com/analogdevicesinc/libiio.git
$ cd libiio
$ cmake ./
$ make
$ sudo make install
```

```
$ git clone https://github.com/analogdevicesinc/libad9361-iio.git
$ cd libad9361-iio
$ cmake ./
$ make
$ sudo make install
```

#### Building under Linux with GCC

```
$ git clone https://github.com/mictronics/multi-sdr-gps-sim
$ cd multi-sdr-gps-sim
```

With HackRF support: `make all HACKRFSDR=yes` (depends on libhackrf)

With ADLAM-PLUTO support: `make all PLUTOSDR=yes` (depends on libiio and libad9361-iio)

Full SDR support: `make all HACKRFSDR=yes PLUTOSDR=yes`

### Online data sources

With `-f` the program downloads the ephemeris and almanac over HTTPS, no login needed:

- RINEX 2 (default): daily GPS broadcast file from the NOAA CORS public bucket (`noaa-cors-pds`), updated about hourly.
- RINEX 3 (`-3`): IGS daily merged broadcast file from BKG (`igs.bkg.bund.de`), published the morning after the day ends.
- Almanac: current SEM almanac from Celestrak (`celestrak.org`).

The files are saved as `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.

The NOAA RINEX 2 files have no leap second line in their header. Whatever ionosphere, UTC or leap second data a file lacks is replaced by neutral values (no ionosphere correction, UTC following from the leap seconds, currently 18 s), so subframe 4 page 18 is always complete. Data that the file does provide is used as it is, and the same Klobuchar parameters that are transmitted are applied to the simulated delays.

### Usage

````
gps-sim [options]
Options:
--nav-file          -e  <filename> RINEX navigation file for GPS ephemeris (required)
--geo-loc           -l  <location> Latitude, Longitude, Height (static mode) e.g. 35.681298,139.766247,10.0
--start             -s  <date,time> Scenario start time YYYY/MM/DD,hh:mm:ss (use 'now' for actual time, an old nav file is shifted to it)
--gain              -g  <gain> Set TX gain, HackRF: 0-47dB, Pluto: -80-0dB (default 0)
--duration          -d  <seconds> Duration in seconds
--target            -t  <distance,bearing,height> Target distance [m], bearing [°] and height [m]
--ppb               -p  <ppb> Set oscillator error in ppb (default 0)
--radio             -r  <name> Set the SDR device type name (default none)
--uri               -U  <uri> ADLAM-Pluto URI
--network           -N  <network> ADLAM-Pluto network IP or hostname (default pluto.local)
--motion            -m  <name> User motion file (dynamic mode)
--iq16                  Set IQ sample size to 16 bit (default 8 bit)
--disable-iono      -I  Disable ionospheric delay for spacecraft scenario
--verbose           -v  Show verbose output and details about simulated channels
--interactive       -i  Use interactive mode
--amplifier         -a  Enable TX amplifier (default OFF)
--use-ftp           -f  Pull actual RINEX navigation file and almanac from online sources
--rinex3            -3  Use RINEX v3 navigation data format
--disable-almanac       Disable transmission of almanac information
--help              -?  Give this help list
--usage                 Give a short usage message
--version           -V  Print program version

SDR device types (use with --radio or -r option):
    none
    iqfile
    hackrf
    plutosdr
````

### Hardware test and practical notes

Tested with a HackRF One (firmware 2026.01.3) at 3 MS/s, TX gain 14 dB, amplifier off, the output going through a cable with 60 dB of attenuation to a Pixel 4 phone. All simulated satellites were tracked at 30 to 45 dB-Hz, the phone locked within a minute with almanac, ephemeris and "used in fix" flags on every satellite, reported the commanded position to about 1.3 m horizontally and the correct time. Its height was about 15 m low: the simulator only models a simple ionosphere delay and no troposphere, and phones apply their own models.

```
mkdir -p ~/gps-tx && cd ~/gps-tx
gps-sim -f -s now -l 48.0,10.0,500 -r hackrf -g 14 -d 1800
```

- Run it from a separate directory: `-f` overwrites `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.
- With `-s now` and a fresh download the ephemeris is used as it is, so the simulated satellites are the ones the almanac puts above the horizon at that place. An old nav file is shifted to the current time instead, which makes the simulated satellites disagree with a real almanac; receivers that use assistance data may then distrust them. Use fresh data for receiver tests.
- Keep the gain constant for the first minutes. A cold receiver needs 30 to 60 s of signal for the ephemeris and about 12.5 minutes for the whole almanac. In the program `t` and `g` raise and lower the gain by 1 dB and `x` quits.
- Leave the amplifier (`-a`) off.

Transmitting on the GPS L1 frequency is regulated in most countries and disturbs every receiver within reach. Keep the output conducted (cable and attenuation, a DC block if the receiver powers an active antenna), use the lowest gain that works, keep runs short and do not attach an antenna. Phones have no antenna connector, so even a cabled setup radiates a little.

### License

Copyright &copy; 2021 Mictronics
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
