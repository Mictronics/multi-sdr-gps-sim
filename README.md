# multi-sdr-gps-sim

multi-sdr-gps-sim generates a GPS L1 baseband signal IQ data stream, which is then transmitted by a
software-defined radio (SDR) platform. Supported at the moment are HackRF, ADLAM-Pluto, LimeSDR (Mini) and binary IQ file output.
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

Best results with an SDR clocked by a TCXO or OCXO; see the comparison under *Hardware test and practical notes*.

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

#### LimeSuiteNG for LimeSDR (built from source)

The LimeSDR backend uses LimeSuiteNG's LimeSuite API wrapper (`lime/LimeSuite.h`, `liblimesuiteng-legacyapi`).
The LimeSuiteNG 25.1 packages from repo.myriadrf.org do not transmit on a LimeSDR Mini v1, not even with their
own `limeTRX` (the host sends packets, none reach the FPGA), so build the `develop` branch. Tested with commit 4032e99
on Ubuntu 24.04.

1. Remove the LimeSuiteNG packages (if installed) but keep their udev rules, so only one copy of the library exists:

```
$ sudo apt-mark manual limesuiteng-udev
$ sudo apt remove limesuiteng liblimesuiteng limesuiteng-legacyapi limesuiteng-dev limesuiteng-cli \
      limesuiteng-gui limesuiteng-soapysdr gr-limesuiteng limepcie-dkms
```

The old LimeSuite (`liblimesuite-dev`) cannot stay either, it installs the same `lime/LimeSuite.h`.

2. Build dependencies: the GUI needs wxWidgets, the GNU Radio blocks `gnuradio-dev`, `pybind11-dev` and
`python3-pygccxml`. GLEW is downloaded by the build when it is missing.

```
$ sudo apt install git cmake g++ libusb-1.0-0-dev libwxgtk3.2-dev gnuradio-dev pybind11-dev python3-pygccxml
```

3. Clone (a full clone, so a known commit can be checked out again), configure, build:

```
$ cd ~/git
$ git clone -b develop https://github.com/myriadrf/LimeSuiteNG.git
$ cd LimeSuiteNG
$ git checkout 4032e99        # optional: the tested commit
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
      -DBUILD_GUI=ON -DBUILD_PLUGINS=ON -DBUILD_PLUGINS_GNURADIO=ON \
      -DBUILD_PLUGINS_SOAPYSDR=OFF -DBUILD_PLUGINS_AMARISOFT=OFF \
      -DBUILD_DRIVERS=OFF -DBUILD_DOCS=OFF -DINSTALL_UDEV_RULES=OFF \
      -DGR_PYTHON_DIR=/usr/lib/python3/dist-packages
$ cmake --build build -j$(nproc)
```

- `CMAKE_INSTALL_PREFIX=/usr`: GNU Radio Companion and Python only find the blocks there without extra paths.
- `GR_PYTHON_DIR`: without it the Python module goes to `/usr/lib/python3.12/site-packages`, which Ubuntu's Python
  does not search, and `import gnuradio.limesuiteng` fails.
- `INSTALL_UDEV_RULES=OFF` because `limesuiteng-udev` provides them; without that package use `ON`.
- `BUILD_DRIVERS=OFF`: the PCIe kernel driver is not needed for USB boards.

4. Install and check:

```
$ sudo cmake --install build
$ sudo ldconfig
$ limeDevice --full                                   # lists the board, firmware and gateware
$ python3 -c "import gnuradio.limesuiteng; print('ok')"
```

5. Rebuild gps-sim against it (`make clean; make all LIMESDR=yes ...`) and check with `ldd gps-sim | grep lime`
that it loads `/usr/lib/liblimesuiteng-legacyapi.so.1.0.0`.

To update later: `git pull` (or check out another commit), run `cmake --build build` and the install again. To remove the
installed files: `sudo xargs rm < build/install_manifest.txt && sudo ldconfig` (the manifest lists only the files of the
last install).

A quick TX check without gps-sim, sending zeros (only LO leakage), into a contained setup only:

```
$ head -c 12000000 /dev/zero > zeros.bin
$ limeConfig --initialize --samplerate=3e6 --rxen=1 --rxlo=1575.42e6 --txen=1 --txlo=1575.42e6 --txpath=Band1 --txlpf=6e6
$ limeTRX -i zeros.bin --looptx --time=5000 -l debug 2>&1 | grep -E "Tx0|rror"
```

A working TX shows `USB ep:03 Tx0: 9.0 MB/s` lines with `u:0` (no underruns) and a growing `dma:` count. The final
`FPGA packet ingresed:` count stayed 0 on the tested Mini v1 even with TX working, so do not judge by it.

#### Building under Linux with GCC

```
$ git clone https://github.com/mictronics/multi-sdr-gps-sim
$ cd multi-sdr-gps-sim
```

With HackRF support: `make all HACKRFSDR=yes` (depends on libhackrf)

With ADLAM-PLUTO support: `make all PLUTOSDR=yes` (depends on libiio and libad9361-iio)

With LimeSDR support: `make all LIMESDR=yes` (depends on LimeSuiteNG built from source, see above). Tested with LimeSuiteNG commit 4032e99, LimeSDR Mini v1 (gateware 1.30): a Pixel 4 and a Samsung A33 lock at `-g 10` (Pixel from `-g 5`, 25-35 dB-Hz) over a cable with 60 dB of attenuation. The Pixel's fix was 0.5 m south, 0.1 m west and 0.8 m high of the commanded 48.0, 10.0, 500 m; the A33 tracks less stably and less accurately on the LimeSDR than on the HackRF or Pluto. The Mini runs from its VCTCXO (about 1.4 ppm off here); leave `-p` at 0, the simulator keeps code and carrier consistent with it.

Full SDR support: `make all HACKRFSDR=yes PLUTOSDR=yes LIMESDR=yes`

### Online data sources

With `-f` the program downloads the ephemeris and almanac over HTTPS, no login needed:

- RINEX 2 (default): daily GPS broadcast file from the NOAA CORS public bucket (`noaa-cors-pds`), updated about hourly, though its records can lag several hours behind. With `-s now`, a file that does not reach the start time is replaced by the RINEX 3 file below.
- RINEX 3 (`-3`): merged broadcast file of the day from BKG (`igs.bkg.bund.de`): `BRDC00WRD_S`, built from real-time streams and reaching the current hour, then `BRDC00WRD_R`, then the previous days' files.
- Almanac: current SEM almanac from the US Coast Guard (`navcen.uscg.gov`), then Celestrak (`celestrak.org`), then NAVCEN's daily archive for today and up to three days back. Each source gets 20 s and two retries; a reply that is no SEM almanac (an HTML error page, fewer than 24 satellites) is skipped. If every source fails, the `almanac.sem` of an earlier download is used. The status window shows which source was used.

The files are saved as `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.

The NOAA RINEX 2 files have no leap second line in their header. Whatever ionosphere, UTC or leap second data a file lacks is replaced by neutral values (no ionosphere correction, UTC following from the leap seconds, currently 18 s), so subframe 4 page 18 is always complete. Data that the file does provide is used as it is, and the same Klobuchar parameters that are transmitted are applied to the simulated delays.

### Usage

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
--radio             -r  <name> Set the SDR device type name (default none)
--uri               -U  <uri> ADLAM-Pluto URI
--network           -N  <network> ADLAM-Pluto network IP or hostname (default pluto.local)
--motion            -m  <name> User motion file (dynamic mode)
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
    none
    iqfile
    hackrf
    plutosdr
    limesdr
````

### Signal level (--cn0)

Without `--cn0` the signal carries no noise, so the C/N0 a receiver reports depends on the TX gain, the cable and the receiver. With `--cn0 45` the simulator adds its own noise floor and sets every satellite relative to it: 45 dB-Hz for a satellite overhead, less for lower ones (path loss and antenna pattern, about 34 dB-Hz at the horizon). SBAS GEOs follow the same model.

The receiver then reports the requested values once the simulated noise is well above its own noise. To set it up, start at a low gain and raise it step by step (`t` in the program) until the receiver's C/N0 stops rising; from there on a higher gain changes nothing but the RF level, so stay at the lowest gain that reaches it. The output is reproducible: the same `-e`, `-s` and `--cn0` give the same `iqdata.bin`.

Measured with an ADALM-Pluto (OCXO mod) over the 60 dB cable into a u-blox NEO-M8T, `--cn0 45`: each 5 dB of gain brought the reported C/N0 closer to the requested one (-12.0 dB at `-g -45`, -7.7 at -40, -4.4 at -35, -2.6 at -30, -1.9 at -25), so `-g -25` is the working point on that setup, about 20 dB above the gain without noise. The elevation shape matched within ±0.3 dB on every satellite. At `-g -25` the receiver reported less than requested by:

| `--cn0` | Reported - requested |
|---|---|
| 30 | -0.9 dB (4 satellites tracked) |
| 40 | -1.6 dB |
| 45 | -1.9 dB |
| 50 | -2.8 dB |

The offset is about 1.5 dB of loss (receiver and the 3 MS/s signal) plus, at high levels, the other satellites' signals adding to the noise. Expect similar values from other receivers, not exact ones.

### SBAS

Besides the GPS satellites the simulator sends SBAS GEO signals on L1 (250 bit/s, rate 1/2 convolutional code, PRN 120-158). By default it picks the system whose GEO stands highest at the user position and sends up to three of its GEOs from the table in `sbas.c` (at 48° N 10° E EGNOS 150, 136 and 121), each at rest in its orbital slot. `--sbas 136,5.0` sends only the given PRN at the given longitude, `--disable-sbas` sends none (the output is then the same as without SBAS support).

The messages carry no real corrections, but everything a receiver needs to use the service:

- PRN mask for GPS 1-32 (MT1), fast corrections of zero with UDRE 3 m for satellites with ephemeris and "not monitored" for the others (MT2-4), long-term corrections of zero with the current IODEs (MT25), degradation parameters (MT7, MT10).
- GEO navigation message (MT9, not for ranging) and an almanac of the GEOs being sent (MT17).
- Ionospheric grid around the user (MT18, MT26), computed from the same Klobuchar model that delays the simulated signal, so a receiver's SBAS iono correction matches the delay it sees.

Tested with a u-blox NEO-M8T over a cable with 60 dB of attenuation and a DC block, HackRF at `-g 20`, amplifier off. The receiver decodes every message type, reports the EGNOS service and gets an SBAS fix. On the Pluto (OCXO) the SBAS fix was 0.18 m horizontally against 0.10 m GPS only (height +1.4 m vs +1.0 m): with zero corrections SBAS cannot improve the fix; the small height difference comes from the receiver taking the ionosphere from the grid instead of Klobuchar. Its iono correction agrees with the simulated delay within 0.35 m per satellite. A cold receiver searches one GEO at first and adds the others within a minute of their MT17. Synchronising to the first GEO took from a few seconds to two minutes after a cold start. Phones track the GEOs (a Pixel 4 at about 45 dB-Hz) but Android does not show whether SBAS is used.

### Hardware test and practical notes

Tested with a HackRF One (firmware 2026.01.3) at 3 MS/s, TX gain 14 dB, amplifier off, the output going through a cable with 60 dB of attenuation to a Pixel 4 phone. All simulated satellites were tracked at 30 to 45 dB-Hz, the phone locked within a minute with almanac, ephemeris and "used in fix" flags on every satellite, reported the commanded position to about 1.3 m horizontally and the correct time. Its height was about 15 m low: the simulator then modelled no troposphere while phones correct for one (a troposphere delay has been added since).

```
mkdir -p ~/gps-tx && cd ~/gps-tx
gps-sim -f -s now -l 48.0,10.0,500 -r hackrf -g 14 -d 1800
```

All three SDRs were compared on a u-blox NEO-M8T over the same cable (60 dB, DC block), GPS only, 5 minutes each from a cold start. Residual is the receiver's pseudorange minus the simulated one at the commanded position, per satellite, with the receiver clock removed:

| SDR | Setting | C/N0 | Fix error (horizontal, height) | Residuals |
|---|---|---|---|---|
| ADALM-Pluto (OCXO mod) | `-g -45` | 32-42 dB-Hz | 0.2 m, +1.8 m | -1.2 to +0.8 m |
| LimeSDR Mini v1 (stock VCTCXO) | `-g 10` | 36-45 dB-Hz | 0.5 m, +0.7 m | -1.4 to +2.5 m |
| HackRF One (OCXO mod) | `-g 20` | 37-47 dB-Hz | 1.7 m, +3.4 m | -4.6 to +9.6 m |

The Pluto and the HackRF used here are modified with an OCXO; a stock unit's crystal is less precise and less stable (see the note at the top), so expect them to do worse without such a mod or an external reference. The LimeSDR Mini ran from its own VCTCXO.

On all three, weak low satellites read slightly long and strong high ones slightly short, most on the HackRF. A badly seated SMA connector once cost about 10 dB and gave the HackRF errors of up to 34 m (a 15-21 m fix error): if the level drops at an unchanged gain, reseat the connectors before looking for other causes.

- Run it from a separate directory: `-f` overwrites `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.
- With `-s now` and a fresh download the ephemeris is used as it is, so the simulated satellites are the ones the almanac puts above the horizon at that place. An old nav file is shifted to the current time instead, which makes the simulated satellites disagree with a real almanac; receivers that use assistance data may then distrust them. Use fresh data for receiver tests.
- Keep the gain constant for the first minutes. A cold receiver needs 30 to 60 s of signal for the ephemeris and about 12.5 minutes for the whole almanac. In the program `t` and `g` raise and lower the gain by 1 dB and `x` quits.
- Leave the amplifier (`-a`) off.

Transmitting on the GPS L1 frequency is regulated in most countries and disturbs every receiver within reach. Keep the output conducted (cable and attenuation, a DC block if the receiver powers an active antenna), use the lowest gain that works, keep runs short and do not attach an antenna. Phones have no antenna connector, so even a cabled setup radiates a little.

### License

Copyright &copy; 2026 Mictronics
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
