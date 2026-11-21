# Building

[Back to the README](../README.md)

## Dependencies

Always needed: a C11 compiler, make, libm, pthread, libcurl, zlib and ncurses (with its panel library). On Debian or Ubuntu:

```
$ sudo apt install build-essential libcurl4-openssl-dev zlib1g-dev libncurses-dev
```

Per SDR, only for the backends you build:

| Backend | Make flag | Library |
|---|---|---|
| HackRF | `HACKRFSDR=yes` | libhackrf (`libhackrf-dev`) |
| ADALM-Pluto | `PLUTOSDR=yes` | libiio and libad9361-iio, see below |
| LimeSDR (Mini) | `LIMESDR=yes` | LimeSuiteNG `develop` built from source, see below |

The IQ file output (`-r iqfile`) needs no SDR library and is always built.

## Build

```
$ git clone https://github.com/mictronics/multi-sdr-gps-sim
$ cd multi-sdr-gps-sim
$ make all HACKRFSDR=yes PLUTOSDR=yes LIMESDR=yes   # leave out the flags of SDRs you do not have
```

Changing a flag rebuilds everything that depends on it. `make OPT=-O0` builds for debugging, `make test` runs the unit tests (see [development](development.md)), `make clean` removes the build.

## libiio and libad9361-iio for the ADALM-Pluto

Distribution packages (`libiio-dev`, `libad9361-dev`) work if they are recent enough; the `libad9361-dev` of Debian 9 (stretch) and older lacks a function the simulator needs. Otherwise build both from source.

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

## LimeSuiteNG for LimeSDR (built from source)

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
