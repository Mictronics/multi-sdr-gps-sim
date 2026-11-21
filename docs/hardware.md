# Hardware tests and practical notes

[Back to the README](../README.md)

All tests ran conducted: the SDR output through a cable with 60 dB of attenuation, a DC block in front of receivers that power an active antenna, the amplifier off, no antenna. The HackRF One and the ADALM-Pluto used here are modified with an OCXO; a stock unit's crystal is less precise and less stable, so expect them to do worse without such a mod or an external reference. The LimeSDR Mini v1 ran from its own VCTCXO (about 1.4 ppm off; leave `-p` at 0, code and carrier stay consistent with it).

## Working points

Gains at which receivers tracked over the 60 dB cable, without `--cn0`:

| SDR | Phones (Pixel 4, Samsung A33) | u-blox NEO-M8T |
|---|---|---|
| HackRF One | `-g 14`, 30-45 dB-Hz | `-g 20` |
| ADALM-Pluto | `-g -50`, 30-38 dB-Hz | `-g -45` |
| LimeSDR Mini v1 | `-g 10` (Pixel from `-g 5`, 25-35 dB-Hz) | `-g 10` |

Phones have no antenna connector; they pick up what leaks from the cable, so their numbers depend on where they lie.

## Phones

Tested with a HackRF One (firmware 2026.01.3) at 3 MS/s, TX gain 14 dB, amplifier off, the output going through a cable with 60 dB of attenuation to a Pixel 4 phone. All simulated satellites were tracked at 30 to 45 dB-Hz, the phone locked within a minute with almanac, ephemeris and "used in fix" flags on every satellite, reported the commanded position to about 1.3 m horizontally and the correct time. Its height was about 15 m low: the simulator then modelled no troposphere while phones correct for one (a troposphere delay has been added since).

```
mkdir -p ~/gps-tx && cd ~/gps-tx
gps-sim -f -s now -l 48.0,10.0,500 -r hackrf -g 14 -d 1800
```

With the LimeSDR the Pixel's fix was 0.5 m south, 0.1 m west and 0.8 m high of the commanded 48.0, 10.0, 500 m; the A33 tracks less stably and less accurately on the LimeSDR than on the HackRF or Pluto. The Samsung A33 shows no stable almanac flag even under the real sky; use a Pixel to check almanac changes.

## SDR comparison on a u-blox NEO-M8T

All three SDRs were compared on a u-blox NEO-M8T over the same cable (60 dB, DC block), GPS only, 5 minutes each from a cold start. Residual is the receiver's pseudorange minus the simulated one at the commanded position, per satellite, with the receiver clock removed:

| SDR | Setting | C/N0 | Fix error (horizontal, height) | Residuals |
|---|---|---|---|---|
| ADALM-Pluto (OCXO mod) | `-g -45` | 32-42 dB-Hz | 0.2 m, +1.8 m | -1.2 to +0.8 m |
| LimeSDR Mini v1 (stock VCTCXO) | `-g 10` | 36-45 dB-Hz | 0.5 m, +0.7 m | -1.4 to +2.5 m |
| HackRF One (OCXO mod) | `-g 20` | 37-47 dB-Hz | 1.7 m, +3.4 m | -4.6 to +9.6 m |

On all three, weak low satellites read slightly long and strong high ones slightly short, most on the HackRF. A badly seated SMA connector once cost about 10 dB and gave the HackRF errors of up to 34 m (a 15-21 m fix error): if the level drops at an unchanged gain, reseat the connectors before looking for other causes.

## Signal level (--cn0)

With `--cn0` the reported C/N0 only reaches the requested value once the simulated noise is well above the receiver's own noise, so the gain has to be raised until the C/N0 stops rising. Ramps at `--cn0 45` over the 60 dB cable into a cold u-blox NEO-M8T, 150 s per step, mean of the last 60 s over 10-12 satellites (reported minus requested, the elevation shape matched within ±0.4 dB on every satellite):

| Gain above the no-noise point | +0 | +5 | +10 | +15 | +20 |
|---|---|---|---|---|---|
| ADALM-Pluto (`-g -45` … `-25`) | -12.0 | -7.7 | -4.4 | -2.6 | **-1.9** |
| HackRF One (`-g 20` … `40`) | -6.7 | -3.9 | -2.5 | -1.7 | **-1.3** |
| LimeSDR Mini v1 (`-g 10` … `30`) | -8.3 | -4.3 | -3.8 | -1.8 | **-1.4** |

So the working points with `--cn0` are about 20 dB above those without noise: **Pluto `-g -25`, HackRF `-g 40`, LimeSDR `-g 30`** on this setup. A further 5 dB would gain 0.2-0.4 dB. The Pluto ramp was measured before the 0.34 dB compensation below; with it, its last value would be about -1.6 dB. The Lime's step from `-g 15` to `-g 20` gave only 0.5 dB, the next one 2 dB, possibly an uneven TX gain step; it was not repeated. Every run had an SBAS fix except the Pluto at +0 dB, where the GEOs were not acquired.

At the Pluto's working point the offset depends on the requested level:

| `--cn0` | Reported - requested |
|---|---|
| 30 | -0.9 dB (4 satellites tracked) |
| 40 | -1.6 dB |
| 45 | -1.9 dB |
| 50 | -2.8 dB |

The offset is about 1.5 dB of loss plus, at high levels, the other satellites' signals adding to the noise. 0.34 dB of the loss came from the simulator: with one code chip value per sample at 3 MS/s the signal correlates that much worse with the ideal code, which `--cn0` now compensates (these values were measured before, expect about 0.3 dB less offset). The remaining ~1.1 dB is the receiver's own loss; expect similar values from other receivers, not exact ones.

## SBAS

Tested with a u-blox NEO-M8T over a cable with 60 dB of attenuation and a DC block, HackRF at `-g 20`, amplifier off. The receiver decodes every message type, reports the EGNOS service and gets an SBAS fix. On the Pluto (OCXO) the SBAS fix was 0.18 m horizontally against 0.10 m GPS only (height +1.4 m vs +1.0 m): with zero corrections SBAS cannot improve the fix; the small height difference comes from the receiver taking the ionosphere from the grid instead of Klobuchar. Its iono correction agrees with the simulated delay within 0.35 m per satellite. With a good signal (Pluto, 37-42 dB-Hz on the GEOs) a cold M8T synchronised to each GEO 7-48 s after first seeing it and had its SBAS fix 76-88 s after the start, in three runs with and three without MT17; MT17 is what lets it name the system (EGNOS). Slow synchronisation seen earlier came from a weak signal. A single GEO works too: with `--sbas 150,13.0` the cold M8T had its SBAS fix after 86 and 187 s in two runs (the later one found 150 only after 131 s). Phones track the GEOs (a Pixel 4 at about 45 dB-Hz) but Android does not show whether SBAS is used.

## Practical notes

- Run it from a separate directory: `-f` overwrites `rinex2.gz`, `rinex3.gz` and `almanac.sem` in the working directory.
- With `-s now` and a fresh download the ephemeris is used as it is, so the simulated satellites are the ones the almanac puts above the horizon at that place. An old nav file is shifted to the current time instead, which makes the simulated satellites disagree with a real almanac; receivers that use assistance data may then distrust them. Use fresh data for receiver tests.
- Keep the gain constant for the first minutes. A cold receiver needs 30 to 60 s of signal for the ephemeris and about 12.5 minutes for the whole almanac. In the program `t` and `g` raise and lower the gain by 1 dB and `x` quits.
- When the simulation ends (`-d` or the end of a motion file) the transmitter is switched off as soon as the last samples are sent, without waiting for `x` (checked on all three SDRs).
- Leave the amplifier (`-a`) off.

Transmitting on the GPS L1 frequency is regulated in most countries and disturbs every receiver within reach. Keep the output conducted (cable and attenuation, a DC block if the receiver powers an active antenna), use the lowest gain that works, keep runs short and do not attach an antenna. Phones have no antenna connector, so even a cabled setup radiates a little.
