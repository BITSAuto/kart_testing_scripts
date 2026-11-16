# Kart test bench

Measures the kart's drive-by-wire behaviour (steering command map, rate, delay, dropped
commands, backlash, throttle/brake, turning radius) for the digital twin and the RL
controller. Talks to the ESP32 with the `Drive_By_Wire/ver1.py` frame format
(`*A..B{throttle}C..D{steering}E..I{brake}J..#`, 9600 8N1).

## Setup (test laptop, Ubuntu)

```bash
./setup.sh              # apt, venv, dialout, brltty/ModemManager fixes, udev rules, self-test
./setup.sh --udev       # with the kart plugged in: names /dev/kart_dbw and /dev/kart_vectornav
```
No internet on the laptop: on a machine with internet run `./setup.sh --make-bundle`
(x86_64 / Python 3.10 = Ubuntu 22.04 + Humble by default; `--py 3.12` for 24.04), copy the folder, then
`./setup.sh --offline`. Log out and back in after the first setup (dialout).
`./setup.sh --self-test [--full]` re-checks everything against a fake kart.

## Run

```bash
./run_dashboard.sh          # real kart (config.toml)
./run_dashboard.sh --sim    # dry run against sim/fake_kart.py
```
**SPACE (hold)** = deadman for anything that moves; release stops it. **A/D (hold)** jog,
**ENTER** done, **ESC/X** abort. Health tab: health check, SPACE auto-repeat test,
calibration status. Data tab: browse runs.

Every procedure also runs without the dashboard
(`.venv/bin/python procedures/p04_slew_rate.py --no-deadman ...`, someone on the brake).

## Order

| | procedure | needs |
|---|---|---|
| p00 | serial probe: line formats, suggests `[encoder] regex` (`--pulse 40` to find the field) | |
| p01 | health check | |
| p02 | steering command map: left/right D values, constant vs proportional | |
| p03 | zero, locks, ticks to wheel-angle table (`--zero-only` after every ESP32 power cycle) | p02 |
| p04 | steering rate each way (`--throttle N` rolling) | p03 |
| p05 | command delay, stop delay, overshoot | p04 |
| p06 | dropped commands vs spacing | p05 |
| p07 / p08 | backlash / centre repeatability (you type angles) | p03 |
| p09 | throttle to speed, accel, coast, brake (VectorNav) | |
| p10 | full-lock circles (VectorNav) | p03 |
| p11 | random sequence for model fitting | p03 |
| p12 | manual keyboard drive, recorded | p02 |

Do p02 to p08 parked (brake held automatically) or with the front wheels lifted, before driving.
Results: `data/` (see `data/README.md`). Settings and limits: `config.toml`.

## Before the first real run

- Set `[serial] port` (or use `--udev`), and `[vectornav]` source/port.
- Run p00: the encoder regex in `config.toml` is a guess until p00 shows the real lines.
- The firmware has no fail-safe if the laptop disconnects: these scripts stop the kart on
  abort, crash, closed dashboard and lost encoder data, but a pulled USB cable leaves the
  last command active. Keep a hand on the brake.
