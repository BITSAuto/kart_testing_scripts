# Kart test bench: field debugging guide (for an LLM assistant)

You are helping someone at the kart, usually on the Toughbook laptop (Ubuntu 22.04 + ROS 2
Humble, Python 3.10, no GPU), often without internet. Project background:
`../HANDOFF.md`. User docs: `README.md`, `data/README.md`.

## Rules for you

- **Never move the kart yourself.** Don't run motion procedures (anything marked `[M]` in
  `procedures/registry.py`), `--no-deadman`, or any script that sends steering or throttle
  frames, unless the user asks for that specific action and confirms someone is at the
  brake. The firmware has **no fail-safe**: a crashed script or pulled cable leaves the last
  command active.
- Read-only checks are fine to run: `tools/diagnose.py`, `tools/serial_devices.py list`,
  `p01_health_check.py` without `--motion-check`, `p00_serial_probe.py` without `--pulse`.
  The `--listen` options never send anything. p00 and p01 do send one neutral frame
  (D50, B0: stop, no throttle), which is safe.
- Don't hand-edit `data/calibration/*.json`. Rerun the procedure that writes it. Previous
  versions are in `data/calibration/history/` if something needs restoring.
- Config changes go in `config.toml`; tell the user what you changed and why.
- Before claiming a fix works, reproduce it on the fake kart: `./run_dashboard.sh --sim`, or
  `.venv/bin/python tools/self_test.py` (quick) / `--full`.

## First thing to run

```bash
.venv/bin/python tools/diagnose.py --listen 3      # read-only snapshot: devices, config,
                                                   # calibration, recent runs + errors
```
Then look in the newest run folder: `data/<test>/<time>/`. It has `meta.json` (status,
args), `summary.json` (results or `error`), `events.csv` (safety_stop, soft_limit, stall,
no_motion, stop_retry, trial_interrupted), `serial_tx.csv` (every frame and why it was sent,
in the `note` column), and `serial_rx.csv` (every line received and the parsed count).

## How it works (just enough to debug)

- **Frames** (from `Drive_By_Wire/ver1.py`):
  `*A{a}B{throttle}C{c}D{steering}E..H..I{brake}J{reverse}#` at 9600 8N1. Steering D: 50 =
  stop, one side = left, the other = right (p02 finds which). Every frame carries the full
  state. The kart may **drop a frame sent within ~0.2 s of the previous one**, so every
  intentional send waits `steering.min_command_gap_s`, and stops, throttle 0 and brake are
  sent 3× (`link.send_reliable`).
- **Encoder:** text lines on the same port (or `[encoder] port`). `[encoder] regex` capture
  group 1 is the raw count, timestamped on arrival (`kart/link.py`). Position = (raw − zero)
  × sign, left positive. Degrees come from the p03 table.
- **Process model:** the dashboard (`dashboard/app.py`) runs each procedure as a subprocess
  with `--tui`.
  - stdout: plain text goes to the log; `@{json}` messages (`tel`, `prompt`, `status`,
    `check`, `result`).
  - stdin: `hb` (SPACE repeat), `jog left|right|stop`, `key x`, `answer id text`, `abort`.
    EOF on stdin = abort.
  - `procedures/monitor.py` runs while idle (read-only).
- **Safety:** `kart/guard.py`, a 50 Hz watchdog.
  - Trips (`Aborted`) on abort, stdin EOF, serial fault, encoder stale while steering
    (`encoder.stale_s`), a move longer than `max_move_s` (`uncalibrated_pulse_s` before p03),
    overspeed, throttle on too long, or a motion command outside a motion block.
  - Deadman release = `Paused`: the trial is redone.
  - Soft limits stop steering inside the locks. A stall near a lock stops steering; away from
    a lock it only logs `no_motion` (usually a dropped command).
- **Calibration chain:**
  - p02 → `steer_map`
  - p03 → `zero`, `range`
  - p04 → `slew`
  - p05 → `latency`
  - p06 → `drop`
  - p07 → `backlash`
  - p08 → `repeatability`
  - p09 → `longitudinal`
  - p10 → `circles`
  - p13 → `step_steer`
  - p14 → `vehicle`

  Procedures refuse to start if a `needs` entry is missing (`--force` overrides).

## Symptom → cause → fix

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `setup.sh` says it needs a newer Python | an old copy (it now needs 3.10+) | update the folder; `PYTHON=python3.10 ./setup.sh` |
| `ModuleNotFoundError: tomli` | 3.10 without the deps | `./setup.sh` or `--offline` (bundle has tomli) |
| port missing (`/dev/ttyUSB1` doesn't exist) | different number / unplugged / brltty grabbed it | `tools/serial_devices.py list`; kernel log in diagnose; `./setup.sh --udev` for `/dev/kart_dbw` |
| port appears then vanishes | **brltty** claims CH340/CP210x | `./setup.sh` masks it (or `--remove-brltty`), replug |
| `Permission denied` | not in dialout, or not re-logged in | `id -nG`; `sudo usermod -aG dialout $USER`, then log out/in |
| `could not exclusively lock port` / busy | ver1.py, picocom, the monitor or another test has it | `fuser -v /dev/ttyUSB*`; quit the other program |
| garbage / random resets of the ESP32 on plug-in | ModemManager probing with AT commands | udev ignore rule from `setup.sh`; or `--disable-modemmanager` |
| no lines at all | wrong port or baud, ESP32 off | `diagnose.py --listen 3`; `picocom -b 9600 /dev/ttyUSB1` (Ctrl-A Ctrl-X exits) |
| lines only right after a frame | firmware answers frames instead of streaming | `[serial] poll_interval_s = 0.1` (repeats count toward the drop window) |
| lines arrive but `encoder parsing` FAIL | regex doesn't match the real format | `p00_serial_probe.py --pulse 40` (needs SPACE); copy its suggested regex into `[encoder] regex` |
| encoder parsed but slow (< 10/s) | firmware print rate | timing results get coarse; ask for a faster rate or a millis timestamp in each line |
| "no encoder data while steering" abort | encoder rate low or delayed | raise `encoder.stale_s`; check rate in the health check |
| encoder at rest jitters | electrical noise / quadrature edge | raise `encoder.noise_ticks` |
| angles nonsense after a power cycle | incremental encoder reset; zero is stale | `p03_center_and_range.py --zero-only`; health check warns if pos is outside the lock range |
| left/right swapped | steer_map wrong or wiring changed | rerun p02, answer the "did it turn LEFT" question carefully |
| steering doesn't move | D value in deadband, brake interlock?, motor power | p02 table (`pulses.csv`); try `--values 0,100`; check the motor supply |
| "steering moved longer than X s" abort | before p03 the limit is `uncalibrated_pulse_s` (0.6 s); after, `max_move_s` | expected while uncalibrated; run p03; or adjust in `[safety]` |
| "stalled at the lock" during jog | holding the key against the lock | normal: steering stops itself |
| many `no_motion` / `stop_retry` events | commands dropped (sent too close) | look at `serial_tx.csv` spacing; raise `min_command_gap_s`; p06 quantifies it |
| move_to never gets close / oscillates | minimum step ≈ rate × gap is bigger than the tolerance | raise `move_tolerance_deg`; expected kart behaviour |
| deadman box never turns green | terminal key auto-repeat off/slow; over SSH; a dialog or input has focus | Health tab → "Test SPACE auto-repeat"; enable repeat (`xset r on`); click a chart so nothing is focused |
| "deadman released" mid-trial without letting go | repeat gap > `deadman_repeat_s` (slow laptop/SSH) | raise `deadman_repeat_s` / `deadman_initial_s` |
| VectorNav no data (`source=serial`) | port/baud, or ASCII output not enabled | `diagnose`; VN must output `$VNINS` (and `$VNYMR` for yaw rate) |
| VectorNav no data (`source=ros2`) | ROS not sourced, wrong domain or topics, rclpy missing | `source /opt/ros/humble/setup.bash` before `./run_dashboard.sh`; same `ROS_DOMAIN_ID` as the Orin; check topic names in `[vectornav.ros2]`; `ros2 topic list` |
| p09/p10/p13 refuse: "needs VectorNav data" | as above, or no GNSS fix | wait for a fix outdoors |
| overspeed abort | throttle too high for `max_speed_kmh` | use p09's `throttle_for_15_kmh`; check the limit |
| kart keeps rolling after a test | throttle-0 frame dropped (guard re-sends if VN shows motion) | brake by hand; check `serial_tx.csv` for repeated throttle-0 frames |
| "brake (I=1) left ON" | aborted test while braking (deliberate) | the next test or ver1 releases it; steering tests release it at the end on success |
| dashboard garbled / tiny | terminal too small or no 256 colours | ≥ 120×36 terminal, `TERM=xterm-256color` |
| `ver1.py` NameError on start | known bug in the user's repo (not ours) | don't use it for tests; tell the user |

## Useful one-liners

```bash
.venv/bin/python procedures/p01_health_check.py                 # health check without the dashboard
.venv/bin/python procedures/p00_serial_probe.py --listen 3      # line formats (no motion)
.venv/bin/python tools/serial_devices.py list                   # USB serial devices
.venv/bin/python tools/self_test.py --skip-host                 # does the code still work (fake kart)
column -s, -t < data/<test>/<run>/serial_tx.csv | tail -30      # what was sent and why
grep -E 'safety_stop|soft_limit|stall|no_motion|stop_retry|interrupted' data/<test>/<run>/events.csv
```

Analysing a run in Python: `serial_rx.csv` columns `t, wall, source, enc_raw, line` and
`serial_tx.csv` columns `t, wall, frame, a, throttle, c, steering, ..., brake, reverse, note`.
`t` is seconds since the run started (same clock for both), so join them on `t`.

## Where things are

`config.toml` (all knobs) · `kart/` (library: link, guard, steering, harness, channel,
recorder, vectornav, config) · `procedures/` (p00–p14, monitor, registry) · `dashboard/` ·
`sim/fake_kart.py` (fake ESP32 + VectorNav; `--help` lists its parameters, e.g.
`--respond-only`, `--proportional`, `--drop-prob`, to reproduce field behaviour) · `tools/`
(diagnose, self_test, scripted_operator, serial_devices) · `setup.sh` · `bundle/wheels`
(offline install).
