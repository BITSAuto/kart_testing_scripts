#!/usr/bin/env bash
# Kart test bench setup for the test laptop (Ubuntu 22.04/24.04).
#
#   ./setup.sh                 apt packages, venv, dialout, brltty/ModemManager fixes,
#                              generic udev rules, then the quick self-test
#   ./setup.sh --udev          also name the plugged-in ESP32 / VectorNav (/dev/kart_dbw,
#                              /dev/kart_vectornav) and point config.toml at them
#   ./setup.sh --offline       install Python packages from bundle/wheels (no internet)
#   ./setup.sh --make-bundle [--platform manylinux2014_x86_64] [--py 3.12]
#                              (on a machine WITH internet) download wheels for the laptop
#   ./setup.sh --remove-brltty      purge brltty instead of just masking its udev rule
#   ./setup.sh --disable-modemmanager   stop ModemManager entirely (default: only ignore our ports)
#   ./setup.sh --self-test [--full]     only run the self-test
#   ./setup.sh --no-apt --no-system --skip-self-test --dry-run --yes
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${VENV_DIR:-$HERE/.venv}"
BUNDLE="$HERE/bundle/wheels"
PY=${PYTHON:-python3}
APT=1 SYSTEM=1 UDEV=0 OFFLINE=0 MAKE_BUNDLE=0 REMOVE_BRLTTY=0 DISABLE_MM=0
SELFTEST=1 SELFTEST_ONLY=0 FULL="" DRY=0 YES=0 PLATFORM=manylinux2014_x86_64 PYVER=3.12
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-apt) APT=0 ;; --no-system) SYSTEM=0; APT=0 ;; --udev) UDEV=1 ;;
    --offline) OFFLINE=1; APT=0 ;; --make-bundle) MAKE_BUNDLE=1 ;;
    --platform) PLATFORM=$2; shift ;; --py) PYVER=$2; shift ;;
    --remove-brltty) REMOVE_BRLTTY=1 ;; --disable-modemmanager) DISABLE_MM=1 ;;
    --self-test) SELFTEST_ONLY=1 ;; --full) FULL=--full ;; --skip-self-test) SELFTEST=0 ;;
    --dry-run) DRY=1 ;; --yes|-y) YES=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac; shift
done
say()  { printf '\033[1;36m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[setup]\033[0m %s\n' "$*"; }
run()  { if [[ $DRY == 1 ]]; then echo "  (dry-run) $*"; else "$@"; fi; }
sudo_run() { if [[ $DRY == 1 ]]; then echo "  (dry-run) sudo $*"; else sudo "$@"; fi; }
ask() { [[ $YES == 1 ]] && return 0; read -r -p "$1 [y/N] " a; [[ $a =~ ^[Yy] ]]; }

# ------------------------------------------------------------------ offline bundle
if [[ $MAKE_BUNDLE == 1 ]]; then
  say "downloading wheels for $PLATFORM / Python $PYVER into $BUNDLE"
  mkdir -p "$BUNDLE"
  "$PY" -m pip download --only-binary=:all: --platform "$PLATFORM" --python-version "$PYVER" \
      --implementation cp -d "$BUNDLE" -r "$HERE/requirements.txt" pip
  (cd "$BUNDLE" && sha256sum *.whl > SHA256SUMS && echo "$PLATFORM py$PYVER $(date -I)" > TARGET)
  say "done: copy the whole kart_testing_modules folder to the laptop and run ./setup.sh --offline"
  exit 0
fi

if [[ $SELFTEST_ONLY == 1 ]]; then exec "$VENV/bin/python" "$HERE/tools/self_test.py" $FULL; fi

"$PY" -c 'import sys; assert sys.version_info >= (3, 11)' 2>/dev/null \
  || { echo "need Python 3.11+ (Ubuntu 24.04 has 3.12)"; exit 1; }

# ------------------------------------------------------------------------ apt
if [[ $APT == 1 ]]; then
  PKGS=(python3-venv python3-pip usbutils picocom)
  MISSING=(); for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p"); done
  if [[ ${#MISSING[@]} -gt 0 ]]; then
    say "apt install ${MISSING[*]}"
    sudo_run apt-get update -qq && sudo_run apt-get install -y "${MISSING[@]}"
  else say "apt packages already installed"; fi
fi

# ----------------------------------------------------------------------- venv
# --system-site-packages keeps a sourced ROS 2 rclpy importable (vectornav.source = "ros2")
if [[ ! -x "$VENV/bin/python" ]]; then
  say "creating $VENV"
  if ! run "$PY" -m venv --system-site-packages "$VENV" 2>/dev/null; then
    [[ $OFFLINE == 1 ]] || { echo "python3 -m venv failed: sudo apt install python3-venv"; exit 1; }
    warn "no ensurepip (python3-venv missing): bootstrapping pip from the bundle"
    run "$PY" -m venv --system-site-packages --without-pip "$VENV"
    PIPWHL=$(ls "$BUNDLE"/pip-*.whl | head -1)
    run "$VENV/bin/python" "$PIPWHL/pip" install -q --no-index --find-links "$BUNDLE" pip
  fi
fi
if [[ $OFFLINE == 1 ]]; then
  [[ -d "$BUNDLE" ]] || { echo "no $BUNDLE: make it with --make-bundle on a machine with internet"; exit 1; }
  [[ -f "$BUNDLE/TARGET" ]] && say "bundle built for: $(cat "$BUNDLE/TARGET"); this machine: $(uname -m)"
  (cd "$BUNDLE" && sha256sum -c --quiet SHA256SUMS) || { echo "bundle checksum mismatch"; exit 1; }
  run "$VENV/bin/python" -m pip install -q --no-index --find-links "$BUNDLE" -r "$HERE/requirements.txt"
else
  run "$VENV/bin/python" -m pip install -q --upgrade pip
  run "$VENV/bin/python" -m pip install -q -r "$HERE/requirements.txt"
fi
say "python packages ready in $VENV"

# ------------------------------------------------------------- system settings
if [[ $SYSTEM == 1 ]]; then
  if ! id -nG "$USER" | grep -qw dialout; then
    say "adding $USER to dialout (serial port access; takes effect after logging out and in)"
    sudo_run usermod -aG dialout "$USER"
  else say "$USER already in dialout"; fi

  # brltty's udev rule claims CH340/CP210x adapters as braille displays: the port vanishes.
  if [[ -f /usr/lib/udev/rules.d/85-brltty.rules || -f /lib/udev/rules.d/85-brltty.rules ]]; then
    if [[ $REMOVE_BRLTTY == 1 ]]; then
      say "removing brltty"; sudo_run apt-get remove -y brltty
    else
      say "masking brltty's USB-serial grab (brltty stays installed; --remove-brltty to purge)"
      sudo_run ln -sf /dev/null /etc/udev/rules.d/85-brltty.rules
      sudo_run systemctl mask --now brltty-udev.service 2>/dev/null || true
    fi
  fi

  if [[ $DISABLE_MM == 1 ]] && systemctl is-active -q ModemManager; then
    say "disabling ModemManager (a built-in WWAN modem will stop working)"
    sudo_run systemctl disable --now ModemManager
  fi

  RULES=/etc/udev/rules.d/70-kart-serial.rules
  TMP=$(mktemp)
  if [[ $UDEV == 1 ]]; then
    "$PY" "$HERE/tools/serial_devices.py" list
    mapfile -t NODES < <(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
    [[ ${#NODES[@]} -gt 0 ]] || { echo "no USB serial devices plugged in"; exit 1; }
    read -r -p "number of the drive-by-wire ESP32 (empty = skip): " i_dbw
    read -r -p "number of the VectorNav (empty = skip): " i_vn
    ARGS=(); [[ -n $i_dbw ]] && ARGS+=(--dbw "${NODES[$((i_dbw-1))]}")
    [[ -n $i_vn ]] && ARGS+=(--vn "${NODES[$((i_vn-1))]}")
    "$PY" "$HERE/tools/serial_devices.py" rules "${ARGS[@]}" --notes "$TMP.notes" > "$TMP"
    cat "$TMP.notes"
  else
    "$PY" "$HERE/tools/serial_devices.py" generic-rules > "$TMP"
    [[ -f $RULES ]] && grep -q 'SYMLINK' "$RULES" && { say "keeping existing named rules in $RULES"; rm -f "$TMP"; TMP=""; }
  fi
  if [[ -n $TMP ]]; then
    say "installing $RULES"
    sudo_run install -m 0644 "$TMP" "$RULES"
    sudo_run udevadm control --reload-rules
    sudo_run udevadm trigger --subsystem-match=tty
    rm -f "$TMP" "$TMP.notes"
  fi
  if [[ $UDEV == 1 ]] && ask "point config.toml at /dev/kart_dbw (and /dev/kart_vectornav)?"; then
    [[ -n $i_dbw ]] && run sed -i '0,/^port = .*/s||port = "/dev/kart_dbw"|' "$HERE/config.toml"
    [[ -n $i_vn ]] && run sed -i '/^\[vectornav\]/,/^\[/{s|^port = .*|port = "/dev/kart_vectornav"|;s|^source = .*|source = "serial"|}' "$HERE/config.toml"
    say "config.toml updated"
  fi
fi

# ----------------------------------------------------------------- self-test
if [[ $SELFTEST == 1 && $DRY == 0 ]]; then
  say "self-test (fake kart; the real kart is not touched)"
  "$VENV/bin/python" "$HERE/tools/self_test.py" $FULL || warn "self-test reported failures (see above)"
fi
say "done. Start the dashboard: ./run_dashboard.sh   (dry run: ./run_dashboard.sh --sim)"
