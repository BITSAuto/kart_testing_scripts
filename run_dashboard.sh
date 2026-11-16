#!/usr/bin/env bash
# Start the kart test bench dashboard. Creates the virtualenv on first run.
#   ./run_dashboard.sh            real kart (config.toml)
#   ./run_dashboard.sh --sim      dry run against the fake kart
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -x "$HERE/.venv/bin/python" ]] || "$HERE/setup.sh"
exec "$HERE/.venv/bin/python" "$HERE/dashboard/app.py" "$@"
