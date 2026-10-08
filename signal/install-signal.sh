#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required. Install Python 3.11 or newer and rerun this script." >&2
  exit 1
fi

if ! python3 -c 'import sys; raise SystemExit(sys.version_info < (3, 11))'; then
  echo "Python 3.11 or newer is required for signal processing dependencies." >&2
  exit 1
fi

python3 -m venv "$VENV_DIR"
cd "$SCRIPT_DIR"
"$VENV_DIR/bin/python" -m pip install -r "$SCRIPT_DIR/requirements.txt"

if command -v code >/dev/null 2>&1; then
  code --install-extension ms-python.black-formatter
  code --install-extension ms-toolsai.jupyter
  code --install-extension janisdd.vscode-edit-csv
else
  echo "VS Code CLI not found; install these extensions manually:" >&2
  echo "  ms-python.black-formatter" >&2
  echo "  ms-toolsai.jupyter" >&2
  echo "  janisdd.vscode-edit-csv" >&2
fi

printf '\nSignal processing environment is ready. Activate it with:\n'
printf '  source %q/bin/activate\n' "$VENV_DIR"