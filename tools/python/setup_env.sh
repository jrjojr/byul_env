#!/usr/bin/env sh
set -eu

PYTHON_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
VENV="$PYTHON_ROOT/.venv"
cd "$PYTHON_ROOT"

command -v python3 >/dev/null 2>&1 || {
    echo "[ERROR] Python 3 was not found. Install Python 3.10 or newer." >&2
    exit 1
}

if [ ! -x "$VENV/bin/python" ]; then
    echo "[INFO] Creating shared Python environment: $VENV"
    python3 -m venv "$VENV"
fi

"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r requirements-dev.txt -r requirements-package.txt
echo "[OK] Shared Python environment is ready: $VENV"
