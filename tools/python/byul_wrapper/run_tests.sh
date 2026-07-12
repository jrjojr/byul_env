#!/usr/bin/env sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if [ -n "${BYUL_PYTHON:-}" ] && [ -x "$BYUL_PYTHON" ]; then
    PYTHON=$BYUL_PYTHON
elif [ -x "$ROOT/../.venv/bin/python" ]; then
    PYTHON=$ROOT/../.venv/bin/python
elif command -v python3 >/dev/null 2>&1; then
    PYTHON=$(command -v python3)
else
    echo "[ERROR] Python 3 was not found. Run tools/python/setup_env.sh first." >&2
    exit 1
fi

exec "$PYTHON" "$ROOT/run_tests.py" "$@"
