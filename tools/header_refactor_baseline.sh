#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if [ -n "${BYUL_PYTHON:-}" ]; then
    exec "$BYUL_PYTHON" "$SCRIPT_DIR/header_refactor_baseline.py" "$@"
fi

if [ -x "$SCRIPT_DIR/python/.venv/bin/python" ]; then
    exec "$SCRIPT_DIR/python/.venv/bin/python" \
        "$SCRIPT_DIR/header_refactor_baseline.py" "$@"
fi

if command -v python3 >/dev/null 2>&1; then
    exec python3 "$SCRIPT_DIR/header_refactor_baseline.py" "$@"
fi

echo "ERROR: Python 3 is unavailable; set BYUL_PYTHON or create tools/python/.venv." >&2
exit 2
