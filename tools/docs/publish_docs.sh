#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPOSITORY_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)

if [ -n "${BYUL_PYTHON:-}" ]; then
    if ! "$BYUL_PYTHON" --version >/dev/null 2>&1; then
        echo "BYUL_PYTHON is not an executable Python interpreter: $BYUL_PYTHON" >&2
        exit 127
    fi
    PYTHON=$BYUL_PYTHON
elif [ -x "$REPOSITORY_ROOT/tools/python/.venv/bin/python" ] &&
     "$REPOSITORY_ROOT/tools/python/.venv/bin/python" --version >/dev/null 2>&1; then
    PYTHON=$REPOSITORY_ROOT/tools/python/.venv/bin/python
elif command -v python3 >/dev/null 2>&1 && python3 --version >/dev/null 2>&1; then
    PYTHON=python3
elif command -v python >/dev/null 2>&1 && python --version >/dev/null 2>&1; then
    PYTHON=python
else
    echo "No Python 3 interpreter was found. Set BYUL_PYTHON." >&2
    exit 127
fi

exec "$PYTHON" "$SCRIPT_DIR/publish_docs.py" "$@"
