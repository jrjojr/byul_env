#!/usr/bin/env sh
set -eu

PROJECT_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ENTRYPOINT="$PROJECT_ROOT/byul_grid/byul_grid.py"
BYUL_SOURCE="$PROJECT_ROOT/../../byul"

case "$(uname -s)" in
    Linux*)
        BYUL_BUILD="$PROJECT_ROOT/../../build_release"
        BYUL_LIBRARY="$BYUL_BUILD/lib/libbyul.so"
        ;;
    Darwin*)
        BYUL_BUILD="$PROJECT_ROOT/../../build_macos_release"
        BYUL_LIBRARY="$BYUL_BUILD/lib/libbyul.dylib"
        ;;
    *)
        echo "[ERROR] Unsupported platform. Use run_byul_grid.bat on Windows." >&2
        exit 1
        ;;
esac

if [ ! -f "$BYUL_SOURCE/CMakeLists.txt" ]; then
    echo "[ERROR] byul_env source was not found: $BYUL_SOURCE/CMakeLists.txt" >&2
    exit 1
fi

if [ ! -f "$BYUL_LIBRARY" ]; then
    command -v cmake >/dev/null 2>&1 || {
        echo "[ERROR] CMake was not found." >&2
        exit 1
    }
    echo "[INFO] Configuring BYUL Release..."
    cmake -S "$BYUL_SOURCE" -B "$BYUL_BUILD" -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] Building the current BYUL Release target..."
    cmake --build "$BYUL_BUILD" --target byul --parallel
fi

if [ ! -f "$BYUL_LIBRARY" ]; then
    echo "[ERROR] Built BYUL library was not found: $BYUL_LIBRARY" >&2
    exit 1
fi

export BYUL_LIBRARY_PATH="$BYUL_LIBRARY"

if [ -x "$PROJECT_ROOT/.venv/bin/python" ]; then
    exec "$PROJECT_ROOT/.venv/bin/python" "$ENTRYPOINT" "$@"
fi

command -v python3 >/dev/null 2>&1 || {
    echo "[ERROR] Python 3 was not found." >&2
    exit 1
}
exec python3 "$ENTRYPOINT" "$@"
