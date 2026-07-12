# BYUL Grid

BYUL Grid is a cross-platform PySide6 application for visualizing and testing
BYUL's grid-based navigation systems. Its zoomable grid displays cells,
terrain, routes, dynamic replanning, NPC movement, and maze-generation results.

## Setup and run

Run the following commands from `tools/byul_grid`.

### Windows

```bat
py -3 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
run_byul_grid.bat
```

The launcher uses `build_win_msvc\bin\Release\byul.dll`. If the library
is missing, it builds the `byul` Release target from the current repository.

### Linux and macOS

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
chmod +x run_byul_grid.sh
./run_byul_grid.sh
```

The launcher uses the current repository build: `build_release/lib/libbyul.so`
on Linux or `build_macos_release/lib/libbyul.dylib` on macOS.

On every platform, set `BYUL_LIBRARY_PATH` to use a specific native library.

## Tests

Install the development dependencies before running tests:

```bat
.venv\Scripts\python -m pip install -r requirements-dev.txt
.venv\Scripts\python -m pytest byul_grid\wrapper\tests
```

On Linux and macOS, use `.venv/bin/python` and the path
`byul_grid/wrapper/tests` instead.

## Standalone package

Standalone packaging is separate from the BYUL engine's default build.

Windows:

```bat
build_byul_grid.bat
```

Linux and macOS:

```sh
chmod +x build_byul_grid.sh
./build_byul_grid.sh
```

Run `python build_byul_grid.py --help` for packaging options. A configured BYUL
CMake build tree also provides the `byul_grid_zip` target for creating a portable
application archive.

For cx_Freeze architecture, troubleshooting, and package layout details, see
[cx_Freeze Packaging](../ko/cx-freeze-packaging.ko.md).
