# BYUL Grid

BYUL Grid is a cross-platform PySide6 application for visualizing and testing
BYUL's grid-based navigation systems. Its zoomable grid displays cells,
terrain, routes, dynamic replanning, NPC movement, and maze-generation results.

## Setup and run

BYUL Grid shares the Python environment under `tools/python` with the standalone
`byul_wrapper` package.

### Windows

```bat
tools\python\setup_env.bat
tools\python\byul_grid\run_byul_grid.bat
```

The launcher uses `build_win_msvc\bin\Release\byul.dll`. If the library
is missing, it builds the `byul` Release target from the current repository.

### Linux and macOS

```sh
tools/python/setup_env.sh
tools/python/byul_grid/run_byul_grid.sh
```

The launcher uses the current repository build: `build_release/lib/libbyul.so`
on Linux or `build_macos_release/lib/libbyul.dylib` on macOS.

On every platform, set `BYUL_LIBRARY_PATH` to use a specific native library.

## Tests

The shared environment installs the development dependencies. Run:

```bat
tools\python\.venv\Scripts\python -m pytest tools\python\byul_wrapper\tests
```

On Linux and macOS, use `tools/python/.venv/bin/python`.

## Standalone package

Standalone packaging is separate from the BYUL engine's default build.

Windows:

```bat
tools\python\byul_grid\build_byul_grid.bat
```

Linux and macOS:

```sh
tools/python/byul_grid/build_byul_grid.sh
```

Run `python tools/python/byul_grid/build_byul_grid.py --help` for packaging
options. A configured BYUL
CMake build tree also provides the `byul_grid_zip` target for creating a portable
application archive.

For cx_Freeze architecture, troubleshooting, and package layout details, see
[cx_Freeze Packaging](../../tools/python/byul_grid/docs/cx-freeze-packaging.ko.md).
