# BYUL Grid

BYUL Grid is the cross-platform PySide6 visualization and simulation tool for
BYUL's grid-based navigation systems. It displays cells, terrain, routes,
dynamic replanning, NPC movement, and future maze-generation results through a
zoomable and scrollable grid interface.

The repository root [`LICENSE`](../../LICENSE) and
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md) apply to this tool.

## Windows

```bat
py -3 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
run_byul_grid.bat
```

BYUL Grid loads `build_win_msvc_release\bin\Release\byul.dll` from the current
repository. If it is missing, the launcher builds the `byul` Release target;
it does not use the copy installed under `%USERPROFILE%`.

## Linux and macOS

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
chmod +x run_byul_grid.sh
./run_byul_grid.sh
```

The launcher uses the current repository build. Linux loads
`build_release/lib/libbyul.so`; macOS loads
`build_macos_release/lib/libbyul.dylib`.

On every platform, `BYUL_LIBRARY_PATH` may point to a specific native library.

## Development dependencies

```text
requirements-dev.txt
```

Install them with the platform's virtual-environment Python when running tests.

## Standalone packaging

cx_Freeze packaging is intentionally separate from the BYUL engine's default
build. `build_byul_grid.py` automatically creates `.venv-package` and installs
the packaging dependencies when the requirements change. The `.bat` and `.sh`
files are thin platform launchers for this Python script.

Detailed packaging architecture, the three Windows startup failures, their root
causes, and fixes are documented in
[`docs/ko/cx-freeze-packaging.ko.md`](../../docs/ko/cx-freeze-packaging.ko.md).

On Windows:

```bat
build_byul_grid.bat
```

or on Linux and macOS:

```sh
chmod +x build_byul_grid.sh
./build_byul_grid.sh
```

Run `python build_byul_grid.py --help` to see options such as a custom virtual
environment path or forced dependency installation.

From a configured BYUL CMake build tree, the `grid_zip` target builds
the current `byul` shared-library target, passes that exact library to
cx_Freeze, and creates `byul-grid-0.1.0-<platform>-<configuration>.zip`. Extract the archive and
run `byul_grid/byul_grid.exe` on Windows or `byul_grid/byul_grid` on Unix-like
platforms; no installation step is required.

Standalone packages use an OS and architecture-specific directory, for example
`build/windows-x86_64/byul_grid`, `build/linux-x86_64/byul_grid`, or
`build/macos-arm64/byul_grid`. The platform directory can contain multiple
applications, and each application directory is its own cx_Freeze root. No
additional `bin` directory is used. Windows
places `byul.dll` beside the executable. Linux places `libbyul.so` in `lib`,
and macOS places `libbyul.dylib` there. License documents are placed in
`licenses`, and runtime images are placed in `assets/images`. If the platform
output directory is locked, packaging
prints the reason and asks whether to use a new timestamped output directory. Pass
`--on-locked new` to choose a new directory without prompting, or
`--on-locked cancel` to cancel immediately. Pass `--output` to select a
different directory. The native library from the current repository build is
included in the package.

The Windows executable icon is built from
`assets/icons/byul_grid.ico`; the transparent PNG source is kept beside it for
use by other platform packaging formats.

Before overwriting a platform package, the build removes that platform's entire
output directory and recreates it. Outputs under the former
`build/byul_grid/<platform>` hierarchy are removed during migration, while
the former flat `build/<platform>` output is also cleaned. Packages for other
operating systems in the new hierarchy are preserved.

## Tests

Windows:

```bat
.venv\Scripts\python -m pytest byul_grid\wrapper\tests
```

Linux and macOS:

```sh
.venv/bin/python -m pytest byul_grid/wrapper/tests
```
