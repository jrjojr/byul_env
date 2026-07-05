# BYUL Runtime And Test Execution

This document describes how to run BYUL build outputs. For build requirements,
see [Build Environment](build-environment.md).

## Output Layout

Typical preset outputs:

```text
Linux Debug:
  build_debug/bin/       test executables and helper executables
  build_debug/lib/       libbyul.so

Linux Release:
  build_release/bin/     test executables and helper executables
  build_release/lib/     libbyul.so

Windows MinGW cross:
  build_win_release/     Windows build tree
  package_tmp/.../bin/   byul.dll, test_*.exe, MinGW runtime DLLs
  package_tmp/.../lib/   import/static archives
```

For deployment checks, prefer the `package_zip` layout or the generated zip over
raw build-tree paths.

## Run Tests With CTest

```bash
cd byul_env/byul
ctest --test-dir ../build_debug --output-on-failure
```

Run selected tests:

```bash
ctest --test-dir ../build_debug -R test_byul --output-on-failure
ctest --test-dir ../build_debug -R test_number_theory --output-on-failure
ctest --test-dir ../build_debug -R test_dstar_lite --output-on-failure
```

`-R` is a regular expression. Use an end anchor for the exact integration test:

```bash
ctest --test-dir ../build_debug -R 'test_byul$' --output-on-failure
```

## Run Linux Test Executables Directly

```bash
./build_debug/bin/test_byul
./build_debug/bin/test_core
./build_debug/bin/test_number_theory
./build_debug/bin/test_dstar_lite
```

Linux test executables use this RPATH:

```text
$ORIGIN/../lib
```

So `build_debug/bin/test_byul` normally finds `build_debug/lib/libbyul.so`
automatically.

## Link And Run An External Linux Program

External programs need headers and `libbyul.so` at link time, then a runtime path
to `libbyul.so`.

```bash
g++ main.cpp \
  -I./build_debug/package_tmp/home/sajang/byul/include/byul \
  -L./build_debug/package_tmp/home/sajang/byul/lib \
  -lbyul \
  -Wl,-rpath,'$ORIGIN/../lib' \
  -o my_app
```

Expected runtime layout:

```text
my_package/
  bin/my_app
  lib/libbyul.so
```

Without RPATH, set `LD_LIBRARY_PATH`:

```bash
LD_LIBRARY_PATH=/path/to/byul/lib ./my_app
```

## Run Windows test_*.exe

On Windows, `byul.dll` must be next to the executable or visible through `PATH`.

```text
byul/
  bin/
    byul.dll
    test_byul.exe
    test_core.exe
    test_number_theory.exe
    test_dstar_lite.exe
    libstdc++-6.dll
    libgcc_s_seh-1.dll
    libwinpthread-1.dll
```

Run from `bin`:

```bat
cd C:\path\to\byul\bin
test_byul.exe
test_number_theory.exe
```

Or add the DLL directory to `PATH`:

```bat
set PATH=C:\path\to\byul\bin;%PATH%
test_byul.exe
```

## Link And Run A Windows Program

At build time, link against the generated import library. At runtime, keep
`byul.dll` and any required MinGW runtime DLLs next to the executable or in
`PATH`.

MinGW example:

```bash
x86_64-w64-mingw32-g++ main.cpp \
  -I/path/to/byul/include/byul \
  -L/path/to/byul/lib/byul \
  -lbyul \
  -o my_app.exe
```

Runtime layout:

```text
app/
  my_app.exe
  byul.dll
  libstdc++-6.dll
  libgcc_s_seh-1.dll
  libwinpthread-1.dll
```

For MSVC builds, link with the generated `.lib` import library and place
`byul.dll` next to the executable or in `PATH`.

## Troubleshooting

Linux:

```bash
ldd ./build_debug/bin/test_byul
LD_LIBRARY_PATH=./build_debug/lib ./build_debug/bin/test_byul
```

Windows:

```text
1. Keep test_*.exe and byul.dll in the same bin directory.
2. Keep MinGW runtime DLLs in the same bin directory for MinGW builds.
3. Add the byul.dll directory to PATH if it is not next to the exe.
4. Do not mix 32-bit and 64-bit outputs.
```
