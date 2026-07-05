# BYUL Build Environment

This document describes the minimum build requirements and installation steps
for BYUL. For running test executables and linking `byul.dll`/`libbyul.so`, see [Runtime And Test Execution](runtime-execution.md).

The primary development machine is Ubuntu. Most Linux and Windows
MinGW cross builds are expected to run from Ubuntu, while MSVC builds require a
Windows Visual Studio environment.

## Build Model

BYUL uses CMake presets.

```text
Primary work environment: Ubuntu
Primary build command: cmake --preset ... / cmake --build --preset ...
Package target: package_zip
Public output: byul.dll or libbyul.so, public headers
Development output: module static libraries, module test executables, integration test executable
```

`package_zip` can be run directly. It updates `all`, installs into a temporary
package layout, and creates `byul.zip`.

## Ubuntu Requirements

Install the base toolchain:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  zip \
  gdb \
  gcc-mingw-w64-x86-64 \
  g++-mingw-w64-x86-64 \
  mingw-w64-tools
```

For SDL or GPU helper targets, install the matching development packages:

```bash
sudo apt install -y libsdl2-dev libvulkan-dev mesa-common-dev
```

The `win_sdl_release` preset expects `external/SDL3-3.2.18/cmake` to exist and
uses it through `CMAKE_PREFIX_PATH`.

## Ubuntu Builds

```bash
cd byul_env/byul
cmake --preset linux-debug
cmake --build --preset build-linux-debug

cmake --preset linux-release
cmake --build --preset build-linux-release
```

## Windows MinGW Cross Build From Ubuntu

```bash
cd byul_env/byul
cmake --preset win-release
cmake --build --preset build-win-release
```

This uses `byul/cmake/toolchain-mingw64.cmake` and expects these tools in PATH:

```text
x86_64-w64-mingw32-gcc
x86_64-w64-mingw32-g++
x86_64-w64-mingw32-windres
```

## Windows Native MinGW

These presets are for Windows MSYS2 MinGW, not Ubuntu:

```text
win-native
win-native-debug
```

Example MSYS2 packages:

```bash
pacman -S --needed \
  mingw-w64-x86_64-cmake \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-make \
  mingw-w64-x86_64-pkgconf \
  mingw-w64-x86_64-zip
```

## MSVC Build Analysis

The `win-msvc-release` and `win-msvc-debug` presets use the Visual Studio 2022
generator and a Windows `cmake.exe` path. They cannot run directly on Ubuntu in
the current configuration.

Supported paths:

```text
Use MinGW64 from Ubuntu for Windows DLL/EXE cross builds.
Use a Windows machine, Windows VM, or Windows CI runner for MSVC presets.
```

Not currently configured or recommended:

```text
Running Visual Studio CMake through Wine.
Using Ubuntu clang-cl as an MSVC ABI replacement.
```

## Tests

CTest is run directly because `testPresets` is currently empty.

```bash
ctest --test-dir ../build_debug --output-on-failure
ctest --test-dir ../build_release --output-on-failure
```

For MSVC multi-config builds on Windows:

```bash
ctest --test-dir ../build_win_msvc_release -C Release --output-on-failure
ctest --test-dir ../build_win_msvc_debug -C Debug --output-on-failure
```

## Presets

```text
linux-debug          Ubuntu Debug build
linux-release        Ubuntu Release build
win-release          Windows DLL cross build from Ubuntu with MinGW64
win_sdl_release      Windows SDL executable cross build from Ubuntu with MinGW64
win-native           Windows MSYS2 MinGW Release build
win-native-debug     Windows MSYS2 MinGW Debug build
win-msvc-release     Windows Visual Studio 2022 Release build
win-msvc-debug       Windows Visual Studio 2022 Debug build
```
