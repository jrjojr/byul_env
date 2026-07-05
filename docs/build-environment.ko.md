# BYUL 빌드 환경 구성

이 문서는 BYUL을 빌드하기 위한 최소 구성 요건과 설치 방법을 정리합니다.
현재 권장 작업 환경은 Ubuntu입니다. Linux 빌드와 Windows MinGW 크로스 빌드는
대부분 Ubuntu에서 실행하고, MSVC 빌드는 Windows Visual Studio 환경에서
확인하는 구조입니다.

## 빌드 기준

BYUL은 CMake preset 중심으로 빌드합니다.

```text
주 작업 환경: Ubuntu
주 빌드 명령: cmake --preset ... / cmake --build --preset ...
패키지 target: package_zip
최종 배포물: byul.dll 또는 libbyul.so, public headers
개발 산출물: 모듈 static library, 모듈별 test exe, 통합 test exe
```

`package_zip`은 단독 실행 대상입니다. 먼저 `all` target을 최신 상태로 만든 뒤
임시 package layout에 설치하고 `byul.zip`을 생성합니다.

## Ubuntu 최소 요건

Ubuntu에서 Linux 빌드와 Windows MinGW 크로스 빌드를 모두 수행하려면 다음이
필요합니다.

```text
필수:
  cmake
  make
  gcc
  g++
  libc 개발 헤더
  zip
  pkg-config

테스트/디버그 권장:
  gdb
  ctest는 cmake 패키지에 포함

Windows MinGW 크로스 빌드:
  gcc-mingw-w64-x86-64
  g++-mingw-w64-x86-64
  mingw-w64-tools

그래픽/SDL 관련 target을 빌드할 경우:
  libsdl2-dev 또는 프로젝트가 요구하는 SDL 패키지
  Vulkan/OpenGL 관련 개발 패키지
```

Ubuntu 설치 예:

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

SDL 또는 GPU 보조 실행 파일을 함께 빌드해야 하면 환경에 맞게 추가 설치합니다.

```bash
sudo apt install -y libsdl2-dev libvulkan-dev mesa-common-dev
```

현재 `win_sdl_release` preset은 `external/SDL3-3.2.18/cmake` 경로를
`CMAKE_PREFIX_PATH`로 사용합니다. 이 preset을 사용하려면 해당 외부 SDL3
패키지가 같은 위치에 준비되어 있어야 합니다.

## Ubuntu Linux 빌드

Debug 빌드:

```bash
cd byul_env/byul
cmake --preset linux-debug
cmake --build --preset build-linux-debug
```

Release 빌드:

```bash
cd byul_env/byul
cmake --preset linux-release
cmake --build --preset build-linux-release
```

빌드 preset은 `package_zip`을 실행합니다. 결과물은 build directory의 package
layout과 `byul.zip`에 생성됩니다.

## Ubuntu에서 Windows MinGW 크로스 빌드

Windows DLL용 MinGW64 크로스 빌드:

```bash
cd byul_env/byul
cmake --preset win-release
cmake --build --preset build-win-release
```

이 preset은 다음 toolchain을 사용합니다.

```text
byul/cmake/toolchain-mingw64.cmake
```

현재 toolchain은 다음 컴파일러 이름을 기대합니다.

```text
x86_64-w64-mingw32-gcc
x86_64-w64-mingw32-g++
x86_64-w64-mingw32-windres
```

즉 Ubuntu에서는 MinGW 크로스 컴파일러가 PATH에서 보여야 합니다.

## Windows native MinGW 빌드

다음 preset은 Ubuntu용이 아니라 Windows MSYS2 MinGW shell에서 실행하는
구성입니다.

```text
win-native
win-native-debug
```

Windows MSYS2에서 필요한 패키지 예:

```bash
pacman -S --needed \
  mingw-w64-x86_64-cmake \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-make \
  mingw-w64-x86_64-pkgconf \
  mingw-w64-x86_64-zip
```

실행 예:

```bash
cd /path/to/byul_env/byul
cmake --preset win-native
cmake --build --preset build-win-native
```

## MSVC 빌드 분석

현재 preset은 다음과 같습니다.

```text
win-msvc-release
win-msvc-debug
```

이 preset은 Visual Studio 2022 generator를 사용하고, `cmakeExecutable`도 Windows
경로를 가리킵니다.

```text
Visual Studio 17 2022
C:/Program Files/Microsoft Visual Studio/2022/Community/.../cmake.exe
```

따라서 현재 구성 그대로는 Ubuntu에서 직접 실행할 수 없습니다. Ubuntu에는
Visual Studio generator와 MSVC compiler toolchain이 없기 때문입니다.

Ubuntu에서 가능한 선택지는 다음입니다.

```text
가능:
  MinGW64 크로스 컴파일로 Windows DLL/EXE 생성
  Windows VM, Windows 머신, GitHub Actions Windows runner에서 MSVC preset 실행
  원격 Windows 빌드 서버에서 MSVC preset 실행

현재 비권장/미구성:
  Wine 위에서 Visual Studio CMake를 실행하는 방식
  Ubuntu clang-cl로 MSVC ABI를 흉내 내는 방식
```

MSVC 산출물을 정식으로 보장하려면 Windows 환경 또는 Windows CI가 필요합니다.
Ubuntu에서 `win-msvc-*` preset을 직접 돌리는 것은 현재 목표가 아닙니다.

## 테스트 실행

CMake test preset은 아직 비어 있으므로 CTest를 직접 실행합니다.

Linux Debug:

```bash
ctest --test-dir ../build_debug --output-on-failure
```

Linux Release:

```bash
ctest --test-dir ../build_release --output-on-failure
```

Windows MinGW 크로스 빌드 산출물은 Ubuntu에서 바로 실행하지 않습니다. Wine을
쓰면 일부 실행은 가능할 수 있지만, 현재 공식 검증 경로로 문서화하지 않습니다.

MSVC multi-config 빌드는 Windows에서 configuration을 지정합니다.

```bash
ctest --test-dir ../build_win_msvc_release -C Release --output-on-failure
ctest --test-dir ../build_win_msvc_debug -C Debug --output-on-failure
```

## 주요 preset

```text
linux-debug          Ubuntu Debug 빌드
linux-release        Ubuntu Release 빌드
win-release          Ubuntu에서 MinGW64로 Windows DLL 크로스 빌드
win_sdl_release      Ubuntu에서 MinGW64로 Windows SDL 실행 파일 크로스 빌드
win-native           Windows MSYS2 MinGW Release 빌드
win-native-debug     Windows MSYS2 MinGW Debug 빌드
win-msvc-release     Windows Visual Studio 2022 Release 빌드
win-msvc-debug       Windows Visual Studio 2022 Debug 빌드
```

## 주요 target

```text
all                 library, 보조 실행 파일, 테스트 실행 파일 빌드
byul                최종 shared library target만 빌드
test_core           core 모듈 테스트 실행 파일 빌드
test_number_theory  number_theory 모듈 테스트 실행 파일 빌드
test_dstar_lite     D* Lite 모듈 테스트 실행 파일 빌드
test_byul           통합 테스트 실행 파일 빌드
test                등록된 CTest 테스트 실행
install             CMAKE_INSTALL_PREFIX 아래로 설치
package_zip         all 갱신, package layout 설치, byul.zip 생성
uninstall           CMake가 기록한 설치 파일 제거
```
