# BYUL 실행 방법

이 문서는 BYUL 빌드 산출물을 실행하는 방법을 정리합니다. 빌드 환경과 설치
요건은 [빌드 환경 구성](build-environment.ko.md)을 먼저 확인합니다.

## 산출물 위치

CMake preset으로 빌드하면 일반적으로 다음 위치에 산출물이 만들어집니다.

```text
Linux Debug:
  build_debug/bin/       테스트 실행 파일, 보조 실행 파일
  build_debug/lib/       libbyul.so

Linux Release:
  build_release/bin/     테스트 실행 파일, 보조 실행 파일
  build_release/lib/     libbyul.so

Windows MinGW cross:
  build_win_release/     Windows용 빌드 트리
  package_tmp/.../bin/   byul.dll, test_*.exe, MinGW runtime DLL
  package_tmp/.../lib/   import/static archive
```

`package_zip`을 실행하면 임시 package layout과 `byul.zip`이 만들어집니다. 실제
배포나 다른 PC 확인은 build tree보다 package layout 또는 zip을 기준으로 보는
것이 안전합니다.

## CTest로 실행

가장 권장하는 테스트 실행 방법은 CTest입니다.

```bash
cd byul_env/byul
ctest --test-dir ../build_debug --output-on-failure
```

특정 테스트만 실행할 수도 있습니다.

```bash
ctest --test-dir ../build_debug -R test_byul --output-on-failure
ctest --test-dir ../build_debug -R test_number_theory --output-on-failure
ctest --test-dir ../build_debug -R test_dstar_lite --output-on-failure
```

`-R`은 정규식 매칭입니다. 예를 들어 `test_byul`은 `test_byul_tick`까지 같이
잡을 수 있습니다. 통합 테스트 실행 파일만 정확히 실행하려면 다음처럼 끝 매칭을
사용합니다.

```bash
ctest --test-dir ../build_debug -R 'test_byul$' --output-on-failure
```

## Linux에서 테스트 실행 파일 직접 실행

Linux Debug 빌드 후 테스트 실행 파일은 `build_debug/bin/` 아래에 있습니다.

```bash
../build_debug/bin/test_byul
../build_debug/bin/test_core
../build_debug/bin/test_number_theory
../build_debug/bin/test_dstar_lite
```

저장소 루트에서 실행한다면 다음처럼 실행합니다.

```bash
./build_debug/bin/test_byul
./build_debug/bin/test_number_theory
```

Linux 테스트 실행 파일은 빌드 설정에서 다음 RPATH를 가집니다.

```text
$ORIGIN/../lib
```

따라서 `build_debug/bin/test_byul`은 보통 자동으로
`build_debug/lib/libbyul.so`를 찾습니다.

## Linux에서 libbyul.so를 사용하는 외부 실행 파일

외부 프로그램이 `libbyul.so`를 사용하려면 두 가지가 필요합니다.

```text
컴파일/링크 시점:
  public header include 경로
  libbyul.so 링크 경로

실행 시점:
  dynamic loader가 libbyul.so를 찾을 수 있는 경로
```

간단한 예:

```bash
g++ main.cpp \
  -I./build_debug/package_tmp/home/sajang/byul/include/byul \
  -L./build_debug/package_tmp/home/sajang/byul/lib \
  -lbyul \
  -Wl,-rpath,'$ORIGIN/../lib' \
  -o my_app
```

실행 파일 옆 구조를 다음처럼 두면 rpath로 찾을 수 있습니다.

```text
my_package/
  bin/my_app
  lib/libbyul.so
```

rpath를 넣지 않았다면 실행 전에 `LD_LIBRARY_PATH`를 지정합니다.

```bash
LD_LIBRARY_PATH=/path/to/byul/lib ./my_app
```

시스템 경로에 설치한 경우에는 `/etc/ld.so.conf.d/` 등록과 `ldconfig`를 사용할
수 있지만, BYUL 개발 중에는 package layout이나 rpath를 우선 사용합니다.

## Windows에서 test_*.exe 실행

Windows에서는 DLL 검색 규칙 때문에 실행 파일이 `byul.dll`을 찾을 수 있어야
합니다. 가장 단순한 구조는 `exe`와 `dll`을 같은 `bin` 디렉터리에 두는 것입니다.

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

MinGW 크로스 빌드에서는 `byul.dll` 외에 MinGW runtime DLL도 필요합니다. 현재
CMake 설정은 다음 DLL을 package `bin`에 함께 넣도록 구성되어 있습니다.

```text
libstdc++-6.dll
libgcc_s_seh-1.dll
libwinpthread-1.dll
```

Windows에서 실행 예:

```bat
cd C:\path\to\byul\bin
test_byul.exe
test_number_theory.exe
```

`byul.dll`이 같은 디렉터리에 없다면 `PATH`에 `byul.dll`이 있는 디렉터리를
추가해야 합니다.

```bat
set PATH=C:\path\to\byul\bin;%PATH%
test_byul.exe
```

## Windows에서 byul.dll을 외부 프로그램에 연결

Windows 실행 파일이 `byul.dll`을 사용하려면 빌드 시 import library가 필요하고,
실행 시 `byul.dll`이 필요합니다.

```text
빌드 시:
  include/byul/*.h
  lib/byul import library 또는 MinGW import archive

실행 시:
  byul.dll
  MinGW 빌드라면 MinGW runtime DLL
```

MinGW 예:

```bash
x86_64-w64-mingw32-g++ main.cpp \
  -I/path/to/byul/include/byul \
  -L/path/to/byul/lib/byul \
  -lbyul \
  -o my_app.exe
```

실행 배치는 다음처럼 둡니다.

```text
app/
  my_app.exe
  byul.dll
  libstdc++-6.dll
  libgcc_s_seh-1.dll
  libwinpthread-1.dll
```

MSVC 빌드라면 Visual Studio가 생성한 `.lib` import library로 링크하고,
실행 시 `byul.dll`을 `exe` 옆이나 `PATH`에 둡니다.

## package_zip 결과 확인

패키지를 만든 뒤에는 zip 내부를 기준으로 실행 구조를 확인합니다.

```bash
cd byul_env/byul
cmake --build --preset build-linux-debug
```

또는 직접 target을 실행합니다.

```bash
cmake --build ../build_debug --target package_zip -- -j8
```

Linux package layout 예:

```text
build_debug/package_tmp/home/sajang/byul/
  bin/test_byul
  bin/test_number_theory
  lib/libbyul.so
  include/byul/*.h
```

Windows package layout 예:

```text
build_win_release/package_tmp/home/sajang/cross_win/byul/
  bin/byul.dll
  bin/test_byul.exe
  bin/test_number_theory.exe
  bin/libstdc++-6.dll
  bin/libgcc_s_seh-1.dll
  bin/libwinpthread-1.dll
  include/byul/*.h
```

## 실행 문제 점검

Linux에서 `libbyul.so`를 찾지 못하면 다음을 확인합니다.

```bash
ldd ./build_debug/bin/test_byul
LD_LIBRARY_PATH=./build_debug/lib ./build_debug/bin/test_byul
```

Windows에서 `byul.dll`을 찾지 못하면 다음을 확인합니다.

```text
1. test_*.exe와 byul.dll이 같은 bin 디렉터리에 있는가
2. MinGW runtime DLL이 같은 bin 디렉터리에 있는가
3. byul.dll이 있는 디렉터리가 PATH에 들어 있는가
4. 32비트/64비트 빌드가 섞이지 않았는가
```
