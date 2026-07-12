# BYUL Python Wrapper 테스트

이 문서는 새 환경에서 `tools/python/byul_wrapper/tests`를 실행하기 위해 필요한
Python 확인, 가상환경 생성, dependency 설치, BYUL shared library 준비와 테스트
명령을 순서대로 설명한다. 명령은 별도 설명이 없으면 저장소 루트에서 실행한다.

## 1. 요구 환경

- Python 3.10 이상
- Python과 같은 architecture로 build한 BYUL shared library
- Python `venv`와 `pip`
- 전체 테스트에는 `cffi`와 `pytest`
- BYUL을 직접 build할 경우 CMake와 platform compiler

Platform별 library 이름:

```text
Windows  byul.dll
Linux    libbyul.so
macOS    libbyul.dylib
```

Windows에서는 Python을 다음 순서로 확인한다.

1. `BYUL_PYTHON` 환경 변수
2. `tools/python/.venv/Scripts/python.exe`
3. `C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe`
4. `py -3`
5. `python`

Linux와 macOS에서는 `BYUL_PYTHON`, `tools/python/.venv/bin/python`, `python3`
순서로 확인한다.

## 2. Python 가상환경과 dependency 설치

### 자동 설치

Setup script는 공용 가상환경 `tools/python/.venv`를 만들고 runtime, test와 package
dependency를 설치한다. `byul_wrapper` package도 editable mode로 설치된다.

Windows:

```bat
tools\python\setup_env.bat
```

Linux 또는 macOS:

```sh
tools/python/setup_env.sh
```

설치되는 테스트 관련 주요 package는 다음과 같다.

```text
byul-wrapper (editable)
cffi 1.17.1
pytest 8.4.1
```

### 수동 설치

Setup script를 사용하지 않을 경우 `tools/python`에서 직접 설치한다.

Windows:

```bat
cd tools\python
python -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

Linux 또는 macOS:

```sh
cd tools/python
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-dev.txt
```

설치 확인:

```bat
tools\python\.venv\Scripts\python.exe --version
tools\python\.venv\Scripts\python.exe -c "import cffi, pytest; print(cffi.__version__, pytest.__version__)"
```

Linux와 macOS에서는 interpreter 경로를 `tools/python/.venv/bin/python`으로 바꾼다.

기존 `.venv`가 더 이상 존재하지 않는 Python installation을 가리키면 해당
가상환경을 새 Python으로 다시 생성한 뒤 dependency를 설치한다.

## 3. BYUL shared library 준비

이미 build한 shared library가 있다면 다음 절의 `BYUL_LIBRARY_PATH` 지정으로
이동한다. 직접 build할 경우 platform에 맞는 BYUL CMake preset을 사용한다.

Windows MSVC Release:

```bat
cd byul
cmake --preset win-msvc
cmake --build --preset build-win-msvc-release
cd ..
```

기본 DLL 경로:

```text
build_win_msvc/bin/Release/byul.dll
```

Linux Debug:

```sh
cd byul
cmake --preset linux-debug
cmake --build ../build_debug
cd ..
```

기본 library 경로:

```text
build_debug/lib/libbyul.so
```

다른 build preset과 compiler 준비 방법은 `docs/ko/build-environment.ko.md`를
따른다.

## 4. Shared library 지정

Wrapper 동작 테스트는 현재 platform과 Python architecture에 맞는 BYUL shared
library가 필요하다.

Windows PowerShell:

```powershell
$env:BYUL_LIBRARY_PATH = (Resolve-Path build_win_msvc/bin/Release/byul.dll).Path
```

Windows 명령 프롬프트:

```bat
set "BYUL_LIBRARY_PATH=C:\absolute\path\to\byul.dll"
```

Linux:

```sh
export BYUL_LIBRARY_PATH="$(pwd)/build_release/lib/libbyul.so"
```

macOS:

```sh
export BYUL_LIBRARY_PATH="$(pwd)/build_macos_release/lib/libbyul.dylib"
```

생성기와 header parser만 검사하는 단위 테스트에는 shared library가 필요하지 않다.

Library 경로 확인:

```powershell
Test-Path $env:BYUL_LIBRARY_PATH
```

Linux 또는 macOS:

```sh
test -f "$BYUL_LIBRARY_PATH"
```

## 5. pytest 전체 실행

### 통합 실행 script

`run_tests.py`가 `argparse`로 test framework, 범위와 DLL 경로를 처리한다. 별도 option이
없으면 pytest로 `tests` 디렉터리 전체를 실행한다.

Windows:

```bat
tools\python\byul_wrapper\run_tests.bat
```

Linux 또는 macOS:

```sh
sh tools/python/byul_wrapper/run_tests.sh
```

DLL 경로를 command line에서 지정할 수 있다.

```bat
tools\python\byul_wrapper\run_tests.bat --library build_win_msvc\bin\Release\byul.dll
```

```sh
sh tools/python/byul_wrapper/run_tests.sh \
  --library build_release/lib/libbyul.so
```

사용 가능한 option:

```text
--framework pytest|unittest  실행 framework 선택, 기본값 pytest
--unit-only                 DLL이 필요 없는 생성기/parser/runner test만 실행
--library PATH              BYUL shared library 경로 지정
--quiet                     test 출력 축소
--dry-run                   실제 실행 없이 하위 명령만 출력
```

도움말:

```bat
tools\python\byul_wrapper\run_tests.bat --help
```

```sh
sh tools/python/byul_wrapper/run_tests.sh --help
```

`run_tests.bat`는 `BYUL_PYTHON`, 공용 `.venv`, Windows Python 3.13 기본 경로,
`py -3`, `python` 순서로 interpreter를 찾는다. `run_tests.sh`는 `BYUL_PYTHON`,
공용 `.venv`, `python3` 순서로 찾는다. 두 script 모두 모든 argument를
`run_tests.py`에 전달한다.

### pytest 직접 실행

Windows:

```bat
tools\python\.venv\Scripts\python.exe -m pytest tools\python\byul_wrapper\tests -v
```

Linux 또는 macOS:

```sh
tools/python/.venv/bin/python -m pytest tools/python/byul_wrapper/tests -v
```

`tools/python/byul_wrapper`에서 실행할 경우:

```bat
..\.venv\Scripts\python.exe -m pytest tests -v
```

## 6. unittest 전체 실행

Python 표준 `unittest` discovery로 `test_*.py` 전체를 실행할 수 있다.

통합 실행 script 사용:

```bat
tools\python\byul_wrapper\run_tests.bat --framework unittest
```

```sh
sh tools/python/byul_wrapper/run_tests.sh --framework unittest
```

Windows:

```bat
tools\python\.venv\Scripts\python.exe -m unittest discover -s tools\python\byul_wrapper\tests -p "test_*.py" -v
```

Linux 또는 macOS:

```sh
tools/python/.venv/bin/python -m unittest discover \
  -s tools/python/byul_wrapper/tests \
  -p 'test_*.py' -v
```

## 7. 생성기 단위 테스트만 실행

DLL을 load하지 않고 ABI 생성기, header parser, test runner와 `c_*` class test
coverage를 검사한다.

권장 명령:

```bat
tools\python\byul_wrapper\run_tests.bat --unit-only
```

```sh
sh tools/python/byul_wrapper/run_tests.sh --unit-only
```

Windows:

```bat
tools\python\.venv\Scripts\python.exe -m pytest tools\python\byul_wrapper\tests\test_generate_wrapper_abi.py tools\python\byul_wrapper\tests\test_header_parser.py tools\python\byul_wrapper\tests\test_tests_runner.py tools\python\byul_wrapper\tests\test_wrapper_coverage.py -v
```

Linux 또는 macOS:

```sh
tools/python/.venv/bin/python -m pytest \
  tools/python/byul_wrapper/tests/test_generate_wrapper_abi.py \
  tools/python/byul_wrapper/tests/test_header_parser.py \
  tools/python/byul_wrapper/tests/test_tests_runner.py \
  tools/python/byul_wrapper/tests/test_wrapper_coverage.py -v
```

표준 `unittest`로 실행할 수도 있다.

```bat
tools\python\.venv\Scripts\python.exe -m unittest discover -s tools\python\byul_wrapper\tests -p "test_generate_wrapper_abi.py" -v
tools\python\.venv\Scripts\python.exe -m unittest discover -s tools\python\byul_wrapper\tests -p "test_header_parser.py" -v
tools\python\.venv\Scripts\python.exe -m unittest discover -s tools\python\byul_wrapper\tests -p "test_tests_runner.py" -v
tools\python\.venv\Scripts\python.exe -m unittest discover -s tools\python\byul_wrapper\tests -p "test_wrapper_coverage.py" -v
```

## 8. 개별 테스트 실행

파일 하나:

```bat
tools\python\.venv\Scripts\python.exe -m pytest tools\python\byul_wrapper\tests\test_navgrid.py -v
```

테스트 하나:

```bat
tools\python\.venv\Scripts\python.exe -m pytest tools\python\byul_wrapper\tests\test_navgrid.py::TestMapMakeAtDegree::test_neighbor_at_degree -v
```

## 9. 테스트 결과 확인

성공 시 process exit code는 0이다. 실패 시 첫 traceback과 함께 다음을 확인한다.

- `BYUL_LIBRARY_PATH`가 테스트할 library의 절대 경로인지
- Python과 shared library의 architecture가 같은지
- DLL과 generated CFFI 선언이 같은 header version인지
- 누락된 native symbol이 있는지
- 실패가 assertion, import, library load 중 어디에서 발생했는지

CI에서는 exit code가 0이 아니면 테스트 실패로 처리한다.
