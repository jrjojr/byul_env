# BYUL Wrapper 설치와 사용법

## 요구 환경

- Python 3.10 이상
- 현재 플랫폼과 Python 아키텍처에 맞는 BYUL shared library
- CFFI

공용 Python 환경은 저장소의 `tools/python/.venv`입니다.

```bat
tools\python\setup_env.bat
```

## DLL 위치 지정

Python import 전에 `BYUL_LIBRARY_PATH`를 지정하는 방법이 가장 명확합니다.

```python
import os

os.environ["BYUL_LIBRARY_PATH"] = (
    r"C:\workspace\build_win_msvc\bin\Release\byul.dll"
)

from byul_wrapper.coord import c_coord
```

래퍼는 환경 변수, package의 `bin`, frozen 실행 파일 주변, 저장소 기본 build
경로 순으로 shared library를 찾습니다.

## `c_coord` 사용

```python
from byul_wrapper.coord import c_coord

coord = c_coord(3, 7)
try:
    print(coord.x, coord.y)
    coord.x = 10
    print(coord.to_tuple())
finally:
    coord.close()
```

context manager 사용을 권장합니다.

```python
with c_coord(3, 7) as coord:
    copied = coord.copy()
    try:
        print(coord.distance(copied))
    finally:
        copied.close()
```

`c_coord`는 순수 Python 좌표 값이 아니라 `coord_t*` wrapper입니다. mutable이며
소유 객체는 반드시 정리해야 합니다.

## `c_navgrid`와 경로 탐색

```python
from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid
from byul_wrapper.route_finder import c_route_finder

with c_navgrid(width=100, height=100) as grid:
    with c_coord(0, 0) as start, c_coord(20, 10) as goal:
        with c_route_finder(grid, start=start, goal=goal) as finder:
            route = finder.find()
            if route is not None:
                try:
                    print(route)
                finally:
                    route.close()
```

## owned와 borrowed

새 객체를 반환하는 create/copy 계열은 일반적으로 Python wrapper가 소유합니다.
부모 내부 주소를 반환하는 get/peek 계열은 borrowed 객체일 수 있습니다.

```text
own=True    wrapper close 시 destroy 호출
own=False   wrapper close 시 native object를 파괴하지 않음
```

정확한 소유권은 각 wrapper와 테스트를 기준으로 확인합니다.

## 오류 진단

- DLL을 못 찾음: `BYUL_LIBRARY_PATH` 확인
- `WinError 193`: Python과 DLL 아키텍처 확인
- symbol 누락: DLL과 생성 ABI 버전 확인
- access violation: owned/borrowed, callback 수명, DLL 내부 오류 확인

ABI 동기화 검사:

```bat
tools\python\byul_wrapper\generate_wrapper_abi.bat --check
```
