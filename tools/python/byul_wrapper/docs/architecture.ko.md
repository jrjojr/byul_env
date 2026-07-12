# BYUL Wrapper 구조와 설계 원칙

구체적인 자동 생성 규칙은 [`wrapper-rules.org`](wrapper-rules.org)에 정의합니다.

## 목표

BYUL의 공개 C ABI를 Python에서 가능한 한 동일한 의미로 호출할 수 있게 합니다.

```text
Python application
    ↓
c_coord / c_navgrid / c_route / c_dstar_lite / ...
    ↓
generated CFFI declarations
    ↓
BYUL public C ABI
    ↓
byul.dll / libbyul.so / libbyul.dylib
```

이 프로젝트의 결과물은 저수준 `c_*` wrapper까지입니다. `c_coord` 위에 별도의
`Coord`를 만들거나 모든 기능을 하나의 `Byul` class로 합치는 일은 범위에
포함하지 않습니다. 필요한 고수준 도메인 API는 이 패키지를 사용하는 별도
애플리케이션 또는 별도 SDK 계층에서 설계합니다.

## 두 계층

### ABI 선언 계층

BYUL public header에서 다음 내용을 `ffi.cdef()` 입력으로 생성합니다.

- public struct와 opaque struct 선언
- enum
- function pointer typedef
- `BYUL_API` 함수 signature

ABI 생성은 수동 header 복사를 제거하지만 소유권까지 결정하지는 않습니다.

### `c_*` 객체 계층

각 C 객체를 대응하는 Python class로 감쌉니다.

```text
coord_t*          → c_coord
coord_list_t*     → c_coord_list
navgrid_t*        → c_navgrid
route_t*          → c_route
route_finder_t*   → c_route_finder
dstar_lite_t*     → c_dstar_lite
```

이 계층은 다음을 책임집니다.

- create/copy 결과를 정확한 destroy 함수로 해제
- get/peek 반환 borrowed pointer를 파괴하지 않음
- borrowed child가 parent보다 오래 살아남지 않게 함
- NULL 반환을 Python 값 또는 예외로 변환
- callback 함수와 userdata handle 수명 보존
- close 후 pointer 재사용 방지
- C 함수 결과를 최소한의 Python 기본형으로 변환

## 의존성 방향

```text
byul_grid ──→ byul_wrapper ──→ BYUL shared library
other app ──→ byul_wrapper ──→ BYUL shared library
```

`byul_wrapper`는 소비자 애플리케이션을 import하지 않습니다.

## API 원칙

- public wrapper class 이름은 C 타입과 대응하는 `c_*` 형식을 사용합니다.
- C 함수와 wrapper method의 관계를 추적할 수 있어야 합니다.
- raw `ffi`와 DLL handle은 wrapper 내부 구현으로 취급합니다.
- 편의를 위해 C 동작을 숨기거나 별도의 값 의미를 추가하지 않습니다.
- 고수준 Python 객체가 필요하면 소비자 계층에서 `c_*`를 조합합니다.
