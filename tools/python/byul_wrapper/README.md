# BYUL Wrapper

`byul_wrapper`는 BYUL C++ 엔진의 공개 C ABI(`byul.dll`, `libbyul.so`,
`libbyul.dylib`)를 Python에서 직접 사용할 수 있게 만드는 독립 저수준 바인딩
프로젝트입니다.

프로젝트의 목표는 BYUL public C API를 Python의 `c_*` wrapper로 빠짐없이
제공하는 것입니다. 범위는 `coord_t`에 대응하는 `c_coord`까지이며, 별도의
고수준 `Coord` 값 객체나 애플리케이션 facade는 이 프로젝트의 목표가 아닙니다.

```python
from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid

with c_coord(10, 20) as coord:
    grid = c_navgrid()
    print(coord.to_tuple())
```

## 문서

- [설치와 사용법](docs/usage.ko.md)
- [구조와 설계 원칙](docs/architecture.ko.md)
- [개발 및 ABI 자동 생성](docs/development.ko.md)
- [DLL과 Python Wrapper 테스트](docs/testing.ko.md)
- [완전 자동 생성 TODO](docs/todo.ko.md)
- [전체 DLL 지원 로드맵](docs/roadmap.ko.md)
- [Wrapper 자동 생성 상세 규칙](docs/wrapper-rules.org)
- [Public C Header 선언 규칙](../../../docs/ko/byul-sdk/header-declaration-rules.org)
- [Public C Header 코멘트 규칙](../../../docs/ko/byul-sdk/header-comment-rules.org)
- [Public C Header 표준 라이선스 블록](../../../docs/ko/byul-sdk/header-license-block.txt)

## 프로젝트 구조

```text
tools/python/byul_wrapper/
├─ pyproject.toml
├─ generate_wrapper_abi.py
├─ byul_wrapper/          저수준 c_* wrapper package
├─ tests/
└─ docs/
```

`byul_grid`는 이 프로젝트의 첫 번째 소비자입니다. wrapper는 GUI, PySide6,
World, NPC 같은 애플리케이션 객체에 의존하지 않습니다.

## 생성 범위

헤더에서 함수·enum·구조체 선언을 읽어 `ffi.cdef()`를 자동 생성합니다. `c_coord`
같은 Python wrapper는 C 객체 생성, 해제, borrowed pointer, callback 수명을
관리합니다. 이 소유권 정보는 C header만으로 판단할 수 없으므로 wrapper class는
검토와 테스트를 거쳐 작성합니다.
