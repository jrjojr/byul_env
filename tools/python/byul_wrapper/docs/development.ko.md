# BYUL Wrapper 개발 가이드

자동 생성기의 명명, 소유권, 변환, lint, test 규칙은
[`wrapper-rules.org`](wrapper-rules.org)를 기준으로 합니다.

## 개발 목표

public C ABI 하나마다 대응하는 저수준 Python `c_*` wrapper를 제공합니다.
고수준 `Coord`, facade, 도메인 서비스는 이 프로젝트에서 작성하지 않습니다.

## ABI 선언 생성

Windows:

```bat
tools\python\byul_wrapper\generate_wrapper_abi.bat
```

Linux 또는 macOS:

```sh
tools/python/byul_wrapper/generate_wrapper_abi.sh
```

변경 여부만 검사:

```bat
tools\python\byul_wrapper\generate_wrapper_abi.bat --check
```

생성기는 `MODULE_HEADERS`에 등록된 public header를 읽고 각 module의 첫 번째
`ffi.cdef()` block을 교체합니다. 생성 block을 직접 수정하지 않습니다.

## 새 wrapper 추가 절차

1. C header가 C 호환 public ABI인지 확인합니다.
2. header 선언과 DLL 구현 signature를 비교합니다.
3. `MODULE_HEADERS`에 header와 대상 module을 등록합니다.
4. ABI 생성기를 실행합니다.
5. 해당 C 타입에 대응하는 `c_*` class를 작성합니다.
6. 모든 반환 pointer의 owned/borrowed/transferred 의미를 확인합니다.
7. destroy, NULL, GC, callback, thread 테스트를 작성합니다.
8. 문서와 coverage 상태를 갱신합니다.

## 소유권 규칙

```text
create / copy  호출자 소유로 반환되는지 확인하고 own=True 적용
get / peek     parent 내부 pointer인지 확인하고 own=False 적용
fetch          Python이 준비한 값 또는 buffer로 복사
close          own=True인 객체에만 정확한 destroy 함수 호출
```

함수 이름만으로 추정하지 않고 C header, 구현, native test를 함께 확인합니다.

## 필수 테스트

- 생성과 destroy
- `close()` 중복 호출
- close 이후 method 호출 차단
- 부모 제거 전후 borrowed child 처리
- NULL 반환
- copy 독립성
- callback 객체 강제 GC
- callback Python 예외
- worker thread 실행 중 종료
- 반복 생성·해제 메모리 안정성
- DLL symbol과 header signature 일치

## 생성기의 한계

자동 생성 가능한 것은 C 선언입니다. 다음 항목은 수동 검토 대상입니다.

- pointer 소유권
- callback 호출 thread와 userdata 수명
- 문자열 encoding과 해제 함수
- 반환 buffer 크기
- native 오류를 Python에 표현하는 방식
- C struct ABI 호환성

ABI 생성 성공만으로 wrapper 완료로 판단하지 않습니다.
