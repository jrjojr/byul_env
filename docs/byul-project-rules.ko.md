# BYUL 프로젝트 작업 규칙

이 문서는 BYUL 저장소에서 코드, 테스트, 빌드 문서, 진단 기록을 다룰 때 먼저
확인해야 하는 프로젝트 전용 규칙입니다.

## 적용 범위

이 규칙은 다음 저장소에 적용합니다.

```text
/home/sajang/docs/byul_env
```

BYUL은 C++17 기반 시뮬레이션 엔진 프로젝트입니다. 코드, 테스트, 빌드 문서,
진단 기록은 이 문서의 기준에 따라 판단합니다.

## 프로젝트 성격

BYUL은 C++17 구현과 C 호환 ABI를 함께 사용하는 시뮬레이션 엔진입니다.

공개 산출물은 다음 구조를 기준으로 합니다.

```text
C++17 내부 구현
C 호환 public ABI
단일 shared library: byul.dll 또는 libbyul.so
모듈별 static library
모듈별 테스트와 통합 테스트
```

공개 API를 수정할 때는 다음 원칙을 지킵니다.

- 공개 헤더에는 `std::vector`, `std::string`, C++ class를 직접 노출하지 않는다.
- 공개 함수는 `extern "C"` 경계와 `BYUL_API` 적용 여부를 확인한다.
- BYUL이 생성한 객체는 BYUL의 destroy/free 함수로 해제하게 한다.
- C++ 예외가 ABI 경계 밖으로 나가지 않게 한다.
- 공개 구조체 레이아웃 변경은 호환성 영향을 먼저 판단한다.

## 작업 전 확인 순서

BYUL 작업을 시작할 때는 작업 성격에 맞게 다음 문서를 먼저 확인합니다.

```text
docs/byul-project-rules.ko.md
README.ko.md
docs/build-environment.ko.md
docs/runtime-execution.ko.md
```

테스트 실행 결과나 논리 검증 관련 작업이면 다음 문서도 확인합니다.

```text
docs/test-byul-result.ko.md
```

특정 모듈 작업이면 해당 모듈의 `CMakeLists.txt`, public header, tests 디렉터리를
함께 확인합니다.

## 코드 작업 기준

기존 모듈 구조와 파일 배치를 우선합니다.

- 새 기능은 관련 모듈 아래에 둔다.
- public header와 내부 구현의 경계를 분리한다.
- 같은 개념의 create/init/free/destroy 패턴을 기존 코드와 맞춘다.
- 테스트만 고칠 때도 실제 코드 버그를 가리는지 함께 판단한다.
- 단순 출력 정리와 동작 변경을 한 커밋 성격 안에서 섞지 않는다.

## 테스트 기준

BYUL 테스트는 CMake 빌드 산출물과 doctest 기반 실행 파일을 기준으로 확인합니다.

대표 실행 대상:

```text
build_debug/bin/test_byul
build_debug/bin/test_number_theory
```

실행 파일과 `byul.dll`/`libbyul.so`의 런타임 연결 방식은
`docs/runtime-execution.ko.md`를 기준으로 확인합니다.

전체 테스트가 통과하더라도 논리적으로 충분히 검증하는지 따로 확인합니다.
특히 다음 유형을 주의합니다.

- 출력은 이상한데 assertion이 너무 느슨해서 통과하는 경우
- `NULL` 또는 빈 자료구조 때문에 테스트가 실제 변경을 검증하지 못하는 경우
- 결과 객체를 검사하면서 다른 객체를 출력하는 경우
- 경로 탐색, 충돌, trajectory처럼 허용 오차와 좌표 의미가 중요한 경우

테스트 결과를 문서에 남길 때는 `docs/` 아래에 한국어 문서로 기록하고,
실패 여부와 논리 검증 항목을 분리합니다.

## 문서 작성 기준

BYUL 문서는 기본적으로 `docs/` 아래에 둡니다.

- 한국어 문서는 `.ko.md`를 사용한다.
- 파일명은 영문 소문자, 숫자, 하이픈을 사용한다.
- 빌드/테스트/진단/규칙 문서는 공개 README와 분리한다.
- 이미 확인한 사실과 후속 수정 후보를 구분해서 쓴다.

README는 프로젝트 소개와 사용자가 바로 알아야 할 큰 구조를 담고, 세부 진단이나
작업 규칙은 `docs/` 문서로 분리합니다.

## TODO와 후속 작업

BYUL의 후속 작업은 코드 수정 대상과 문서 기록 대상을 먼저 구분합니다.

후속 작업은 다음 중 알맞은 방식으로 남깁니다.

- 짧은 진단 기록: `docs/*result*.ko.md`
- 빌드/환경 지식: `docs/build-environment.ko.md`
- 프로젝트 공통 규칙: `docs/byul-project-rules.ko.md`
- 실제 코드 수정이 필요한 항목: 테스트 또는 관련 코드 주석이 아니라 별도 문서에
  먼저 정리한 뒤 수정한다.

사용자가 “진행하라”, “수정하라”처럼 실제 작업을 지시하면 문서에 남긴 후속
항목을 기준으로 코드와 테스트를 수정합니다.
