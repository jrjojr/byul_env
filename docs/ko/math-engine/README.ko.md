# BYUL Math Engine 프로젝트

BYUL Math Engine은 임베디드 CAS 공학용 그래프 계산기를 최종 reference product로
삼는 재사용 가능한 수학 SDK 프로젝트입니다.

계산기 전용 코드로 만들지 않습니다. 게임, 물리 시뮬레이션, 데스크톱 GUI, CLI,
Python wrapper, 교육 도구와 batch/GPU 계산에서도 동일한 C ABI와 계산 계약을
사용하는 것이 핵심 목표입니다.

## 다섯 가지 대표 사용처

```text
BYUL Math Engine
├─ 임베디드 공학 계산
├─ 게임 시뮬레이션
├─ 공학·과학 시뮬레이션
├─ 금융 계산
└─ CAD·강건 기하
```

- 임베디드 공학 계산: CAS 그래프 계산기, 계측기와 현장 진단 장비
- 게임 시뮬레이션: 실시간 물리, collision, entity, projectile와 trajectory
- 공학·과학 시뮬레이션: 기계·물리, 로봇·제어, 항공우주와 전기전자
- 금융 계산: fixed decimal, mpdecimal, rounding, quantize와 재현 가능한 계산
- CAD·강건 기하: raycast, curve/surface, NURBS, mesh, B-rep와 선택 OCCT

게임과 공학은 별도 simulation engine이 아니다. 같은 simulation core를 사용하고
게임 runtime은 속도 모드, 공학·offline 검증은 정밀도 모드를 기본으로 한다. 두
분야 모두 필요에 따라 다른 모드나 계산별 override를 사용할 수 있다.

## 최종 목표

```text
BYUL Math SDK
  ├─ 다른 애플리케이션에 내장 가능한 수학 엔진
  ├─ 안정적인 C ABI와 C++17 내부 구현
  ├─ static/shared build와 임베디드 profile
  └─ reference product: 임베디드 CAS 공학용 그래프 계산기
```

그래프 계산기는 엔진의 유일한 사용처가 아니라 다음 기능을 통합 검증하는 대표
애플리케이션입니다.

- exact와 approximate 수학
- 수치해석
- 단위·수량과 공학 상수
- 표현식, 제한적 CAS와 반복 평가
- Cartesian, parametric, polar와 implicit graph
- 제한된 RAM·flash·계산 예산
- worksheet 저장과 결과 재현성

## 설계 원칙

- 수학 알고리즘은 GUI, keypad, renderer와 worksheet를 모릅니다.
- 표현식 parser는 수학 함수를 다시 구현하지 않습니다.
- 그래프는 compile된 식과 renderer 독립 plot data를 사용합니다.
- 게임과 시뮬레이션은 문자열 parser 없이 수학 C API를 직접 호출할 수 있습니다.
- 다른 앱은 필요한 모듈만 선택해 static 또는 shared library로 연결합니다.
- public API에는 C++ 표준 컨테이너와 외부 backend 타입을 노출하지 않습니다.
- generated parser, GPU와 arbitrary precision은 선택 기능입니다.
- OCCT 기반 B-rep/CAD backend는 full profile의 선택 기능이며 기본 build에는 필요하지
  않습니다.
- mpdecimal은 full profile의 arbitrary decimal 선택 backend이며 minimal profile은
  fixed decimal 또는 integer coefficient/scale 경로를 사용합니다.
- 지원하지 않는 기능은 정밀도를 몰래 낮추지 않고 명시적 상태를 반환합니다.

## 문서 구성

- [전체 수학 엔진 계획](todo-math-engine.org)
- [기초 수학과 숫자 backend](todo-numal.org)
- [수치해석](todo-numeq.org)
- [단위·수량과 공학 상수](todo-mathunits.org)
- [표현식, parser, bytecode와 제한적 CAS](todo-expression.org)
- [기하, 곡선·곡면, 충돌·접촉, NURBS와 B-rep](todo-geometry.org)
- [공통 simulation context와 속도·정밀도 모드](todo-simulation-context.org)
- [프로젝트 아키텍처와 제품 계획](project.ko.md)

## 목표 산출물

초기에는 BYUL의 단일 shared library 배포를 유지하되 내부 target 경계를 먼저
확립합니다. API와 사용 패턴이 안정되면 다음 산출물을 제공하는 방향을 검토합니다.

```text
Shared SDK
  byul_math.dll / libbyul_math.so
  byul_geometry.dll / libbyul_geometry.so             선택
  byul_geometry_occt.dll / libbyul_geometry_occt.so   선택, full/CAD profile
  byul_expression.dll / libbyul_expression.so       선택

Static SDK
  byul_math_static
  byul_geometry_static                               선택
  byul_expression_static                            선택

Applications
  byul_calculator                                   reference product
  byul_math_cli                                     진단·자동화
  Python wrapper                                    교육·검증·도구
```

실제 DLL 분리 전에도 각 모듈은 분리된 CMake target과 한 방향 의존성을 가져야 합니다.
OCCT를 배포하는 package는 LGPL 2.1과 OCCT exception을 적용받는 third-party component를
BYUL license와 명확히 분리하고 notice, license와 source 접근 정보를 포함해야 합니다.

## 구현 profile

```text
full
  desktop/Linux application processor
  f64/f128, arbitrary precision, CAS와 모든 graph mode

compact
  고성능 MCU/RTOS 또는 저사양 application processor
  f32/f64, 제한 행렬·CAS와 주요 graph mode

minimal
  소형 MCU 또는 작은 내장 도구
  기본 함수, 고정 workspace와 제한 expression
```

모든 profile은 가능한 범위에서 같은 public 계약과 conformance corpus를 공유합니다.

## 현재 단계

현재 문서는 구현 전 아키텍처와 마이그레이션 계획입니다. 첫 구현은 GUI가 아니라
기존 수학 기능 inventory, 회귀 테스트, 실제 CMake target 경계와 최소 `f64`
expression evaluator를 확립하는 작업입니다.
