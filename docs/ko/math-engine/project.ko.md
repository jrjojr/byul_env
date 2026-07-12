# BYUL Math Engine 및 임베디드 CAS 그래프 계산기 프로젝트 계획

## 프로젝트 정의

BYUL Math Engine은 여러 애플리케이션에서 재사용할 수 있는 공학·과학 계산 SDK이며,
임베디드 CAS 공학용 그래프 계산기를 최종 reference product로 개발합니다.

이 프로젝트는 하나의 계산기 프로그램과 하나의 거대한 수학 DLL을 동시에 만드는
방식이 아닙니다. 낮은 계층의 독립 수학 target을 먼저 만들고, expression과 plot을
선택 계층으로 올린 뒤 계산기 application에서 조립합니다.

```text
재사용 SDK가 본체
계산기는 대표 제품이자 통합 검증기
```

## 사용자와 사용처

대표 사용처는 다음 다섯 분야로 고정합니다.

```text
1. 임베디드 공학 계산
2. 게임 시뮬레이션
3. 공학·과학 시뮬레이션
4. 금융 계산
5. CAD·강건 기하
```

로봇, 제어, 항공우주, 전기전자, 계측과 신호처리는 공학·과학 시뮬레이션의 세부
profile로 분류합니다. 사용처를 무제한으로 늘리지 않고 이 다섯 분야가 공유할 기반과
선택 모듈을 설계합니다.

### 임베디드 계산기

- 공학·과학 계산
- 제한적 기호 미분·단순화·방정식 해
- 단위와 공학 상수
- 수치 미분·적분·ODE·최적화
- 함수 graph, trace, table과 교점 분석
- 네트워크가 없는 독립 실행

### 다른 애플리케이션

- 게임 공식과 시뮬레이션
- trajectory와 projectile 계산
- 데스크톱 계산 GUI
- CLI batch 계산
- Python 교육·분석 도구
- 장비 firmware와 계측 보조 기능
- 그래프 또는 수식 편집 component

다른 앱은 계산기 UI나 worksheet를 링크하지 않고 필요한 SDK target만 사용합니다.

## 게임과 공학 시뮬레이션 통합

BYUL에서 게임은 제한된 frame/time budget 안에서 수행되는 simulation입니다. 게임과
공학 계산은 동일한 수학, 수치해석, geometry와 simulation API를 사용합니다.

```text
Simulation Core
├─ speed mode
└─ precision mode
```

- speed mode는 게임 runtime, 대량 entity와 실시간 제어를 기본 대상으로 함
- precision mode는 공학 검증, offline trajectory, CAD refinement와 기준 결과를
  기본 대상으로 함
- 게임에서도 projectile 예측, replay 검증과 애매한 collision refinement에 precision
  mode를 사용할 수 있음
- 공학에서도 real-time controller와 hardware-in-the-loop에 speed mode를 사용할 수 있음

두 모드는 별도 알고리즘 복사본이 아니라 공통 API에 적용되는 preset입니다. numeric
backend, tolerance, iteration, integrator, robust geometry, SIMD/GPU와 memory budget을
context에서 선택합니다.

## 논리 모듈

```text
numal
  scalar, integer, rational, decimal, complex, vector, matrix와 predicate 기반
       ↑
mathfunc
  초월함수, 특수함수와 확률분포
       ↑
numeq
  root, 미분, 적분, ODE, 보간, 회귀와 최적화

numal
  ↑
mathunits
  dimension, unit, quantity, registry와 constants

numal + mathfunc + numeq + mathunits
  ↑
expression
  parser, AST, semantic analysis, bytecode, registry와 limited CAS
       ↑
plot
  adaptive sampling, graph analysis와 renderer 독립 plot data
       ↑
calculator / CLI / Python / game / simulation
```

기하 kernel은 다음 독립 축을 이룹니다.

```text
numal
  vector, matrix, interval과 robust predicate
       ↑
geometry
  primitive, curve, surface, mesh, NURBS, B-rep와 collision/contact
       ↑
plot adapter / CAD 도구 / game / simulation / Python
```

고급 CAD 기능은 다음처럼 선택 backend로 분리합니다.

```text
geometry_core
  BYUL 자체 구현, embedded와 실시간 공통
       ↑
geometry_occt
  OCCT adapter, full/CAD profile 전용 선택 기능
```

허용하지 않는 의존성:

```text
numal -> expression, GUI, balix
numeq -> motion_state, unit registry, renderer
mathunits -> expression, numeq, GUI
expression -> calculator GUI
plot -> 특정 display 또는 window toolkit
geometry -> balix, entity, projectile, renderer
```

## 물리적 source 후보

구현 시 실제 repository 구조는 다음을 후보로 합니다.

```text
byul/
  numal/
  mathfunc/
  numeq/
  mathunits/
  geometry/
  expression/
  plot/

apps 또는 tools/
  byul_calculator/
  byul_math_cli/
```

현재 `balix/numeq`의 범용 알고리즘은 루트 `byul/numeq`로 이동하고, 물리 adapter,
PID, MPC와 Kalman filter는 각각 `balix/dynamics`, `balix/control`과
`balix/estimation` 책임으로 분리합니다.

## API 계층

### 직접 계산 API

게임, simulation과 firmware가 문자열 없이 사용합니다.

```c
double y = math_gamma_f64(5.0);
numeq_find_root_f64(callback, userdata, 0.0, 10.0,
                    &options, &result);
```

### 표현식 API

계산기, GUI와 CLI가 사용합니다.

```text
sin(pi / 3)
solve(x^2 - 5*x + 6, x, 0, 10)
9.81 m/s^2
derivative(sin(x)^2, x)
```

### Plot API

compile된 expression과 viewport를 받아 renderer 독립 sample과 polyline 후보를
제공합니다. display, SDL, OpenGL과 framebuffer는 application 책임입니다.

## 제품 분리

### SDK

- UI 없음
- C ABI와 C++17 내부 구현
- static/shared build
- module별 tests와 integration tests
- 선택 backend와 feature manifest
- caller-provided workspace와 cancellation

### Calculator application

- keypad/touch/keyboard 입력
- 수식 편집과 history
- worksheet와 파일 저장
- graph viewport와 renderer
- 진행 상태와 계산 취소
- 설정, 각도 mode, precision과 unit system
- 전원 중단에 안전한 저장

계산기 application 내부에 수학 알고리즘을 중복 구현하지 않습니다.

## 임베디드 전략

첫 제품 검증은 desktop 또는 Linux application processor에서 수행합니다. 이후 같은
conformance corpus를 유지하면서 compact/minimal profile로 축소합니다.

### Full

- heap과 worker thread 사용 가능
- arbitrary precision과 advanced CAS
- implicit graph와 adaptive ODE
- 전체 단위·상수 registry

### Compact

- 제한 heap 또는 arena
- `f32/f64`
- 제한 matrix와 CAS expression growth
- Cartesian, parametric와 polar graph
- 정적 단위 registry

### Minimal

- allocation 없는 경로 우선
- 고정 AST/bytecode/workspace
- 기본 함수와 단순 graph
- 명시적 unsupported 결과

## Parser 전략

1차 기준은 handwritten lexer와 Pratt parser입니다. 동일 최소 grammar의 Flex/Bison
C++ prototype을 만들어 다음을 비교합니다.

- 오류 위치와 recovery
- AST ownership
- grammar 확장성
- generated source 크기
- MCU flash/RAM/stack
- Windows/Linux/macOS 재생성
- fuzz와 sanitizer 안정성

`cmake_starter_1040` 전체는 흡수하지 않습니다. reentrant scanner, pure parser,
`%destructor`, token prefix와 detailed error 원칙만 선별 참고하고 BYUL 전용 maintainer
regeneration helper를 새로 작성합니다.

## 기존 자산 활용

### BYUL repository

- `byul/numal`: scalar, vec3, quaternion, dual quaternion과 geometry
- `byul/number_theory`: 소수와 인수분해
- `balix/numeq`: 기존 solver와 simulation integrator
- `rng/distributions`: 확률분포와 sampling 후보
- Python wrapper generator
- GPU compute tester

### `tools/unit_1003`

- 단위·prefix·quantity 자료의 기준 원본
- 기존 테스트와 단위 symbol 자료
- 새 `mathunits` ABI로 재설계하여 이식
- bit-field ABI, unsigned 지수와 quantity 연산 문제는 회귀 테스트로 보존

### `tools/cmake_starter_1040`

- Flex/Bison 사용 경험과 prototype 참고 자료
- 전체 build framework는 흡수하지 않음
- 권리 확인 없이 source/template 직접 복사 금지

### mpdecimal/libmpdec

- `numal` arbitrary precision decimal의 우선 backend 후보
- fixed decimal과 별도 capability 및 CMake option으로 제공
- `mathunits` scale/offset/quantity와 expression decimal mode에서 사용
- CAS exact 계층을 대체하지 않고 decimal approximation에 사용
- `numeq`의 decimal-compatible 알고리즘에만 선택 적용
- geometry, 실시간 ODE, graph batch와 GPU의 기본 backend로 사용하지 않음
- public C ABI에서 `mpd_t`, context와 allocator를 숨김
- quiet API status를 BYUL result로 변환하고 기본 signal trap을 피함
- Simplified BSD notice와 사용 version/build option을 third-party 기록에 포함

### 기존 BYUL geometry와 collision

- `byul/numal/geom.*`, `plane.*`와 vector/quaternion 공식 inventory
- `balix/collision`의 순수 기하와 시간 기반 collision 구분
- `ground`, `entity`, `projectile`의 raycast·최근접점·교차 helper 조사
- 순수 공간 질의는 독립 `geometry`, 동적 물리 adapter는 `balix`에 배치

### Open CASCADE Technology

- NURBS surface, trimmed surface, B-rep, healing, boolean과 STEP/IGES backend 후보
- FreeCAD Part/CAD module 대신 OCCT를 직접 private backend로 연결
- `TopoDS_Shape`, `Handle(Geom_*)` 등 OCCT type은 public C ABI에 노출하지 않음
- desktop/full/CAD profile에서만 기본 OFF 선택 option으로 제공
- embedded minimal/compact와 일반 SDK build는 OCCT 없이 동작
- shared library 동적 연결을 기본 배포 후보로 사용
- LGPL 2.1, OCCT exception, notice와 source 접근 의무를 package에서 처리
- BYUL license 제한을 OCCT component에 적용하지 않도록 명시
- static link와 OCCT source 편입은 기본 전략에서 제외

### FreeCAD

- CAD 사용 사례, UI, test와 workflow 참고 대상으로 사용
- application/document/workbench/Python/Qt 계층을 BYUL library에 직접 편입하지 않음
- source를 재사용할 예외 상황에는 file별 license와 dependency를 별도 audit

## 단계별 산출물

### 단계 0: 기준 고정

- 기존 함수와 target dependency inventory
- 기존 solver·unit test 결과와 알려진 오류 기록
- 외부 코드의 저작권과 재사용 권리 확인
- 공통 결과·오류·precision context 초안

### 단계 1: 수학 기반

- 독립 `numal` target
- `f64` 기준과 checked integer
- rational, complex와 matrix 최소 기능
- 기준 결과와 cross-platform tests
- fixed decimal과 선택 mpdecimal backend 경계

### 단계 2: 수치해석

- 독립 `numeq` target
- callback, result와 cancellation
- root, 미분, 적분과 범용 ODE
- `balix` physics adapter 분리

### 단계 3: 단위

- 독립 `mathunits` target
- dimension, SI prefix와 quantity
- linear/affine 변환
- 공학·물리 상수 version

### 단계 3A: 강건 기하 기반

- 독립 `geometry` target
- point, line, ray, segment와 주요 primitive
- 거리, projection, closest point와 intersection
- tolerance context와 robust predicate
- query result, contact point와 feature id 계약

### 단계 4: 표현식

- 공통 token, diagnostic와 AST
- Pratt parser와 evaluator
- function registry
- bytecode 반복 평가
- 단위와 수치해석 연결

### 단계 5: Reference calculator

- desktop CLI calculator
- Cartesian graph와 table
- history와 worksheet serialization
- direct API와 expression 결과 비교

### 단계 6: 제한적 CAS와 고급 graph

- 기호 미분과 안전한 단순화
- 다항식과 제한적 기호 적분
- parametric, polar와 implicit graph
- 교점, 극값과 progressive refinement

### 단계 6A: 곡선·곡면과 B-rep

- Bezier, Hermite, spline와 NURBS
- curve-line, curve-curve와 curve-surface intersection
- surface evaluation, trimming과 tessellation
- mesh collision, contact manifold와 penetration query
- B-rep topology, validation과 geometry entity 연결
- OCCT adapter prototype과 자체 core 결과 비교
- third-party notice, license와 source-access package 검증

### 단계 7: 임베디드 profile

- static library와 feature manifest
- fixed workspace와 arena
- flash/RAM/stack benchmark
- framebuffer 또는 장치 UI adapter
- 전원 중단 저장과 watchdog 친화적 취소

### 단계 8: backend와 성능

- `f128`, arbitrary precision와 decimal
- CPU SIMD와 GPU batch
- Flex/Bison 최종 채택 여부 결정
- shared DLL 분리 검토

## 완료 정의

프로젝트는 단순히 계산기 화면이 동작할 때 완료되지 않습니다. 다음 조건을 모두
만족해야 합니다.

- SDK가 계산기 없이 독립적으로 build와 test됩니다.
- 다른 C/C++ 앱이 직접 수학 API를 사용할 수 있습니다.
- 계산기, CLI와 Python이 동일 C ABI를 사용합니다.
- parser를 거친 결과와 직접 API 결과가 일치합니다.
- 단위와 차원 오류가 계산 전에 진단됩니다.
- 기하 query가 교차 여부뿐 아니라 위치, parameter, normal, distance와 접촉점을
  일관된 결과 구조로 제공합니다.
- curve, surface, NURBS와 B-rep의 퇴화·공차 계약이 문서화됩니다.
- 그래프 반복 평가가 재parse 없이 수행됩니다.
- CAS가 exact, conditional, approximate와 unevaluated 결과를 구분합니다.
- 장시간 계산을 취소할 수 있습니다.
- 임베디드 profile별 RAM, flash, stack과 계산 예산이 문서화됩니다.
- 일반 SDK build는 Flex/Bison과 GPU SDK를 요구하지 않습니다.
- 일반·embedded SDK build는 OCCT와 FreeCAD를 요구하지 않습니다.
- OCCT 포함 package는 LGPL/exception 고지와 component 교체 가능성을 검증합니다.
- Windows, Linux와 macOS 결과 계약 및 지원 장치 profile을 검증합니다.

## 첫 작업

첫 구현은 계산기 GUI가 아닙니다.

1. 실제 target과 함수 dependency inventory
2. 기존 solver·numal·unit·geometry/collision 회귀 corpus
3. 독립 target 기반 CMake 구조
4. `f64` result/error/precision API
5. 최소 Pratt expression evaluator

이 기반이 안정된 후 reference calculator를 시작합니다.
