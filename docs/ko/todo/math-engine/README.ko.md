# BYUL Math Engine 프로젝트

BYUL Math Engine은 TI-89 수준의 공학용 CAS·그래프 기능을 첫 완료 기준으로 삼는
임베디드 공학 계산기와 재사용 가능한 수학·simulation SDK 프로젝트입니다. TI-89는
기능 범위 비교 기준일 뿐 명령 호환성이나 화면 layout 복제 대상이 아닙니다.

계산기 전용 코드로 만들지 않습니다. 범용 simulation, 데스크톱 GUI, CLI,
Python wrapper, 교육 도구와 batch/GPU 계산에서도 동일한 C ABI와 계산 계약을
사용하는 것이 핵심 목표입니다.

## 네 가지 대표 사용처

```text
BYUL Math Engine
├─ 임베디드 공학 계산
├─ 범용 시뮬레이션
├─ 금융 계산
└─ CAD·강건 기하
```

- 임베디드 공학 계산: CAS 그래프 계산기, 계측기와 현장 진단 장비
- 범용 시뮬레이션: 실시간 물리, collision, entity, projectile, trajectory,
  기계·물리, 로봇·제어, 항공우주와 전기전자
- 금융 계산: fixed decimal, mpdecimal, rounding, quantize와 재현 가능한 계산
- CAD·강건 기하: raycast, curve/surface, NURBS, mesh, B-rep와 선택 OCCT

게임과 공학·과학은 별도 simulation engine이 아닙니다. 같은 simulation core에
패스트, 정밀 또는 복합 preset을 적용합니다. 복합 preset은 패스트 경로를 기본으로
사용하고 애매하거나 중요한 계산만 정밀 경로로 승격합니다. 실행 preset과
`f32/f64/f128/binary256` 같은 숫자 backend는 서로 독립적으로 선택합니다.

## 첫 함수 범위 기준과 BYUL 독자성

첫 계산기의 기능 범위 기준은 TI-89입니다. symbolic algebra와 calculus, exact와
approximate 결과, 복소수·행렬, 단위·상수, 방정식·ODE와 주요 graph 기능을 단계별
완료 기준으로 사용합니다. TI-Nspire의 document와 application 연결 방식은 장기
참고 대상으로만 두며 첫 구현 범위를 늘리는 기준으로 사용하지 않습니다.

현재 단계에서는 계산기 UI, key layout, menu와 화면 흐름을 작성하지 않습니다.
TI-89는 다음 함수와 결과 계약을 정리할 때만 참고합니다.

- CAS, 방정식, 미분·적분, 행렬, 단위와 graph 함수 inventory
- exact, approximate, inexact와 uncertain 결과 상태
- product profile, 실행 preset과 숫자 backend 조회 API
- graph, table, ODE와 simulation adapter가 공유할 결과 구조
- trajectory, collision과 ODE의 시간축 sample과 검사 정보
- 복합 preset의 정밀 승격 이유, 오차와 사용 예산
- 단위·차원, 금융 rounding·scale·quantize 상태
- resolved configuration과 결과 재현 metadata

미래 UI는 이 API와 metadata를 소비하게 하며 현재 함수 구현 단계에서는 화면이나
조작 방식을 확정하지 않습니다. UI 단계가 시작되면 TI 제품 layout을 복제하지 않고
BYUL 고유 설계를 별도 문서에서 진행합니다.

## 최종 목표

```text
BYUL Math SDK
  ├─ 다른 애플리케이션에 내장 가능한 수학 엔진
  ├─ 안정적인 C ABI와 C++17 내부 구현
  ├─ 모듈별 static build와 루트 단일 shared library
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
- 내부 도구와 test는 필요한 module static target을 연결하고, 외부 동적 사용자는 루트
  `byul.dll` 또는 `libbyul.so` 하나만 연결합니다.
- public API에는 C++ 표준 컨테이너와 외부 backend 타입을 노출하지 않습니다.
- generated parser, GPU와 arbitrary precision은 선택 기능입니다.
- OCCT 기반 B-rep/CAD backend는 full profile의 선택 기능이며 기본 build에는 필요하지
  않습니다.
- mpdecimal은 full profile의 arbitrary decimal 선택 backend이며 minimal profile은
  fixed decimal 또는 integer coefficient/scale 경로를 사용합니다.
- 지원하지 않는 기능은 정밀도를 몰래 낮추지 않고 명시적 상태를 반환합니다.

## 논리 모듈과 의존성

수학 계층은 낮은 수준의 숫자와 대수에서 표현식·plot으로 올라가는 단방향 구조를
사용합니다.

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

고급 CAD 기능은 BYUL 자체 `geometry_core`와 full/CAD profile 전용 선택
`geometry_occt` backend로 분리합니다.

허용하지 않는 의존성은 다음과 같습니다.

```text
numal -> expression, GUI, balix
numeq -> motion_state, unit registry, renderer
mathunits -> expression, numeq, GUI
expression -> calculator GUI
plot -> 특정 display 또는 window toolkit
geometry -> balix, entity, projectile, renderer
```

실제 source directory 후보는 다음과 같습니다.

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

직접 계산 API는 게임, simulation과 firmware가 문자열 parser 없이 사용합니다.

```c
double y = math_gamma_f64(5.0);
numeq_find_root_f64(callback, userdata, 0.0, 10.0,
                    &options, &result);
```

표현식 API는 계산기, GUI와 CLI가 사용합니다.

```text
sin(pi / 3)
solve(x^2 - 5*x + 6, x, 0, 10)
9.81 m/s^2
derivative(sin(x)^2, x)
```

Plot API는 compile된 expression과 viewport를 받아 renderer 독립 sample과 polyline
후보를 제공합니다. Display, SDL, OpenGL과 framebuffer는 application 책임입니다.

## SDK와 reference product 분리

SDK는 UI를 포함하지 않으며 C ABI, C++17 내부 구현, 모듈별 static library, 루트 단일
shared library, test, feature manifest, workspace와 cancellation을 제공합니다.

계산기 application은 다음 제품 기능만 담당합니다.

- keypad, touch와 keyboard 입력
- 수식 편집, history와 worksheet
- graph viewport와 renderer
- 진행 상태와 계산 취소
- 각도 mode, precision과 unit system 설정
- 전원 중단에 안전한 저장

계산기 application 내부에 수학 알고리즘을 중복 구현하지 않습니다.

## 문서 구성

- [전체 수학 엔진 계획](../todo/todo-math-engine.org)
- [기초 수학과 숫자 backend](../todo/todo-numal.org)
- [수치해석](../todo/todo-numeq.org)
- [단위·수량과 공학 상수](../todo/todo-mathunits.org)
- [표현식, parser, bytecode와 제한적 CAS](../todo/todo-expression.org)
- [기하, 곡선·곡면, 충돌·접촉, NURBS와 B-rep](../todo/todo-geometry.org)
- [범용 simulation context와 패스트·정밀·복합 preset](../todo/todo-simulation-context.org)
- [제품 profile과 설정 resolver](../todo/todo-configuration-profiles.org)

## 목표 산출물

모든 내부 모듈은 CMake static library target으로 작성합니다. 외부에 제공하는 BYUL
shared library는 루트 `byul` target 하나뿐이며, 일부 모듈만 담는 축소 DLL을 만들지
않습니다. 루트 DLL은 모든 BYUL public C 함수를 export해야 합니다.

```text
Module targets
  byul_core_static 또는 기존 module static target
  byul_numal_static
  byul_mathfunc_static
  byul_numeq_static
  byul_mathunits_static
  byul_geometry_static
  byul_expression_static
  byul_plot_static

Root shared target
  byul.dll / libbyul.so
  모든 BYUL public C 함수와 runtime capability query 포함

Applications
  byul_calculator                                   reference product
  byul_math_cli                                     진단·자동화
  Python wrapper                                    교육·검증·도구
```

모듈별 DLL은 만들지 않습니다. 모듈 static target은 독립 test와 내부 재사용 단위이며,
루트 DLL은 동일한 module source inventory를 `BYUL_EXPORTS` 조건으로 통합합니다. 단순
static archive 링크에 의존해 미참조 public 함수가 DLL에서 빠지지 않게 합니다.

기능을 사용할 수 없는 build에서도 안정적인 ABI에 포함하기로 정한 함수는 사라지지
않고 명시적인 unsupported 상태를 반환합니다. Public header 선언 목록, ABI manifest와
루트 DLL export 목록을 비교하여 모든 함수 포함 여부를 검사합니다.

현재 CMake 3.20 구조에서는 module source inventory를 static target과 루트 shared
target이 공유하고 각각 `BYUL_STATIC`과 `BYUL_EXPORTS` 조건으로 컴파일하는 방식을
우선합니다. Static archive만 루트 DLL에 링크하면 linker가 미참조 object를 제외할 수
있으므로 전체 public 함수 포함을 보장하는 방법으로 간주하지 않습니다. Whole-archive
또는 object library 방식은 export macro, PIC, 중복 symbol과 Windows/Linux/macOS
동작을 검증한 뒤 선택합니다.

Build와 ABI 검증에는 다음을 포함합니다.

- module source/header inventory와 루트 target inventory 일치
- public header parser가 얻은 함수 목록과 DLL export 목록 일치
- ABI manifest와 runtime `byul_abi_fingerprint()` 일치
- static module test와 루트 DLL integration test가 같은 conformance corpus 사용
- 새 public 함수가 module static target에는 있지만 루트 DLL에서 누락되는 회귀 방지

Public C header와 export는 다음 원본 규칙을 따릅니다.

```text
../rules_env/byul/header-declaration-rules.org
../rules_env/byul/header-comment-rules.org
docs/ko/byul-sdk/header-license-block.txt
```

- 모든 public 함수 선언에는 `BYUL_API`를 적용합니다.
- static target에서는 `BYUL_STATIC`, 루트 DLL 구현에서는 `BYUL_EXPORTS` 의미를
  지킵니다.
- 라이선스 블록은 기준 파일을 첫 줄부터 그대로 사용합니다.
- 파일 수준 Doxygen, C linkage, C ABI type, ownership과 오류 metadata를 lint합니다.
- 루트 umbrella header, 개별 public header, ABI manifest와 DLL export를 함께
  검증합니다.

OCCT 같은 third-party shared dependency는 BYUL이 생성하는 모듈 DLL이 아닙니다. 이를
배포하는 package는 LGPL 2.1과 OCCT exception을 적용받는 component를 BYUL license와
명확히 분리하고 notice, license와 source 접근 정보를 포함해야 합니다.

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

Full profile은 heap과 worker thread, arbitrary precision, advanced CAS, implicit graph,
adaptive ODE와 전체 단위·상수 registry를 사용할 수 있습니다. Compact profile은 제한된
heap 또는 arena, `f32/f64`, 제한 matrix·CAS, 주요 graph mode와 정적 단위 registry를
사용합니다. Minimal profile은 allocation 없는 경로, 고정 AST·bytecode·workspace,
기본 함수와 단순 graph를 우선하며 사용할 수 없는 기능을 명시적으로 보고합니다.

첫 제품 검증은 desktop 또는 Linux application processor에서 수행하고, 동일한
conformance corpus를 유지하면서 compact와 minimal profile로 축소합니다.

## Parser 전략

첫 기준은 handwritten lexer와 Pratt parser입니다. 동일한 최소 grammar의 Flex/Bison
C++ prototype을 만들어 다음을 비교합니다.

- 오류 위치와 recovery
- AST ownership
- grammar 확장성
- generated source 크기
- MCU flash, RAM과 stack
- Windows, Linux와 macOS 재생성
- fuzz와 sanitizer 안정성

`cmake_starter_1040` 전체는 흡수하지 않습니다. Reentrant scanner, pure parser,
`%destructor`, token prefix와 detailed error 원칙만 선별 참고하고 BYUL 전용 maintainer
regeneration helper를 작성합니다.

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
- 기존 test와 단위 symbol 자료
- 새 `mathunits` ABI로 재설계하여 이식
- bit-field ABI, unsigned 지수와 quantity 연산 문제를 회귀 test로 보존

### `tools/cmake_starter_1040`

- Flex/Bison 사용 경험과 prototype 참고 자료
- 전체 build framework는 흡수하지 않음
- 권리 확인 없이 source와 template 직접 복사 금지

### mpdecimal/libmpdec

- `numal` arbitrary precision decimal의 우선 backend 후보
- fixed decimal과 별도 capability 및 CMake option으로 제공
- `mathunits` scale·offset·quantity와 expression decimal mode에서 사용
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
- `TopoDS_Shape`, `Handle(Geom_*)` 같은 OCCT type을 public C ABI에 노출하지 않음
- desktop/full/CAD profile에서만 기본 OFF 선택 option으로 제공
- embedded minimal/compact와 일반 SDK build는 OCCT 없이 동작
- shared library 동적 연결을 기본 배포 후보로 사용
- LGPL 2.1, OCCT exception, notice와 source 접근 의무를 package에서 처리
- BYUL license 제한을 OCCT component에 적용하지 않도록 명시
- static link와 OCCT source 편입은 기본 전략에서 제외

### FreeCAD

- CAD 사용 사례, UI, test와 workflow 참고 대상으로 사용
- application, document, workbench, Python과 Qt 계층을 BYUL library에 직접 편입하지 않음
- source를 재사용하는 예외 상황에는 file별 license와 dependency를 별도 audit

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
- 기준 결과와 cross-platform test
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
- query result, contact point와 feature ID 계약

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
- flash, RAM과 stack benchmark
- framebuffer 또는 장치 UI adapter
- 전원 중단 저장과 watchdog 친화적 취소

### 단계 8: Backend와 성능

- `f128`, arbitrary precision과 decimal
- CPU SIMD와 GPU batch
- Flex/Bison 최종 채택 여부 결정
- 루트 단일 DLL의 전체 public symbol과 ABI manifest 검증

## 완료 정의

- SDK가 계산기 없이 독립적으로 build되고 test됩니다.
- 모든 모듈이 static target으로 build되고 BYUL shared target은 루트 하나만 존재합니다.
- 루트 DLL이 모든 public C 함수를 export하고 header·ABI manifest와 일치합니다.
- 다른 C/C++ application이 직접 수학 API를 사용할 수 있습니다.
- 계산기, CLI와 Python이 동일한 C ABI를 사용합니다.
- parser를 거친 결과와 직접 API 결과가 일치합니다.
- 단위와 차원 오류가 계산 전에 진단됩니다.
- 기하 query가 위치, parameter, normal, distance와 접촉점을 일관되게 제공합니다.
- curve, surface, NURBS와 B-rep의 퇴화·공차 계약이 문서화됩니다.
- 그래프 반복 평가가 재parse 없이 수행됩니다.
- CAS가 exact, conditional, approximate와 unevaluated 결과를 구분합니다.
- 장시간 계산을 취소할 수 있습니다.
- profile별 RAM, flash, stack과 계산 예산이 문서화됩니다.
- 일반 SDK build는 Flex/Bison, GPU SDK, OCCT와 FreeCAD를 요구하지 않습니다.
- OCCT 포함 package는 LGPL/exception 고지와 component 교체 가능성을 검증합니다.
- Windows, Linux와 macOS 결과 계약 및 지원 장치 profile을 검증합니다.

## 현재 단계

현재 문서는 구현 전 아키텍처와 마이그레이션 계획입니다. 첫 구현은 GUI가 아니라
다음 기반을 확립하는 작업입니다.

1. 실제 target과 함수 dependency inventory
2. 기존 solver, numal, unit, geometry/collision 회귀 corpus
3. 독립 target 기반 CMake 구조
4. `f64` result, error와 precision API
5. 최소 Pratt expression evaluator

이 기반이 안정된 뒤 TI-89 수준의 함수와 결과 계약을 작은 수직 단위로 구현하고
검증합니다. 화면과 조작 체계는 현재 범위가 아닙니다.
