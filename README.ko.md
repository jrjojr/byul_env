# 별이의 세계 / BYUL 엔진

언어: [English](README.md) | [한국어](README.ko.md)

> **라이선스 안내**
>
> 이 저장소는 **오픈소스가 아니라 source-available 프로젝트**입니다.
> 개인 학습, 학술 연구, 비상업적 기술 검토와 평가는 허용됩니다.
> 상업적 사용, 재배포, 재판매, 공개 미러링, 라이브러리 패키징은 사전 서면 허가 없이는 금지됩니다.
> 자세한 내용은 [LICENSE](LICENSE)를 확인하십시오.

`byul`은 **별이의 세계**를 위한 C++17 기반 시뮬레이션 엔진입니다.

내부 구현은 C++로 작성하여 구현의 유연성을 확보하고, 외부에는 `extern "C"`와 `BYUL_API`를 통해 C 호환 ABI를 제공합니다. 이 구조를 통해 Unreal Engine, C/C++ 애플리케이션, FFI 기반 환경에서 안정적인 DLL 인터페이스로 사용할 수 있습니다.

`byul`은 처음에는 경로 탐색 코어로 시작했지만, 현재는 route 계획, 미로 생성, 수치 시뮬레이션, 동적 엔티티, trajectory 예측, projectile 시뮬레이션, ground 상호작용, runtime tick 업데이트까지 다루는 모듈형 엔진으로 확장되었습니다.

---

## 핵심 개념

BYUL은 단순한 길찾기 라이브러리가 아닙니다.

BYUL은 별이의 세계 안에서 객체가 다음과 같은 일을 수행하는 방식을 정의하는 실시간 시뮬레이션 코어입니다.

- 계획된 route를 찾는다.
- 격자 공간과 연속 공간을 이동한다.
- trajectory를 생성하고 분석한다.
- 중력, 바람, 항력, ground의 영향을 받는다.
- projectile의 impact를 예측한다.
- runtime tick 기준으로 상태를 갱신한다.

현재 설계는 내부 모듈을 분리된 static library로 유지하면서, 최종 공개 산출물은 하나의 shared library로 제공합니다.

```text
byul.dll / libbyul.so
```

이 방식은 개발 구조는 모듈형으로 유지하고, 배포 구조는 단순하게 유지하기 위한 선택입니다.

---

## 구현 모델

BYUL은 다음 구조로 설명하는 것이 가장 정확합니다.

```text
C++17 구현 + C 호환 ABI + 단일 shared library
```

공개 헤더는 심볼 안정성을 위해 `extern "C"`를 사용하고, 플랫폼별 export/import 처리를 위해 `BYUL_API`를 사용합니다.

```c
#ifdef __cplusplus
extern "C" {
#endif

BYUL_API const char* byul_version_string();
BYUL_API void byul_print_version();

#ifdef __cplusplus
}
#endif
```

따라서 BYUL은 순수 C 라이브러리가 아닙니다. BYUL은 C ABI 경계를 가진 C++ 엔진 DLL입니다.

### ABI 규칙

공개 API는 다음 규칙을 지키는 것을 원칙으로 합니다.

- 공개 헤더에 `std::vector`, `std::string`, C++ class를 노출하지 않는다.
- 공개 함수에는 `BYUL_API`를 붙인다.
- 공개 함수는 `extern "C"` 블록 안에 둔다.
- BYUL에서 생성한 객체는 BYUL의 destroy/free 함수로 해제한다.
- C++ 예외를 DLL 경계 밖으로 던지지 않는다.
- 공개 구조체 레이아웃 변경은 버전 호환성을 고려하여 신중하게 수행한다.

---

## 모듈 아키텍처

엔진은 다음 내부 모듈을 조립하여 구성됩니다.

```text
byul
├─ core
├─ numal
├─ rng
├─ console
├─ navsys
├─ balix
├─ entity
├─ projectile
├─ ground
├─ byul_tick
└─ gpu_comp_tester
```

최상위 `byul.h`는 우산 헤더 역할을 합니다.

```c
#include "byul_config.h"
#include "numal.h"
#include "navsys.h"
#include "balix.h"
#include "entity.h"
#include "projectile.h"
#include "byul_tick.h"
```

---

## 주요 계층

### 1. `navsys` - 격자 기반 route 계획

`navsys`는 격자 기반 navigation 계층입니다.

구성 요소는 다음과 같습니다.

```text
coord
route
navgrid
obstacle
maze
route_finder
dstar_lite
route_carver
```

지원하는 route 탐색 알고리즘은 다음과 같습니다.

- A*
- BFS
- DFS
- Dijkstra
- Greedy Best First
- Fast Marching
- IDA*
- Fringe Search
- RTA*
- SMA*
- Weighted A*
- D* Lite

이 계층의 주요 산출물은 `route_t`입니다.

---

### 2. `balix` - 연속 시뮬레이션 계층

`balix`는 연속 운동, 물리 상태, 제어, 수치 계산을 담당하는 계층입니다.

구성 요소는 다음과 같습니다.

```text
bodyprops
environ
collision
numeq
xform
motion_state
trajectory
controller
```

이 계층은 다음 기능을 제공합니다.

- 물리 속성
- 환경 데이터
- transform
- motion state
- 수치 적분
- PID / MPC 제어 구조
- collision 관련 유틸리티
- trajectory 저장 및 export

---

### 3. `entity` - 세계 객체 계층

`entity`는 일반 세계 객체와 동적 물리 상태를 연결합니다.

중요한 타입은 다음과 같습니다.

```text
entity_t
entity_dynamic_t
```

`entity_dynamic_t`는 다음 요소를 결합합니다.

- base entity data
- transform
- physical properties
- velocity
- angular velocity
- grounded state

이 계층은 객체의 정체성과 물리 시뮬레이션을 연결하는 중간 계층입니다.

---

### 4. `projectile` - 투사체 시뮬레이션 계층

`projectile`은 게임플레이에 가까운 투사체 계층입니다.

구성 요소는 다음과 같습니다.

```text
projectile_core
propulsion
guidance
projectile_predict
projectile_tick
```

C 스타일 구조체 포함 방식으로 다음 계층을 구성합니다.

```text
projectile_t
└─ shell_projectile_t
   └─ rocket_t
      └─ missile_t
         └─ patriot_t
```

이 계층은 다음을 지원합니다.

- 기본 projectile
- explosion radius를 가진 shell
- propulsion을 가진 rocket
- guidance를 가진 missile
- 고급 target tracking
- 전체 trajectory prediction
- impact position/time 출력
- tick 기반 runtime update

---

## Route와 Trajectory의 구분

BYUL은 `route`와 `trajectory`를 의도적으로 분리합니다.

### `route_t`

`route_t`는 `navsys`에 속합니다.

격자 공간에서 계획된 이동 route를 나타냅니다.

저장하는 정보는 다음과 같습니다.

- route coordinates
- visited order
- visited count
- route cost
- success state
- retry count
- average direction data

`route_t`는 다음 질문에 답할 때 사용합니다.

```text
이 entity는 어디로 가야 하는가?
```

### `trajectory_t`

`trajectory_t`는 연속 시뮬레이션 계층에 속합니다.

시간 순서대로 정렬된 motion sample을 저장합니다.

```text
time + motion_state
```

또한 예측된 impact 정보도 저장합니다.

```text
impact_time
impact_pos
```

`trajectory_t`는 다음 질문에 답할 때 사용합니다.

```text
이 object는 시간에 따라 어디에 있게 되는가?
```

### 이름 규칙

```text
route      = navigation/grid 공간에서 계획된 이동 경로
trajectory = 시간 기반 물리 운동 곡선
track      = 실제 이동 기록, 필요하면 추후 추가
trail      = 시각적 잔상 또는 렌더링 효과, 필요하면 추후 추가
course     = 고수준 guidance 방향, 필요하면 추후 추가
```

---

## Projectile Prediction Result

Projectile prediction은 `projectile_result_t`에 결과를 기록합니다.

결과에는 다음 정보가 포함됩니다.

```text
start_pos
target_pos
initial_velocity
impact_time
impact_pos
bool_impacted
trajectory
```

즉 projectile prediction은 단순히 명중 여부만 반환하지 않습니다. 전체 예측 trajectory를 저장하므로 출력, 샘플링, export, 시각화에 사용할 수 있습니다.

---

## 예제: Route Finding

```cpp
TEST_CASE("navsys: find astar") {
    navgrid_t* navgrid = navgrid_create();
    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    for (int y = 1; y < 10; ++y) {
        navgrid_block_coord(navgrid, 5, y);
    }

    route_t* route = navsys_find_astar(navgrid, &start, &goal);
    CHECK(route_get_success(route) == true);

    route_print(route);
    navgrid_print_ascii_with_route(navgrid, route, 2);

    route_destroy(route);
    navgrid_destroy(navgrid);
}
```

---

## 예제: Projectile Prediction

```cpp
TEST_CASE("projectile_predict - basic prediction") {
    projectile_t proj;
    projectile_init(&proj);

    proj.base.xf.pos = {0.0f, 10.0f, 0.0f};
    proj.base.velocity = {10.0f, 20.0f, 0.0f};

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = {50.0f, 0.0f, 0.0f};

    environ_t env;
    environ_init(&env);

    projectile_result_t* result = projectile_result_create();

    bool hit = projectile_predict(
        &proj,
        0.016f,
        &target,
        &env,
        nullptr,
        nullptr,
        nullptr,
        result
    );

    projectile_result_print_detailed(result);

    projectile_result_destroy(result);
}
```

---

## 빌드

BYUL은 CMake를 기본 크로스플랫폼 빌드 시스템으로 사용합니다. 소스 트리에는
모듈별 `CMakeLists.txt`가 여러 개 있지만, 최종적으로는
`byul/CMakeLists.txt`에서 모두 조립됩니다. 플랫폼 선택은
`byul/CMakePresets.json`에서 관리하며 Linux, Windows MinGW 크로스 컴파일,
Windows MinGW native 빌드, Windows MSVC 빌드를 포함합니다.

현재 빌드는 두 가지 표면을 가집니다.

```text
개발 빌드:
  내부 static library
  모듈별 test 실행 파일
  통합 test 실행 파일
  보조 실행 파일
  byul shared library

배포 표면:
  byul.dll / libbyul.so
  공개 헤더
```

기본 build target은 최종 shared library만 만들지 않습니다. `core`, `numal`,
`dstar_lite` 같은 모듈 static library와 `test_core`, `test_numal`,
`test_dstar_lite`, `test_byul` 같은 테스트 실행 파일도 함께 만듭니다. 이
테스트 실행 파일들은 개발 검증용 산출물이며, 기본 공개 배포 표면은 아닙니다.

최종 공개 산출물은 계속 다음 하나의 shared library입니다.

```text
byul.dll / libbyul.so
```

### Preset 빌드

권장 빌드 방법은 `byul` 디렉토리에서 `CMakePresets.json`을 사용하는 것입니다.
빌드 preset은 `package_zip`을 실행해 새 패키지를 만듭니다. `package_zip` target은
먼저 `all`을 최신 상태로 갱신하고, 임시 package layout에 설치한 뒤 `byul.zip`을
생성합니다.

```bash
cd byul_env/byul
cmake --preset linux-debug
cmake --build --preset build-linux-debug
```

주요 preset은 다음과 같습니다.

```text
linux-debug          Ubuntu Debug 빌드
linux-release        Ubuntu Release 빌드
win-release          Linux에서 MinGW64로 Windows DLL 크로스 컴파일
win_sdl_release      Linux에서 MinGW64로 Windows SDL 실행 파일 빌드
win-native           Windows native MSYS2 MinGW Release 빌드
win-native-debug     Windows native MSYS2 MinGW Debug 빌드
win-msvc-release     Windows Visual Studio 2022 Release 빌드
win-msvc-debug       Windows Visual Studio 2022 Debug 빌드
```

빌드 preset 이름은 configure preset 이름 앞에 `build-`를 붙인 형태입니다.

```bash
cmake --preset linux-release
cmake --build --preset build-linux-release

cmake --preset win-release
cmake --build --preset build-win-release

cmake --preset win-native
cmake --build --preset build-win-native

cmake --preset win-msvc-release
cmake --build --preset build-win-msvc-release
```

### 테스트 실행

현재 저장소는 CMake test preset 대신 CTest 명령을 직접 사용합니다. preset으로
configure와 build를 마친 뒤 해당 build directory를 대상으로 CTest를 실행합니다.

```bash
ctest --test-dir ../build_debug --output-on-failure
ctest --test-dir ../build_release --output-on-failure
ctest --test-dir ../build_win_release --output-on-failure
```

MSVC처럼 multi-config build에서는 필요할 때 configuration을 함께 지정합니다.

```bash
ctest --test-dir ../build_win_msvc_release -C Release --output-on-failure
ctest --test-dir ../build_win_msvc_debug -C Debug --output-on-failure
```

### 수동 빌드

수동 generator 명령도 가능하지만, Linux, Windows 크로스 컴파일, Windows native,
MSVC 빌드 디렉토리를 일관되게 유지하려면 preset 사용을 권장합니다.

```bash
cd byul_env/byul
mkdir build
cd build
cmake ..
cmake --build . --target package_zip
ctest --output-on-failure
```

### 주요 Target

Makefile 계열 generator로 configure한 뒤 자주 쓰는 target은 다음과 같습니다.

```text
all              library, 보조 실행 파일, 테스트 실행 파일을 빌드
byul             최종 shared library target만 빌드
test_core        core 모듈 테스트 실행 파일 빌드
test_dstar_lite  D* Lite 모듈 테스트 실행 파일 빌드
test_byul        통합 테스트 실행 파일 빌드
test             등록된 CTest 테스트 실행
install          CMAKE_INSTALL_PREFIX 아래로 설치
package_zip      all 갱신, package layout 설치, byul.zip 생성
uninstall        CMake가 기록한 설치 파일 제거
```

`package_zip`은 단독으로 실행해도 됩니다. 먼저 `all` target을 최신 상태로
갱신한 뒤 임시 package layout에 설치하고, 그 최신 빌드 결과로 zip을 만듭니다.

---

## 설계 방향

현재 방향은 다음과 같습니다.

```text
내부 모듈은 분리한다.
최종 배포는 하나의 BYUL shared library로 유지한다.
```

이 선택은 의도적입니다.

`navsys`, `balix`, `entity`, `projectile`, `ground`, `byul_tick`은 내부적으로 분리된 모듈이지만, 너무 이른 시점에 여러 DLL로 노출하면 ABI 관리와 배포 복잡도가 커집니다.

현재 선호 구조는 다음입니다.

```text
개발: 모듈형 static library
배포: 단일 byul.dll / libbyul.so
```

`byul_navsys.dll` 또는 `byul_projectile.dll` 같은 분리 DLL은 외부 사용 패턴이 명확해진 뒤에 검토합니다.

---

## 프로젝트 상태

현재 BYUL은 다음 영역에서 강점을 가집니다.

- route planning
- maze generation
- continuous motion state modeling
- trajectory storage and export
- projectile prediction
- propulsion/guidance structure
- DLL 지향 C ABI 설계

프로젝트는 route/pathfinding 엔진에서 출발하여, 별이의 세계를 위한 더 넓은 simulation core로 확장되고 있습니다.

---

## 라이선스

Byul World Source-Available Non-Commercial License v1.0

이 저장소는 오픈소스가 아니라 source-available 프로젝트입니다.

- 개인 학습, 학술 연구, 기술 검토, 피드백, 비상업적 평가는 허용됩니다.
- GitHub fork는 개인 학습, 이슈 보고, 기여 논의, 비상업적 기술 검토 목적에 한해 허용됩니다.
- 상업적 사용, 재배포, 재판매, 공개 미러링, 재라이선스, 라이브러리·SDK·플러그인·서비스 패키징은 사전 서면 허가 없이는 금지됩니다.
- 제3자 구성 요소가 있다면 해당 구성 요소는 각자의 라이선스를 따릅니다.

전체 조건은 [LICENSE](LICENSE)를 확인하십시오.

---

## 연락처

**byuldev@outlook.kr**

---

## 개발자 노트

BYUL은 별이의 세계의 movement core입니다.

`route`는 entity가 공간을 어떻게 계획해서 이동하는지를 정의합니다.

`trajectory`는 physical object가 시간에 따라 어떻게 움직이는지를 정의합니다.

이 둘은 navigation, motion, prediction, impact, runtime simulation을 엔진 자체에서 처리하는 별이의 세계의 기반이 됩니다.
