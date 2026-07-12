# GPU Compute Tester

`gpu_comp_tester`는 BYUL의 회전 보간과 GPU 연산 적용 가능성을 확인하기 위한
독립 진단 도구입니다. SDL로 창과 OpenGL 컨텍스트를 만들고, LERP와
SLERP 결과를 두 개의 선으로 그려 차이를 비교합니다.

이 프로그램은 BYUL 엔진이나 SDK의 필수 구성 요소가 아닙니다. CMake의
`EXCLUDE_FROM_ALL` 대상으로 등록되어 일반 `all` 및 Visual Studio `ALL_BUILD`에서는
제외되고, 전용 build 또는 ZIP target으로만 별도 빌드합니다.

## 현재 동작

실행하면 `800x600` 크기의 GPU comp 선택 창이 열립니다. 실행 중 comp를 바꾸면
compute program과 창 제목이 즉시 바뀌고 빨간색 선의 움직임도 달라집니다.

- 빨간색 선: 현재 선택한 compute shader의 결과
- 파란색 선: CPU의 벡터 LERP 결과
- `1`~`4`: SLERP, NLERP, Pulse, Lissajous comp 직접 선택
- `←` / `→`: 이전 또는 다음 comp 선택
- `Esc` 또는 창 닫기: 프로그램 종료
- 콘솔: 현재 comp와 1초마다 FPS 및 CPU SLERP 기준 편차 출력

선택 가능한 comp는 다음과 같습니다.

| 키 | 파일 | 화면 동작 |
|---|---|---|
| `1` | `clock.comp` | 쿼터니언 SLERP로 시계 방향 회전 |
| `2` | `nlerp.comp` | 정규화 선형 보간(NLERP)으로 회전 |
| `3` | `pulse.comp` | 회전하면서 선 길이가 주기적으로 변화 |
| `4` | `lissajous.comp` | 독립적인 8자 궤적 |

GPU SLERP 결과는 SSBO를 통해 읽어 화면에 표시합니다. 같은 입력을 BYUL의 CPU
SLERP로도 계산하고 두 결과의 위치 오차를 측정하므로 compute shader 구현을
수치적으로 확인할 수 있습니다.

## 요구 사항

공통 요구 사항:

- CMake 3.20 이상
- C++17 컴파일러
- OpenGL 4.3 이상과 compute shader를 지원하는 그래픽 드라이버
- 저장소의 `external/glad`

Linux에서는 SDL2와 OpenGL 개발 패키지가 필요합니다.

```bash
sudo apt install -y libsdl2-dev mesa-common-dev
```

Windows에서는 저장소의 `external/SDL3-3.2.18`을 사용합니다. MSVC와 MinGW는
각각 해당 폴더의 전용 CMake 설정과 `SDL3.dll`을 사용합니다.

## 빌드

명령은 `byul` 디렉터리에서 실행합니다.

### Linux Debug

```bash
cmake --preset linux-debug
cmake --build ../build_debug --target gpu_comp_tester -- -j8
```

실행 파일과 셰이더 위치:

```text
build_debug/bin/gpu_comp_tester
build_debug/glsl/shader.vert
build_debug/glsl/shader.frag
build_debug/glsl/clock.comp
build_debug/glsl/nlerp.comp
build_debug/glsl/pulse.comp
build_debug/glsl/lissajous.comp
```

실행:

```bash
../build_debug/bin/gpu_comp_tester
```

### Windows MSVC Release

```bat
cmake --preset win-msvc
cmake --build ../build_win_msvc --config Release --target gpu_comp_tester
```

실행 파일은 일반적으로 다음 위치에 생성됩니다.

```text
build_win_msvc\bin\Release\gpu_comp_tester.exe
```

빌드 후 `SDL3.dll`은 실행 파일 디렉터리로 자동 복사됩니다. 셰이더는
`build_win_msvc\glsl`에 복사됩니다. 실행 파일은 portable 패키지,
일반 설치, 단일 구성 빌드와 MSVC 다중 구성 빌드의 `glsl` 경로를 순서대로
탐색합니다.

다른 preset을 사용할 때도 `gpu_comp_tester` target을 명시적으로 빌드하면 됩니다.

## Portable ZIP 생성

`gpu_comp_tester_zip` target은 `gpu_comp_tester`를 먼저 빌드하고 실행에 필요한 파일을
staging한 뒤 플랫폼별 ZIP을 생성합니다.

Windows MSVC Release:

```bat
cmake --preset win-msvc
cmake --build ../build_win_msvc --config Release --target gpu_comp_tester_zip
```

Linux Release:

```bash
cmake --preset linux-release
cmake --build ../build_release --target gpu_comp_tester_zip -- -j8
```

파일 이름 형식:

```text
gpu-comp-tester-<version>-<platform>-<configuration>.zip
```

Windows Release 예:

```text
build_win_msvc/
└── gpu-comp-tester-0.1.0-windows-x86_64-release.zip
```

ZIP 내부 구조:

```text
gpu_comp_tester/
├── gpu_comp_tester.exe       Windows 실행 파일
├── SDL3.dll                  Windows SDL 런타임
├── glsl/
│   ├── clock.comp
│   ├── nlerp.comp
│   ├── pulse.comp
│   ├── lissajous.comp
│   ├── shader.vert
│   └── shader.frag
└── licenses/
    ├── LICENSE
    ├── THIRD_PARTY_NOTICES.md
    └── SDL3-LICENSE.txt      Windows 패키지에 포함
```

Linux 패키지는 확장자가 없는 `gpu_comp_tester` 실행 파일을 포함하며 시스템의
SDL2와 OpenGL 런타임을 사용합니다. 실행 파일만 필요하면 `gpu_comp_tester` target을
빌드하고, portable 배포 구조와 ZIP이 필요하면 `gpu_comp_tester_zip` target을
빌드합니다. 두 target 모두 일반 `ALL_BUILD`와 분리된 명시적 target입니다.

## 구성

```text
tools/gpu_comp_tester/
├── CMakeLists.txt       빌드, SDL/OpenGL 연결, 셰이더 복사와 설치
├── main.cpp             이벤트 루프와 LERP/SLERP 계산 및 렌더링
├── gpu.h/.cpp           SDL/OpenGL 초기화, 그래픽 셰이더와 UBO 관리
├── gpu_compute.h/.cpp   compute shader와 SSBO 보조 API
├── glsl/                GLSL 셰이더
│   ├── shader.vert      선 정점 처리
│   ├── shader.frag      SLERP/LERP 색상 구분
│   ├── clock.comp       쿼터니언 SLERP compute shader
│   ├── nlerp.comp       쿼터니언 NLERP compute shader
│   ├── pulse.comp       길이 변화 효과 compute shader
│   └── lissajous.comp   8자 궤적 compute shader
└── assets/icons/        애플리케이션 아이콘 PNG 및 Windows ICO
```

프로그램은 BYUL의 `balix` 정적 라이브러리를 연결하고 `quat_t`, `vec3_t` 등의
수치 자료형과 쿼터니언 연산을 사용합니다.

## Compute shader 실행 흐름

프로그램은 OpenGL 4.3 Core 컨텍스트에서 다음 순서로 GPU 연산을 수행합니다.

1. `clock.comp`를 컴파일하고 compute program 생성
2. 두 쿼터니언과 보간값을 UBO binding 0에 기록
3. work group 하나를 dispatch하여 SLERP와 벡터 회전 수행
4. shader storage와 buffer update memory barrier 수행
5. SSBO binding 1에서 회전 결과를 읽어 빨간색 선으로 렌더링
6. BYUL CPU SLERP 결과와 비교하여 위치 오차 누적

쿼터니언은 BYUL의 `(w, x, y, z)` 구조에서 GLSL의 `(x, y, z, w)` 벡터 순서로
명시적으로 변환합니다. UBO와 SSBO의 C++ 구조는 `alignas(16)`을 사용해 GLSL의
`std140` 및 `std430` 배치와 일치시킵니다.

GPU 결과를 매 프레임 CPU로 읽어 오는 방식은 성능 최적화용 구조가 아니라 결과
검증을 위한 의도적인 동기화입니다. 향후 성능 실험에서는 SSBO를 렌더링 단계에서
직접 소비하고 비동기 측정 방식을 사용해야 합니다.

현재 자동화된 테스트는 등록돼 있지 않습니다. 실행 확인에는 SDL 창을 열 수
있는 그래픽 환경이 필요합니다.

## 관련 문서

- [빌드 환경 구성](build-environment.ko.md)
- [BYUL 실행 방법](runtime-execution.ko.md)
