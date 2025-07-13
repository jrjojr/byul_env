# 🌟 Byul's World – Path Finding Engine

`byul` 모듈은 실시간 시뮬레이션 게임의 경로 탐색을 담당하는  
경량 고성능 모듈입니다.  
이 모듈은 미로 생성, 경로 탐색, 실시간 재계산 기능을 중심으로 구성되며,  
`maze`, `dstar_lite`, `route_finder` 세 모듈이 전체 구조의 중심입니다.

---

## 🧩 핵심 구성 요소

### 🌀 1. Maze Generator – `maze/`

맵에 삽입 가능한 **독립형 미로 생성기**입니다.  
`maze_t` 구조체로 생성 후, `map_t`에 삽입하여 장애물 패턴으로 사용됩니다.

- 지원 알고리즘: Binary Tree, Prim, Eller, Kruskal 등
- 동작 방식: `maze_make()`로 생성 → `maze_apply_to_map()`로 삽입
- 활용 목적: 초기 맵 설정, 스테이지 구성, 테스트 자동화 등

#### 주요 인터페이스:
```c
maze_t* maze_new();
void maze_make(maze_t* maze, maze_type_t type);
void maze_apply_to_map(const maze_t* maze, map_t* map);
void maze_free(maze_t*);
```

---

### 🧠 2. D* Lite Module – `dstar_lite/`

동적 환경에서의 경로 재탐색을 위한 **D\* Lite 알고리즘** 구현체입니다.  
`dstar_lite_t`는 알고리즘 상태를 내부에 유지하며,  
`update_vertex`, `compute_shortest_path` 등을 통해 실시간 반응이 가능합니다.

- 키 정렬: `dstar_lite_key` 구조 기반
- 우선순위 큐: 별도 `dstar_lite_pqueue`로 분리
- 휴리스틱 계산: `dstar_lite_utils`로 모듈화

> 단일 호출형이 아닌, **실시간 장애물 변화 대응 경로 탐색기**입니다.

---

### 🚦 3. Static Route Finder – `route_finder/`

A\*, Dijkstra, BFS, JPS 등 **정적 알고리즘**을 통합한 경로 탐색기입니다.  
실시간 반응이 필요 없는 단발성 탐색에 적합합니다.

- `route_finder_t` 구조체로 알고리즘 선택/실행/결과 추출
- `route_finder_find()` → 경로 계산 및 결과 반환

---

## 🛠 보조 구성 요소

### 📌 `coord/`
- 2D 정수 좌표 구조체 (`coord_t`)
- 해시 지원 (`coord_hash`), 리스트 연산 (`coord_list`)

### 📌 `map/`
- `map_t`: 셀 단위로 구성된 맵 구조
- 장애물 위치, 이동 가능 여부 판단
- `map_is_blocked(coord_t*)` 등으로 활용

### 📌 `cost_coord_pq/`
- `coord + cost` 기반 우선순위 큐
- 최소 비용 경로 계산 시 사용
- 직접 구현한 경량 힙 기반 구조체

---

## 📘 용어 설명 – 왜 `path`가 아닌 `route`인가?

별이의 세계에서는 `path`라는 단어 대신 **`route`**를 사용합니다. 이유는 다음과 같습니다:

- `path`는 파일 경로를 떠올리게 합니다. 파일경로인지 길찾기 경로인지 알기가 어렵습니다.
- 게임 내에서는 **'목적지를 향한 전략적 이동 경로'**라는 의미에 더 가까운 `route`를 선택했습니다.
- 특히 정적 탐색(`route_finder`)과 동적 탐색(`dstar_lite`)을 구분할 때 `route`라는 용어가
  **의도, 방향성, 설계된 흐름**을 내포하므로 더 적절합니다.

> 이 용어는 설계 철학의 일환이며, 내부 구조 전반에서 일관되게 사용됩니다.

---

## ▶️ 사용 예제

### 🔹 A* 정적 경로 찾기

고정된 맵에서 가장 일반적인 A* 알고리즘을 사용하려면 다음처럼 작성합니다:

```c
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    std::cout << "default\n";
    map_t* m = map_new();
    // 장애물 삽입 (세로 차단)
    for (int y = 1; y < 10; ++y)
        map_block_coord(m, 5, y);

    route_finder_t* a = route_finder_new(m);
    route_finder_set_goal(a, goal);
    route_finder_set_start(a, start);
    route_finder_set_visited_logging(a, true);
    route_t* p = nullptr;

    p = route_finder_find(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    map_print_ascii_with_visited_count(m, p, 5);
    route_free(p);    
    route_finder_free(a);
    map_free(m);

    coord_free(start);
    coord_free(goal);    
```

### 🧩 요약

| 상황 | 추천 알고리즘 | 특징 |
|------|---------------|------|
| 장애물 변화 O | D* Lite | 상태 보존형, 실시간 대응 |
| 장애물 고정 | A*, Dijkstra 등 | 단발성 호출, 계산 빠름 |
| 알고리즘 선택 자동화 | `route_finder` | 여러개의 알고리즘 통합 |

## ⚙️ 빌드 방법

```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul
mkdir build && cd build
cmake ..
make -j$(nproc)
```

> Windows: `byul.dll` / Linux: `libbyul.so` 생성됨

---

## 🧪 테스트

```bash
cd byul/maze/tests
make
./test_maze
```

> 모든 모듈은 `doctest` 기반 CLI 테스트를 포함하고 있습니다.

---

## 📄 라이센스 – Byul World Public License v1.0

| 허용됨                    | 금지됨                                     |
|----------------------------|---------------------------------------------|
| 개인 학습/분석            | 상업적 사용 (직접/간접 수익 포함)         |
| 연구 및 비상업적 데모 실행 | 재배포, 리패키징, 호스팅, 라이브러리화      |
| 출처 명시한 참고           | 저작권 표기 누락 또는 허위 명시             |

문의: **byuldev@outlook.kr**

> LICENSE 파일 전문을 확인하세요.

---

## 💬 개발자의 말

이 프로젝트는 단순한 경로 탐색기가 아닙니다.  
**게임 세계를 살아 움직이게 만드는 최소 단위 시스템**입니다.

미로는 장애물의 예술이고,  
경로는 존재의 의지이며,  
탐색은 세계를 향한 질문입니다.

© 2025 ByulPapa (byuldev@outlook.kr)  
All rights reserved.