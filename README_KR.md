# 별이의 세계 – 시뮬레이션 및 경로 탐색 엔진

`byul`은 **경로 탐색 엔진**으로 시작하여,
현재는 **미로 생성, 발사체 궤적 예측, 제어 시스템(MPC/PID), 수치 해석**까지 지원하는
**고성능 시뮬레이션 엔진**으로 진화하였습니다.

---

## ✨ 주요 기능
- **경로 탐색**  
  A*, Dijkstra, BFS, D* Lite 알고리즘을 통한 정적/동적 경로 탐색
- **미로 생성**  
  Kruskal, Eller, Binary Tree 등 다양한 알고리즘 제공
- **발사체 예측**  
  중력, 바람, 항력을 고려한 발사체 시뮬레이션 (RK4, Verlet, Euler 통합기)
- **제어 시스템**  
  PID, Bang-Bang, MPC 제어기 지원
- **수치 연산 코어**  
  vec3, quat, dualquat, dualnumber 기반 3D 수학 연산
- **모듈형 아키텍처**  
  `numeq`, `controller`, `trajectory`, `motion_state` 등의 독립 모듈로 구성

---

## 📈 프로젝트 진행 순서
1. **경로 탐색 코어**: A*, Dijkstra, D* Lite + PySide6 시각화
2. **미로 생성기**: Binary Tree → Prim → Eller → Kruskal
3. **수치 연산 핵심**: vec3, quat, dualquat, motion_state 구현
4. **제어 이론**: PID, Bang-Bang, MPC 제어 구조 구현
5. **궤적 시스템**: 다양한 통합기 + 충돌 예측 기능

---

## 🛠 빌드 방법
### Linux / macOS
```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul
mkdir build && cd build
cmake ..
make -j$(nproc)
ctest
```

### Windows (MinGW)
```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul
mkdir build && cd build
cmake -G "MinGW Makefiles" ..
mingw32-make -j4
ctest
```

---

## 🧪 테스트 예제
### A* 경로 탐색
```cpp
TEST_CASE("navsys: find astar") {
    navgrid_t* navgrid = navgrid_create();
    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_astar(navgrid, &start, &goal);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
}
```

### 미로 생성 (Kruskal)
```cpp
TEST_CASE("Kruskal Algorithm Maze Generation") {
    maze_t* maze = maze_make_kruskal(0, 0, 19, 19);
    REQUIRE(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);
    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
```

### 발사체 궤적 예측 (RK4)
```cpp
TEST_CASE("projectile_predict - ground collision") {
    vec3_t start_pos = {0, 500, 0};
    vec3_t target_pos = {0, 0, 0};
    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = target_pos;

    projectile_result_t* result = projectile_result_create();
    environ_t env;
    environ_init(&env);

    bool hit = projectile_predict(
        result, &proj, &entdyn, 500.0f, 1.0f, &env, nullptr, guidance_none);

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time,
           vec3_to_string(&result->impact_pos, 64, buf));
    projectile_result_destroy(result);
}
```

---

## 📘 왜 `path`가 아닌 `route`인가?

`byul`에서는 `path` 대신 `route`라는 용어를 사용합니다:
- `path`는 일반적으로 파일 경로로 인식되어 혼동을 줄 수 있습니다.
- `route`는 **게임 시뮬레이션에서의 이동 계획**을 더 명확히 표현합니다.
- `route_finder`, `dstar_lite`, `navsys` 등 전체적으로 일관된 용어 체계를 유지합니다.

---

## 📩 연락처
**byuldev@outlook.kr**

---

## 📄 라이선스
- **Byul World Public License v1.0**  
  - 개인 학습 및 연구 목적에 한해 자유롭게 사용 가능  
  - 상업적 사용 및 재배포는 금지됨  
  - 자세한 내용은 LICENSE 파일 참조

---

## 💬 개발자 노트
> `byul`은 단순한 길찾기 도구가 아닙니다. 
> 이는 실시간 시뮬레이션 코어로 성장하며, 
> 미래의 **별이의 세계**를 구현하기 위한 기반이 됩니다.
