# Byul's World – Simulation & Pathfinding Engine

`byul`은 **길찾기 엔진**으로 시작하여,  
**미로 생성, 발사체 궤적 예측, 제어 시스템(MPC/PID), 수치해석**까지 확장된  
**경량 고성능 시뮬레이션 엔진**입니다.

> 💖 If you enjoy this project, consider supporting development at [paypal.me/jrjojr](https://paypal.me/jrjojr)

---

## ✨ 주요 특징
- **경로 탐색(Pathfinding)**  
  A*, Dijkstra, BFS, D* Lite 등 정적·동적 알고리즘 지원.
- **미로 생성(Maze Generation)**  
  Binary Tree, Eller, Kruskal 등 다양한 미로 패턴.
- **발사체 궤적 예측(Projectile Prediction)**  
  중력, 항력, 바람을 반영한 RK4/Verlet 기반 궤적 계산.
- **제어 시스템(Control)**  
  PID, MPC(Model Predictive Control) 기반 궤적 제어.
- **수치해석(Core Math)**  
  vec3, quat, dualquat, dualnumber 기반 3D 수학 연산.
- **모듈화 아키텍처**  
  `numeq`, `controller`, `motion_state`, `trajectory` 등 독립 모듈.

---

## 📜 프로젝트 진화
1. **길찾기 엔진 개발**  
   A*, Dijkstra, D* Lite 완성 → `PySide6` 기반 2D 테스트.
2. **미로 생성 기능 추가**  
   Binary Tree, Prim, Eller, Kruskal 알고리즘 구현.
3. **수치해석·물리 코어 추가**  
   vec3, quat, dualquat, motion_state 등 3D 연산 기반 확립.
4. **제어 시스템 통합**  
   PID, Bang-Bang, MPC 기반 경로 및 속도 제어.
5. **발사체 궤적 예측 시스템**  
   Euler, Semi-Implicit, Verlet, RK4 적분기 및 trajectory 기록 기능 구현.

---

## 🚀 빌드 및 실행
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
### 경로 탐색 (A*)
```c
    navgrid_t* m = navgrid_create();

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);
    route_finder_set_start(rf, &start);
    route_finder_set_goal(rf, &goal);

    // 장애물 삽입 (세로 차단)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = route_finder_run(rf);

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    route_finder_destroy(rf);
    navgrid_destroy(m);
```

### 발사체 궤적 예측 (RK4)
```c
TEST_CASE("projectile_predict - ground collision") {
    MESSAGE("\nprojectile_predict - ground collision");
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
        result,          // [out] 발사체 궤적 및 충돌 정보 저장 (projectile_result_t*)
        &proj,           // [in]  발사체 엔티티
        &entdyn,         // [in]  타겟 엔티티 (충돌 판정 대상)
        500.0f,          // [in]  max_time: 예측 최대 시간 (초)
        1.0f,            // [in]  dt: 시뮬레이션 샘플링 간격 (초)
        &env,            // [in]  환경 정보 (중력, 바람 등)
        nullptr,         // [in]  추진기 (없으면 null)
        guidance_none    // [in]  유도 함수 포인터 (없으면 guidance_none)
    );

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, buf, 64));    
    projectile_result_destroy(result);
}
```


## 📘 용어 설명 – 왜 `path`가 아닌 `route`인가?

별이의 세계에서는 `path`라는 단어 대신 **`route`**를 사용합니다. 이유는 다음과 같습니다:

- `path`는 파일 경로를 떠올리게 합니다. 파일경로인지 길찾기 경로인지 알기가 어렵습니다.
- 게임 내에서는 **'목적지를 향한 전략적 이동 경로'**라는 의미에 더 가까운 `route`를 선택했습니다.
- 특히 정적 탐색(`route_finder`)과 동적 탐색(`dstar_lite`)을 구분할 때 `route`라는 용어가
  **의도, 방향성, 설계된 흐름**을 내포하므로 더 적절합니다.

> 이 용어는 설계 철학의 일환이며, 내부 구조 전반에서 일관되게 사용됩니다.

---
문의: **byuldev@outlook.kr**

## 📄 라이선스
- **Byul World Public License v1.0**  
  - 개인 학습 및 연구 목적 사용 허용  
  - 상업적 사용 및 재배포 금지  
  - 상세 내용은 LICENSE 파일 참조

---

## 💬 개발자의 말
> `byul`은 단순한 길찾기를 넘어 **실시간 시뮬레이션 엔진**으로 성장했습니다.  
> 길찾기에서 발사체 궤적까지 이어진 이 여정은,  
> **별이의 세계**를 더 정교하게 만들기 위한 기초입니다.
