#include "doctest.h"
#include "vec3.h"
#include "entity_dynamic.h"
#include "trajectory.h"
#include "entity_spring.h"
#include <iostream>

TEST_CASE("Spring Simulation - Two Entities Converging") {
    // --- 초기 엔티티 설정 ---
    entity_dynamic_t e[2];
    entity_dynamic_init(&e[0]);
    entity_dynamic_init(&e[1]);

    // 엔티티 초기 위치 (L0보다 멀리 배치)
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&e[1].xf.pos, 5.0f, 0.0f, 0.0f);

    // 초기 속도는 0
    vec3_zero(&e[0].velocity);
    vec3_zero(&e[1].velocity);

    // --- 스프링 파라미터 ---
    const float k = 2.0f;      // 스프링 강성
    const float c = 0.1f;      // 감쇠 계수
    const float L0 = 2.0f;     // 목표 거리
    const float dt = 0.1f;     // 시뮬레이션 시간 간격
    const int steps = 50;      // 시뮬레이션 스텝 수

    // --- Trajectory 생성 ---
    trajectory_t* traj = trajectory_create_full(steps * 2);

    // --- 시뮬레이션 실행 ---
    spring_simulate(traj, e, 2, dt, k, c, L0, steps);

    // --- 최종 위치 체크 ---
    float final_dist = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    CHECK(final_dist < 5.0f);          // 서로 가까워져야 함
    CHECK(final_dist > L0 - 0.5f);     // 목표 거리 근처에 도달해야 함

    // --- Trajectory 출력 ---
    // char buffer[TRAJECTORY_STR_BUFSIZE*5];
    // trajectory_to_string(traj, buffer, sizeof(buffer));
    // std::cout << "[Trajectory Output]\n" << buffer << std::endl;
    // trajectory_print(traj);

    // --- 메모리 정리 ---
    trajectory_destroy(traj);
}

TEST_CASE("Pairwise Spring - Triangle Stabilization") {
    entity_dynamic_t e[3];
    for (int i = 0; i < 3; ++i)
        entity_dynamic_init(&e[i]);

    // 초기 삼각형 좌표
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&e[1].xf.pos, 5.0f, 0.0f, 0.0f);
    vec3_init_full(&e[2].xf.pos, 2.5f, 0.0f, 4.0f);

    for (int i = 0; i < 3; ++i)
        vec3_zero(&e[i].velocity);

    const float k = 1.0f;
    const float c = 0.5f;
    const float L0 = 3.0f;
    const float dt = 0.05f;
    const int steps = 100;

    trajectory_t* traj = trajectory_create_full(steps * 3);

    spring_simulate_pairwise(traj, e, 3, dt, k, c, L0, steps);

    // 최종 거리 체크
    float d01 = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    float d12 = vec3_distance(&e[1].xf.pos, &e[2].xf.pos);
    float d20 = vec3_distance(&e[2].xf.pos, &e[0].xf.pos);

    CHECK(d01 > L0 - 0.5f);
    CHECK(d12 > L0 - 0.5f);
    CHECK(d20 > L0 - 0.5f);

    CHECK(d01 < L0 + 0.5f);
    CHECK(d12 < L0 + 0.5f);
    CHECK(d20 < L0 + 0.5f);

    // Trajectory 출력
    // char buffer[TRAJECTORY_STR_BUFSIZE*5];
    // trajectory_to_string(traj, buffer, sizeof(buffer));
    // std::cout << "[Triangle Pairwise Trajectory Output]\n" << buffer << std::endl;
    // trajectory_print(traj);

    trajectory_destroy(traj);
}

TEST_CASE("Spring network approach-and-separate dynamics") {
    // --- 초기 엔티티 설정 (두 사람이 마주보고 있음) ---
    entity_dynamic_t e[2];
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);  // 첫 번째 사람
    vec3_init_full(&e[1].xf.pos, 3.0f, 0.0f, 0.0f);  // 두 번째 사람 (3m 거리)

    for (int i = 0; i < 2; ++i) {
        vec3_zero(&e[i].velocity);
        e[i].props.mass = 70.0f;  // 체중 70kg 정도
    }

    // --- 스프링 설정 ---
    // 자연 길이 L0 = 2.0m (서로 2m 거리에서 균형)
    // k = 50.0f (상당히 강한 스프링)
    // c = 5.0f (약간의 감쇠)
    spring_link_t links[] = {
        {0, 1, 50.0f, 5.0f, 2.0f}
    };

    trajectory_t traj = {};
    trajectory_init_full(&traj, 3000);

    float dt = 0.01f;   // 20ms
    int steps = 200;    // 4초간 시뮬레이션
    spring_simulate_network(&traj, e, 2, links, 1, dt, steps);

    // --- 검사 포인트 ---
    // 1. 두 사람이 L0=2.0m 근처로 접근해야 함
    float dist = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    CHECK(dist == doctest::Approx(2.0f).epsilon(0.2f));  // ±0.2m 허용

    // 2. 속도는 어느 정도 줄어들어야 함
    float v0 = vec3_length(&e[0].velocity);
    float v1 = vec3_length(&e[1].velocity);
    CHECK(v0 < 1.0f);
    CHECK(v1 < 1.0f);

    // trajectory_print(&traj);
    trajectory_free(&traj);
}

// TEST_CASE("Spring network convergence") {
//     // --- 초기 엔티티 설정 (3개 엔티티 삼각형 구조) ---
//     entity_dynamic_t e[3];
//     vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
//     vec3_init_full(&e[1].xf.pos, 5.0f, 0.0f, 0.0f);
//     vec3_init_full(&e[2].xf.pos, 2.5f, 4.0f, 0.0f);

//     for (int i = 0; i < 3; ++i) {
//         vec3_zero(&e[i].velocity);
//         e[i].props.mass = 70.0f;  // 대략 사람 체중
//     }

//     // --- 스프링 링크 설정 ---
//     // 삼각형 연결 (0-1, 1-2, 2-0)
//     spring_link_t links[] = {
//         {0, 1, 80.0f, 5.0f, 5.0f},  // k=80, c=5, L0=5m
//         {1, 2, 80.0f, 5.0f, 5.0f},
//         {2, 0, 80.0f, 5.0f, 5.0f}
//     };

//     trajectory_t traj = {};
//     trajectory_init_full(&traj, 50000);

//     float dt = 0.01f;   // 10ms
//     int steps = 400;   // 총 10초 시뮬레이션
//     spring_simulate_network(&traj, e, 3, links, 3, dt, steps);

//     // --- 검증 포인트 ---
//     // 각 링크가 L0에 가까워야 한다.
//     for (int l = 0; l < 3; ++l) {
//         float dist = vec3_distance(&e[links[l].i].xf.pos, &e[links[l].j].xf.pos);
//         CHECK(dist == doctest::Approx(links[l].L0).epsilon(0.05f));
//     }

//     // 속도 수렴 확인
//     for (int i = 0; i < 3; ++i) {
//         float v = vec3_length(&e[i].velocity);
//         CHECK(v < 0.05f);
//     }

//     // 궤적 디버깅 출력
//     // trajectory_print(&traj);
//     trajectory_free(&traj);
// }

// TEST_CASE("Repulsion network - triangle spacing") {
//     // --- 초기 엔티티 설정 (삼각형) ---
//     entity_dynamic_t e[3];
//     vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
//     vec3_init_full(&e[1].xf.pos, 0.5f, 0.0f, 0.0f);  // 0.5m 간격
//     vec3_init_full(&e[2].xf.pos, 0.25f, 0.0f, 0.4f); // 거의 붙어있는 삼각형

//     for (int i = 0; i < 3; ++i) {
//         vec3_zero(&e[i].velocity);
//         e[i].props.mass = 1.0f;
//     }

//     // --- 링크 (안전거리 L0=1.5m) ---
//     spring_link_t links[] = {
//         {0, 1, 10.0f, 1.0f, 1.5f},
//         {1, 2, 10.0f, 1.0f, 1.5f},
//         {2, 0, 10.0f, 1.0f, 1.5f},
//     };

//     trajectory_t* traj = trajectory_create_full(1500);
//     // --- 시뮬레이션 실행 ---
//     repulsion_simulate_network(traj, e, 3, links, 3, 0.01f, 100);

//     // --- 결과 검증 ---
//     for (int i = 0; i < 3; ++i) {
//         for (int j = i + 1; j < 3; ++j) {
//             vec3_t d;
//             vec3_sub(&d, &e[j].xf.pos, &e[i].xf.pos);
//             float dist = vec3_length(&d);
//             CHECK(dist >= 1.4f); // 최소 안전거리 이상
//         }
//     }
//     // trajectory_print(traj);
//     trajectory_destroy(traj);
// }

// TEST_CASE("Repulsion soft vs hard") {
//     entity_dynamic_t e_soft[2], e_hard[2];

//     // 초기화
//     vec3_init_full(&e_soft[0].xf.pos, 0.0f, 0.0f, 0.0f);
//     vec3_init_full(&e_soft[1].xf.pos, 0.5f, 0.0f, 0.0f);
//     vec3_init_full(&e_hard[0].xf.pos, 0.0f, 0.0f, 0.0f);
//     vec3_init_full(&e_hard[1].xf.pos, 0.5f, 0.0f, 0.0f);

//     for (int i = 0; i < 2; ++i) {
//         vec3_zero(&e_soft[i].velocity);
//         vec3_zero(&e_hard[i].velocity);
//         e_soft[i].props.mass = e_hard[i].props.mass = 1.0f;
//     }

//     // Soft link
//     spring_link_t links_soft[] = { {0, 1, 5.0f, 1.0f, 1.5f} };
//     // Hard link
//     spring_link_t links_hard[] = { {0, 1, 50.0f, 5.0f, 1.5f} };

//     trajectory_t* traj_soft = trajectory_create_full(500);
//     trajectory_t* traj_hard = trajectory_create_full(500);

//     repulsion_simulate_network(traj_soft, e_soft, 2, links_soft, 1, 0.01f, 500);
//     repulsion_simulate_network(traj_hard, e_hard, 2, links_hard, 1, 0.01f, 500);

//     // Soft 모드는 속도 변화가 느리고, Hard는 초기에 큰 반발 속도를 보임
//     CHECK(vec3_length(&e_soft[0].velocity) < vec3_length(&e_hard[0].velocity));

//     // trajectory_print(traj_soft);
//     // trajectory_print(traj_hard);

//     trajectory_destroy(traj_soft);
//     trajectory_destroy(traj_hard);
// }

// TEST_CASE("Repulsion push 1m in 10s") {
//     entity_dynamic_t e[2];
//     vec3_init_full(&e[0].xf.pos, -0.5f, 0.0f, 0.0f);
//     vec3_init_full(&e[1].xf.pos,  0.5f, 0.0f, 0.0f);

//     for (int i = 0; i < 2; ++i) {
//         vec3_zero(&e[i].velocity);
//         e[i].props.mass = 1.0f;
//     }

//     spring_link_t links[] = {
//         {0, 1, 0.02f, 0.0f, 1.0f} // k=0.02, c=0, L0 무시
//     };

//     trajectory_t traj = {};
//     trajectory_init_full(&traj, 1000);
//     repulsion_simulate_network(&traj, e, 2, links, 1, 0.1f, 100); // 10s

//     // trajectory_print(&traj); // 최종 위치 확인
//     trajectory_free(&traj);
// }
// TEST_CASE("Push simulation - 10s, 1m displacement") {
//     entity_dynamic_t e[2];
//     for (int i = 0; i < 2; ++i) {
//         vec3_zero(&e[i].velocity);
//         vec3_zero(&e[i].xf.pos);
//         e[i].props.mass = 1.0f;  // 1kg
//     }
//     // 서로 반대 방향으로 밀기
//     // vec3_t forces[2] = { {0.1f, 0, 0}, {-0.1f, 0, 0} }; // 0.1N
//     // 서로 반대 방향으로 밀기 (힘 크기 줄임)
//     vec3_t forces[2] = { {0.02f, 0, 0}, {-0.02f, 0, 0} }; // 0.02N


//     trajectory_t traj = {};
//     trajectory_init_full(&traj, 1000);

//     float dt = 0.1f;  // 10ms
//     int steps = 100;  // 10s

//     push_simulate_network(&traj, e, 2, forces, dt, steps);

//     // 10초 동안 이동 거리 확인 (~1m)
//     CHECK(e[0].xf.pos.x > 0.9f);
//     CHECK(e[1].xf.pos.x < -0.9f);

//     CHECK(e[0].xf.pos.x < 1.1f);
//     CHECK(e[1].xf.pos.x > -1.1f);

//     // trajectory_print(&traj);
//     trajectory_free(&traj);
// }
