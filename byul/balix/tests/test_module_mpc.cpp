#include "doctest.h"
#include <chrono>
#include <iostream>

extern "C" {
    #include "internal/numeq_mpc.h"
    #include "internal/numeq_model.h"
    #include "internal/trajectory.h"
}

float dummy_cost(
    const linear_state_t* sim_state,
    const vec3_t* target,
    const vec3_t* accel,
    void* userdata)
{
    (void)userdata;
    float dx = target->x - sim_state->position.x;
    float dy = target->y - sim_state->position.y;
    float dz = target->z - sim_state->position.z;
    return dx * dx + dy * dy + dz * dz;
}

TEST_CASE("MPC default cost function produces positive cost") {
    // 현재 상태
    motion_state_t ms = {};
    ms.linear.position = {0.0f, 0.0f, 0.0f};
    ms.linear.velocity = {1.0f, 0.0f, 0.0f};
    ms.linear.acceleration = {0.0f, 0.0f, 0.0f};
    quat_identity(&ms.angular.orientation);
    ms.angular.angular_velocity = {0.0f, 0.0f, 0.0f};
    ms.angular.angular_acceleration = {0.0f, 0.0f, 0.0f};

    // 목표 상태
    motion_state_t target = {};
    target.linear.position = {3.0f, 0.0f, 0.0f};
    quat_identity(&target.angular.orientation);

    // 가속도 제어 입력
    vec3_t accel = {0.0f, 0.0f, 0.0f};
    vec3_t ang_accel = {0.0f, 0.0f, 0.0f};

    float cost = numeq_mpc_cost_default(&ms, &target, NULL);
    CHECK(cost > 0.0f);
}

TEST_CASE("MPC trajectory init and free") {
    trajectory_t* traj = trajectory_create_full(10);
    CHECK(traj != nullptr);
    CHECK(traj->samples != nullptr);
    CHECK(traj->capacity == 10);

    trajectory_destroy(traj);    
}

TEST_CASE("MPC directional target structure basic") {
    quat_t oq;
    quat_identity(&oq);
    mpc_direction_target_t dir_target = {
        .direction = {1.0f, 0.0f, 0.0f},
        .orientation = oq, // 단위 쿼터니언 값
        .weight_dir = 2.0f,
        .weight_rot = 1.0f,
        .duration = 1.0f
    };
    CHECK(dir_target.direction.x == doctest::Approx(1.0f));
    CHECK(dir_target.weight_dir == doctest::Approx(2.0f));
    CHECK(dir_target.duration == doctest::Approx(1.0f));
}

// ---------------------------------------------------------
// 기본 비용 함수: 단순 위치 오차 제곱합
// ---------------------------------------------------------
static float test_cost_fn(
    const motion_state_t* state,
    const motion_state_t* target,
    void* userdata)
{
    vec3_t diff;
    vec3_sub(&diff, &state->linear.position, &target->linear.position);
    return vec3_length_sq(&diff);
}

// 간단한 비용 함수: 목표 위치와의 거리 제곱
float simple_cost_fn(const motion_state_t* state,
                     const motion_state_t* target,
                     void* userdata)
{
    vec3_t diff;
    vec3_sub(&diff, &state->linear.position, &target->linear.position);
    return vec3_length_sq(&diff); // 거리 제곱
}

TEST_CASE("numeq_mpc_solve basic test") {
    motion_state_t current = {};
    current.linear.position = {0, 0, 0};
    current.linear.velocity = {0, 0, 0};

    motion_state_t target = {};
    target.linear.position = {10, 0, 0}; // x축으로 10m 떨어진 목표

    mpc_config_t config = {};
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 3;
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj); // trajectory 초기화

    bool ok = numeq_mpc_solve(
        &current,
        &target,
        nullptr,        // 환경 없음
        nullptr,        // 바디 특성 없음
        &config,
        &result,
        &traj,
        simple_cost_fn,
        nullptr
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x >= 0.0f);  // 목표가 x축 +방향이므로 가속도 양수
    CHECK(result.cost >= 0.0f);             // 비용은 음수가 될 수 없음
    CHECK(traj.count > 0);                  // 샘플이 기록되어야 함

    // 마지막 샘플이 목표 위치와 가까워야 함
    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);
        
    CHECK(dist < 10.0f); // 최소한 접근해야 함 (10m 이내로)

    trajectory_print(&traj);
    trajectory_free(&traj);
}


TEST_CASE("numeq_mpc_solve basic test v1") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 3;
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    bool ok = numeq_mpc_solve(
        &current,
        &target,
        nullptr,
        nullptr,
        &config,
        &result,
        &traj,
        numeq_mpc_cost_default,
        &config
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x >= 0.0f);
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 1.0f); // 더 엄격하게 검사

    trajectory_print(&traj);
    trajectory_free(&traj);
}

TEST_CASE("numeq_mpc_solve basic test v2") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0}; // 10m 앞 목표로 설정!

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 5;   // 더 길게 예측
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    bool ok = numeq_mpc_solve(
        &current,
        &target,
        nullptr,
        nullptr,
        &config,
        &result,
        &traj,
        numeq_mpc_cost_default,
        &config
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x > 0.0f);  // x축으로 가속해야 함
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 10.0f); // 최소한 접근
    trajectory_print(&traj);
    trajectory_free(&traj);
}

TEST_CASE("numeq_mpc_solve basic test v3") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0}; // 10m 앞 목표로 설정!

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 5;   // 더 길게 예측
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    bool ok = numeq_mpc_solve(
        &current,
        &target,
        &env,
        &body,
        &config,
        &result,
        &traj,
        numeq_mpc_cost_default,
        &config
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x > 0.0f);  // x축으로 가속해야 함
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 10.0f); // 최소한 접근
    trajectory_print(&traj);
    trajectory_free(&traj);
}

TEST_CASE("numeq_mpc_solve_fast basic test") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0}; // 10m 앞 목표로 설정!

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 5;   // 더 길게 예측
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    bool ok = numeq_mpc_solve_fast(
        &current,
        &target,
        &env,
        &body,
        &config,
        &result,
        &traj,
        numeq_mpc_cost_default,
        &config
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x > 0.0f);  // x축으로 가속해야 함
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 10.0f); // 최소한 접근
    trajectory_print(&traj);
    trajectory_free(&traj);
}

TEST_CASE("numeq_mpc_solve_coarse2fine basic test") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0}; // 10m 앞 목표로 설정!

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 5;   // 더 길게 예측
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    bool ok = numeq_mpc_solve_coarse2fine(
        &current,
        &target,
        &env,
        &body,
        &config,
        &result,
        &traj,
        numeq_mpc_cost_default,
        &config
    );

    CHECK(ok == true);
    CHECK(result.desired_accel.x > 0.0f);  // x축으로 가속해야 함
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 10.0f); // 최소한 접근
    trajectory_print(&traj);
    trajectory_free(&traj);
}

static float run_mpc_test(
    const char* name,
    bool (*mpc_fn)(
        const motion_state_t*, 
        const motion_state_t*, 
        const environ_t*, 
        const bodyprops_t*, 
        const mpc_config_t*,
        mpc_output_t*, trajectory_t*, mpc_cost_func, void*))
{
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0}; // x=10m 목표

    environ_t env;
    environ_init(&env);
    env.gravity = {0.0f, -9.8f, 0.0f}; // 동일한 중력 조건
    env.wind = {0.0f, 0.0f, 0.0f};

    bodyprops_t body;
    bodyprops_init(&body);
    body.mass = 1.0f;

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.horizon_sec = 5;
    config.step_dt = 0.1f;
    config.output_trajectory = true;

    mpc_output_t result = {};
    trajectory_t traj = {};
    trajectory_init(&traj);

    auto start = std::chrono::high_resolution_clock::now();
    bool ok = mpc_fn(&current, &target, &env, &body, &config,
                     &result, &traj, numeq_mpc_cost_default, &config);
    auto end = std::chrono::high_resolution_clock::now();

    long duration = std::chrono::duration_cast<
        std::chrono::microseconds>(end - start).count();

    if (!ok) {
        printf("%s: FAILED\n", name);
        trajectory_free(&traj);
        return -1.0f;
    }

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    printf("[%s] time=%ld us | cost=%.3f | final_dist=%.3f | final_vel=%.3f\n",
           name, duration, result.cost, dist,
           vec3_length(&last.state.linear.velocity));

    trajectory_free(&traj);
    return dist;
}

TEST_CASE("MPC Benchmark") {
    printf("\n===== MPC Benchmark (5s horizon) =====\n");
    float d1 = run_mpc_test("basic", numeq_mpc_solve);
    float d2 = run_mpc_test("fast", numeq_mpc_solve_fast);
    float d3 = run_mpc_test("coarse2fine", numeq_mpc_solve_coarse2fine);

    CHECK(d1 >= 0.0f);
    CHECK(d2 >= 0.0f);
    CHECK(d3 >= 0.0f);
}
