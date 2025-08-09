#include "doctest.h"
#include <chrono>
#include <iostream>

extern "C" {
    #include "numeq_mpc.h"
    #include "numeq_model.h"
    #include "trajectory.h"
}

TEST_CASE("MPC default cost function produces positive cost") {
    motion_state_t ms = {};
    ms.linear.velocity = {1.0f, 0.0f, 0.0f};
    quat_identity(&ms.angular.orientation);

    motion_state_t target = {};
    target.linear.position = {3.0f, 0.0f, 0.0f};
    quat_identity(&target.angular.orientation);

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
    mpc_direction_target_t dir_target;

    mpc_direction_target_init(&dir_target);
	dir_target.weight_dir = 2.0f;
	dir_target.weight_rot = 1.0f;
	dir_target.duration = 1.0f;

    CHECK(dir_target.direction.x == doctest::Approx(1.0f));
    CHECK(dir_target.weight_dir == doctest::Approx(2.0f));
    CHECK(dir_target.duration == doctest::Approx(1.0f));
}

TEST_CASE("numeq_mpc_solve_coarse2fine basic test") {
    motion_state_t current;
    motion_state_init(&current);

    motion_state_t target;
    motion_state_init(&target);
    target.linear.position = {10, 0, 0};

    mpc_config_t config;
    mpc_config_init(&config);
    config.max_accel = 5.0f;
    config.max_ang_accel = 0.0f;
    config.horizon_sec = 5;
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
    CHECK(result.desired_accel.x > 0.0f);
    CHECK(result.cost >= 0.0f);
    CHECK(traj.count > 0);

    const trajectory_sample_t& last = traj.samples[traj.count - 1];
    float dist = vec3_distance(
        &last.state.linear.position, &target.linear.position);

    CHECK(dist < 10.0f);
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
    target.linear.position = {10, 0, 0};

    environ_t env;
    environ_init(&env);
    env.gravity = {0.0f, -9.8f, 0.0f};
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

    trajectory_print(&traj);
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
