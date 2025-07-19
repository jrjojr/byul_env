#include "doctest.h"

extern "C" {
    #include "internal/numeq_mpc.h"
    #include "internal/numeq_model.h"
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
    trajectory_t traj;
    CHECK(trajectory_init(&traj, 10) == true);
    CHECK(traj.samples != nullptr);
    CHECK(traj.capacity == 10);

    trajectory_free(&traj);
    CHECK(traj.samples == nullptr);
    CHECK(traj.capacity == 0);
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
