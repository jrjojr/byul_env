#include "doctest.h"

extern "C" {
    #include "internal/numeq_mpc.h"
    #include "internal/numeq_model.h"
}

float dummy_cost(
    const state_vector_t* sim_state,
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
    state_vector_t state = {
        .position = {0.0f, 0.0f, 0.0f},
        .velocity = {1.0f, 0.0f, 0.0f},
        .acceleration = {0.0f, 0.0f, 0.0f}
    };
    vec3_t target = {3.0f, 0.0f, 0.0f};
    vec3_t accel = {0.0f, 0.0f, 0.0f};
    float cost = numeq_mpc_cost_default(&state, &target, &accel, NULL);
    CHECK(cost > 0.0f);
}

TEST_CASE("MPC trajectory init and free") {
    mpc_trajectory_t traj;
    CHECK(mpc_trajectory_init(&traj, 10) == true);
    CHECK(traj.samples != nullptr);
    CHECK(traj.capacity == 10);
    mpc_trajectory_free(&traj);
    CHECK(traj.samples == nullptr);
}

TEST_CASE("MPC directional target structure basic") {
    mpc_direction_target_t dir_target = {
        .direction = {1.0f, 0.0f, 0.0f},
        .weight = 2.0f,
        .duration = 1.0f
    };
    CHECK(dir_target.direction.x == doctest::Approx(1.0f));
    CHECK(dir_target.weight == doctest::Approx(2.0f));
}
