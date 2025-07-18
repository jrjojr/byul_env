#include "doctest.h"

extern "C" {
#include "internal/numeq_integrator.h"
}

TEST_CASE("Euler Integration: basic update") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {1, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, 0, 0};
    float dt = 1.0f;

    numeq_integrate_euler(&state, &accel, dt);

    CHECK(state.velocity.x == doctest::Approx(1.0f));
    CHECK(state.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Semi-Implicit Euler: acceleration applied first") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {2, 0, 0};
    float dt = 0.5f;

    numeq_integrate_semi_implicit(&state, &accel, dt);

    CHECK(state.velocity.x == doctest::Approx(1.0f));
    CHECK(state.position.x == doctest::Approx(0.5f)); // uses updated v
}

TEST_CASE("Verlet Integration: past position affects update") {
    vec3_t current = {1.0f, 0.0f, 0.0f};
    vec3_t prev    = {0.0f, 0.0f, 0.0f};
    vec3_t accel   = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f;

    numeq_integrate_verlet(&current, &prev, &accel, dt);

    CHECK(current.x == doctest::Approx(2.0f));
}

TEST_CASE("RK4 Integration: acceleration effect (simple)") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {1.0f, 0.0f, 0.0f};
    float dt = 1.0f;

    numeq_integrate_rk4(&state, &accel, dt);

    CHECK(state.velocity.x > 0.9f);
    CHECK(state.position.x > 0.4f);
}

TEST_CASE("Unified integrator selector dispatches correctly") {
    integrator_config_t cfg = {
        .type = INTEGRATOR_EULER,
        .time_step = 1.0f
    };
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {1, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, 0, 0};

    numeq_integrate(&state, &accel, &cfg);

    CHECK(state.position.x == doctest::Approx(1.0f));
}
