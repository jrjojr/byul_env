#include "doctest.h"

extern "C" {
#include "internal/numeq_integrator.h"
}

TEST_CASE("Euler Integration: basic update") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {1, 0, 0},
            .acceleration = {0, 0, 0}
        }
    };
    vec3_t accel = {0, 0, 0};
    float dt = 1.0f;

    numeq_integrate_euler(&state, &accel, dt);

    CHECK(state.linear.velocity.x == doctest::Approx(1.0f));
    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Semi-Implicit Euler: acceleration applied first") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {0, 0, 0},
            .acceleration = {0, 0, 0}
        }
    };
    vec3_t accel = {2, 0, 0};
    float dt = 0.5f;

    numeq_integrate_semi_implicit(&state, &accel, dt);

    CHECK(state.linear.velocity.x == doctest::Approx(1.0f));
    CHECK(state.linear.position.x == doctest::Approx(0.5f));
}

TEST_CASE("Verlet Integration: past position affects update") {
    motion_state_t state = {
        .linear = {
            .position = {1.0f, 0.0f, 0.0f},
            .velocity = {0.0f, 0.0f, 0.0f},
            .acceleration = {0.0f, 0.0f, 0.0f}
        }
    };
    motion_state_t prev_state = state;
    prev_state.linear.position = {0.0f, 0.0f, 0.0f};
    vec3_t accel = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f;

    numeq_integrate_verlet(&state, &prev_state, &accel, dt);

    CHECK(state.linear.position.x == doctest::Approx(2.0f));
}


TEST_CASE("RK4 Integration: acceleration effect (simple)") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {0, 0, 0},
            .acceleration = {0, 0, 0}
        }
    };
    vec3_t accel = {1.0f, 0.0f, 0.0f};
    float dt = 1.0f;

    numeq_integrate_rk4(&state, &accel, dt);

    CHECK(state.linear.velocity.x > 0.9f);
    CHECK(state.linear.position.x > 0.4f);
}

TEST_CASE("Unified integrator selector dispatches correctly") {
    integrator_config_t cfg = {
        .type = INTEGRATOR_EULER,
        .time_step = 1.0f,
        .linear_accel = {0, 0, 0},
        .angular_accel = {0, 0, 0},
        .prev_state = nullptr
    };
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {1, 0, 0},
            .acceleration = {0, 0, 0}
        }
    };

    numeq_integrate(&state, &cfg);

    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}
