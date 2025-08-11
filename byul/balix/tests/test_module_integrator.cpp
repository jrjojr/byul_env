#include "doctest.h"

extern "C" {
#include "numeq_integrator.h"
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

    integrator_step_euler(&state, dt);

    CHECK(state.linear.velocity.x == doctest::Approx(1.0f));
    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Semi-Implicit Euler: acceleration applied first") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {0, 0, 0},
            .acceleration = {2, 0, 0}
        }
    };
    float dt = 0.5f;

    integrator_step_semi_implicit(&state, dt);

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

    integrator_step_verlet(&state, &prev_state, dt);

    CHECK(state.linear.position.x == doctest::Approx(2.0f));
}

TEST_CASE("RK4 Integration: acceleration effect (simple)") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {0, 0, 0},
            .acceleration = {1.0f, 0.0f, 0.0f}
        }
    };

    float dt = 1.0f;
    integrator_step_motion_rk4(&state, dt);

    CHECK(state.linear.velocity.x > 0.9f);
    CHECK(state.linear.position.x > 0.4f);
}


TEST_CASE("Unified integrator selector dispatches correctly") {
    integrator_t intgr = {};

    motion_state_t state = {};
    motion_state_init(&state);
    state.linear.velocity = {1.0f, 0.0f, 0.0f};

    integrator_init_full(&intgr, INTEGRATOR_EULER, &state, nullptr, nullptr, nullptr);


    integrator_step(&intgr, 1.0f);
    state = intgr.state;

    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Unified integrator selector dispatches correctly v1") {
    integrator_t intgr;

    motion_state_t state = {};
    state.linear.velocity = {1.0f, 0.0f, 0.0f};

    integrator_init(&intgr);
    motion_state_assign(&intgr.state, &state);

    integrator_step(&intgr, 1.0f);
    state = intgr.state;

    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Unified integrator selector dispatches correctly v2") {
    integrator_t intgr;

    motion_state_t state;
    motion_state_init(&state);     // position=(0,0,0), velocity=(0,0,0)
    state.linear.velocity = {1.0f, 0.0f, 0.0f};    

    integrator_init(&intgr);  // default config (RK4)
    motion_state_assign(&intgr.state, &state);

    integrator_step(&intgr, 1.0f);
    state = intgr.state;

    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}
