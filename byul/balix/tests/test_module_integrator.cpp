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

    numeq_integrate_euler(&state, dt);

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

    numeq_integrate_semi_implicit(&state, dt);

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

    numeq_integrate_verlet(&state, &prev_state, dt);

    CHECK(state.linear.position.x == doctest::Approx(2.0f));
}

TEST_CASE("RK4 Integration: acceleration effect (simple)") {
    motion_state_t state = {
        .linear = {
            .position = {0, 0, 0},
            .velocity = {0, 0, 0},
            .acceleration = {1.0f, 0.0f, 0.0f}  // 가속도 직접 설정
        }
    };

    float dt = 1.0f;
    numeq_integrate_motion_rk4(&state, dt);

    CHECK(state.linear.velocity.x > 0.9f);  // 예상: ~1.0
    CHECK(state.linear.position.x > 0.4f);  // 예상: ~0.5
}


TEST_CASE("Unified integrator selector dispatches correctly") {
    integrator_config_t cfg = {
        .type = INTEGRATOR_EULER,
        .time_step = 1.0f,
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

TEST_CASE("Unified integrator selector dispatches correctly v1") {
    integrator_config_t cfg;
    integrator_config_init_full(
        &cfg,
        INTEGRATOR_EULER,  // 오일러 방식
        1.0f,              // dt = 1초
        nullptr,           // prev_state 없음
        nullptr            // userdata 없음
    );

    motion_state_t state;
    state.linear.position = {0.0f, 0.0f, 0.0f};
    state.linear.velocity = {1.0f, 0.0f, 0.0f};
    state.linear.acceleration = {0.0f, 0.0f, 0.0f};

    numeq_integrate(&state, &cfg);

    CHECK(state.linear.position.x == doctest::Approx(1.0f));
}

TEST_CASE("Unified integrator selector dispatches correctly v2") {
    integrator_config_t cfg;
    integrator_config_init(&cfg);  // 기본 설정 (예: RK4, dt=0.016f)

    motion_state_t state;
    motion_state_init(&state);     // position=(0,0,0), velocity=(0,0,0)
    state.linear.velocity = {1.0f, 0.0f, 0.0f};  // 초기 속도를 직접 지정

    numeq_integrate(&state, &cfg);

    // 기본 dt (0.016f)를 적용하면 position.x ≈ 0.016f
    CHECK(state.linear.position.x == doctest::Approx(cfg.time_step));
}
