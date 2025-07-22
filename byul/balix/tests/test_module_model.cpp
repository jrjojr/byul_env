#include "doctest.h"

extern "C" {
    #include "internal/numeq_model.h"
    #include "internal/common.h"
}

// 환경: 중력만, 바람 없음
static environ_t test_env = {
    .gravity = {0.0f, -9.8f, 0.0f},
    .wind = {0.0f, 0.0f, 0.0f},
    .air_density = 0.0f,
    .humidity = 0.0f,
    .temperature = 20.0f,
    .pressure = 101325.0f
};

// 물체: 질량 1kg, 드래그 제거
static bodyprops_t test_body_no_drag = {
    .mass = 1.0f,
    .drag_coef = 0.0f,
    .cross_section = 0.0f,
    .restitution = 1.0f,
    .friction = 0.0f
};

TEST_CASE("Model: a(t) under gravity only") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {10, 10, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t a;
    numeq_model_accel_at(0.0f, &state, &test_env, &test_body_no_drag, &a);
    CHECK(a.x == doctest::Approx(0.0f));
    CHECK(a.y == doctest::Approx(-9.8f));
}

TEST_CASE("Model: v(t) includes acceleration (gravity)") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {1.0f, 0.0f, 0.0f},
        .acceleration = {0, -9.8f, 0}
    };
    vec3_t v;
    numeq_model_vel_at(1.0f, &state, &test_env, &test_body_no_drag, &v);
    CHECK(v.x == doctest::Approx(1.0f));
    CHECK(v.y == doctest::Approx(-9.8f));
}

TEST_CASE("Model: p(t) includes velocity and gravity") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 10, 0},
        .acceleration = {0, -9.8f, 0}
    };
    vec3_t p;
    numeq_model_pos_at(1.0f, &state, &test_env, &test_body_no_drag, &p);
    CHECK(p.y == doctest::Approx(10.0f - 0.5f * 9.8f));
}

TEST_CASE("Model: is_apex true near zero vy") {
    // linear_state_t state = { .velocity = {0.0f, FLOAT_EPSILON-0.000001, 0.0f} };
    linear_state_t state = { .velocity = {0.0f, FLOAT_EPSILON, 0.0f} };
    vec3_print(&state.velocity);
    CHECK(numeq_model_is_apex(&state));
}

TEST_CASE("Model: is_grounded below or at ground level") {
    linear_state_t s1 = { .position = {0.0f, 0.01f, 0.0f} };
    linear_state_t s2 = { .position = {0.0f, -0.01f, 0.0f} };
    CHECK(numeq_model_is_grounded(&s1, 0.0f) == false);
    CHECK(numeq_model_is_grounded(&s2, 0.0f) == true);
}

TEST_CASE("Model: default bounce reflects velocity") {
    vec3_t vin = {5.0f, -3.0f, 0.0f};
    vec3_t normal = {0.0f, 1.0f, 0.0f};
    vec3_t vout;

    bool ok = numeq_model_default_bounce(&vin, &normal, 0.8f, &vout);

    CHECK(ok == true);
    CHECK(vout.y == doctest::Approx(2.4f));  // 반사: -vy * 0.8
}

TEST_CASE("Model: predict vs predict_rk4 under gravity") {
    linear_state_t state0;
    //  = {
    //     .position = {0, 0, 0},
    //     .velocity = {10, 10, 0},
    //     .acceleration = {0, 0, 0} // 초기 가속도는 사용하지 않음
    // };
    linear_state_init(&state0);
    state0.velocity = {10, 10, 0};

    // --- 1초 후 상태 예측 (기본 공식) ---
    linear_state_t out_basic;
    numeq_model_predict(1.0f, &state0, &test_env, &test_body_no_drag, &out_basic);

    // --- 1초 후 상태 예측 (RK4 적분기 기반, 60스텝 = 60Hz) ---
    linear_state_t out_rk4;
    numeq_model_predict_rk4(1.0f, &state0, &test_env, &test_body_no_drag, 60, &out_rk4);

    // --- 비교: 중력만 있는 경우 두 결과가 거의 동일해야 함 ---
    CHECK(out_basic.position.x == doctest::Approx(out_rk4.position.x).epsilon(1e-4));
    CHECK(out_basic.position.y == doctest::Approx(out_rk4.position.y).epsilon(1e-4));
    CHECK(out_basic.velocity.y == doctest::Approx(out_rk4.velocity.y).epsilon(1e-4));
}

TEST_CASE("Model: predict_rk4 convergence test") {
    linear_state_t state0 = {
        .position = {0, 0, 0},
        .velocity = {5, 5, 0},
        .acceleration = {0, 0, 0}
    };

    // RK4 with 10 steps
    linear_state_t out_rk4_10;
    numeq_model_predict_rk4(1.0f, &state0, &test_env, &test_body_no_drag, 10, &out_rk4_10);

    // RK4 with 100 steps (should be more precise)
    linear_state_t out_rk4_100;
    numeq_model_predict_rk4(1.0f, &state0, &test_env, &test_body_no_drag, 100, &out_rk4_100);

    // 두 결과가 큰 차이를 보이지 않아야 함 (convergence)
    CHECK(out_rk4_10.position.y == doctest::Approx(out_rk4_100.position.y).epsilon(1e-3));
}

// --- 테스트용 환경 및 물체 속성 ---
static environ_t test_env_no_drag = {
    .gravity = {0.0f, 0.0f, 0.0f},
    .wind = {0.0f, 0.0f, 0.0f},
    .air_density = 0.0f,
    .humidity = 0.0f,
    .temperature = 20.0f,
    .pressure = 101325.0f
};

static bodyprops_t test_body_unit = {
    .mass = 1.0f,
    .drag_coef = 0.0f,
    .cross_section = 0.0f,
    .restitution = 1.0f,
    .friction = 0.0f
};

TEST_CASE("Model: collision prediction between two moving objects") {
    // --- 1. 초기 상태 설정 ---
    linear_state_t my_state = {
        .position = {0, 0, 0},
        .velocity = {1, 0, 0},   // 오른쪽으로 1 m/s
        .acceleration = {0, 0, 0}
    };

    linear_state_t other_state = {
        .position = {5, 0, 0},
        .velocity = {-1, 0, 0},  // 왼쪽으로 1 m/s
        .acceleration = {0, 0, 0}
    };

    float radius_sum = 0.5f;   // 두 개체의 합 반경
    float max_time = 10.0f;    // 10초까지 예측
    float time_step = 0.1f;    // 0.1초 간격

    float collision_time;
    vec3_t collision_point;

    // --- 2. 충돌 예측 ---
    bool hit = numeq_model_predict_collision(
        &my_state, &other_state,
        &test_env_no_drag, &test_body_unit, &test_body_unit,
        radius_sum, max_time, time_step,
        &collision_time, &collision_point
    );

    // --- 3. 검증 ---
    CHECK(hit == true);
    // 이론상 두 개체는 (5 / (1+1)) = 2.5초 후에 충돌해야 함
    CHECK(collision_time == doctest::Approx(2.5f).epsilon(0.1f));

    // 충돌 지점은 중간점 (2.5, 0, 0)에 가까워야 함
    CHECK(collision_point.x == doctest::Approx(2.5f).epsilon(0.1f));
    CHECK(collision_point.y == doctest::Approx(0.0f));
    CHECK(collision_point.z == doctest::Approx(0.0f));
}

TEST_CASE("Model: no collision when objects diverge") {
    linear_state_t my_state = {
        .position = {0, 0, 0},
        .velocity = {1, 0, 0},
        .acceleration = {0, 0, 0}
    };

    linear_state_t other_state = {
        .position = {5, 0, 0},
        .velocity = {1, 0, 0},   // 같은 방향으로 1 m/s
        .acceleration = {0, 0, 0}
    };

    float collision_time;
    vec3_t collision_point;

    bool hit = numeq_model_predict_collision(
        &my_state, &other_state,
        &test_env_no_drag, &test_body_unit, &test_body_unit,
        0.5f, 10.0f, 0.1f,
        &collision_time, &collision_point
    );

    CHECK(hit == false);
    CHECK(collision_time == doctest::Approx(-1.0f));
}