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
