#include "doctest.h"

extern "C" {
    #include "numeq_model.h"
    #include "float_common.h"
}

// 환경: 중력만, 바람 없음
static environ_t test_env;

// 물체: 질량 1kg, 드래그 제거
static bodyprops_t test_body_no_drag;

TEST_CASE("Model: a(t) under gravity only") {
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = { 10, 10, 0 };

    environ_init(&test_env);
    // test_env.gravity = vec3_t{0.0f,0.0f,0.0f};
    bodyprops_init(&test_body_no_drag);

    vec3_t a;
    numeq_model_accel_at(0.0f, &state, &test_env, &test_body_no_drag, &a);
    CHECK(a.x == doctest::Approx(0.0f).epsilon(0.5f));
    CHECK(a.y == doctest::Approx(-9.8f).epsilon(0.5f));
}

TEST_CASE("Model: v(t) includes acceleration (gravity)") {
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = { 1.0f, 0.0f, 0.0f };
    
    vec3_t v;
    numeq_model_vel_at(1.0f, &state, &test_env, &test_body_no_drag, &v);
    CHECK(v.x == doctest::Approx(1.0f).epsilon(0.5f));
    CHECK(v.y == doctest::Approx(-9.8f).epsilon(0.5f));
}

TEST_CASE("Model: p(t) includes velocity and gravity") {
    linear_state_t state;
	linear_state_init(&state);
    state.velocity = { 0, 10, 0 };
    
    vec3_t p;
    numeq_model_pos_at(1.0f, &state, &test_env, &test_body_no_drag, &p);
    CHECK(p.y == doctest::Approx(10.0f - 0.5f * 9.8f).epsilon(0.5f));
}

TEST_CASE("Model: default bounce reflects velocity") {
    vec3_t vin = {5.0f, -3.0f, 0.0f};
    vec3_t normal = {0.0f, 1.0f, 0.0f};
    vec3_t vout;

    bool ok = numeq_model_bounce(&vin, &normal, 0.8f, &vout);

    CHECK(ok == true);
    CHECK(vout.y == doctest::Approx(2.4f));  // 반사: -vy * 0.8
}

TEST_CASE("Model: predict vs predict_rk4 (no gravity, no drag)") {
    // 무중력 환경 생성
    environ_t env_no_gravity = test_env;
    env_no_gravity.gravity = {0, 0, 0};  // 중력 제거

    bodyprops_t body_no_drag = test_body_no_drag;
    body_no_drag.drag_coef = 0.0f;       // 항력 제거

    linear_state_t state0;
    linear_state_init(&state0);
    state0.velocity = {10, 10, 0};       // 초기 속도

    // --- 1초 후 상태 예측 (기본 공식) ---
    linear_state_t out_basic;
    numeq_model_calc(1.0f, &state0, &env_no_gravity, &body_no_drag, &out_basic);

    // --- 1초 후 상태 예측 (RK4 적분기 기반, 60스텝 = 60Hz) ---
    linear_state_t out_rk4;
    numeq_model_calc_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 60, &out_rk4);

    // --- 비교: 무중력 + 무항력 상황에서 두 결과가 같아야 함 ---
    CHECK(out_basic.position.x == doctest::Approx(out_rk4.position.x).epsilon(1e-4));
    CHECK(out_basic.position.y == doctest::Approx(out_rk4.position.y).epsilon(1e-4));
    CHECK(out_basic.velocity.y == doctest::Approx(out_rk4.velocity.y).epsilon(0.5f));
}

TEST_CASE("Model: predict_rk4 convergence test (no gravity)") {
    environ_t env_no_gravity = test_env;
    env_no_gravity.gravity = {0, 0, 0};

    bodyprops_t body_no_drag = test_body_no_drag;
    body_no_drag.drag_coef = 0.0f;

    linear_state_t state0;
    linear_state_init(&state0);
    state0.velocity = { 5, 5, 0 };

    // RK4 with 10 steps
    linear_state_t out_rk4_10;
    numeq_model_calc_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 10, &out_rk4_10);

    // RK4 with 100 steps (should be more precise)
    linear_state_t out_rk4_100;
    numeq_model_calc_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 100, &out_rk4_100);

    // 두 결과가 큰 차이를 보이지 않아야 함 (convergence)
    CHECK(out_rk4_10.position.x == doctest::Approx(out_rk4_100.position.x).epsilon(1e-3));
    CHECK(out_rk4_10.position.y == doctest::Approx(out_rk4_100.position.y).epsilon(1e-3));
}


// --- 테스트용 환경 및 물체 속성 ---
static environ_t test_env_no_drag;

static bodyprops_t test_body_unit;

TEST_CASE("Model: collision prediction between two moving objects") {
	environ_init(&test_env_no_drag);
	bodyprops_init(&test_body_unit);
    test_body_unit.restitution = 1.0f;
    // --- 1. 초기 상태 설정 ---
    linear_state_t my_state;
    linear_state_init(&my_state);
    my_state.velocity = { 1, 0, 0 };

    linear_state_t other_state;
    linear_state_init(&other_state);

    other_state.position = { 5, 0, 0 };
    other_state.velocity = { -1, 0, 0 };


    float radius_sum = 0.5f;   // 두 개체의 합 반경
    float collision_time;
    vec3_t collision_point;

    // --- 2. 충돌 예측 (방정식 기반) ---
    bool hit = numeq_model_calc_collision(
        &my_state, &other_state,
        radius_sum,
        &collision_time, &collision_point
    );

    // --- 3. 검증 ---
    CHECK(hit == true);
    // 이론상 두 개체는 2.5초 후 충돌 (거리 5m, 속도합 2m/s)
    CHECK(collision_time == doctest::Approx(2.5f).epsilon(0.1f));

    // 충돌 지점은 (2.5, 0, 0)에 가까워야 함
    CHECK(collision_point.x == doctest::Approx(2.5f).epsilon(0.1f));
    CHECK(collision_point.y == doctest::Approx(0.0f));
    CHECK(collision_point.z == doctest::Approx(0.0f));
}

TEST_CASE("Model: no collision when objects diverge") {
    linear_state_t my_state;
	linear_state_init(&my_state);

    my_state.velocity = { 1, 0, 0 };

    linear_state_t other_state;
    linear_state_init(&other_state);

    other_state.position = { 5, 0, 0 };
    other_state.velocity = { 1, 0, 0 };   // 같은 방향으로 1 m/s


    float collision_time;
    vec3_t collision_point;

    bool hit = numeq_model_calc_collision(
        &my_state, &other_state,
        0.5f,
        &collision_time, &collision_point
    );

    CHECK(hit == false);
    CHECK(collision_time == doctest::Approx(-1.0f));
}

TEST_CASE("Model: predict vs predict_rk4 under gravity (difference check)") {
    // 중력이 포함된 테스트 환경
    environ_t env_gravity = test_env;
    env_gravity.gravity = {0, -9.81f, 0};   // 중력 가속도 적용

    bodyprops_t body_no_drag = test_body_no_drag;
    body_no_drag.drag_coef = 0.0f;          // 항력 제거 (중력 효과만 확인)

    // 초기 상태
    linear_state_t state0;
    linear_state_init(&state0);
    state0.velocity = {10, 10, 0};          // 초기 속도

    // --- 1초 후 상태 예측 (기본 등가속도 공식) ---
    linear_state_t out_basic;
    numeq_model_calc(1.0f, &state0, &env_gravity, &body_no_drag, &out_basic);

    // --- 1초 후 상태 예측 (RK4 적분, 60스텝 = 60Hz) ---
    linear_state_t out_rk4;
    numeq_model_calc_rk4(1.0f, &state0, &env_gravity, &body_no_drag, 60, &out_rk4);

    // --- 비교 결과 출력 ---
    MESSAGE("Basic (p, v): (", out_basic.position.x, ", ", out_basic.position.y, "), (", 
            out_basic.velocity.x, ", ", out_basic.velocity.y, ")");
    MESSAGE("RK4   (p, v): (", out_rk4.position.x, ", ", out_rk4.position.y, "), (", 
            out_rk4.velocity.x, ", ", out_rk4.velocity.y, ")");

    // 차이가 존재하는지 확인 (중력 포함시 RK4와 basic의 결과는 같지 않을 수 있음)
    CHECK(out_basic.position.y != doctest::Approx(out_rk4.position.y).epsilon(1e-4));
    CHECK(out_basic.velocity.y != doctest::Approx(out_rk4.velocity.y).epsilon(1e-4));
}
