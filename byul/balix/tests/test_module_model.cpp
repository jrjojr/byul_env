#include "doctest.h"

extern "C" {
    #include "numeq_model.h"
    #include "scalar.h"
}

// Environment: gravity only, no wind
static environ_t test_env;

// Body: mass 1 kg, drag removed
static bodyprops_t test_body_no_drag;

TEST_CASE("Model: a(t) under gravity only") {
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = { 10, 10, 0 };

    environ_init(&test_env);
    bodyprops_init(&test_body_no_drag);

    vec3_t a;
    numeq_model_accel_predict(0.0f, &state, &test_env, &test_body_no_drag, &a);
    CHECK(a.x == doctest::Approx(0.0f).epsilon(0.5f));
    CHECK(a.y == doctest::Approx(-9.8f).epsilon(0.5f));
}

TEST_CASE("Model: v(t) includes acceleration (gravity)") {
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = { 1.0f, 0.0f, 0.0f };
    
    vec3_t v;
    numeq_model_vel_predict(1.0f, &state, &test_env, &test_body_no_drag, &v);
    CHECK(v.x == doctest::Approx(1.0f).epsilon(0.5f));
    CHECK(v.y == doctest::Approx(-9.8f).epsilon(0.5f));
}

TEST_CASE("Model: p(t) includes velocity and gravity") {
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = { 0, 10, 0 };
    
    vec3_t p;
    numeq_model_pos_predict(1.0f, &state, &test_env, &test_body_no_drag, &p);
    CHECK(p.y == doctest::Approx(10.0f - 0.5f * 9.8f).epsilon(0.5f));
}

TEST_CASE("Model: default bounce reflects velocity") {
    vec3_t vin = {5.0f, -3.0f, 0.0f};
    vec3_t normal = {0.0f, 1.0f, 0.0f};
    vec3_t vout;

    bool ok = numeq_model_bounce(&vin, &normal, 0.8f, &vout);

    CHECK(ok == true);
    CHECK(vout.y == doctest::Approx(2.4f));  // reflection: -vy * 0.8
}

TEST_CASE("Model: predict vs predict_rk4 (no gravity, no drag)") {
    // Create zero-gravity environment
    environ_t env_no_gravity = test_env;
    env_no_gravity.gravity = {0, 0, 0};  // remove gravity

    bodyprops_t body_no_drag = test_body_no_drag;
    body_no_drag.drag_coef = 0.0f;       // remove drag

    linear_state_t state0;
    linear_state_init(&state0);
    state0.velocity = {10, 10, 0};       // initial velocity

    // --- Predict state after 1 second (basic formula) ---
    linear_state_t out_basic;
    numeq_model_predict(1.0f, &state0, &env_no_gravity, &body_no_drag, &out_basic);

    // --- Predict state after 1 second (RK4 integration, 60 steps = 60Hz) ---
    linear_state_t out_rk4;
    numeq_model_predict_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 60, &out_rk4);

    // --- Comparison: should be equal in no-gravity and no-drag case ---
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
    numeq_model_predict_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 10, &out_rk4_10);

    // RK4 with 100 steps (should be more precise)
    linear_state_t out_rk4_100;
    numeq_model_predict_rk4(1.0f, &state0, &env_no_gravity, &body_no_drag, 100, &out_rk4_100);

    // Difference between results should be small (convergence)
    CHECK(out_rk4_10.position.x == doctest::Approx(out_rk4_100.position.x).epsilon(1e-3));
    CHECK(out_rk4_10.position.y == doctest::Approx(out_rk4_100.position.y).epsilon(1e-3));
}


// --- Test environment and body properties ---
static environ_t test_env_no_drag;
static bodyprops_t test_body_unit;

TEST_CASE("Model: collision prediction between two moving objects") {
    environ_init(&test_env_no_drag);
    bodyprops_init(&test_body_unit);
    test_body_unit.restitution = 1.0f;
    // --- 1. Initial states ---
    linear_state_t my_state;
    linear_state_init(&my_state);
    my_state.velocity = { 1, 0, 0 };

    linear_state_t other_state;
    linear_state_init(&other_state);

    other_state.position = { 5, 0, 0 };
    other_state.velocity = { -1, 0, 0 };

    float radius_sum = 0.5f;   // sum of both radii
    float collision_time;
    vec3_t collision_point;

    // --- 2. Collision prediction (equation-based) ---
    bool hit = numeq_model_predict_collision(
        &my_state, &other_state,
        radius_sum,
        &collision_time, &collision_point
    );

    // --- 3. Validation ---
    CHECK(hit == true);
    // Theoretically, they collide at 2.5 s (distance 5m, relative speed 2 m/s)
    CHECK(collision_time == doctest::Approx(2.5f).epsilon(0.1f));

    // Collision point should be near (2.5, 0, 0)
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
    other_state.velocity = { 1, 0, 0 };   // moving in the same direction

    float collision_time;
    vec3_t collision_point;

    bool hit = numeq_model_predict_collision(
        &my_state, &other_state,
        0.5f,
        &collision_time, &collision_point
    );

    CHECK(hit == false);
    CHECK(collision_time == doctest::Approx(-1.0f));
}

TEST_CASE("Model: predict vs predict_rk4 under gravity (difference check)") {
    // Environment with gravity
    environ_t env_gravity = test_env;
    env_gravity.gravity = {0, -9.81f, 0};   // gravity acceleration

    bodyprops_t body_no_drag = test_body_no_drag;
    body_no_drag.drag_coef = 0.0f;          // remove drag (only gravity effect)

    // Initial state
    linear_state_t state0;
    linear_state_init(&state0);
    state0.velocity = {10, 10, 0};          // initial velocity

    // --- Predict after 1 second (basic constant acceleration formula) ---
    linear_state_t out_basic;
    numeq_model_predict(1.0f, &state0, &env_gravity, &body_no_drag, &out_basic);

    // --- Predict after 1 second (RK4 integration, 60 steps = 60Hz) ---
    linear_state_t out_rk4;
    numeq_model_predict_rk4(1.0f, &state0, &env_gravity, &body_no_drag, 60, &out_rk4);

    // --- Output comparison ---
    MESSAGE("Basic (p, v): (", out_basic.position.x, ", ", out_basic.position.y, "), (", 
            out_basic.velocity.x, ", ", out_basic.velocity.y, ")");
    MESSAGE("RK4   (p, v): (", out_rk4.position.x, ", ", out_rk4.position.y, "), (", 
            out_rk4.velocity.x, ", ", out_rk4.velocity.y, ")");

    // There should be differences (RK4 and basic formula may differ under gravity)
    CHECK(out_basic.position.y != doctest::Approx(out_rk4.position.y).epsilon(1e-4));
    CHECK(out_basic.velocity.y != doctest::Approx(out_rk4.velocity.y).epsilon(1e-4));
}
