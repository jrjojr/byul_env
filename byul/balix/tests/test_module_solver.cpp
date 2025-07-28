#include "doctest.h"
#include <math.h>

extern "C" {
    #include "numeq_solver.h"
}

// --------------------- TEST CASES -----------------------

TEST_CASE("Quadratic solver returns correct real roots") {
    float x1, x2;
      // x^2 -3x + 2 = 0 → x=1,2
    bool ok = numeq_solve_quadratic(1.0f, -3.0f, 2.0f, &x1, &x2);
    CHECK(ok);
    CHECK((x1 == doctest::Approx(1.0f) || x2 == doctest::Approx(1.0f)));
    CHECK((x1 == doctest::Approx(2.0f) || x2 == doctest::Approx(2.0f)));
}

TEST_CASE("Bisection finds root of sin(x) near pi") {
    auto sin_func = [](float x, void*) -> float {
        return sinf(x);
    };
    float root;
    bool ok = numeq_solve_bisection(sin_func, nullptr, 3.0f, 3.5f, 1e-5f, &root);
    CHECK(ok);
    CHECK(root == doctest::Approx(3.14159f).epsilon(0.001f));
}

TEST_CASE("Apex solver computes correct peak") {
    linear_state_t state;
	linear_state_init(&state);
    state.velocity = { 2, 10, 0 };
        state.acceleration = { 0, -9.8f, 0 };

    vec3_t apex;
    float t_apex;
    bool ok = numeq_solve_apex(&state, &apex, &t_apex);
    CHECK(ok);
    CHECK(t_apex == doctest::Approx(10.0f / 9.8f).epsilon(0.01));
    CHECK(apex.y > 5.0f);
}

TEST_CASE("Solve velocity for flat range") {
    float v;
    bool ok = numeq_solve_velocity_for_range(100.0f, 9.8f, &v);
    CHECK(ok);
    CHECK(v > 0.0f);
}

TEST_CASE("Solve time to reach specific Y position") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 10, 0},
        .acceleration = {0, -9.8f, 0}
    };
    float t;
    bool ok = numeq_solve_time_for_y(&state, 5.0f, &t);
    CHECK(ok);
    CHECK(t > 0.0f);
    CHECK(t == doctest::Approx(0.76f).epsilon(0.1));
}

TEST_CASE("Solve time to reach target XZ position") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {10, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t target = {50.0f, 0, 0};
    float t;
    bool ok = numeq_solve_time_for_position(&state, &target, 0.01f, 10.0f, &t);
    CHECK(ok);
    CHECK(t == doctest::Approx(5.0f).epsilon(0.01));
}

TEST_CASE("Solve time when projectile stops (horizontal motion)") {
    linear_state_t state = {
        .position = {0, 0, 0},
        .velocity = {0.5f, 0.0f, 0.0f},
        .acceleration = {-0.1f, 0.0f, 0.0f}
    };
    float t;
    bool ok = numeq_solve_stop_time(&state, 0.01f, &t);
    CHECK(ok);
    CHECK(t > 4.0f);  // v = 0.5, a = -0.1 → t ≈ 5
}

TEST_CASE("Solve vec3 function to approach target") {
    auto moving_func = [](float t, vec3_t* out, void*) {
        out->x = t;
        out->y = t * t;
        out->z = 0.0f;
    };
    vec3_t target = {2.0f, 4.0f, 0.0f};  // y = x^2 → 최소점 t = 2
    float t_min;
    bool ok = numeq_solve_time_for_vec3(
        moving_func, nullptr, &target, 0.0f, 5.0f, 1e-4f, &t_min);
    CHECK(ok);
    CHECK(t_min == doctest::Approx(2.0f).epsilon(0.01f));
}

