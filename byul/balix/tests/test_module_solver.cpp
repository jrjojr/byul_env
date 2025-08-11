#include "doctest.h"
#include <math.h>

extern "C" {
    #include "numeq_solver.h"
}

// --------------------- TEST CASES -----------------------

TEST_CASE("Quadratic solver returns correct real roots") {
    float x1, x2;
      // x^2 -3x + 2 = 0 -> x=1,2
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
