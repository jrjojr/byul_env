#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "numal.h"
}

TEST_CASE("dualnumber basic operation test") {
    // Example explanation:
    // dualnumber is a core tool for automatic differentiation.
    // It has the form a + b*e, where e^2 = 0, representing an infinitesimal.
    // When a function is evaluated with a dualnumber, its derivative is automatically computed.
    // Example: f(x + e) = f(x) + f'(x)*e -> f'(x) is the coefficient of e.

    dualnumber_t a;
    dualnumber_init_full(&a, 3.0f, 2.0f);

    dualnumber_t b;
    dualnumber_init_full(&b, 1.0f, 4.0f);

    dualnumber_t add;
    dualnumber_add(&add, &a, &b);
    CHECK(add.re == doctest::Approx(4.0f));
    CHECK(add.du == doctest::Approx(6.0f));

    dualnumber_t mul;
    dualnumber_mul(&mul, &a, &b);
    CHECK(mul.re == doctest::Approx(3.0f));
    CHECK(mul.du == doctest::Approx(14.0f));

    // Auto differentiation example: f(x) = x^3 -> f'(x) = 3 * x^2
    float x = 2.0f;
    dualnumber_t x_dn;
    dualnumber_init_full(&x_dn, x, 1.0f); // e coefficient 1 -> for automatic differentiation
    
    dualnumber_t f;
    dualnumber_powf(&f, &x_dn, 3.0f);

    CHECK(f.re == doctest::Approx(8.0f));    // 2^3 = 8
    CHECK(f.du == doctest::Approx(12.0f));   // 3 * 2^2 = 12 -> f'(x) = 12
}
