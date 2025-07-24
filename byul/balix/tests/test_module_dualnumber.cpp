#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/numal.h"
}

TEST_CASE("dualnumber 기본 연산 테스트") {
    // MESSAGE("");
// MESSAGE(
//     "\n💡 dualnumber는 자동 미분의 핵심 도구입니다.\n"
//     "   형태는 a + bε 이며, ε^2 = 0 인 특수한 기호로, 무한소를 나타냅니다.\n"
//     "   함수에 dualnumber를 넣으면 도함수가 자동으로 계산됩니다.\n"
//     "   예: f(x + ε) = f(x) + f'(x)ε → f'(x)는 ε 항의 계수로 자동 추출됩니다.\n"
// );

    // dualnumber_t* a = dualnumber_init_full(3.0f, 2.0f);
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

    // 🎯 자동 미분 예제: f(x) = x^3 → f'(x) = 3x^2
    float x = 2.0f;
    dualnumber_t x_dn;
    dualnumber_init_full(&x_dn, x, 1.0f); // ε 계수 1 → 자동 미분용
    
    dualnumber_t f;
    dualnumber_powf(&f, &x_dn, 3.0f);

    CHECK(f.re == doctest::Approx(8.0f));    // 2^3 = 8
    CHECK(f.du == doctest::Approx(12.0f));   // 3*2^2 = 12 → f'(x) = 12
}
