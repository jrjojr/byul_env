#include "doctest.h"

extern "C" {
    #include "float_common.h"
}

TEST_CASE("float_equal: similar and dissimilar values") {
    CHECK(float_equal(1.000001f, 1.000002f));
    CHECK(float_equal(1.000001f, 1.000003f));
    CHECK(float_equal(1.000001f, 1.000009f));

    CHECK(float_equal(1.00001f, 1.000019f));
    CHECK(float_equal(1.00001f, 1.000001f));

    CHECK_FALSE(float_equal(1.00001f, 1.000020f));
    CHECK_FALSE(float_equal(1.00001f, 1.000000f));

    CHECK_FALSE(float_equal(1.0f, 1.1f));
}

TEST_CASE("float_compare: ordering (value-based)") {
    CHECK(float_compare(1.0f, 2.0f, nullptr) < 0);
    CHECK(float_compare(2.0f, 1.0f, nullptr) > 0);
    CHECK(float_compare(1.0f, 1.0f, nullptr) == 0);
}

TEST_CASE("int_compare: ordering (value-based)") {
    CHECK(int_compare(3, 7, nullptr) < 0);
    CHECK(int_compare(10, 5, nullptr) > 0);
    CHECK(int_compare(42, 42, nullptr) == 0);
}

TEST_CASE("float_equal tests") {
    CHECK(float_equal(1.00001f, 1.000019f));
    CHECK(float_equal(1.00001f, 1.000001f));
    CHECK_FALSE(float_equal(1.00001f, 1.000020f));
    CHECK_FALSE(float_equal(1.00001f, 1.000000f));
}

TEST_CASE("float_zero tests") {
    CHECK(float_zero(1e-9f));
    CHECK_FALSE(float_zero(1e-4f));
}

TEST_CASE("float_safe_div tests") {
    CHECK(float_safe_div(10.0f, 2.0f, -1.0f) == doctest::Approx(5.0f));
    CHECK(float_safe_div(10.0f, 0.0f, -1.0f) == doctest::Approx(-1.0f));
}

TEST_CASE("square, clamp, sign") {
    CHECK(square(3.0f) == doctest::Approx(9.0f));
    CHECK(clamp(5.0f, 1.0f, 4.0f) == doctest::Approx(4.0f));
    CHECK(clamp(-2.0f, -1.0f, 1.0f) == doctest::Approx(-1.0f));
    CHECK(sign(3.0f) == doctest::Approx(1.0f));
    CHECK(sign(-3.0f) == doctest::Approx(-1.0f));
    CHECK(sign(0.0f) == doctest::Approx(0.0f));
}

TEST_CASE("deg2rad, rad2deg") {
    CHECK(deg2rad(180.0f) == doctest::Approx(3.1415926f).epsilon(0.001f));
    CHECK(rad2deg(3.1415926f) == doctest::Approx(180.0f).epsilon(0.001f));
}

TEST_CASE("lerp and inverse lerp") {
    CHECK(lerp(0.0f, 10.0f, 0.5f) == doctest::Approx(5.0f));
    CHECK(inv_lerp(0.0f, 10.0f, 5.0f) == doctest::Approx(0.5f));
}

TEST_CASE("remap and clamp01") {
    CHECK(remap(0.0f, 10.0f, 100.0f, 200.0f, 5.0f) == doctest::Approx(150.0f));
    CHECK(clamp01(1.2f) == doctest::Approx(1.0f));
    CHECK(clamp01(-0.2f) == doctest::Approx(0.0f));
    CHECK(clamp01(0.5f) == doctest::Approx(0.5f));
}

TEST_CASE("smoothstep") {
    CHECK(smoothstep(0.0f, 1.0f, 0.0f) == doctest::Approx(0.0f));
    CHECK(smoothstep(0.0f, 1.0f, 0.5f) == doctest::Approx(0.5f).epsilon(0.1f));
    CHECK(smoothstep(0.0f, 1.0f, 1.0f) == doctest::Approx(1.0f));
}

TEST_CASE("float_equal_tol: 공차 기반 비교") {
    CHECK(float_equal_tol(1.0f, 1.00001f, 1e-4f) == true);

    CHECK(float_equal_tol(1.0f, 0.99991f, 1e-4f) == true);

    CHECK(float_equal_tol(1.0f, 1.0002f, 1e-4f) == false);
    CHECK(float_equal_tol(1.0f, 0.9998f, 1e-4f) == false);

    // tol이 음수일 경우 자동 보정
    CHECK(float_equal_tol(1.0f, 1.00005f, -1e-4f) == true);
}

TEST_CASE("float_equal_tol_all: 양/음수 방향 공차 개별 비교") {
    float a = 1.0f;

    // tol_pos = 0.002, tol_neg = 0.001
    CHECK(float_equal_tol_all(a, 1.002f, 0.002f, 0.001f) == true);  // 양수 방향 허용
    CHECK(float_equal_tol_all(a, 1.003f, 0.002f, 0.001f) == false); // 양수 방향 초과
    CHECK(float_equal_tol_all(a, 0.999f, 0.002f, 0.001f) == true);  // 음수 방향 허용
    CHECK(float_equal_tol_all(a, 0.998f, 0.002f, 0.001f) == false); // 음수 방향 초과

    // tol_pos / tol_neg에 음수가 들어올 경우 자동 보정
    CHECK(float_equal_tol_all(a, 1.001f, -0.002f, -0.001f) == true);
}

TEST_CASE("float_equal_tol_all: 경계 조건") {
    float a = 1.0f;

    // 경계값에서 true 반환
    CHECK(float_equal_tol_all(a, 1.002f, 0.002f, 0.001f) == true);  // 양수 경계
    CHECK(float_equal_tol_all(a, 0.999f, 0.002f, 0.001f) == true);  // 음수 경계

    // 경계 초과 시 false
    CHECK(float_equal_tol_all(a, 1.0021f, 0.002f, 0.001f) == false);
    CHECK(float_equal_tol_all(a, 0.9989f, 0.002f, 0.001f) == false);
}