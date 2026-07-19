#include "doctest.h"
#include <cmath>
#include <cstdint>

#include "coord.h"
#include "internal/coord_ops.hpp"
#include "scalar.h"

namespace {
constexpr double kPi = 3.14159265358979323846;

int legacy_wrap_reference(std::int64_t value) {
    constexpr std::int64_t range =
        static_cast<std::int64_t>(COORD_MAX) - COORD_MIN + 1;
    std::int64_t offset = value - COORD_MIN;
    offset = ((offset % range) + range) % range;
    return static_cast<int>(COORD_MIN + offset);
}
}

TEST_CASE("coord ABI layout diagnostics match the compiled value type") {
    CHECK(coord_sizeof() == sizeof(coord_t));
    CHECK(coord_alignof() == alignof(coord_t));
    CHECK(coord_offsetof_x() == offsetof(coord_t, x));
    CHECK(coord_offsetof_y() == offsetof(coord_t, y));
}

TEST_CASE("Wrap-Around Test") {
    coord_t a;
    coord_init_full(&a, COORD_MAX, 0);

    coord_t b;
    coord_init_full(&b, 1, 0);

    coord_t result;
    coord_add(&result, &a, &b);

    CHECK(result.x == COORD_MIN);
    CHECK(result.y == 0);
}

TEST_CASE("Legacy coord wrapping matches an int64 reference") {
    constexpr int range = COORD_MAX - COORD_MIN + 1;
    const int inputs[] = {
        COORD_MIN - range,
        COORD_MIN - 1,
        COORD_MIN,
        -1,
        0,
        1,
        COORD_MAX,
        COORD_MAX + 1,
        COORD_MAX + range,
    };

    for (const int input : inputs) {
        CAPTURE(input);
        coord_t coord;
        coord_init_full(&coord, input, -input);
        CHECK(coord.x == legacy_wrap_reference(input));
        CHECK(coord.y == legacy_wrap_reference(-static_cast<std::int64_t>(input)));
    }

    coord_t value{COORD_MAX, COORD_MIN};
    const coord_t delta{1, -1};
    coord_t result{};

    coord_add(&result, &value, &delta);
    CHECK(result.x == legacy_wrap_reference(
        static_cast<std::int64_t>(value.x) + delta.x));
    CHECK(result.y == legacy_wrap_reference(
        static_cast<std::int64_t>(value.y) + delta.y));

    coord_sub(&result, &value, &delta);
    CHECK(result.x == legacy_wrap_reference(
        static_cast<std::int64_t>(value.x) - delta.x));
    CHECK(result.y == legacy_wrap_reference(
        static_cast<std::int64_t>(value.y) - delta.y));

    value = coord_t{150000000, -150000000};
    coord_mul(&result, &value, 2);
    CHECK(result.x == legacy_wrap_reference(
        static_cast<std::int64_t>(value.x) * 2));
    CHECK(result.y == legacy_wrap_reference(
        static_cast<std::int64_t>(value.y) * 2));
}

TEST_CASE("Legacy coord operations preserve established aliasing behavior") {
    coord_t value{COORD_MAX, COORD_MIN};
    coord_t other{1, -1};

    coord_add(&value, &value, &other);
    CHECK(value.x == COORD_MIN);
    CHECK(value.y == COORD_MAX);

    coord_sub(&other, &value, &other);
    CHECK(other.x == COORD_MAX);
    CHECK(other.y == COORD_MIN);

    coord_mul(&value, &value, 1);
    CHECK(value.x == COORD_MIN);
    CHECK(value.y == COORD_MAX);

    const coord_t before_divide = value;
    coord_div(&value, &value, 0);
    CHECK(value.x == before_divide.x);
    CHECK(value.y == before_divide.y);
}

TEST_CASE("Distance and Angle Test") {
    coord_t origin;
    coord_init(&origin);

    coord_t east;
    coord_init_full(&east, 1, 0);

    coord_t north;
    coord_init_full(&north, 0, 1);

    coord_t west;
    coord_init_full(&west, -1, 0);

    coord_t south;
    coord_init_full(&south, 0, -1);

     CHECK(doctest::Approx(coord_distance(&origin, &east)).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(coord_distance(&origin, &north)).epsilon(1e-6) == 1.0f);

     CHECK(doctest::Approx(coord_degree(&origin, &east)).epsilon(1e-6) == 0.0);
    CHECK(doctest::Approx(coord_degree(&origin, &north)).epsilon(1e-6) == 90.0);
    CHECK(doctest::Approx(coord_degree(&origin, &west)).epsilon(1e-6) == 180.0);
    CHECK(doctest::Approx(coord_degree(&origin, &south)).epsilon(1e-6) == 270.0);
}

TEST_CASE("Add/Sub Test") {
    coord_t a;
    coord_init_full(&a, 5, 10);

    coord_t b;
    coord_init_full(&b, 3, -4);

    coord_t result;

    coord_add(&result, &a, &b);
    CHECK(result.x == 8);
    CHECK(result.y == 6);

    coord_sub(&result, &a, &b);
    CHECK(result.x == 2);
    CHECK(result.y == 14);
}

TEST_CASE("Next To Goal Test") {
    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal;
    coord_init_full(&goal, 3, 4);

    coord_t next;
    coord_next_to_goal(&next, &start, &goal);

    CHECK(next.x == 1);
    CHECK(next.y == 1);
}

TEST_CASE("Legacy direction baselines cover identical points and aliased start") {
    coord_t start{0, 0};
    const coord_t goal{3, -4};

    CHECK(coord_angle(&start, &start) == 0.0);
    CHECK(coord_degree(&start, &start) == 0.0);

    coord_next_to_goal(&start, &start, &goal);
    CHECK(start.x == 1);
    CHECK(start.y == -1);
}


TEST_CASE("Angle in Radian Test") {
    coord_t origin;
    coord_init(&origin);

    coord_t east, north, west, south;
    coord_init_full(&east, 1, 0);
    coord_init_full(&north, 0, 1);
    coord_init_full(&west, -1, 0);
    coord_init_full(&south, 0, -1);

     CHECK(doctest::Approx(coord_angle(&origin, &east)).epsilon(SCALAR_EPSILON) == 0.0);
    CHECK(doctest::Approx(coord_angle(&origin, &north)).epsilon(SCALAR_EPSILON) == kPi / 2.0);
    CHECK(doctest::Approx(coord_angle(&origin, &west)).epsilon(SCALAR_EPSILON) == kPi);
    CHECK(doctest::Approx(coord_angle(&origin, &south)).epsilon(SCALAR_EPSILON) == 3.0 * kPi / 2.0);
}

TEST_CASE("Internal coord value functors form consistent container contracts") {
    using byul::navsys::detail::coord_equal;
    using byul::navsys::detail::coord_hash;
    using byul::navsys::detail::coord_less;

    const coord_equal equal;
    const coord_hash hash;
    const coord_less less;

    const coord_t a{1, 2};
    const coord_t a_copy{1, 2};
    const coord_t a_copy2{1, 2};
    const coord_t b{1, 3};
    const coord_t c{2, 0};

    CHECK(equal(a, a));
    CHECK(equal(a, a_copy));
    CHECK(equal(a_copy, a_copy2));
    CHECK(equal(a, a_copy2));
    CHECK_FALSE(equal(a, b));
    CHECK(hash(a) == hash(a_copy));
    CHECK(hash(a_copy) == hash(a_copy2));

    CHECK_FALSE(less(a, a));
    CHECK_FALSE(less(a, a_copy));
    CHECK_FALSE(less(a_copy, a));
    CHECK(less(a, b));
    CHECK(less(b, c));
    CHECK(less(a, c));
    CHECK_FALSE(less(b, a));
}
