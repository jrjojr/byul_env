#include "doctest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>

#include "coord.h"
#include "internal/coord_ops.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kCoordTestEpsilon = 1e-6;

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

TEST_CASE("Checked coord construction validates range and preserves outputs") {
    coord_t value{71, 72};
    CHECK(coord_init_checked(&value, 3, -4) == NAVSYS_STATUS_OK);
    CHECK(value.x == 3);
    CHECK(value.y == -4);

    const coord_t preserved = value;
    CHECK(coord_init_checked(
        &value, BYUL_COORD_COMPONENT_MAX + INT32_C(1), 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == preserved.x);
    CHECK(value.y == preserved.y);
    CHECK(coord_init_checked(nullptr, 0, 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    coord_t* created = reinterpret_cast<coord_t*>(1);
    CHECK(coord_create_checked(
        BYUL_COORD_COMPONENT_MAX + INT32_C(1), 0, &created)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(created == reinterpret_cast<coord_t*>(1));
    CHECK(coord_create_checked(5, 6, &created) == NAVSYS_STATUS_OK);
    REQUIRE(created != nullptr);
    CHECK(created->x == 5);
    CHECK(created->y == 6);

    coord_t* copied = reinterpret_cast<coord_t*>(1);
    CHECK(coord_copy_checked(created, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(copied != created);
    CHECK(copied->x == created->x);
    CHECK(copied->y == created->y);
    copied->x = 9;
    CHECK(created->x == 5);

    coord_t* preserved_pointer = reinterpret_cast<coord_t*>(1);
    CHECK(coord_copy_checked(nullptr, &preserved_pointer)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved_pointer == reinterpret_cast<coord_t*>(1));
    CHECK(coord_create_checked(0, 0, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(coord_copy_checked(created, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    coord_destroy(copied);
    coord_destroy(created);
}

TEST_CASE("Checked coord arithmetic is alias-safe and failure-atomic") {
    coord_t value{10, -20};
    const coord_t delta{3, 4};

    CHECK(coord_add_checked(&value, &value, &delta)
        == NAVSYS_STATUS_OK);
    CHECK(value.x == 13);
    CHECK(value.y == -16);

    coord_t rhs{3, 4};
    CHECK(coord_sub_checked(&rhs, &value, &rhs)
        == NAVSYS_STATUS_OK);
    CHECK(rhs.x == 10);
    CHECK(rhs.y == -20);

    CHECK(coord_mul_checked(&value, &value, -2)
        == NAVSYS_STATUS_OK);
    CHECK(value.x == -26);
    CHECK(value.y == 32);
    CHECK(coord_div_checked(&value, &value, 3)
        == NAVSYS_STATUS_OK);
    CHECK(value.x == -8);
    CHECK(value.y == 10);

    const coord_t before_failure = value;
    const coord_t max_value{
        static_cast<int>(BYUL_COORD_COMPONENT_MAX),
        static_cast<int>(BYUL_COORD_COMPONENT_MIN)
    };
    const coord_t one{1, -1};
    const coord_t negative_one{-1, 1};
    CHECK(coord_add_checked(&value, &max_value, &one)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == before_failure.x);
    CHECK(value.y == before_failure.y);
    CHECK(coord_sub_checked(&value, &max_value, &negative_one)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == before_failure.x);
    CHECK(value.y == before_failure.y);
    CHECK(coord_mul_checked(&value, &max_value, 2)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == before_failure.x);
    CHECK(value.y == before_failure.y);
    CHECK(coord_div_checked(&value, &max_value, 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == before_failure.x);
    CHECK(value.y == before_failure.y);

    const coord_t outside_policy{
        static_cast<int>(BYUL_COORD_COMPONENT_MAX + INT32_C(1)),
        0
    };
    CHECK(coord_add_checked(&value, &outside_policy, &one)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value.x == before_failure.x);
    CHECK(value.y == before_failure.y);
    CHECK(coord_add_checked(nullptr, &value, &one)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(coord_sub_checked(&value, nullptr, &one)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(coord_mul_checked(&value, nullptr, 1)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(coord_div_checked(&value, nullptr, 1)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
}

TEST_CASE("Checked coord queries preserve scalar outputs on failure") {
    const coord_t min_value{
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min()
    };
    const coord_t max_value{
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max()
    };

    int order = 71;
    CHECK(coord_compare_canonical(&min_value, &max_value, &order)
        == NAVSYS_STATUS_OK);
    CHECK(order == -1);
    CHECK(coord_compare_canonical(&max_value, &min_value, &order)
        == NAVSYS_STATUS_OK);
    CHECK(order == 1);
    CHECK(coord_compare_canonical(&max_value, &max_value, &order)
        == NAVSYS_STATUS_OK);
    CHECK(order == 0);
    order = 71;
    CHECK(coord_compare_canonical(nullptr, &max_value, &order)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(order == 71);

    double distance = -1.0;
    CHECK(coord_distance_f64(&min_value, &max_value, &distance)
        == NAVSYS_STATUS_OK);
    CHECK(std::isfinite(distance));
    CHECK(distance > 0.0);
    const double preserved_distance = distance;
    CHECK(coord_distance_f64(&min_value, nullptr, &distance)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(distance == preserved_distance);

    int64_t manhattan = -1;
    CHECK(coord_manhattan_distance_i64(
        &min_value, &max_value, &manhattan) == NAVSYS_STATUS_OK);
    CHECK(manhattan == INT64_C(8589934590));
    const int64_t preserved_manhattan = manhattan;
    CHECK(coord_manhattan_distance_i64(
        nullptr, &max_value, &manhattan)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(manhattan == preserved_manhattan);
}

TEST_CASE("Checked coord direction distinguishes identical points") {
    const coord_t origin{0, 0};
    const coord_t east{1, 0};
    const coord_t north{0, 1};
    const coord_t west{-1, 0};
    const coord_t south{0, -1};

    double angle = -1.0;
    CHECK(coord_angle_rad(&origin, &east, &angle) == NAVSYS_STATUS_OK);
    CHECK(angle == doctest::Approx(0.0).epsilon(kCoordTestEpsilon));
    CHECK(coord_angle_rad(&origin, &north, &angle) == NAVSYS_STATUS_OK);
    CHECK(angle == doctest::Approx(kPi / 2.0).epsilon(kCoordTestEpsilon));
    CHECK(coord_angle_rad(&origin, &west, &angle) == NAVSYS_STATUS_OK);
    CHECK(angle == doctest::Approx(kPi).epsilon(kCoordTestEpsilon));
    CHECK(coord_angle_rad(&origin, &south, &angle) == NAVSYS_STATUS_OK);
    CHECK(angle == doctest::Approx(3.0 * kPi / 2.0)
        .epsilon(kCoordTestEpsilon));

    angle = 71.0;
    CHECK(coord_angle_rad(&origin, &origin, &angle)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(angle == 71.0);
    CHECK(coord_angle_deg(nullptr, &east, &angle)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(angle == 71.0);
    CHECK(coord_angle_deg(&origin, &north, &angle)
        == NAVSYS_STATUS_OK);
    CHECK(angle == doctest::Approx(90.0).epsilon(kCoordTestEpsilon));

    coord_t start{0, 0};
    const coord_t goal{3, -4};
    CHECK(coord_step_toward(&start, &goal, &start)
        == NAVSYS_STATUS_OK);
    CHECK(start.x == 1);
    CHECK(start.y == -1);

    coord_t goal_alias{3, -4};
    const coord_t start_value{0, 0};
    CHECK(coord_step_toward(&start_value, &goal_alias, &goal_alias)
        == NAVSYS_STATUS_OK);
    CHECK(goal_alias.x == 1);
    CHECK(goal_alias.y == -1);

    const coord_t preserved = start;
    CHECK(coord_step_toward(nullptr, &goal, &start)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(start.x == preserved.x);
    CHECK(start.y == preserved.y);
}

TEST_CASE("coord_format implements a failure-atomic two-call byte contract") {
    const coord_t value{-200000000, 200000000};
    constexpr char expected[] = "(-200000000, 200000000)";
    constexpr size_t expected_size = sizeof(expected);

    size_t required = 71;
    CHECK(coord_format(&value, nullptr, 0, &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == expected_size);

    char exact[expected_size];
    std::fill(std::begin(exact), std::end(exact), '#');
    required = 71;
    CHECK(coord_format(&value, exact, sizeof(exact), &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == expected_size);
    CHECK(std::string(exact) == expected);

    char short_buffer[expected_size - 1];
    std::fill(
        std::begin(short_buffer),
        std::end(short_buffer),
        '#');
    const std::string short_before(
        std::begin(short_buffer),
        std::end(short_buffer));
    required = 71;
    CHECK(coord_format(
        &value, short_buffer, sizeof(short_buffer), &required)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(required == expected_size);
    CHECK(std::string(
        std::begin(short_buffer),
        std::end(short_buffer)) == short_before);

    char oversized[64];
    std::fill(std::begin(oversized), std::end(oversized), '#');
    required = 71;
    CHECK(coord_format(&value, oversized, sizeof(oversized), &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == expected_size);
    CHECK(std::string(oversized) == expected);

    char preserved[8] = "keep";
    required = 71;
    CHECK(coord_format(nullptr, preserved, sizeof(preserved), &required)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(std::string(preserved) == "keep");
    CHECK(required == 71);
    CHECK(coord_format(&value, nullptr, 1, &required)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(required == 71);
    CHECK(coord_format(&value, preserved, 0, &required)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(std::string(preserved) == "keep");
    CHECK(required == 71);
    CHECK(coord_format(&value, preserved, sizeof(preserved), nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(std::string(preserved) == "keep");
}

TEST_CASE("Legacy coord string and clone APIs forward to checked contracts") {
    const coord_t value{0, 0};
    char exact[sizeof("(0, 0)")];
    CHECK(coord_to_string(&value, sizeof(exact), exact) == exact);
    CHECK(std::string(exact) == "(0, 0)");

    char short_buffer[sizeof("(0, 0)") - 1] = "keep";
    CHECK(coord_to_string(
        &value, sizeof(short_buffer), short_buffer) == nullptr);
    CHECK(std::string(short_buffer) == "keep");

    const coord_t goal{2, -3};
    coord_t* next = coord_clone_next_to_goal(&value, &goal);
    REQUIRE(next != nullptr);
    CHECK(next->x == 1);
    CHECK(next->y == -1);
    coord_destroy(next);

    CHECK(coord_clone_next_to_goal(nullptr, &goal) == nullptr);
    CHECK(coord_clone_next_to_goal(&value, nullptr) == nullptr);
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

     CHECK(doctest::Approx(coord_angle(&origin, &east)).epsilon(kCoordTestEpsilon) == 0.0);
    CHECK(doctest::Approx(coord_angle(&origin, &north)).epsilon(kCoordTestEpsilon) == kPi / 2.0);
    CHECK(doctest::Approx(coord_angle(&origin, &west)).epsilon(kCoordTestEpsilon) == kPi);
    CHECK(doctest::Approx(coord_angle(&origin, &south)).epsilon(kCoordTestEpsilon) == 3.0 * kPi / 2.0);
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
