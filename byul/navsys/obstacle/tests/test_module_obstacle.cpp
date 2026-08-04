//test_coord.cpp

#include "doctest.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <locale.h>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

extern "C" {
#include "obstacle.h"

#include "console.h"
#include "navgrid.h"
}

#include "../internal/obstacle_private.hpp"

static_assert(std::is_standard_layout_v<obstacle_t>);
static_assert(offsetof(obstacle_t, x0) == 0);
static_assert(offsetof(obstacle_t, y0) == 4);
static_assert(offsetof(obstacle_t, width) == 8);
static_assert(offsetof(obstacle_t, height) == 12);
static_assert(offsetof(obstacle_t, blocked) == 16);
static_assert(sizeof(obstacle_t) == (sizeof(void*) == 8 ? 24 : 20));
static_assert(alignof(obstacle_t) == alignof(void*));
static_assert(ENCLOSURE_OPEN_UNKNOWN == 0);
static_assert(ENCLOSURE_OPEN_RIGHT == 1);
static_assert(ENCLOSURE_OPEN_UP == 2);
static_assert(ENCLOSURE_OPEN_LEFT == 3);
static_assert(ENCLOSURE_OPEN_DOWN == 4);
static_assert(SPIRAL_CLOCKWISE == 0);
static_assert(SPIRAL_COUNTER_CLOCKWISE == 1);

using obstacle_coord_pair_t = std::pair<int, int>;

std::vector<obstacle_coord_pair_t> sorted_blocked_coords(
    const obstacle_t* obstacle) {
    size_t count = 0;
    REQUIRE(obstacle_export_blocked(
        obstacle, nullptr, 0, &count) == NAVSYS_STATUS_OK);
    std::vector<coord_t> coords(count);
    REQUIRE(obstacle_export_blocked(
        obstacle,
        coords.empty() ? nullptr : coords.data(),
        coords.size(),
        &count) == NAVSYS_STATUS_OK);

    std::vector<obstacle_coord_pair_t> result;
    result.reserve(count);
    for (const coord_t& coord : coords) {
        result.emplace_back(coord.x, coord.y);
    }
    std::sort(result.begin(), result.end());
    return result;
}

void check_obstacle_golden(
    const obstacle_t* obstacle,
    int x0,
    int y0,
    int width,
    int height,
    std::initializer_list<obstacle_coord_pair_t> expected) {
    REQUIRE(obstacle != nullptr);
    int actual_x0 = 0;
    int actual_y0 = 0;
    obstacle_fetch_origin(obstacle, &actual_x0, &actual_y0);
    CHECK(actual_x0 == x0);
    CHECK(actual_y0 == y0);
    CHECK(obstacle_get_width(obstacle) == width);
    CHECK(obstacle_get_height(obstacle) == height);

    std::vector<obstacle_coord_pair_t> sorted_expected(expected);
    std::sort(sorted_expected.begin(), sorted_expected.end());
    CHECK(sorted_blocked_coords(obstacle) == sorted_expected);
}

void check_all_blocked_inside_extent(const obstacle_t* obstacle) {
    REQUIRE(obstacle != nullptr);
    for (const auto& point : sorted_blocked_coords(obstacle)) {
        CHECK(obstacle_is_inside(obstacle, point.first, point.second));
    }
}

std::vector<obstacle_coord_pair_t> translated_blocked_coords(
    const obstacle_t* obstacle, int dx, int dy) {
    std::vector<obstacle_coord_pair_t> translated;
    for (const auto& point : sorted_blocked_coords(obstacle)) {
        translated.emplace_back(point.first + dx, point.second + dy);
    }
    std::sort(translated.begin(), translated.end());
    return translated;
}

std::vector<obstacle_coord_pair_t> rotated_90_blocked_coords(
    const obstacle_t* obstacle) {
    std::vector<obstacle_coord_pair_t> rotated;
    for (const auto& point : sorted_blocked_coords(obstacle)) {
        rotated.emplace_back(-point.second, point.first);
    }
    std::sort(rotated.begin(), rotated.end());
    return rotated;
}

std::vector<obstacle_coord_pair_t> diagonal_reflected_blocked_coords(
    const obstacle_t* obstacle) {
    std::vector<obstacle_coord_pair_t> reflected;
    for (const auto& point : sorted_blocked_coords(obstacle)) {
        reflected.emplace_back(point.second, point.first);
    }
    std::sort(reflected.begin(), reflected.end());
    return reflected;
}

struct obstacle_cancel_fixture_t {
    int calls;
    int cancel_after;
};

TEST_CASE("Obstacle opaque ABI reports current and ABI1 compatibility") {
    CHECK(obstacle_get_abi_version() == BYUL_OBSTACLE_ABI_VERSION);
    CHECK(obstacle_get_abi_fingerprint() == BYUL_OBSTACLE_ABI_FINGERPRINT);
    CHECK(obstacle_sizeof() == sizeof(obstacle_t));
    CHECK(obstacle_alignof() == alignof(obstacle_t));

    obstacle_abi_mismatch_t mismatch = OBSTACLE_ABI_VERSION_MISMATCH;
    CHECK(obstacle_check_abi(
        BYUL_OBSTACLE_ABI_VERSION,
        BYUL_OBSTACLE_ABI_FINGERPRINT,
        &mismatch) == NAVSYS_STATUS_OK);
    CHECK(mismatch == OBSTACLE_ABI_MATCH);
    CHECK(obstacle_check_abi(
        1u, UINT64_C(0x4f42535401000018), &mismatch)
        == NAVSYS_STATUS_OK);
    CHECK(mismatch == OBSTACLE_ABI_MATCH);
    CHECK(obstacle_check_abi(
        BYUL_OBSTACLE_ABI_VERSION,
        BYUL_OBSTACLE_ABI_FINGERPRINT ^ UINT64_C(1),
        &mismatch) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == OBSTACLE_ABI_FINGERPRINT_MISMATCH);
    CHECK(obstacle_check_abi(99u, 0, &mismatch)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == OBSTACLE_ABI_VERSION_MISMATCH);
    CHECK(obstacle_check_abi(
        BYUL_OBSTACLE_ABI_VERSION,
        BYUL_OBSTACLE_ABI_FINGERPRINT,
        nullptr) == NAVSYS_STATUS_INVALID_ARGUMENT);
}

bool cancel_obstacle_overlay(void* userdata) {
    auto* fixture = static_cast<obstacle_cancel_fixture_t*>(userdata);
    ++fixture->calls;
    return fixture->calls >= fixture->cancel_after;
}

bool throw_obstacle_overlay_cancel(void*) {
    throw std::runtime_error("obstacle overlay cancellation failure");
}

TEST_CASE("obstacle checked rect generators use half-open extent and seed replay") {
    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    CHECK(options.struct_size == sizeof(options));
    CHECK(options.abi_version == OBSTACLE_GENERATE_OPTIONS_ABI_VERSION);
    CHECK(options.raster_rule == OBSTACLE_RASTER_CELL_CENTER);

    obstacle_t* filled = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    REQUIRE(obstacle_generate_filled_rect(
        1, 2, 2, 2, &options, &filled) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        filled, 1, 2, 2, 2,
        {{1, 2}, {2, 2}, {1, 3}, {2, 3}});
    obstacle_destroy(filled);

    obstacle_t* empty = nullptr;
    REQUIRE(obstacle_generate_filled_rect(
        -4, 7, 0, 3, nullptr, &empty) == NAVSYS_STATUS_OK);
    check_obstacle_golden(empty, -4, 7, 0, 0, {});
    obstacle_destroy(empty);

    options.max_cells = 3;
    obstacle_t* limited = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_filled_rect(
        0, 0, 2, 2, &options, &limited) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(limited == nullptr);
    CHECK(obstacle_generate_filled_rect(
        0, 0, -1, 2, nullptr, &limited) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(limited == nullptr);

    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.seed = 17;
    obstacle_t* random = nullptr;
    REQUIRE(obstacle_generate_random_rect(
        5, -2, 4, 3, 0.5, &options, &random) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        random, 5, -2, 4, 3,
        {{6, -2}, {7, -2}, {8, -2},
         {5, -1}, {6, -1}, {7, -1},
         {5, 0}, {7, 0}, {8, 0}});
    obstacle_t* replay = nullptr;
    REQUIRE(obstacle_generate_random_rect(
        5, -2, 4, 3, 0.5, &options, &replay) == NAVSYS_STATUS_OK);
    CHECK(obstacle_equal(random, replay));
    obstacle_destroy(replay);
    obstacle_destroy(random);

    obstacle_t* invalid = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_random_rect(
        0, 0, 1, 1, std::numeric_limits<double>::quiet_NaN(),
        nullptr, &invalid) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);
}

TEST_CASE("obstacle checked line is symmetric and failure atomic") {
    const coord_t start = {-3, -3};
    const coord_t end = {-1, -2};
    obstacle_t* forward = nullptr;
    obstacle_t* reverse = nullptr;
    REQUIRE(obstacle_generate_line(
        &start, &end, 0, nullptr, &forward) == NAVSYS_STATUS_OK);
    REQUIRE(obstacle_generate_line(
        &end, &start, 0, nullptr, &reverse) == NAVSYS_STATUS_OK);
    CHECK(obstacle_equal(forward, reverse));
    check_obstacle_golden(
        forward, -3, -3, 3, 2,
        {{-3, -3}, {-2, -3}, {-2, -2}, {-1, -2}});
    obstacle_destroy(reverse);
    obstacle_destroy(forward);

    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.raster_rule = OBSTACLE_RASTER_ALL_TOUCHED;
    const coord_t diagonal_start = {0, 0};
    const coord_t diagonal_end = {1, 1};
    obstacle_t* all_touched = nullptr;
    REQUIRE(obstacle_generate_line(
        &diagonal_start, &diagonal_end, 0, &options, &all_touched)
        == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        all_touched, 0, 0, 2, 2,
        {{0, 0}, {1, 0}, {0, 1}, {1, 1}});
    obstacle_t* all_touched_reverse = nullptr;
    REQUIRE(obstacle_generate_line(
        &diagonal_end, &diagonal_start, 0, &options,
        &all_touched_reverse) == NAVSYS_STATUS_OK);
    CHECK(obstacle_equal(all_touched, all_touched_reverse));
    obstacle_destroy(all_touched_reverse);
    obstacle_destroy(all_touched);

    obstacle_cancel_fixture_t cancel{0, 1};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.cancel_func = cancel_obstacle_overlay;
    options.cancel_userdata = &cancel;
    obstacle_t* cancelled = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_line(
        &start, &end, 1, &options, &cancelled)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(cancelled == nullptr);
}

TEST_CASE("obstacle checked polygon supports explicit fill and outline") {
    const coord_t triangle[] = {{0, 0}, {2, 0}, {0, 2}};
    obstacle_t* filled = nullptr;
    REQUIRE(obstacle_generate_polygon(
        triangle, 3, OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &filled) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        filled, 0, 0, 3, 3,
        {{0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {0, 2}});
    obstacle_destroy(filled);

    obstacle_t* outline = nullptr;
    REQUIRE(obstacle_generate_polygon_outline(
        triangle, 3, 0, nullptr, &outline) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        outline, 0, 0, 3, 3,
        {{0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {0, 2}});
    obstacle_destroy(outline);

    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.raster_rule = OBSTACLE_RASTER_ALL_TOUCHED;
    obstacle_t* touched = nullptr;
    REQUIRE(obstacle_generate_polygon(
        triangle, 3, OBSTACLE_POLYGON_EVEN_ODD,
        &options, &touched) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        touched, 0, 0, 3, 3,
        {{0, 0}, {1, 0}, {2, 0},
         {0, 1}, {1, 1}, {2, 1},
         {0, 2}, {1, 2}});
    obstacle_destroy(touched);

    obstacle_t* touched_outline = nullptr;
    REQUIRE(obstacle_generate_polygon_outline(
        triangle, 3, 0, &options, &touched_outline)
        == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        touched_outline, 0, 0, 3, 3,
        {{0, 0}, {1, 0}, {2, 0},
         {0, 1}, {1, 1}, {2, 1},
         {0, 2}, {1, 2}});
    obstacle_destroy(touched_outline);

    options.max_cells = 7;
    obstacle_t* limited = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_polygon(
        triangle, 3, OBSTACLE_POLYGON_EVEN_ODD,
        &options, &limited) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(limited == nullptr);

    const coord_t twice_wound[] = {
        {0, 0}, {2, 0}, {2, 2}, {0, 2},
        {0, 0}, {2, 0}, {2, 2}, {0, 2}};
    obstacle_t* even_odd = nullptr;
    obstacle_t* non_zero = nullptr;
    REQUIRE(obstacle_generate_polygon(
        twice_wound, 8, OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &even_odd) == NAVSYS_STATUS_OK);
    REQUIRE(obstacle_generate_polygon(
        twice_wound, 8, OBSTACLE_POLYGON_NON_ZERO,
        nullptr, &non_zero) == NAVSYS_STATUS_OK);
    CHECK_FALSE(obstacle_is_coord_blocked(even_odd, 1, 1));
    CHECK(obstacle_is_coord_blocked(non_zero, 1, 1));
    CHECK(sorted_blocked_coords(even_odd).size() == 8);
    CHECK(sorted_blocked_coords(non_zero).size() == 9);
    obstacle_destroy(non_zero);
    obstacle_destroy(even_odd);

    const coord_t collinear[] = {{0, 0}, {1, 0}, {2, 0}};
    obstacle_t* invalid = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_polygon(
        collinear, 3, OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &invalid) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);
}

TEST_CASE("obstacle checked rect outline and triangle conveniences are exact") {
    obstacle_t* rectangle = nullptr;
    REQUIRE(obstacle_generate_rect_outline(
        10, 20, 5, 4, 1, nullptr, &rectangle) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        rectangle, 10, 20, 5, 4,
        {{10, 20}, {11, 20}, {12, 20}, {13, 20}, {14, 20},
         {10, 21}, {14, 21}, {10, 22}, {14, 22},
         {10, 23}, {11, 23}, {12, 23}, {13, 23}, {14, 23}});
    obstacle_destroy(rectangle);

    const coord_t a{0, 0};
    const coord_t b{2, 0};
    const coord_t c{0, 2};
    obstacle_t* triangle = nullptr;
    REQUIRE(obstacle_generate_triangle(
        &a, &b, &c, OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &triangle) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        triangle, 0, 0, 3, 3,
        {{0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {0, 2}});

    obstacle_t* outline = nullptr;
    REQUIRE(obstacle_generate_triangle_outline(
        &a, &b, &c, 0, nullptr, &outline) == NAVSYS_STATUS_OK);
    CHECK(obstacle_equal(triangle, outline));
    obstacle_destroy(outline);
    obstacle_destroy(triangle);

    obstacle_t* invalid = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_rect_outline(
        0, 0, 4, 5, 2, nullptr, &invalid)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);
    CHECK(obstacle_generate_triangle(
        &a, nullptr, &c, OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &invalid) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);

    const coord_t collinear{1, 0};
    CHECK(obstacle_generate_triangle_outline(
        &a, &collinear, &b, 0, nullptr, &invalid)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);

    obstacle_cancel_fixture_t cancel{0, 1};
    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.cancel_func = cancel_obstacle_overlay;
    options.cancel_userdata = &cancel;
    CHECK(obstacle_generate_rect_outline(
        0, 0, 5, 5, 1, &options, &invalid)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(invalid == nullptr);
}

TEST_CASE("lossless legacy generators match their canonical targets") {
    obstacle_t* legacy_rect = obstacle_make_rect_all_blocked(1, 2, 3, 2);
    obstacle_t* canonical_rect = nullptr;
    REQUIRE(obstacle_generate_filled_rect(
        1, 2, 3, 2, nullptr, &canonical_rect) == NAVSYS_STATUS_OK);
    REQUIRE(legacy_rect != nullptr);
    CHECK(obstacle_equal(legacy_rect, canonical_rect));
    obstacle_destroy(canonical_rect);
    obstacle_destroy(legacy_rect);

    const coord_t first{7, 9};
    const coord_t second{1, 2};
    obstacle_t* legacy_ring = obstacle_make_torus(&first, &second, 2);
    obstacle_t* canonical_ring = nullptr;
    REQUIRE(obstacle_generate_rect_outline(
        1, 2, 7, 8, 2, nullptr, &canonical_ring) == NAVSYS_STATUS_OK);
    REQUIRE(legacy_ring != nullptr);
    CHECK(obstacle_equal(legacy_ring, canonical_ring));
    obstacle_destroy(canonical_ring);
    obstacle_destroy(legacy_ring);

    const coord_t center{-3, 4};
    obstacle_t* legacy_cross = obstacle_make_cross(&center, 2, 1);
    obstacle_cross_desc_t desc{};
    REQUIRE(obstacle_cross_desc_init(&desc) == NAVSYS_STATUS_OK);
    desc.center = center;
    desc.arm_length_cells = 2;
    desc.radius_cells = 1;
    obstacle_t* canonical_cross = nullptr;
    REQUIRE(obstacle_generate_cross(&desc, nullptr, &canonical_cross)
        == NAVSYS_STATUS_OK);
    REQUIRE(legacy_cross != nullptr);
    CHECK(obstacle_equal(legacy_cross, canonical_cross));
    obstacle_destroy(canonical_cross);
    obstacle_destroy(legacy_cross);
}

TEST_CASE("canonical generator shapes satisfy count extent and translation properties") {
    obstacle_t* rectangle = nullptr;
    REQUIRE(obstacle_generate_rect_outline(
        -4, -3, 9, 7, 2, nullptr, &rectangle) == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(rectangle).size() == 48);
    CHECK_FALSE(obstacle_is_coord_blocked(rectangle, 0, 0));
    check_all_blocked_inside_extent(rectangle);

    obstacle_t* moved_rectangle = nullptr;
    REQUIRE(obstacle_generate_rect_outline(
        7, -10, 9, 7, 2, nullptr, &moved_rectangle)
        == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(moved_rectangle)
        == translated_blocked_coords(rectangle, 11, -7));
    check_all_blocked_inside_extent(moved_rectangle);
    obstacle_destroy(moved_rectangle);

    obstacle_t* rotated_rectangle = nullptr;
    REQUIRE(obstacle_generate_rect_outline(
        -3, -4, 7, 9, 2, nullptr, &rotated_rectangle)
        == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(rotated_rectangle)
        == rotated_90_blocked_coords(rectangle));
    obstacle_destroy(rotated_rectangle);
    obstacle_destroy(rectangle);

    obstacle_enclosure_desc_t enclosure{};
    REQUIRE(obstacle_enclosure_desc_init(&enclosure) == NAVSYS_STATUS_OK);
    enclosure.x0 = -3;
    enclosure.y0 = 5;
    enclosure.width = 7;
    enclosure.height = 7;
    enclosure.wall_thickness_cells = 2;
    enclosure.open_side = OBSTACLE_ENCLOSURE_OPEN_LEFT;
    enclosure.aperture_offset_cells = 2;
    enclosure.aperture_length_cells = 3;
    obstacle_t* opened = nullptr;
    REQUIRE(obstacle_generate_enclosure(&enclosure, nullptr, &opened)
        == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(opened).size() == 34);
    CHECK_FALSE(obstacle_is_coord_blocked(opened, -3, 7));
    CHECK_FALSE(obstacle_is_coord_blocked(opened, -2, 9));
    check_all_blocked_inside_extent(opened);

    obstacle_enclosure_desc_t rotated_enclosure = enclosure;
    rotated_enclosure.x0 = -11;
    rotated_enclosure.y0 = -3;
    rotated_enclosure.width = 7;
    rotated_enclosure.height = 7;
    rotated_enclosure.open_side = OBSTACLE_ENCLOSURE_OPEN_UP;
    rotated_enclosure.aperture_offset_cells = 2;
    obstacle_t* rotated_opened = nullptr;
    REQUIRE(obstacle_generate_enclosure(
        &rotated_enclosure, nullptr, &rotated_opened) == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(rotated_opened)
        == rotated_90_blocked_coords(opened));
    obstacle_destroy(rotated_opened);
    obstacle_destroy(opened);

    obstacle_cross_desc_t cross{};
    REQUIRE(obstacle_cross_desc_init(&cross) == NAVSYS_STATUS_OK);
    cross.center = {4, -6};
    cross.arm_length_cells = 3;
    cross.radius_cells = 1;
    obstacle_t* cross_obstacle = nullptr;
    REQUIRE(obstacle_generate_cross(&cross, nullptr, &cross_obstacle)
        == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(cross_obstacle).size() == 45);
    for (const auto& point : sorted_blocked_coords(cross_obstacle)) {
        const int dx = point.first - cross.center.x;
        const int dy = point.second - cross.center.y;
        CHECK(obstacle_is_coord_blocked(
            cross_obstacle, cross.center.x - dy, cross.center.y + dx));
        CHECK(obstacle_is_coord_blocked(
            cross_obstacle, cross.center.x - dx, point.second));
    }
    check_all_blocked_inside_extent(cross_obstacle);
    obstacle_destroy(cross_obstacle);

    obstacle_spiral_desc_t spiral{};
    REQUIRE(obstacle_spiral_desc_init(&spiral) == NAVSYS_STATUS_OK);
    spiral.center = {-7, 9};
    spiral.max_radius_cells = 3;
    spiral.pitch_cells = 1;
    spiral.path_radius_cells = 1;
    spiral.clip_rule = OBSTACLE_SPIRAL_CLIP_OUTPUT;
    obstacle_t* clipped = nullptr;
    REQUIRE(obstacle_generate_spiral(&spiral, nullptr, &clipped)
        == NAVSYS_STATUS_OK);
    for (const auto& point : sorted_blocked_coords(clipped)) {
        CHECK(std::abs(point.first - spiral.center.x) <= 3);
        CHECK(std::abs(point.second - spiral.center.y) <= 3);
    }
    check_all_blocked_inside_extent(clipped);

    obstacle_spiral_desc_t reflected_spiral = spiral;
    reflected_spiral.center = {spiral.center.y, spiral.center.x};
    reflected_spiral.direction = OBSTACLE_SPIRAL_COUNTER_CLOCKWISE;
    obstacle_t* reflected = nullptr;
    REQUIRE(obstacle_generate_spiral(
        &reflected_spiral, nullptr, &reflected) == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(reflected)
        == diagonal_reflected_blocked_coords(clipped));
    obstacle_destroy(reflected);
    obstacle_destroy(clipped);
}

TEST_CASE("canonical random rectangle has a separate deterministic distribution gate") {
    constexpr int width = 64;
    constexpr int height = 64;
    constexpr int seed_count = 8;
    constexpr double probability = 0.25;
    size_t total_blocked = 0;
    for (uint64_t seed = 0; seed < seed_count; ++seed) {
        obstacle_generate_options_t options{};
        REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
        options.seed = seed;
        obstacle_t* sample = nullptr;
        REQUIRE(obstacle_generate_random_rect(
            0, 0, width, height, probability, &options, &sample)
            == NAVSYS_STATUS_OK);
        const size_t count = sorted_blocked_coords(sample).size();
        CHECK(count > 850);
        CHECK(count < 1200);
        total_blocked += count;
        check_all_blocked_inside_extent(sample);
        if (seed == 0) {
            obstacle_t* translated = nullptr;
            REQUIRE(obstacle_generate_random_rect(
                13, -17, width, height, probability, &options, &translated)
                == NAVSYS_STATUS_OK);
            CHECK(sorted_blocked_coords(translated)
                == translated_blocked_coords(sample, 13, -17));
            obstacle_destroy(translated);
        }
        obstacle_destroy(sample);
    }
    const double observed = static_cast<double>(total_blocked)
        / static_cast<double>(width * height * seed_count);
    CHECK(observed > 0.24);
    CHECK(observed < 0.26);
}

TEST_CASE("canonical generators reject overflowing coordinate domains atomically") {
    const int32_t minimum = std::numeric_limits<int32_t>::min();
    const int32_t maximum = std::numeric_limits<int32_t>::max();
    obstacle_t* output = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_filled_rect(
        maximum, 0, 2, 1, nullptr, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    const coord_t low{minimum, 0};
    const coord_t high{maximum, 0};
    CHECK(obstacle_generate_line(
        &low, &high, 1, nullptr, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    const coord_t huge[] = {{minimum, 0}, {maximum, 0}, {minimum, 1}};
    CHECK(obstacle_generate_polygon(
        huge, 3, OBSTACLE_POLYGON_EVEN_ODD, nullptr, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
    CHECK(obstacle_generate_polygon_outline(
        huge, 3, 0, nullptr, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
    CHECK(obstacle_generate_triangle(
        &huge[0], &huge[1], &huge[2], OBSTACLE_POLYGON_EVEN_ODD,
        nullptr, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    obstacle_cross_desc_t cross{};
    REQUIRE(obstacle_cross_desc_init(&cross) == NAVSYS_STATUS_OK);
    cross.center = {maximum, 0};
    cross.arm_length_cells = 1;
    CHECK(obstacle_generate_cross(&cross, nullptr, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    obstacle_spiral_desc_t spiral{};
    REQUIRE(obstacle_spiral_desc_init(&spiral) == NAVSYS_STATUS_OK);
    spiral.center = {minimum, 0};
    spiral.max_radius_cells = 1;
    CHECK(obstacle_generate_spiral(&spiral, nullptr, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
}

TEST_CASE("obstacle checked enclosure owns aperture corner overlap") {
    obstacle_enclosure_desc_t desc{};
    REQUIRE(obstacle_enclosure_desc_init(&desc) == NAVSYS_STATUS_OK);
    CHECK(desc.struct_size == sizeof(desc));
    CHECK(desc.abi_version == OBSTACLE_ENCLOSURE_DESC_ABI_VERSION);
    desc.x0 = 10;
    desc.y0 = 20;
    desc.width = 5;
    desc.height = 5;
    desc.open_side = OBSTACLE_ENCLOSURE_OPEN_LEFT;
    desc.aperture_offset_cells = 0;
    desc.aperture_length_cells = 5;
    obstacle_t* enclosure = nullptr;
    REQUIRE(obstacle_generate_enclosure(
        &desc, nullptr, &enclosure) == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        enclosure, 10, 20, 5, 5,
        {{11, 20}, {12, 20}, {13, 20}, {14, 20},
         {14, 21}, {14, 22}, {14, 23},
         {11, 24}, {12, 24}, {13, 24}, {14, 24}});
    obstacle_destroy(enclosure);

    obstacle_cancel_fixture_t cancel{0, 1};
    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.cancel_func = cancel_obstacle_overlay;
    options.cancel_userdata = &cancel;
    desc.aperture_length_cells = 0;
    obstacle_t* invalid = reinterpret_cast<obstacle_t*>(uintptr_t{1});
    CHECK(obstacle_generate_enclosure(
        &desc, &options, &invalid) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(invalid == nullptr);
    CHECK(cancel.calls == 0);
}

TEST_CASE("obstacle checked cross separates arm length and raster radius") {
    obstacle_cross_desc_t desc{};
    REQUIRE(obstacle_cross_desc_init(&desc) == NAVSYS_STATUS_OK);
    CHECK(desc.struct_size == sizeof(desc));
    CHECK(desc.abi_version == OBSTACLE_CROSS_DESC_ABI_VERSION);
    desc.arm_length_cells = 1;
    obstacle_t* thin = nullptr;
    REQUIRE(obstacle_generate_cross(&desc, nullptr, &thin)
        == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        thin, -1, -1, 3, 3,
        {{-1, 0}, {0, -1}, {0, 0}, {0, 1}, {1, 0}});
    obstacle_destroy(thin);

    desc.radius_cells = 1;
    obstacle_t* thick = nullptr;
    REQUIRE(obstacle_generate_cross(&desc, nullptr, &thick)
        == NAVSYS_STATUS_OK);
    CHECK(obstacle_get_width(thick) == 5);
    CHECK(obstacle_get_height(thick) == 5);
    CHECK(sorted_blocked_coords(thick).size() == 21);
    CHECK(obstacle_is_coord_blocked(thick, -2, 0));
    CHECK(obstacle_is_coord_blocked(thick, 0, 2));
    CHECK_FALSE(obstacle_is_coord_blocked(thick, -2, -2));
    obstacle_destroy(thick);
}

TEST_CASE("obstacle checked spiral uses radius pitch direction and clip") {
    obstacle_spiral_desc_t desc{};
    REQUIRE(obstacle_spiral_desc_init(&desc) == NAVSYS_STATUS_OK);
    CHECK(desc.struct_size == sizeof(desc));
    CHECK(desc.abi_version == OBSTACLE_SPIRAL_DESC_ABI_VERSION);
    CHECK(desc.pitch_cells == 1);
    desc.max_radius_cells = 2;
    desc.pitch_cells = 2;
    desc.clip_rule = OBSTACLE_SPIRAL_CLIP_OUTPUT;
    obstacle_t* clockwise = nullptr;
    REQUIRE(obstacle_generate_spiral(&desc, nullptr, &clockwise)
        == NAVSYS_STATUS_OK);
    check_obstacle_golden(
        clockwise, -2, -2, 5, 5,
        {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2},
         {1, 2}, {0, 2}, {-1, 2}, {-2, 2},
         {-2, 1}, {-2, 0}, {-2, -1}, {-2, -2},
         {-1, -2}, {0, -2}, {1, -2}, {2, -2}});

    desc.direction = OBSTACLE_SPIRAL_COUNTER_CLOCKWISE;
    obstacle_t* counter_clockwise = nullptr;
    REQUIRE(obstacle_generate_spiral(
        &desc, nullptr, &counter_clockwise) == NAVSYS_STATUS_OK);
    CHECK(sorted_blocked_coords(counter_clockwise).size() == 17);
    CHECK(obstacle_is_coord_blocked(counter_clockwise, 0, 1));
    CHECK_FALSE(obstacle_is_coord_blocked(clockwise, 0, 1));
    obstacle_destroy(counter_clockwise);
    obstacle_destroy(clockwise);

    REQUIRE(obstacle_spiral_desc_init(&desc) == NAVSYS_STATUS_OK);
    desc.max_radius_cells = 1;
    desc.path_radius_cells = 1;
    obstacle_t* path_only = nullptr;
    REQUIRE(obstacle_generate_spiral(&desc, nullptr, &path_only)
        == NAVSYS_STATUS_OK);
    CHECK(obstacle_get_width(path_only) == 5);
    CHECK(obstacle_get_height(path_only) == 5);
    CHECK(sorted_blocked_coords(path_only).size() == 25);
    obstacle_destroy(path_only);

    desc.clip_rule = OBSTACLE_SPIRAL_CLIP_OUTPUT;
    obstacle_t* clipped = nullptr;
    REQUIRE(obstacle_generate_spiral(&desc, nullptr, &clipped)
        == NAVSYS_STATUS_OK);
    CHECK(obstacle_get_width(clipped) == 3);
    CHECK(obstacle_get_height(clipped) == 3);
    CHECK(sorted_blocked_coords(clipped).size() == 9);
    obstacle_destroy(clipped);
}

TEST_CASE("obstacle all-touched raster preserves tiny geometry symmetries") {
    obstacle_generate_options_t options{};
    REQUIRE(obstacle_generate_options_init(&options) == NAVSYS_STATUS_OK);
    options.raster_rule = OBSTACLE_RASTER_ALL_TOUCHED;

    using scaled_pair_t = std::pair<int64_t, int64_t>;
    const auto orient = [](const scaled_pair_t& a,
                           const scaled_pair_t& b,
                           const scaled_pair_t& p) {
        const int64_t cross = (b.first - a.first) * (p.second - a.second)
            - (b.second - a.second) * (p.first - a.first);
        return (cross > 0) - (cross < 0);
    };
    const auto on_segment = [&](const scaled_pair_t& p,
                                const scaled_pair_t& a,
                                const scaled_pair_t& b) {
        return orient(a, b, p) == 0
            && std::min(a.first, b.first) <= p.first
            && p.first <= std::max(a.first, b.first)
            && std::min(a.second, b.second) <= p.second
            && p.second <= std::max(a.second, b.second);
    };
    const auto segments_touch = [&](const scaled_pair_t& a,
                                    const scaled_pair_t& b,
                                    const scaled_pair_t& c,
                                    const scaled_pair_t& d) {
        const int abc = orient(a, b, c);
        const int abd = orient(a, b, d);
        const int cda = orient(c, d, a);
        const int cdb = orient(c, d, b);
        return (abc == 0 && on_segment(c, a, b))
            || (abd == 0 && on_segment(d, a, b))
            || (cda == 0 && on_segment(a, c, d))
            || (cdb == 0 && on_segment(b, c, d))
            || ((abc < 0) != (abd < 0) && (cda < 0) != (cdb < 0));
    };
    const auto brute_reference = [&](const coord_t& start, const coord_t& end) {
        std::vector<obstacle_coord_pair_t> result;
        const scaled_pair_t a = {2LL * start.x, 2LL * start.y};
        const scaled_pair_t b = {2LL * end.x, 2LL * end.y};
        for (int y = std::min(start.y, end.y);
             y <= std::max(start.y, end.y); ++y) {
            for (int x = std::min(start.x, end.x);
                 x <= std::max(start.x, end.x); ++x) {
                const scaled_pair_t square[] = {
                    {2LL * x - 1, 2LL * y - 1},
                    {2LL * x + 1, 2LL * y - 1},
                    {2LL * x + 1, 2LL * y + 1},
                    {2LL * x - 1, 2LL * y + 1}};
                bool touched = square[0].first <= a.first
                    && a.first <= square[2].first
                    && square[0].second <= a.second
                    && a.second <= square[2].second;
                for (size_t side = 0; side < 4 && !touched; ++side) {
                    touched = segments_touch(
                        a, b, square[side], square[(side + 1) % 4]);
                }
                if (touched) result.emplace_back(x, y);
            }
        }
        std::sort(result.begin(), result.end());
        return result;
    };

    for (int y0 = -2; y0 <= 2; ++y0) {
        for (int x0 = -2; x0 <= 2; ++x0) {
            for (int y1 = -2; y1 <= 2; ++y1) {
                for (int x1 = -2; x1 <= 2; ++x1) {
                    const coord_t start = {x0, y0};
                    const coord_t end = {x1, y1};
                    obstacle_t* forward = nullptr;
                    obstacle_t* reverse = nullptr;
                    REQUIRE(obstacle_generate_line(
                        &start, &end, 0, &options, &forward)
                        == NAVSYS_STATUS_OK);
                    REQUIRE(obstacle_generate_line(
                        &end, &start, 0, &options, &reverse)
                        == NAVSYS_STATUS_OK);
                    CHECK(obstacle_equal(forward, reverse));
                    CHECK(sorted_blocked_coords(forward)
                        == brute_reference(start, end));
                    obstacle_destroy(reverse);
                    obstacle_destroy(forward);
                }
            }
        }
    }

    const coord_t polygon[] = {
        {-3, -1}, {2, -2}, {4, 1}, {1, 4}, {-2, 3}};
    const coord_t rotated[] = {
        {1, -3}, {2, 2}, {-1, 4}, {-4, 1}, {-3, -2}};
    const coord_t reversed[] = {
        {-2, 3}, {1, 4}, {4, 1}, {2, -2}, {-3, -1}};
    obstacle_t* baseline = nullptr;
    obstacle_t* rotation = nullptr;
    obstacle_t* reversal = nullptr;
    REQUIRE(obstacle_generate_polygon(
        polygon, 5, OBSTACLE_POLYGON_NON_ZERO,
        &options, &baseline) == NAVSYS_STATUS_OK);
    REQUIRE(obstacle_generate_polygon(
        rotated, 5, OBSTACLE_POLYGON_NON_ZERO,
        &options, &rotation) == NAVSYS_STATUS_OK);
    REQUIRE(obstacle_generate_polygon(
        reversed, 5, OBSTACLE_POLYGON_NON_ZERO,
        &options, &reversal) == NAVSYS_STATUS_OK);
    CHECK(obstacle_equal(baseline, reversal));
    std::vector<obstacle_coord_pair_t> rotated_expected;
    for (const auto& point : sorted_blocked_coords(baseline)) {
        rotated_expected.emplace_back(-point.second, point.first);
    }
    std::sort(rotated_expected.begin(), rotated_expected.end());
    CHECK(sorted_blocked_coords(rotation) == rotated_expected);
    obstacle_destroy(reversal);
    obstacle_destroy(rotation);
    obstacle_destroy(baseline);
}

TEST_CASE("obstacle core ABI layout and legacy neighbor export baseline") {
    obstacle_t* obstacle = obstacle_create_full(10, 20, 3, 3);
    REQUIRE(obstacle != nullptr);
    REQUIRE(obstacle_block_coord(obstacle, 10, 20));

    coord_list_t* neighbors = obstacle_clone_neighbors(obstacle, 11, 21);
    REQUIRE(neighbors != nullptr);
    CHECK(coord_list_size(neighbors) == 7);
    CHECK(obstacle_is_coord_blocked(obstacle, 10, 20));

    coord_list_destroy(neighbors);
    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle canonical neighborhood APIs preserve legacy results") {
    obstacle_t* obstacle = obstacle_create_full(0, 0, 5, 5);
    REQUIRE(obstacle != nullptr);
    REQUIRE(obstacle_block_coord(obstacle, 1, 1));

    size_t count = 777;
    CHECK(obstacle_export_neighbors(
        obstacle, 2, 2, false, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 8);

    coord_t short_buffer[2] = {{91, 92}, {93, 94}};
    CHECK(obstacle_export_neighbors(
        obstacle, 2, 2, true, short_buffer, 2, &count)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(count == 7);
    CHECK(short_buffer[0].x == 91);
    CHECK(short_buffer[0].y == 92);
    CHECK(short_buffer[1].x == 93);
    CHECK(short_buffer[1].y == 94);

    coord_t exported[8]{};
    REQUIRE(obstacle_export_neighbors(
        obstacle, 2, 2, true, exported, 8, &count)
        == NAVSYS_STATUS_OK);
    REQUIRE(count == 7);

    coord_list_t* canonical = obstacle_create_neighbors(obstacle, 2, 2);
    coord_list_t* legacy = obstacle_clone_neighbors(obstacle, 2, 2);
    REQUIRE(canonical != nullptr);
    REQUIRE(legacy != nullptr);
    REQUIRE(coord_list_size(canonical) == count);
    REQUIRE(coord_list_size(legacy) == count);
    for (size_t index = 0; index < count; ++index) {
        coord_t from_canonical{};
        coord_t from_legacy{};
        REQUIRE(coord_list_fetch(canonical, index, &from_canonical)
            == NAVSYS_STATUS_OK);
        REQUIRE(coord_list_fetch(legacy, index, &from_legacy)
            == NAVSYS_STATUS_OK);
        CHECK(coord_equal(&from_canonical, &exported[index]));
        CHECK(coord_equal(&from_legacy, &exported[index]));
    }

    CHECK(obstacle_export_neighbors_range(
        obstacle, 2, 2, 0, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 8);
    CHECK(obstacle_export_neighbors_range(
        obstacle, 2, 2, 1, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 25);

    coord_t selected{81, 82};
    CHECK(obstacle_fetch_neighbor_at_degree(
        obstacle, 2, 2, 0.0, &selected) == NAVSYS_STATUS_OK);
    CHECK(selected.x == 3);
    CHECK(selected.y == 2);
    const coord_t center{2, 2};
    const coord_t goal{2, 4};
    CHECK(obstacle_fetch_neighbor_at_goal(
        obstacle, &center, &goal, &selected) == NAVSYS_STATUS_OK);
    CHECK(selected.x == 2);
    CHECK(selected.y == 3);

    const coord_t east_goal{4, 2};
    CHECK(obstacle_export_neighbors_at_degree_range(
        obstacle, &center, &east_goal, -45.0, 45.0, 2,
        nullptr, 0, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 8);

    selected = {71, 72};
    CHECK(obstacle_fetch_neighbor_at_degree(
        obstacle, 9, 9, 0.0, &selected) == NAVSYS_STATUS_NOT_FOUND);
    CHECK(selected.x == 71);
    CHECK(selected.y == 72);

    coord_list_destroy(legacy);
    coord_list_destroy(canonical);
    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle checked square and line raster are failure atomic") {
    obstacle_t* square = obstacle_create_full(0, 0, 7, 7);
    REQUIRE(square != nullptr);

    size_t changed = 999;
    REQUIRE(obstacle_block_square(square, 3, 3, 1, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 9);
    CHECK(coord_hash_size(obstacle_get_blocked_coords(square)) == 9);
    REQUIRE(obstacle_block_square(square, 3, 3, 1, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 0);

    obstacle_t* snapshot = obstacle_copy(square);
    REQUIRE(snapshot != nullptr);
    changed = 888;
    CHECK(obstacle_block_square(square, 8, 8, 1, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(changed == 888);
    CHECK(obstacle_equal(square, snapshot));
    CHECK(obstacle_block_square(square, 3, 3, -1, &changed)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(changed == 888);
    CHECK(obstacle_equal(square, snapshot));

    obstacle_t* canonical = obstacle_create_full(0, 0, 7, 7);
    obstacle_t* legacy = obstacle_create_full(0, 0, 7, 7);
    REQUIRE(canonical != nullptr);
    REQUIRE(legacy != nullptr);
    REQUIRE(obstacle_block_line(
        canonical, 1, 1, 5, 3, 1, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed > 0);
    obstacle_block_straight(legacy, 1, 1, 5, 3, 1);
    CHECK(obstacle_equal(canonical, legacy));

    obstacle_destroy(legacy);
    obstacle_destroy(canonical);
    obstacle_destroy(snapshot);
    obstacle_destroy(square);
}

TEST_CASE("obstacle checked lifecycle and mutation preserve outputs on failure") {
    obstacle_t* obstacle = nullptr;
    REQUIRE(obstacle_create_checked(10, 20, -4, -3, &obstacle)
        == NAVSYS_STATUS_OK);
    REQUIRE(obstacle != nullptr);
    CHECK(obstacle->x0 == 10);
    CHECK(obstacle->y0 == 20);
    CHECK(obstacle->width == -4);
    CHECK(obstacle->height == -3);

    obstacle_t* preserved_output = obstacle;
    CHECK(obstacle_copy_checked(nullptr, &preserved_output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved_output == obstacle);
    CHECK(obstacle_copy_checked(obstacle, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    bool changed = true;
    CHECK(obstacle_set_blocked(obstacle, 10, 19, true, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(changed);
    CHECK(obstacle_set_blocked(nullptr, 7, 18, true, &changed)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(changed);

    CHECK(obstacle_set_blocked(obstacle, 7, 18, true, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);
    CHECK(obstacle_is_coord_blocked(obstacle, 7, 18));
    CHECK(obstacle_set_blocked(obstacle, 7, 18, true, &changed)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(changed);
    CHECK(obstacle_block_coord(obstacle, 7, 18));

    obstacle_t* copied = nullptr;
    REQUIRE(obstacle_copy_checked(obstacle, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(obstacle_equal(obstacle, copied));
    CHECK(copied->blocked != obstacle->blocked);

    CHECK(obstacle_set_blocked(obstacle, 7, 18, false, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);
    CHECK_FALSE(obstacle_unblock_coord(obstacle, 7, 18));
    CHECK(obstacle_is_coord_blocked(copied, 7, 18));

    coord_hash_t* saved_blocked = obstacle->blocked;
    obstacle->blocked = nullptr;
    changed = true;
    CHECK(obstacle_set_blocked(obstacle, 7, 18, true, &changed)
        == NAVSYS_STATUS_CORRUPT_STATE);
    CHECK(changed);
    obstacle->blocked = saved_blocked;

    obstacle_destroy(copied);
    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle blocked export is allocation-free and two-call") {
    obstacle_t* obstacle = nullptr;
    REQUIRE(obstacle_create_checked(0, 0, 4, 4, &obstacle)
        == NAVSYS_STATUS_OK);

    size_t count = 999;
    CHECK(obstacle_export_blocked(obstacle, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 0);

    bool changed = false;
    REQUIRE(obstacle_set_blocked(obstacle, 1, 2, true, &changed)
        == NAVSYS_STATUS_OK);
    REQUIRE(changed);
    REQUIRE(obstacle_set_blocked(obstacle, 3, 0, true, &changed)
        == NAVSYS_STATUS_OK);
    REQUIRE(changed);

    CHECK(obstacle_export_blocked(obstacle, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    REQUIRE(count == 2);

    coord_t short_buffer[1] = {{123, 456}};
    size_t required = 777;
    CHECK(obstacle_export_blocked(
        obstacle, short_buffer, 1, &required)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(required == 2);
    CHECK(short_buffer[0].x == 123);
    CHECK(short_buffer[0].y == 456);

    size_t preserved_count = 888;
    CHECK(obstacle_export_blocked(obstacle, nullptr, 1, &preserved_count)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved_count == 888);

    coord_t exact[2] = {};
    CHECK(obstacle_export_blocked(obstacle, exact, 2, &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == 2);
    const bool first_is_a = exact[0].x == 1 && exact[0].y == 2;
    const bool first_is_b = exact[0].x == 3 && exact[0].y == 0;
    const bool second_is_a = exact[1].x == 1 && exact[1].y == 2;
    const bool second_is_b = exact[1].x == 3 && exact[1].y == 0;
    const bool contains_both = (first_is_a && second_is_b)
        || (first_is_b && second_is_a);
    CHECK(contains_both);

    coord_hash_t* saved_blocked = obstacle->blocked;
    obstacle->blocked = nullptr;
    preserved_count = 999;
    CHECK(obstacle_export_blocked(obstacle, exact, 2, &preserved_count)
        == NAVSYS_STATUS_CORRUPT_STATE);
    CHECK(preserved_count == 999);
    obstacle->blocked = saved_blocked;

    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle core origin translation and resize preserve extent invariant") {
    obstacle_t* obstacle = obstacle_create_full(10, 20, 5, 5);
    REQUIRE(obstacle != nullptr);
    REQUIRE(obstacle_block_coord(obstacle, 11, 21));

    obstacle_set_origin(obstacle, 100, 200);
    CHECK_FALSE(obstacle_is_coord_blocked(obstacle, 11, 21));
    CHECK(obstacle_is_coord_blocked(obstacle, 101, 201));
    CHECK(obstacle_is_inside(obstacle, 101, 201));

    REQUIRE(obstacle_block_coord(obstacle, 100, 200));
    REQUIRE(obstacle_block_coord(obstacle, 104, 204));
    obstacle_set_width(obstacle, 2);
    CHECK(obstacle_is_coord_blocked(obstacle, 100, 200));
    CHECK(obstacle_is_coord_blocked(obstacle, 101, 201));
    CHECK_FALSE(obstacle_is_coord_blocked(obstacle, 104, 204));

    obstacle_set_height(obstacle, 1);
    CHECK(obstacle_is_coord_blocked(obstacle, 100, 200));
    CHECK_FALSE(obstacle_is_coord_blocked(obstacle, 101, 201));
    CHECK(coord_hash_size(obstacle_get_blocked_coords(obstacle)) == 1);

    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle core uses overflow-safe normalized half-open extents") {
    obstacle_t* negative = obstacle_create_full(10, 20, -3, -2);
    REQUIRE(negative != nullptr);
    CHECK(obstacle_is_inside(negative, 7, 18));
    CHECK(obstacle_is_inside(negative, 9, 19));
    CHECK_FALSE(obstacle_is_inside(negative, 10, 19));
    CHECK_FALSE(obstacle_is_inside(negative, 9, 20));
    CHECK(obstacle_block_coord(negative, 7, 18));
    CHECK_FALSE(obstacle_block_coord(negative, 10, 19));
    obstacle_destroy(negative);

    obstacle_t* empty = obstacle_create_full(3, 4, 0, 7);
    REQUIRE(empty != nullptr);
    CHECK_FALSE(obstacle_is_inside(empty, 3, 4));
    CHECK_FALSE(obstacle_block_coord(empty, 3, 4));
    obstacle_destroy(empty);

    const int max_value = std::numeric_limits<int>::max();
    obstacle_t* high = obstacle_create_full(max_value - 1, 0, 3, 1);
    REQUIRE(high != nullptr);
    CHECK(obstacle_is_inside(high, max_value - 1, 0));
    CHECK(obstacle_is_inside(high, max_value, 0));
    CHECK(obstacle_block_coord(high, max_value, 0));
    obstacle_destroy(high);
}

TEST_CASE("obstacle origin translation overflow preserves the original object") {
    obstacle_t* obstacle = obstacle_create_full(0, 0, 3, 3);
    REQUIRE(obstacle != nullptr);
    REQUIRE(obstacle_block_coord(obstacle, 1, 1));
    obstacle_t* snapshot = obstacle_copy(obstacle);
    REQUIRE(snapshot != nullptr);

    obstacle_set_origin(obstacle, std::numeric_limits<int>::max(), 0);
    CHECK(obstacle_equal(obstacle, snapshot));
    CHECK(obstacle_hash(obstacle) == obstacle_hash(snapshot));
    CHECK(obstacle_is_coord_blocked(obstacle, 1, 1));

    obstacle_destroy(snapshot);
    obstacle_destroy(obstacle);
}

TEST_CASE("obstacle core copy is independent on successful allocation") {
    obstacle_t* source = obstacle_create_full(-2, -3, 7, 9);
    REQUIRE(source != nullptr);
    REQUIRE(obstacle_block_coord(source, 1, 2));
    obstacle_t* copied = obstacle_copy(source);
    REQUIRE(copied != nullptr);
    CHECK(obstacle_equal(source, copied));
    CHECK(obstacle_hash(source) == obstacle_hash(copied));

    REQUIRE(obstacle_block_coord(source, 2, 3));
    CHECK_FALSE(obstacle_equal(source, copied));
    CHECK_FALSE(obstacle_is_coord_blocked(copied, 2, 3));

    obstacle_destroy(copied);
    obstacle_destroy(source);
}

TEST_CASE("obstacle overlays preserve overlapping sources and base terrain") {
    obstacle_t* first = obstacle_create_full(0, 0, 8, 8);
    obstacle_t* second = obstacle_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(grid != nullptr);

    REQUIRE(obstacle_block_coord(first, 3, 4));
    REQUIRE(obstacle_block_coord(second, 3, 4));
    const navcell_t base{TERRAIN_TYPE_MOUNTAIN, 81};
    REQUIRE(navgrid_set_cell(grid, 3, 4, &base));

    obstacle_apply_to_navgrid(first, grid);
    obstacle_apply_to_navgrid(second, grid);
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    obstacle_remove_from_navgrid(first, grid);
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    obstacle_remove_from_navgrid(second, grid);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 3, 4, nullptr));

    navcell_t fetched{};
    REQUIRE(navgrid_fetch_cell(grid, 3, 4, &fetched) == 0);
    CHECK(fetched.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(fetched.height == 81);

    navgrid_destroy(grid);
    obstacle_destroy(second);
    obstacle_destroy(first);
}

TEST_CASE("obstacle checked overlays preserve provenance and diagnose owner mismatch") {
    obstacle_t* first = obstacle_create_full(0, 0, 8, 8);
    obstacle_t* second = obstacle_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    navgrid_t* other_grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(grid != nullptr);
    REQUIRE(other_grid != nullptr);
    REQUIRE(obstacle_block_coord(first, 3, 4));
    REQUIRE(obstacle_block_coord(first, 4, 4));
    REQUIRE(obstacle_block_coord(second, 3, 4));
    const navcell_t base{TERRAIN_TYPE_MOUNTAIN, 81};
    REQUIRE(navgrid_set_cell(grid, 3, 4, &base));

    obstacle_navgrid_overlay_token_t first_token{};
    obstacle_navgrid_overlay_token_t second_token{};
    size_t changed = 999;
    REQUIRE(obstacle_apply_to_navgrid_checked(
        first, grid, nullptr, &first_token, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 2);
    CHECK(first_token.struct_size == sizeof(first_token));
    CHECK(first_token.abi_version
        == OBSTACLE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION);
    REQUIRE(obstacle_apply_to_navgrid_checked(
        second, grid, nullptr, &second_token, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 0);

    obstacle_navgrid_overlay_token_t other_token{};
    REQUIRE(obstacle_apply_to_navgrid_checked(
        second, other_grid, nullptr, &other_token, &changed)
        == NAVSYS_STATUS_OK);
    const obstacle_navgrid_overlay_token_t preserved = second_token;
    changed = 777;
    CHECK(obstacle_remove_from_navgrid_checked(
        other_grid, &second_token, &changed) == NAVSYS_STATUS_INVALIDATED);
    CHECK(second_token.owner_cookie == preserved.owner_cookie);
    CHECK(second_token.overlay == preserved.overlay);
    CHECK(changed == 777);
    CHECK(is_coord_blocked_navgrid(other_grid, 3, 4, nullptr));

    REQUIRE(obstacle_remove_from_navgrid_checked(
        grid, &first_token, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK(first_token.owner_cookie == 0);
    CHECK(first_token.overlay == 0);
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 4, 4, nullptr));
    CHECK(obstacle_remove_from_navgrid_checked(
        grid, &first_token, &changed) == NAVSYS_STATUS_INVALIDATED);

    obstacle_navgrid_overlay_token_t bad_version = second_token;
    ++bad_version.abi_version;
    changed = 555;
    CHECK(obstacle_remove_from_navgrid_checked(
        grid, &bad_version, &changed) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(changed == 555);
    REQUIRE(obstacle_remove_from_navgrid_checked(
        grid, &second_token, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 3, 4, nullptr));

    navcell_t fetched{};
    REQUIRE(navgrid_fetch_cell(grid, 3, 4, &fetched) == 0);
    CHECK(fetched.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(fetched.height == 81);
    REQUIRE(obstacle_remove_from_navgrid_checked(
        other_grid, &other_token, &changed) == NAVSYS_STATUS_OK);

    navgrid_destroy(other_grid);
    navgrid_destroy(grid);
    obstacle_destroy(second);
    obstacle_destroy(first);
}

TEST_CASE("obstacle checked overlay cancellation and bounds are failure atomic") {
    obstacle_t* obstacle = obstacle_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_8, nullptr);
    REQUIRE(obstacle != nullptr);
    REQUIRE(grid != nullptr);
    REQUIRE(obstacle_block_coord(obstacle, 1, 1));
    REQUIRE(obstacle_block_coord(obstacle, 2, 2));

    obstacle_cancel_fixture_t fixture{0, 5};
    obstacle_navgrid_apply_options_t options{
        sizeof(obstacle_navgrid_apply_options_t),
        OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION,
        OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE,
        cancel_obstacle_overlay,
        &fixture
    };
    obstacle_navgrid_overlay_token_t token{31, 32, 33, 34};
    size_t changed = 77;
    CHECK(obstacle_apply_to_navgrid_checked(
        obstacle, grid, &options, &token, &changed)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(token.owner_cookie == 33);
    CHECK(token.overlay == 34);
    CHECK(changed == 77);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 1, 1, nullptr));
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 2, nullptr));

    options.cancel_func = throw_obstacle_overlay_cancel;
    CHECK(obstacle_apply_to_navgrid_checked(
        obstacle, grid, &options, &token, &changed)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(token.owner_cookie == 33);
    CHECK(changed == 77);

    options.cancel_func = nullptr;
    ++options.abi_version;
    CHECK(obstacle_apply_to_navgrid_checked(
        obstacle, grid, &options, &token, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);
    --options.abi_version;
    options.merge_policy = 99u;
    CHECK(obstacle_apply_to_navgrid_checked(
        obstacle, grid, &options, &token, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);

    REQUIRE(obstacle_block_coord(obstacle, 6, 6));
    options.merge_policy = OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE;
    CHECK(obstacle_apply_to_navgrid_checked(
        obstacle, grid, &options, &token, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(token.owner_cookie == 33);
    CHECK(changed == 77);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 1, 1, nullptr));

    navgrid_destroy(grid);
    obstacle_destroy(obstacle);
}

TEST_CASE("legacy obstacle generators preserve exact raster golden baseline") {
    SUBCASE("filled and random rectangles") {
        obstacle_t* filled = obstacle_make_rect_all_blocked(1, 2, 2, 2);
        check_obstacle_golden(
            filled, 1, 2, 2, 2,
            {{1, 2}, {1, 3}, {2, 2}, {2, 3}});
        obstacle_destroy(filled);

        CHECK(obstacle_make_rect_random_blocked(1, 2, 2, 2, 0.0f)
            == nullptr);

        obstacle_t* full = obstacle_make_rect_random_blocked(
            1, 2, 2, 2, 1.0f);
        check_obstacle_golden(
            full, 1, 2, 2, 2,
            {{1, 2}, {1, 3}, {2, 2}, {2, 3}});
        obstacle_destroy(full);

        obstacle_t* nan_ratio = obstacle_make_rect_random_blocked(
            1,
            2,
            2,
            2,
            std::numeric_limits<float>::quiet_NaN());
        check_obstacle_golden(nan_ratio, 1, 2, 2, 2, {});
        obstacle_destroy(nan_ratio);
    }

    SUBCASE("beam direction and endpoint legacy semantics") {
        coord_t start{0, 0};
        coord_t goal{3, 0};
        obstacle_t* forward = obstacle_make_beam(&start, &goal, 0);
        check_obstacle_golden(forward, 0, 0, 3, 0, {});
        obstacle_destroy(forward);

        obstacle_t* reverse = obstacle_make_beam(&goal, &start, 0);
        check_obstacle_golden(reverse, 3, 0, -3, 0, {});
        obstacle_destroy(reverse);

        coord_t vertical_goal{0, 3};
        obstacle_t* vertical = obstacle_make_beam(
            &start, &vertical_goal, 0);
        check_obstacle_golden(vertical, 0, 0, 0, 3, {});
        obstacle_destroy(vertical);

        coord_t same{2, 2};
        obstacle_t* zero_length = obstacle_make_beam(&same, &same, 0);
        check_obstacle_golden(zero_length, 2, 2, 0, 0, {});
        obstacle_destroy(zero_length);

        coord_t diagonal_goal{3, 3};
        obstacle_t* diagonal = obstacle_make_beam(
            &start, &diagonal_goal, 0);
        check_obstacle_golden(
            diagonal, 0, 0, 3, 3, {{1, 1}, {2, 2}});
        obstacle_destroy(diagonal);
    }

    SUBCASE("rectangular ring and minimum enclosure") {
        coord_t start{0, 0};
        coord_t goal{2, 2};
        obstacle_t* ring = obstacle_make_torus(&start, &goal, 1);
        check_obstacle_golden(
            ring, 0, 0, 3, 3,
            {{0, 0}, {0, 1}, {0, 2}, {1, 0},
             {1, 2}, {2, 0}, {2, 1}, {2, 2}});
        obstacle_destroy(ring);

        obstacle_t* enclosure = obstacle_make_enclosure(
            &start, &goal, 1, ENCLOSURE_OPEN_LEFT);
        check_obstacle_golden(
            enclosure, 0, 0, 3, 3,
            {{0, 0}, {0, 2}, {1, 0}, {1, 2},
             {2, 0}, {2, 1}, {2, 2}});
        obstacle_destroy(enclosure);
    }

    SUBCASE("cross and radius-clipped spiral") {
        coord_t center{0, 0};
        obstacle_t* cross = obstacle_make_cross(&center, 1, 0);
        check_obstacle_golden(
            cross, -1, -1, 3, 3,
            {{-1, 0}, {0, -1}, {0, 0}, {0, 1}, {1, 0}});
        obstacle_destroy(cross);

        obstacle_t* clockwise = obstacle_make_spiral(
            &center, 1, 1, 0, 0, SPIRAL_CLOCKWISE);
        check_obstacle_golden(
            clockwise, -1, -1, 3, 3,
            {{-1, -1}, {-1, 0}, {-1, 1}, {0, 0},
             {0, 1}, {1, 0}, {1, 1}});
        obstacle_destroy(clockwise);

        obstacle_t* counter_clockwise = obstacle_make_spiral(
            &center, 1, 1, 0, 0, SPIRAL_COUNTER_CLOCKWISE);
        check_obstacle_golden(
            counter_clockwise, -1, -1, 3, 3,
            {{-1, -1}, {0, -1}, {0, 0}, {0, 1},
             {1, -1}, {1, 0}, {1, 1}});
        obstacle_destroy(counter_clockwise);
    }

    SUBCASE("triangle fill outline and degenerate input") {
        coord_t a{0, 0};
        coord_t b{2, 0};
        coord_t c{0, 2};
        const std::initializer_list<obstacle_coord_pair_t> triangle = {
            {0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {2, 0}};

        obstacle_t* filled = obstacle_make_triangle(&a, &b, &c);
        check_obstacle_golden(filled, 0, 0, 3, 3, triangle);
        obstacle_destroy(filled);

        obstacle_t* outline = obstacle_make_triangle_torus(
            &a, &b, &c, 0);
        check_obstacle_golden(outline, 0, 0, 3, 3, triangle);
        obstacle_destroy(outline);

        coord_t collinear_b{1, 1};
        coord_t collinear_c{2, 2};
        obstacle_t* degenerate = obstacle_make_triangle(
            &a, &collinear_b, &collinear_c);
        check_obstacle_golden(degenerate, 0, 0, 3, 3, {});
        obstacle_destroy(degenerate);
    }

    SUBCASE("polygon fill boundary self-intersection and large origin") {
        coord_list_t* square = coord_list_create();
        REQUIRE(square != nullptr);
        coord_t point{0, 0};
        REQUIRE(coord_list_push_back(square, &point));
        point = {2, 0};
        REQUIRE(coord_list_push_back(square, &point));
        point = {2, 2};
        REQUIRE(coord_list_push_back(square, &point));
        point = {0, 2};
        REQUIRE(coord_list_push_back(square, &point));

        obstacle_t* filled = obstacle_make_polygon(square);
        check_obstacle_golden(
            filled, 0, 0, 3, 3,
            {{0, 0}, {0, 1}, {1, 0}, {1, 1}});
        obstacle_destroy(filled);

        obstacle_t* outline = obstacle_make_polygon_torus(square, 0);
        check_obstacle_golden(
            outline, 0, 0, 3, 3,
            {{0, 0}, {0, 1}, {0, 2}, {1, 0},
             {1, 2}, {2, 0}, {2, 1}, {2, 2}});
        obstacle_destroy(outline);
        coord_list_destroy(square);

        coord_list_t* bowtie = coord_list_create();
        REQUIRE(bowtie != nullptr);
        point = {0, 0};
        REQUIRE(coord_list_push_back(bowtie, &point));
        point = {2, 2};
        REQUIRE(coord_list_push_back(bowtie, &point));
        point = {0, 2};
        REQUIRE(coord_list_push_back(bowtie, &point));
        point = {2, 0};
        REQUIRE(coord_list_push_back(bowtie, &point));
        obstacle_t* self_intersecting = obstacle_make_polygon(bowtie);
        check_obstacle_golden(
            self_intersecting, 0, 0, 3, 3, {{0, 0}, {1, 0}});
        obstacle_destroy(self_intersecting);
        coord_list_destroy(bowtie);

        coord_list_t* large = coord_list_create();
        REQUIRE(large != nullptr);
        point = {1000000000, 1000000000};
        REQUIRE(coord_list_push_back(large, &point));
        point = {1000000002, 1000000000};
        REQUIRE(coord_list_push_back(large, &point));
        point = {1000000002, 1000000002};
        REQUIRE(coord_list_push_back(large, &point));
        point = {1000000000, 1000000002};
        REQUIRE(coord_list_push_back(large, &point));
        obstacle_t* large_filled = obstacle_make_polygon(large);
        check_obstacle_golden(
            large_filled, 1000000000, 1000000000, 3, 3, {});
        obstacle_destroy(large_filled);
        coord_list_destroy(large);
    }
}

TEST_CASE("obstacle_make_rect_all_blocked - full blocking") {
    obstacle_t* obs = obstacle_make_rect_all_blocked(10, 20, 5, 5);
    REQUIRE(obs != nullptr);
    CHECK(obstacle_get_width(obs) == 5);
    CHECK(obstacle_get_height(obs) == 5);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    CHECK(coord_hash_length(blocked) == 25);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);

}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.0f);
    REQUIRE(obs == nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 0);
    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.5") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.5f);
    REQUIRE(obs != nullptr);

    int blocked = coord_hash_length(obstacle_get_blocked_coords(obs));
    CHECK(blocked >= 3);     // Too low means incorrect random values
    CHECK(blocked <= 22);    // Too high means ratio issue

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);    

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 1.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 1.0f);
    REQUIRE(obs != nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 25);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_beam") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 0);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_beam power up") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 1);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_torus minimum size") {
    coord_t start{0, 0};
    coord_t goal{6, 6}; // width = 7, height = 7
    int thickness = 2;

    obstacle_t* obs = obstacle_make_torus(&start, &goal, thickness);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_torus too small should fail") {
    coord_t start{0, 0};
    coord_t goal{3, 3}; // width = 4, height = 4 < 2*thickness+1
    int thickness = 2;

    obstacle_t* obs = obstacle_make_torus(&start, &goal, thickness);
    REQUIRE(obs == nullptr);
}

TEST_CASE("obstacle_make_enclosure open LEFT") {
    coord_t start{0, 0};
    coord_t goal{6, 6};
    int thickness = 1;

    obstacle_t* obs = obstacle_make_enclosure(
        &start, &goal, thickness, ENCLOSURE_OPEN_LEFT);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_enclosure fully closed") {
    coord_t start{0, 0};
    coord_t goal{6, 6};
    int thickness = 1;

    obstacle_t* obs = obstacle_make_enclosure(
        &start, &goal, thickness, ENCLOSURE_OPEN_UNKNOWN);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_cross") {
    SUBCASE("center point only (length = 0, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 0, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("thin cross (length = 2, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 2, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("thick cross (length = 3, range = 1)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 3, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("invalid input (null center)") {
        obstacle_t* obs = obstacle_make_cross(nullptr, 3, 1);
        REQUIRE(obs == nullptr);
    }

    SUBCASE("invalid input (negative range)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 2, -1);
        REQUIRE(obs == nullptr);
    }

    SUBCASE("invalid input (negative length)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, -1, 1);
        REQUIRE(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_spiral direction") {
    coord_t center = {20, 20};
    int radius = 5;
    int turns = 8;
    int range = 0;
    int gap = 2;

    SUBCASE("clockwise spiral") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, range, gap, SPIRAL_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("counter-clockwise spiral") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, range, gap, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("counter-clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }
}

TEST_CASE("obstacle_make_triangle") {
    SUBCASE("basic triangle generate.") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("reverse triangllle") {
        coord_t a = {12, 10};
        coord_t b = {9, 15};
        coord_t c = {15, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("up_left diagonal triangle") {
        coord_t a = {10, 10};
        coord_t b = {15, 15};
        coord_t c = {10, 20};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("input is null that fail") {
        coord_t a = {10, 10};
        obstacle_t* obs = obstacle_make_triangle(&a, nullptr, nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_triangle_torus") {
    SUBCASE("basic outline torus, thickness = 0") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("fat outline torus, thickness = 1") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("more fat torus, thickness = 2") {
        coord_t a = {8, 8};
        coord_t b = {16, 9};
        coord_t c = {12, 16};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 2);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("fault input - thickness < 0") {
        coord_t a = {0, 0};
        coord_t b = {1, 0};
        coord_t c = {0, 1};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, -1);
        CHECK(obs == nullptr);
    }

    SUBCASE("fault input - NULL coord") {
        coord_t a = {0, 0};
        obstacle_t* obs = obstacle_make_triangle_torus(&a, nullptr, nullptr, 1);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon") {
    SUBCASE("generic pentagon polygon generate") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {10, 10};
        coord_list_push_back(list, &tmp);

        tmp = {15, 10};        
        coord_list_push_back(list, &tmp);

        tmp = {17, 15};        
        coord_list_push_back(list, &tmp);

        tmp = {12, 18};
        coord_list_push_back(list, &tmp);
        
        tmp = {8, 14};
        coord_list_push_back(list, &tmp);

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon(list);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("lack of input (2 point )") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);

        tmp = {1, 1};
        coord_list_push_back(list, &tmp);

        REQUIRE(coord_list_length(list) == 2);

        obstacle_t* obs = obstacle_make_polygon(list);
        CHECK(obs == nullptr);

        coord_list_destroy(list);
    }

    SUBCASE("input is NULL") {
        obstacle_t* obs = obstacle_make_polygon(nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon_torus") {
    SUBCASE("basic pentagon torus generate - thickness = 0") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {10, 10};
        coord_list_push_back(list, &tmp);

        tmp = {15, 10};
        coord_list_push_back(list, &tmp);

        tmp = {17, 15};
        coord_list_push_back(list, &tmp);

        tmp = {12, 18};
        coord_list_push_back(list, &tmp);

        tmp = {8, 14};
        coord_list_push_back(list, &tmp);        

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("thickness = 1 more fat") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {5, 5};
        coord_list_push_back(list, &tmp);

        tmp = {10, 5};
        coord_list_push_back(list, &tmp);

        tmp = {12, 10};
        coord_list_push_back(list, &tmp);

        tmp = {7, 13};
        coord_list_push_back(list, &tmp);

        tmp = {3, 9};
        coord_list_push_back(list, &tmp);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("lack of coord (2 point)") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);

        tmp = {1, 1};
        coord_list_push_back(list, &tmp);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        CHECK(obs == nullptr);
        coord_list_destroy(list);
    }

    SUBCASE("NULL list") {
        obstacle_t* obs = obstacle_make_polygon_torus(nullptr, 0);
        CHECK(obs == nullptr);
    }

    SUBCASE("minus thickness is fail") {
        coord_list_t* list = coord_list_create();
        
        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);
       
        tmp = {1, 0};
        coord_list_push_back(list, &tmp);
        
        tmp = {1, 1};
        coord_list_push_back(list, &tmp);        

        obstacle_t* obs = obstacle_make_polygon_torus(list, -1);
        CHECK(obs == nullptr);
        coord_list_destroy(list);
    }
}

TEST_CASE("obstacle_block_straight") {
    SUBCASE("range = 0, one line blocking") {
        obstacle_t* obs = obstacle_create_full(10, 10, 20, 20);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 15, 15, 25, 20, 0);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 1, block adjacent") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 20, 10, 1);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked); // 디버깅용

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid); // 두꺼운 직선 확인

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 2, vertical blocking") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 10, 5, 10, 20, 2);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 0, diagonal blocking") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 15, 15, 0);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }
}
