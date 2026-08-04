#include "doctest.h"

#include "navgrid.h"
#include "route_carver.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

struct carve_snapshot_t {
    int result;
    std::string footprint;
};

navgrid_t* make_fully_blocked_grid(
    int width, int height, navgrid_dir_mode_t mode) {
    navgrid_t* grid = navgrid_create_full(width, height, mode, nullptr);
    if (!grid) return nullptr;

    std::vector<coord_t> coords;
    coords.reserve(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) coords.push_back({x, y});
    }
    navgrid_overlay_id_t overlay = 0;
    std::size_t changed = 0;
    if (navgrid_apply_blocked_overlay(
            grid, coords.data(), coords.size(), &overlay, &changed)
            != NAVSYS_STATUS_OK
        || overlay == 0 || changed != coords.size()) {
        navgrid_destroy(grid);
        return nullptr;
    }
    return grid;
}

std::string snapshot_footprint(
    const navgrid_t* grid, int width, int height) {
    std::string result;
    for (int y = 0; y < height; ++y) {
        if (y != 0) result.push_back('/');
        for (int x = 0; x < width; ++x) {
            result.push_back(
                is_coord_blocked_navgrid(grid, x, y, nullptr) ? '#' : '.');
        }
    }
    return result;
}

carve_snapshot_t snapshot_beam(
    navgrid_dir_mode_t mode,
    int width,
    int height,
    coord_t start,
    coord_t goal,
    int range) {
    navgrid_t* grid = make_fully_blocked_grid(width, height, mode);
    REQUIRE(grid != nullptr);
    const int result = route_carve_beam(grid, &start, &goal, range);
    const std::string footprint = snapshot_footprint(grid, width, height);
    navgrid_destroy(grid);
    return {result, footprint};
}

carve_snapshot_t snapshot_bomb(
    navgrid_dir_mode_t mode,
    int width,
    int height,
    coord_t center,
    int range) {
    navgrid_t* grid = make_fully_blocked_grid(width, height, mode);
    REQUIRE(grid != nullptr);
    const int result = route_carve_bomb(grid, &center, range);
    const std::string footprint = snapshot_footprint(grid, width, height);
    navgrid_destroy(grid);
    return {result, footprint};
}

navgrid_carve_options_t canonical_options(
    std::uint32_t radius,
    navgrid_carve_metric_t metric,
    std::uint32_t flags) {
    return {
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        radius,
        static_cast<std::uint32_t>(metric),
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        flags | NAVGRID_CARVE_CLIP_TO_EXTENT | NAVGRID_CARVE_ATOMIC,
        0,
        1024,
        nullptr,
        nullptr
    };
}

carve_snapshot_t snapshot_canonical_line(
    navgrid_dir_mode_t mode,
    int width,
    int height,
    coord_t start,
    coord_t goal,
    std::uint32_t radius,
    navgrid_carve_metric_t metric,
    std::uint32_t endpoint_flags) {
    navgrid_t* grid = make_fully_blocked_grid(width, height, mode);
    REQUIRE(grid != nullptr);
    const navgrid_carve_options_t options = canonical_options(
        radius, metric, endpoint_flags);
    std::size_t changed = 0;
    REQUIRE(navgrid_carve_line(
        grid, &start, &goal, &options, &changed) == NAVSYS_STATUS_OK);
    const std::string footprint = snapshot_footprint(grid, width, height);
    navgrid_destroy(grid);
    return {static_cast<int>(changed), footprint};
}

carve_snapshot_t snapshot_canonical_area(
    navgrid_dir_mode_t mode,
    int width,
    int height,
    coord_t center,
    std::uint32_t radius,
    navgrid_carve_metric_t metric) {
    navgrid_t* grid = make_fully_blocked_grid(width, height, mode);
    REQUIRE(grid != nullptr);
    const navgrid_carve_options_t options = canonical_options(radius, metric, 0);
    std::size_t changed = 0;
    REQUIRE(navgrid_carve_area(grid, &center, &options, &changed)
        == NAVSYS_STATUS_OK);
    const std::string footprint = snapshot_footprint(grid, width, height);
    navgrid_destroy(grid);
    return {static_cast<int>(changed), footprint};
}

void check_snapshot(
    const carve_snapshot_t& actual,
    int expected_result,
    const char* expected_footprint) {
    CHECK(actual.result == expected_result);
    CHECK(actual.footprint == expected_footprint);
}

} // namespace

TEST_CASE("route_carver legacy beam range and topology footprint is frozen") {
    const coord_t start{2, 2};
    const coord_t goal{6, 4};
    const char* thin =
        "#########/#########/#########/###.#####/####...##/"
        "#########/#########/#########/#########";

    for (const navgrid_dir_mode_t mode : {NAVGRID_DIR_4, NAVGRID_DIR_8}) {
        check_snapshot(snapshot_beam(mode, 9, 9, start, goal, -1), 4, thin);
        check_snapshot(snapshot_beam(mode, 9, 9, start, goal, 0), 4, thin);
    }

    check_snapshot(
        snapshot_beam(NAVGRID_DIR_4, 9, 9, start, goal, 1), 13,
        "#########/#########/###.#####/##.#...##/###.....#/"
        "####...##/#########/#########/#########");
    check_snapshot(
        snapshot_beam(NAVGRID_DIR_4, 9, 9, start, goal, 2), 38,
        "#########/##...####/#.......#/#......../#......../"
        "##......./###.....#/#########/#########");
    check_snapshot(
        snapshot_beam(NAVGRID_DIR_8, 9, 9, start, goal, 1), 20,
        "#########/#########/##...####/##......#/##......#/"
        "###.....#/#########/#########/#########");
    check_snapshot(
        snapshot_beam(NAVGRID_DIR_8, 9, 9, start, goal, 2), 44,
        "#########/#.....###/#......../#......../#......../"
        "#......../##......./#########/#########");
}

TEST_CASE("route_carver legacy bomb range and topology footprint is frozen") {
    const coord_t center{4, 4};
    const char* point =
        "#########/#########/#########/#########/####.####/"
        "#########/#########/#########/#########";

    for (const navgrid_dir_mode_t mode : {NAVGRID_DIR_4, NAVGRID_DIR_8}) {
        check_snapshot(snapshot_bomb(mode, 9, 9, center, -1), 1, point);
        check_snapshot(snapshot_bomb(mode, 9, 9, center, 0), 1, point);
    }

    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_4, 9, 9, center, 1), 5,
        "#########/#########/#########/####.####/###...###/"
        "####.####/#########/#########/#########");
    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_4, 9, 9, center, 2), 21,
        "#########/#########/###...###/##.....##/##.....##/"
        "##.....##/###...###/#########/#########");
    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_8, 9, 9, center, 1), 9,
        "#########/#########/#########/###...###/###...###/"
        "###...###/#########/#########/#########");
    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_8, 9, 9, center, 2), 25,
        "#########/#########/##.....##/##.....##/##.....##/"
        "##.....##/##.....##/#########/#########");
}

TEST_CASE("route_carver legacy endpoint and bounded clipping behavior is frozen") {
    check_snapshot(
        snapshot_beam(
            NAVGRID_DIR_4, 5, 5, {-1, 2}, {2, 2}, 0),
        3, "#####/#####/...##/#####/#####");
    check_snapshot(
        snapshot_beam(
            NAVGRID_DIR_4, 5, 5, {2, 2}, {5, 2}, 0),
        2, "#####/#####/###../#####/#####");
    check_snapshot(
        snapshot_beam(
            NAVGRID_DIR_8, 5, 5, {0, 0}, {2, 0}, 1),
        8, "....#/....#/#####/#####/#####");
    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_8, 5, 5, {0, 0}, 1),
        4, "..###/..###/#####/#####/#####");
    check_snapshot(
        snapshot_bomb(NAVGRID_DIR_8, 5, 5, {-1, 0}, 1),
        0, "#####/#####/#####/#####/#####");
}

TEST_CASE("route_carver legacy invalid inputs collapse to zero") {
    const coord_t coord{0, 0};
    CHECK(route_carve_beam(nullptr, &coord, &coord, 0) == 0);
    CHECK(route_carve_beam(nullptr, nullptr, nullptr, 0) == 0);
    CHECK(route_carve_bomb(nullptr, &coord, 0) == 0);
    CHECK(route_carve_bomb(nullptr, nullptr, 0) == 0);
}

TEST_CASE("route_carver migration keeps only proven equivalent profiles") {
    const carve_snapshot_t legacy_horizontal = snapshot_beam(
        NAVGRID_DIR_8, 5, 3, {0, 1}, {2, 1}, 0);
    const carve_snapshot_t canonical_horizontal = snapshot_canonical_line(
        NAVGRID_DIR_8,
        5,
        3,
        {0, 1},
        {2, 1},
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_CARVE_INCLUDE_END);
    CHECK(legacy_horizontal.result == canonical_horizontal.result);
    CHECK(legacy_horizontal.footprint == canonical_horizontal.footprint);

    const carve_snapshot_t legacy_diagonal = snapshot_beam(
        NAVGRID_DIR_8, 9, 9, {2, 2}, {6, 4}, 0);
    const carve_snapshot_t canonical_diagonal = snapshot_canonical_line(
        NAVGRID_DIR_8,
        9,
        9,
        {2, 2},
        {6, 4},
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_CARVE_INCLUDE_END);
    check_snapshot(
        legacy_diagonal, 4,
        "#########/#########/#########/###.#####/####...##/"
        "#########/#########/#########/#########");
    check_snapshot(
        canonical_diagonal, 6,
        "#########/#########/###.#####/###...###/#####..##/"
        "#########/#########/#########/#########");

    const carve_snapshot_t legacy_square = snapshot_bomb(
        NAVGRID_DIR_8, 9, 9, {4, 4}, 2);
    const carve_snapshot_t canonical_square = snapshot_canonical_area(
        NAVGRID_DIR_8,
        9,
        9,
        {4, 4},
        2,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE);
    CHECK(legacy_square.result == canonical_square.result);
    CHECK(legacy_square.footprint == canonical_square.footprint);

    const carve_snapshot_t legacy_dir4 = snapshot_bomb(
        NAVGRID_DIR_4, 9, 9, {4, 4}, 2);
    const carve_snapshot_t canonical_diamond = snapshot_canonical_area(
        NAVGRID_DIR_4,
        9,
        9,
        {4, 4},
        2,
        NAVGRID_CARVE_MANHATTAN_DIAMOND);
    check_snapshot(
        legacy_dir4, 21,
        "#########/#########/###...###/##.....##/##.....##/"
        "##.....##/###...###/#########/#########");
    check_snapshot(
        canonical_diamond, 13,
        "#########/#########/####.####/###...###/##.....##/"
        "###...###/####.####/#########/#########");
}
