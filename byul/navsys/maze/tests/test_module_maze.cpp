#include "doctest.h"
#include <locale.h>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <vector>

extern "C" {
#include "maze.h"
#include "maze_binary.h"
#include "maze_eller.h"
#include "maze_hunt_and_kill.h"
#include "maze_kruskal.h"
#include "maze_sidewinder.h"
#include "console.h"
#include "obstacle_core.h"

}

#ifndef BYUL_AGGREGATE_TEST
#include "internal/maze_private.hpp"
#endif

namespace {

struct maze_cancel_fixture_t {
    int calls;
    int cancel_after;
};

bool cancel_maze_overlay(void* userdata) {
    auto* fixture = static_cast<maze_cancel_fixture_t*>(userdata);
    ++fixture->calls;
    return fixture->calls >= fixture->cancel_after;
}

bool throw_maze_overlay_cancel(void*) {
    throw std::runtime_error("maze overlay cancellation failure");
}

#ifndef BYUL_AGGREGATE_TEST
bool cancel_maze_generation(void*) {
    return true;
}

bool throw_maze_generation_cancel(void*) {
    throw std::runtime_error("maze generation cancellation failure");
}

using maze_generator_t = navsys_status_t (*)(
    int32_t,
    int32_t,
    uint32_t,
    uint32_t,
    byul_maze_generation_context&,
    maze_t**) noexcept;
#endif

struct maze_topology_t {
    bool queries_ok = true;
    bool border_blocked = true;
    bool logical_cells_open = true;
    bool connected = false;
    size_t node_count = 0;
    size_t edge_count = 0;
};

maze_topology_t analyze_logical_topology(
    const maze_t* maze,
    int32_t origin_x,
    int32_t origin_y,
    int width,
    int height) {
    maze_topology_t result;
    auto blocked = [&](int x, int y) {
        bool value = false;
        if (byul_maze_is_blocked(
                maze, origin_x + x, origin_y + y, &value)
            != NAVSYS_STATUS_OK) {
            result.queries_ok = false;
            return true;
        }
        return value;
    };

    for (int x = 0; x < width; ++x) {
        result.border_blocked = result.border_blocked
            && blocked(x, 0) && blocked(x, height - 1);
    }
    for (int y = 0; y < height; ++y) {
        result.border_blocked = result.border_blocked
            && blocked(0, y) && blocked(width - 1, y);
    }

    const int columns = (width - 1) / 2;
    const int rows = (height - 1) / 2;
    result.node_count = static_cast<size_t>(columns) * rows;
    std::vector<uint8_t> reached(result.node_count, uint8_t{0});
    std::queue<int> pending;
    if (result.node_count != 0 && !blocked(1, 1)) {
        reached[0] = 1;
        pending.push(0);
    }

    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const int x = 1 + column * 2;
            const int y = 1 + row * 2;
            result.logical_cells_open =
                result.logical_cells_open && !blocked(x, y);
            if (column + 1 < columns && !blocked(x + 1, y)) {
                ++result.edge_count;
            }
            if (row + 1 < rows && !blocked(x, y + 1)) {
                ++result.edge_count;
            }
        }
    }

    static constexpr int delta_column[4] = {0, 0, -1, 1};
    static constexpr int delta_row[4] = {-1, 1, 0, 0};
    while (!pending.empty()) {
        const int node = pending.front();
        pending.pop();
        const int column = node % columns;
        const int row = node / columns;
        const int x = 1 + column * 2;
        const int y = 1 + row * 2;
        for (int direction = 0; direction < 4; ++direction) {
            const int next_column = column + delta_column[direction];
            const int next_row = row + delta_row[direction];
            if (next_column < 0 || next_row < 0
                || next_column >= columns || next_row >= rows) {
                continue;
            }
            const int midpoint_x = x + delta_column[direction];
            const int midpoint_y = y + delta_row[direction];
            if (blocked(midpoint_x, midpoint_y)) continue;
            const int next = next_row * columns + next_column;
            if (reached[next] != 0) continue;
            reached[next] = 1;
            pending.push(next);
        }
    }
    size_t reached_count = 0;
    for (const uint8_t value : reached) reached_count += value != 0 ? 1u : 0u;
    result.connected = reached_count == result.node_count;
    return result;
}

} // namespace

TEST_CASE("maze dispatcher preserves its ABI-1 enum and Kruskal fallback") {
    static_assert(MAZE_TYPE_RECURSIVE == 0);
    static_assert(MAZE_TYPE_PRIM == 1);
    static_assert(MAZE_TYPE_BINARY == 2);
    static_assert(MAZE_TYPE_ELLER == 3);
    static_assert(MAZE_TYPE_ALDOUS_BRODER == 4);
    static_assert(MAZE_TYPE_WILSON == 5);
    static_assert(MAZE_TYPE_HUNT_AND_KILL == 6);
    static_assert(MAZE_TYPE_SIDEWINDER == 7);
    static_assert(MAZE_TYPE_RECURSIVE_DIVISION == 8);
    static_assert(MAZE_TYPE_KRUSKAL == 9);
    static_assert(MAZE_TYPE_ROOM_BLEND == 10);
    static_assert(sizeof(maze_type_t) == 4);
    static_assert(std::is_same_v<
        decltype(&maze_make),
        maze_t* (*)(int, int, int, int, maze_type_t)>);

    maze_t* expected = maze_make_kruskal(7, -3, 3, 3);
    REQUIRE(expected != nullptr);
    for (const int invalid : {-1, 11, std::numeric_limits<int>::max()}) {
        maze_t* actual = maze_make(
            7, -3, 3, 3, static_cast<maze_type_t>(invalid));
        REQUIRE(actual != nullptr);
        CHECK(maze_equal(actual, expected));
        CHECK(maze_hash(actual) == maze_hash(expected));
        maze_destroy(actual);
    }
    maze_destroy(expected);
}

TEST_CASE("Binary Tree checked API preserves explicit bias lattices") {
    static_assert(BYUL_MAZE_BINARY_BIAS_NORTH_WEST == 0);
    static_assert(BYUL_MAZE_BINARY_BIAS_NORTH_EAST == 1);
    static_assert(BYUL_MAZE_BINARY_BIAS_SOUTH_WEST == 2);
    static_assert(BYUL_MAZE_BINARY_BIAS_SOUTH_EAST == 3);
    static_assert(sizeof(byul_maze_binary_bias_t) == 4);

    struct bias_case_t {
        byul_maze_binary_bias_t bias;
        int open_midpoints[3][2];
        uint32_t expected_hash;
    };
    const bias_case_t cases[] = {
        {BYUL_MAZE_BINARY_BIAS_NORTH_WEST,
            {{2, 1}, {1, 2}, {2, 3}}, UINT32_C(470646451)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_EAST,
            {{2, 1}, {2, 3}, {3, 2}}, UINT32_C(499477593)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_WEST,
            {{2, 1}, {1, 2}, {2, 3}}, UINT32_C(470646451)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_EAST,
            {{2, 1}, {2, 3}, {3, 2}}, UINT32_C(499477593)}
    };
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(1),
        UINT64_C(1000),
        UINT64_C(25),
        nullptr,
        nullptr
    };

    for (const bias_case_t& fixture : cases) {
        CAPTURE(static_cast<int>(fixture.bias));
        bool supported = false;
        REQUIRE(byul_maze_binary_bias_is_supported(
            fixture.bias, &supported) == NAVSYS_STATUS_OK);
        CHECK(supported);

        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_binary_tree(
            -2, 7, 5, 5, fixture.bias, &options, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == fixture.expected_hash);
        const maze_topology_t topology =
            analyze_logical_topology(maze, -2, 7, 5, 5);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.node_count == 4);
        CHECK(topology.edge_count == 3);

        for (int y = 1; y < 4; ++y) {
            for (int x = 1; x < 4; ++x) {
                if ((x & 1) == (y & 1)) continue;
                bool expected_open = false;
                for (const auto& midpoint : fixture.open_midpoints) {
                    expected_open = expected_open
                        || (x == midpoint[0] && y == midpoint[1]);
                }
                bool blocked = true;
                REQUIRE(byul_maze_is_blocked(
                    maze, -2 + x, 7 + y, &blocked) == NAVSYS_STATUS_OK);
                CHECK(blocked == !expected_open);
            }
        }
        maze_destroy(maze);

        maze = nullptr;
        REQUIRE(byul_maze_generate_binary_tree(
            0, 0, 3, 3, fixture.bias, &options, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == UINT32_C(663082931));
        maze_destroy(maze);
    }
}

TEST_CASE("Binary Tree checked API rejects invalid dimensions and bias") {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(1000),
        UINT64_C(0),
        nullptr,
        nullptr
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_binary_tree(
        0, 0, 2, 3, BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_binary_tree(
        0, 0, 4, 5, BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_binary_tree(
        std::numeric_limits<int32_t>::max(), 0, 3, 3,
        BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_binary_tree(
        0, 0, 3, 3, static_cast<byul_maze_binary_bias_t>(4),
        &options, &output) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);
    bool supported = true;
    CHECK(byul_maze_binary_bias_is_supported(
        static_cast<byul_maze_binary_bias_t>(-1), &supported)
        == NAVSYS_STATUS_UNSUPPORTED);

    CHECK(maze_make_binary(0, 0, 2, 3) == nullptr);
    CHECK(maze_make_binary(0, 0, 4, 5) == nullptr);
}

TEST_CASE("Binary Tree bias corpus preserves topology and root corridors") {
    struct hash_case_t {
        byul_maze_binary_bias_t bias;
        uint64_t seed;
        uint32_t expected_hash;
    };
    const hash_case_t hashes[] = {
        {BYUL_MAZE_BINARY_BIAS_NORTH_WEST, UINT64_C(0), UINT32_C(314326785)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_WEST, UINT64_C(1), UINT32_C(333622325)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_WEST, UINT64_C(17), UINT32_C(637696577)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_WEST, UINT64_MAX, UINT32_C(456164093)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_EAST, UINT64_C(0), UINT32_C(19337297)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_EAST, UINT64_C(1), UINT32_C(481215691)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_EAST, UINT64_C(17), UINT32_C(212513867)},
        {BYUL_MAZE_BINARY_BIAS_NORTH_EAST, UINT64_MAX, UINT32_C(65445043)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_WEST, UINT64_C(0), UINT32_C(342668731)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_WEST, UINT64_C(1), UINT32_C(276670393)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_WEST, UINT64_C(17), UINT32_C(714964413)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_WEST, UINT64_MAX, UINT32_C(397381761)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, UINT64_C(0), UINT32_C(49710731)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, UINT64_C(1), UINT32_C(405786447)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, UINT64_C(17), UINT32_C(137609167)},
        {BYUL_MAZE_BINARY_BIAS_SOUTH_EAST, UINT64_MAX, UINT32_C(7968567)}
    };

    for (const hash_case_t& fixture : hashes) {
        CAPTURE(static_cast<int>(fixture.bias));
        CAPTURE(fixture.seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            fixture.seed,
            UINT64_C(16),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_binary_tree(
            -5, 8, 9, 9, fixture.bias, &options, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == fixture.expected_hash);
        const maze_topology_t topology =
            analyze_logical_topology(maze, -5, 8, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.node_count == 16);
        CHECK(topology.edge_count == 15);

        const bool north = fixture.bias == BYUL_MAZE_BINARY_BIAS_NORTH_WEST
            || fixture.bias == BYUL_MAZE_BINARY_BIAS_NORTH_EAST;
        const bool west = fixture.bias == BYUL_MAZE_BINARY_BIAS_NORTH_WEST
            || fixture.bias == BYUL_MAZE_BINARY_BIAS_SOUTH_WEST;
        const int corridor_y = north ? 1 : 7;
        const int corridor_x = west ? 1 : 7;
        for (int x = 2; x < 8; x += 2) {
            bool blocked = true;
            REQUIRE(byul_maze_is_blocked(
                maze, -5 + x, 8 + corridor_y, &blocked)
                == NAVSYS_STATUS_OK);
            CHECK_FALSE(blocked);
        }
        for (int y = 2; y < 8; y += 2) {
            bool blocked = true;
            REQUIRE(byul_maze_is_blocked(
                maze, -5 + corridor_x, 8 + y, &blocked)
                == NAVSYS_STATUS_OK);
            CHECK_FALSE(blocked);
        }
        maze_destroy(maze);
    }
}

TEST_CASE("Binary Tree two-choice draws remain deterministically unbiased") {
    constexpr uint64_t sample_count = UINT64_C(512);
    constexpr uint64_t choices_per_sample = UINT64_C(9);
    constexpr uint64_t minimum_horizontal =
        sample_count * choices_per_sample * UINT64_C(43) / UINT64_C(100);
    constexpr uint64_t maximum_horizontal =
        sample_count * choices_per_sample * UINT64_C(57) / UINT64_C(100);

    for (int bias_value = BYUL_MAZE_BINARY_BIAS_NORTH_WEST;
         bias_value <= BYUL_MAZE_BINARY_BIAS_SOUTH_EAST;
         ++bias_value) {
        const auto bias = static_cast<byul_maze_binary_bias_t>(bias_value);
        const bool east = bias == BYUL_MAZE_BINARY_BIAS_NORTH_EAST
            || bias == BYUL_MAZE_BINARY_BIAS_SOUTH_EAST;
        const bool south = bias == BYUL_MAZE_BINARY_BIAS_SOUTH_WEST
            || bias == BYUL_MAZE_BINARY_BIAS_SOUTH_EAST;
        const int horizontal_delta = east ? 1 : -1;
        const int vertical_delta = south ? 1 : -1;
        uint64_t horizontal_count = 0;

        for (uint64_t seed = 0; seed < sample_count; ++seed) {
            const byul_maze_generate_options_t options{
                sizeof(byul_maze_generate_options_t),
                BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
                seed,
                UINT64_C(16),
                UINT64_C(81),
                nullptr,
                nullptr
            };
            maze_t* maze = nullptr;
            REQUIRE(byul_maze_generate_binary_tree(
                0, 0, 9, 9, bias, &options, &maze) == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            for (int y = 1; y < 8; y += 2) {
                for (int x = 1; x < 8; x += 2) {
                    const bool both_choices = x + horizontal_delta * 2 > 0
                        && x + horizontal_delta * 2 < 9
                        && y + vertical_delta * 2 > 0
                        && y + vertical_delta * 2 < 9;
                    if (!both_choices) continue;
                    bool horizontal_blocked = true;
                    bool vertical_blocked = true;
                    REQUIRE(byul_maze_is_blocked(
                        maze, x + horizontal_delta, y, &horizontal_blocked)
                        == NAVSYS_STATUS_OK);
                    REQUIRE(byul_maze_is_blocked(
                        maze, x, y + vertical_delta, &vertical_blocked)
                        == NAVSYS_STATUS_OK);
                    REQUIRE(horizontal_blocked != vertical_blocked);
                    horizontal_count += horizontal_blocked ? 0u : 1u;
                }
            }
            maze_destroy(maze);
        }
        CAPTURE(bias_value);
        CAPTURE(horizontal_count);
        CHECK(horizontal_count >= minimum_horizontal);
        CHECK(horizontal_count <= maximum_horizontal);
    }
}

TEST_CASE("Binary Tree step and cancellation work scale with raster cells") {
    for (const uint32_t extent : {UINT32_C(9), UINT32_C(17), UINT32_C(33)}) {
        const uint64_t logical_axis = (extent - 1) / 2;
        const uint64_t logical_cells = logical_axis * logical_axis;
        maze_cancel_fixture_t poll_fixture{0, std::numeric_limits<int>::max()};
        byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            UINT64_C(23),
            logical_cells,
            static_cast<uint64_t>(extent) * extent,
            cancel_maze_overlay,
            &poll_fixture
        };
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_binary_tree(
            0, 0, extent, extent, BYUL_MAZE_BINARY_BIAS_SOUTH_EAST,
            &options, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(poll_fixture.calls
            == 1 + static_cast<int>(extent * extent + logical_cells));
        maze_destroy(maze);

        options.max_steps = logical_cells - 1;
        options.cancel_func = nullptr;
        options.cancel_userdata = nullptr;
        maze = reinterpret_cast<maze_t*>(uintptr_t{1});
        CHECK(byul_maze_generate_binary_tree(
            0, 0, extent, extent, BYUL_MAZE_BINARY_BIAS_SOUTH_EAST,
            &options, &maze) == NAVSYS_STATUS_LIMIT_REACHED);
        CHECK(maze == nullptr);
    }

    maze_cancel_fixture_t cancel_fixture{0, 83};
    const byul_maze_generate_options_t cancel_options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(23),
        UINT64_C(16),
        UINT64_C(81),
        cancel_maze_overlay,
        &cancel_fixture
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_binary_tree(
        0, 0, 9, 9, BYUL_MAZE_BINARY_BIAS_SOUTH_EAST,
        &cancel_options, &output) == NAVSYS_STATUS_CANCELLED);
    CHECK(output == nullptr);
    CHECK(cancel_fixture.calls == cancel_fixture.cancel_after);
}

TEST_CASE("maze checked dispatcher validates options and routes algorithms") {
    static_assert(BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER == 0);
    static_assert(BYUL_MAZE_ALGORITHM_ROOM_BLEND == 10);
    static_assert(std::is_same_v<
        decltype(&byul_maze_algorithm_is_supported),
        navsys_status_t (*)(byul_maze_algorithm_t, bool*)>);

    bool supported = true;
    CHECK(byul_maze_algorithm_is_supported(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL, &supported)
        == NAVSYS_STATUS_OK);
    CHECK(supported);
    CHECK(byul_maze_algorithm_is_supported(
        static_cast<byul_maze_algorithm_t>(-1), &supported)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(byul_maze_algorithm_is_supported(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(123),
        0,
        0,
        nullptr,
        nullptr
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        0, 0, 3, 3, &options, &output) == NAVSYS_STATUS_OK);
    REQUIRE(output != nullptr);
    maze_destroy(output);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    options.struct_size = sizeof(options) - 1;
    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        0, 0, 3, 3, &options, &output) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);
    options.struct_size = sizeof(options);

    ++options.abi_version;
    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        0, 0, 3, 3, &options, &output) == NAVSYS_STATUS_UNSUPPORTED);
    --options.abi_version;

    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        std::numeric_limits<int32_t>::max(), 0, 3, 3,
        &options, &output) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        0, 0, 4, 3, &options, &output) == NAVSYS_STATUS_UNSUPPORTED);

    options.max_cells = 8;
    CHECK(byul_maze_generate(
        BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL,
        0, 0, 3, 3, &options, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
}

TEST_CASE("public checked dispatcher executes every advertised algorithm") {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(20260804),
        0,
        0,
        nullptr,
        nullptr
    };
    for (int value = BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER;
         value <= BYUL_MAZE_ALGORITHM_ROOM_BLEND; ++value) {
        const auto algorithm = static_cast<byul_maze_algorithm_t>(value);
        CAPTURE(value);
        bool supported = false;
        REQUIRE(byul_maze_algorithm_is_supported(algorithm, &supported)
            == NAVSYS_STATUS_OK);
        REQUIRE(supported);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate(
            algorithm, -6, 4, 9, 9, &options, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        const maze_topology_t topology =
            analyze_logical_topology(maze, -6, 4, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        maze_destroy(maze);
    }
}

#ifndef BYUL_AGGREGATE_TEST
TEST_CASE("maze generation context fixes PCG32 replay and Kruskal limits") {
    byul_maze_generation_context rng(123, 0, nullptr, nullptr);
    const uint32_t expected[] = {
        UINT32_C(101485362),
        UINT32_C(1376678935),
        UINT32_C(696086775),
        UINT32_C(3567161484),
        UINT32_C(1719793513),
        UINT32_C(2216846136),
        UINT32_C(1965434202),
        UINT32_C(1660622145)
    };
    for (const uint32_t value : expected) CHECK(rng.next_u32() == value);

    byul_maze_generation_context first_context(123, 100, nullptr, nullptr);
    byul_maze_generation_context second_context(123, 100, nullptr, nullptr);
    maze_t* first = nullptr;
    maze_t* second = nullptr;
    REQUIRE(byul_maze_generate_kruskal_internal(
        -3, 7, 7, 7, first_context, &first) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_generate_kruskal_internal(
        -3, 7, 7, 7, second_context, &second) == NAVSYS_STATUS_OK);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    CHECK(maze_equal(first, second));
    CHECK(maze_hash(first) == maze_hash(second));
    maze_destroy(second);
    maze_destroy(first);

    byul_maze_generation_context limited_context(123, 1, nullptr, nullptr);
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_kruskal_internal(
        0, 0, 7, 7, limited_context, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
    CHECK(limited_context.steps() == 1);

    byul_maze_generation_context cancelled_context(
        123, 100, cancel_maze_generation, nullptr);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_kruskal_internal(
        0, 0, 7, 7, cancelled_context, &output)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(output == nullptr);

    byul_maze_generation_context callback_failure_context(
        123, 100, throw_maze_generation_cancel, nullptr);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_kruskal_internal(
        0, 0, 7, 7, callback_failure_context, &output)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(output == nullptr);
}

TEST_CASE("internal generators replay and honor step limits") {
    const maze_generator_t generators[] = {
        byul_maze_generate_recursive_internal,
        byul_maze_generate_prim_internal,
        byul_maze_generate_binary_internal,
        byul_maze_generate_eller_internal,
        byul_maze_generate_hunt_and_kill_internal,
        byul_maze_generate_sidewinder_internal,
        byul_maze_generate_recursive_division_internal,
        byul_maze_generate_aldous_broder_internal,
        byul_maze_generate_wilson_internal
    };

    for (const maze_generator_t generate : generators) {
        byul_maze_generation_context first_context(456, 1000, nullptr, nullptr);
        byul_maze_generation_context second_context(456, 1000, nullptr, nullptr);
        maze_t* first = nullptr;
        maze_t* second = nullptr;
        REQUIRE(generate(-4, 6, 7, 9, first_context, &first)
            == NAVSYS_STATUS_OK);
        REQUIRE(generate(-4, 6, 7, 9, second_context, &second)
            == NAVSYS_STATUS_OK);
        REQUIRE(first != nullptr);
        REQUIRE(second != nullptr);
        CHECK(maze_equal(first, second));
        CHECK(maze_hash(first) == maze_hash(second));
        maze_destroy(second);
        maze_destroy(first);

        byul_maze_generation_context limited_context(456, 1, nullptr, nullptr);
        maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
        CHECK(generate(0, 0, 7, 7, limited_context, &output)
            == NAVSYS_STATUS_LIMIT_REACHED);
        CHECK(output == nullptr);
        CHECK(limited_context.steps() == 1);
    }
}

TEST_CASE("generator seed corpus satisfies declared logical topology") {
    struct generator_case_t {
        const char* name;
        maze_generator_t generate;
        bool perfect;
        uint32_t seed_zero_hash;
    };
    const generator_case_t generators[] = {
        {"recursive", byul_maze_generate_recursive_internal,
            true, UINT32_C(303425655)},
        {"prim", byul_maze_generate_prim_internal,
            true, UINT32_C(857387639)},
        {"binary", byul_maze_generate_binary_internal,
            true, UINT32_C(49710731)},
        {"eller", byul_maze_generate_eller_internal,
            true, UINT32_C(789167229)},
        {"aldous-broder", byul_maze_generate_aldous_broder_internal,
            true, UINT32_C(744322881)},
        {"wilson", byul_maze_generate_wilson_internal,
            true, UINT32_C(424385079)},
        {"hunt-and-kill", byul_maze_generate_hunt_and_kill_internal,
            true, UINT32_C(26398801)},
        {"sidewinder", byul_maze_generate_sidewinder_internal,
            true, UINT32_C(915955447)},
        {"recursive-division",
            byul_maze_generate_recursive_division_internal,
            true, UINT32_C(669558005)},
        {"kruskal", byul_maze_generate_kruskal_internal,
            true, UINT32_C(73237245)},
        {"room-blend", byul_maze_generate_room_blend_internal,
            false, UINT32_C(453713525)}
    };
    const uint64_t seeds[] = {
        UINT64_C(0), UINT64_C(1), UINT64_C(2), UINT64_C(17),
        UINT64_C(123), UINT64_C(0xffffffffffffffff)
    };

    for (const generator_case_t& generator : generators) {
        CAPTURE(generator.name);
        for (const uint64_t seed : seeds) {
            CAPTURE(seed);
            byul_maze_generation_context context(
                seed, UINT64_C(1000000), nullptr, nullptr);
            maze_t* maze = nullptr;
            REQUIRE(generator.generate(-5, 8, 9, 9, context, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            if (seed == 0) CHECK(maze_hash(maze) == generator.seed_zero_hash);
            const maze_topology_t topology =
                analyze_logical_topology(maze, -5, 8, 9, 9);
            CHECK(topology.queries_ok);
            CHECK(topology.border_blocked);
            CHECK(topology.logical_cells_open);
            CHECK(topology.connected);
            if (generator.perfect) {
                CHECK(topology.edge_count + 1 == topology.node_count);
            } else {
                CHECK(topology.edge_count + 1 >= topology.node_count);
            }
            maze_destroy(maze);
        }
    }
}

TEST_CASE("Eller checked API validates options and failure atomicity") {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(16),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 2, 3, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 4, 5, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(
        std::numeric_limits<int32_t>::max(), 0, 3, 3, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 9, 9, nullptr, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(80);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 9, 9, &options, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(81);
    options.max_steps = UINT64_C(15);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 9, 9, &options, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    maze_cancel_fixture_t cancel_fixture{0, 83};
    options.max_steps = UINT64_C(16);
    options.cancel_func = cancel_maze_overlay;
    options.cancel_userdata = &cancel_fixture;
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_eller(0, 0, 9, 9, &options, &output)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(output == nullptr);
    CHECK(cancel_fixture.calls == cancel_fixture.cancel_after);

    CHECK(maze_make_eller(0, 0, 2, 3) == nullptr);
    CHECK(maze_make_eller(0, 0, 4, 5) == nullptr);
}

TEST_CASE("Eller checked API replays the corrected lattice") {
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(16),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    maze_t* first = nullptr;
    maze_t* second = nullptr;
    REQUIRE(byul_maze_generate_eller(
        -5, 8, 9, 9, &options, &first) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_generate_eller(
        -5, 8, 9, 9, &options, &second) == NAVSYS_STATUS_OK);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    CHECK(maze_hash(first) == UINT32_C(789167229));
    CHECK(maze_hash(first) == maze_hash(second));
    const maze_topology_t topology =
        analyze_logical_topology(first, -5, 8, 9, 9);
    CHECK(topology.queries_ok);
    CHECK(topology.border_blocked);
    CHECK(topology.logical_cells_open);
    CHECK(topology.connected);
    CHECK(topology.edge_count + 1 == topology.node_count);
    maze_destroy(second);
    maze_destroy(first);
}

TEST_CASE("Eller corrected lattice has stable tiny goldens") {
    struct fixture_t {
        uint32_t width;
        uint32_t height;
        uint32_t expected_hash;
    };
    const fixture_t fixtures[] = {
        {3, 3, UINT32_C(910439850)},
        {5, 5, UINT32_C(778966597)},
        {9, 9, UINT32_C(789167229)}
    };

    for (const fixture_t& fixture : fixtures) {
        CAPTURE(fixture.width);
        CAPTURE(fixture.height);
        byul_maze_generation_context context(
            UINT64_C(0), UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_eller_internal(
            -5, 8, fixture.width, fixture.height, context, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == fixture.expected_hash);

        const maze_topology_t topology = analyze_logical_topology(
            maze, -5, 8,
            static_cast<int>(fixture.width),
            static_cast<int>(fixture.height));
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Eller corrected lattice remains perfect across 100 seeds") {
    for (uint64_t seed = 0; seed < 100; ++seed) {
        CAPTURE(seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            seed,
            UINT64_C(16),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_eller(
            13, -21, 9, 9, &options, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);

        const maze_topology_t topology =
            analyze_logical_topology(maze, 13, -21, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Hunt-and-Kill phase contract has stable tiny goldens") {
    struct fixture_t {
        uint32_t width;
        uint32_t height;
        uint32_t expected_hash;
    };
    const fixture_t fixtures[] = {
        {3, 3, UINT32_C(910439850)},
        {5, 5, UINT32_C(874550531)},
        {9, 9, UINT32_C(26398801)}
    };

    for (const fixture_t& fixture : fixtures) {
        CAPTURE(fixture.width);
        CAPTURE(fixture.height);
        byul_maze_generation_context context(
            UINT64_C(0), UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_hunt_and_kill_internal(
            -5, 8, fixture.width, fixture.height, context, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == fixture.expected_hash);

        const maze_topology_t topology = analyze_logical_topology(
            maze, -5, 8,
            static_cast<int>(fixture.width),
            static_cast<int>(fixture.height));
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Hunt-and-Kill remains perfect across 100 seeds") {
    for (uint64_t seed = 0; seed < 100; ++seed) {
        CAPTURE(seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            seed,
            UINT64_C(1296),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_hunt_and_kill(
            13, -21, 9, 9, &options, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);

        const maze_topology_t topology =
            analyze_logical_topology(maze, 13, -21, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Hunt-and-Kill step count equals the logical cell count") {
    const uint32_t extents[][2] = {
        {3, 3}, {3, 31}, {31, 3}, {31, 31}
    };
    for (const auto& extent : extents) {
        CAPTURE(extent[0]);
        CAPTURE(extent[1]);
        const uint64_t logical_cells =
            static_cast<uint64_t>((extent[0] - 1) / 2)
            * ((extent[1] - 1) / 2);
        for (uint64_t seed = 0; seed < 32; ++seed) {
            CAPTURE(seed);
            byul_maze_generation_context context(
                seed, logical_cells, nullptr, nullptr);
            maze_t* maze = nullptr;
            REQUIRE(byul_maze_generate_hunt_and_kill_internal(
                7, -13, extent[0], extent[1], context, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            CHECK(context.steps() == logical_cells);
            maze_destroy(maze);
        }
    }
}

TEST_CASE("Hunt-and-Kill handles one-dimensional logical grids") {
    const int extents[][2] = {{3, 9}, {9, 3}};
    for (const auto& extent : extents) {
        CAPTURE(extent[0]);
        CAPTURE(extent[1]);
        byul_maze_generation_context context(
            UINT64_C(17), UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_hunt_and_kill_internal(
            -11, 23,
            static_cast<uint32_t>(extent[0]),
            static_cast<uint32_t>(extent[1]),
            context, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);

        const maze_topology_t topology = analyze_logical_topology(
            maze, -11, 23, extent[0], extent[1]);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Hunt-and-Kill checked API validates options and failure atomicity") {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(1296),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 2, 3, &options, &output) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 4, 5, &options, &output) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        std::numeric_limits<int32_t>::max(), 0, 3, 3, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        std::numeric_limits<int32_t>::min(), 0,
        UINT32_MAX, 3, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 9, 9, nullptr, &output) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(80);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 9, 9, &options, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(81);
    options.max_steps = UINT64_C(1);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 9, 9, &options, &output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    maze_cancel_fixture_t cancel_fixture{0, 83};
    options.max_steps = UINT64_C(1296);
    options.cancel_func = cancel_maze_overlay;
    options.cancel_userdata = &cancel_fixture;
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_hunt_and_kill(
        0, 0, 9, 9, &options, &output) == NAVSYS_STATUS_CANCELLED);
    CHECK(output == nullptr);
    CHECK(cancel_fixture.calls == cancel_fixture.cancel_after);

    CHECK(maze_make_hunt_and_kill(0, 0, 2, 3) == nullptr);
    CHECK(maze_make_hunt_and_kill(0, 0, 4, 5) == nullptr);
}

TEST_CASE("Hunt-and-Kill checked API replays and matches dispatcher") {
    const uint64_t seeds[] = {
        UINT64_C(0), UINT64_C(1), UINT64_C(17), UINT64_MAX
    };
    for (const uint64_t seed : seeds) {
        CAPTURE(seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            seed,
            UINT64_C(1296),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* direct = nullptr;
        maze_t* replay = nullptr;
        maze_t* dispatched = nullptr;
        REQUIRE(byul_maze_generate_hunt_and_kill(
            -5, 8, 9, 9, &options, &direct) == NAVSYS_STATUS_OK);
        REQUIRE(byul_maze_generate_hunt_and_kill(
            -5, 8, 9, 9, &options, &replay) == NAVSYS_STATUS_OK);
        REQUIRE(byul_maze_generate(
            BYUL_MAZE_ALGORITHM_HUNT_AND_KILL,
            -5, 8, 9, 9, &options, &dispatched) == NAVSYS_STATUS_OK);
        REQUIRE(direct != nullptr);
        REQUIRE(replay != nullptr);
        REQUIRE(dispatched != nullptr);
        if (seed == 0) CHECK(maze_hash(direct) == UINT32_C(26398801));
        CHECK(maze_hash(direct) == maze_hash(replay));
        CHECK(maze_hash(direct) == maze_hash(dispatched));
        maze_destroy(dispatched);
        maze_destroy(replay);
        maze_destroy(direct);
    }

    maze_t* legacy = maze_make_hunt_and_kill(-5, 8, 9, 9);
    REQUIRE(legacy != nullptr);
    const maze_topology_t topology =
        analyze_logical_topology(legacy, -5, 8, 9, 9);
    CHECK(topology.queries_ok);
    CHECK(topology.border_blocked);
    CHECK(topology.logical_cells_open);
    CHECK(topology.connected);
    CHECK(topology.edge_count + 1 == topology.node_count);
    maze_destroy(legacy);
}

TEST_CASE("Sidewinder corrected first-row contract has stable tiny goldens") {
    struct fixture_t {
        uint32_t width;
        uint32_t height;
        uint32_t expected_hash;
    };
    const fixture_t fixtures[] = {
        {3, 3, UINT32_C(910439850)},
        {5, 5, UINT32_C(778966597)},
        {9, 9, UINT32_C(915955447)}
    };

    for (const fixture_t& fixture : fixtures) {
        CAPTURE(fixture.width);
        CAPTURE(fixture.height);
        byul_maze_generation_context context(
            UINT64_C(0), UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_sidewinder_internal(
            -5, 8, fixture.width, fixture.height, context, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);
        CHECK(maze_hash(maze) == fixture.expected_hash);

        for (uint32_t x = 1; x + 1 < fixture.width; ++x) {
            bool blocked = true;
            REQUIRE(byul_maze_is_blocked(
                maze, -5 + static_cast<int32_t>(x), 9, &blocked)
                == NAVSYS_STATUS_OK);
            CHECK_FALSE(blocked);
        }
        const maze_topology_t topology = analyze_logical_topology(
            maze, -5, 8,
            static_cast<int>(fixture.width),
            static_cast<int>(fixture.height));
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Sidewinder corrected lattice remains perfect across 100 seeds") {
    for (uint64_t seed = 0; seed < 100; ++seed) {
        CAPTURE(seed);
        byul_maze_generation_context context(
            seed, UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_sidewinder_internal(
            13, -21, 9, 9, context, &maze) == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);

        for (int32_t x = 14; x < 21; ++x) {
            bool blocked = true;
            REQUIRE(byul_maze_is_blocked(maze, x, -20, &blocked)
                == NAVSYS_STATUS_OK);
            CHECK_FALSE(blocked);
        }
        const maze_topology_t topology =
            analyze_logical_topology(maze, 13, -21, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Sidewinder handles one-dimensional logical grids") {
    const uint32_t extents[][2] = {{3, 9}, {9, 3}};
    for (const auto& extent : extents) {
        CAPTURE(extent[0]);
        CAPTURE(extent[1]);
        byul_maze_generation_context context(
            UINT64_C(17), UINT64_C(1000000), nullptr, nullptr);
        maze_t* maze = nullptr;
        REQUIRE(byul_maze_generate_sidewinder_internal(
            -11, 23, extent[0], extent[1], context, &maze)
            == NAVSYS_STATUS_OK);
        REQUIRE(maze != nullptr);

        const maze_topology_t topology = analyze_logical_topology(
            maze, -11, 23,
            static_cast<int>(extent[0]),
            static_cast<int>(extent[1]));
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
        maze_destroy(maze);
    }
}

TEST_CASE("Sidewinder checked API exposes four reflection-related sweeps") {
    static_assert(BYUL_MAZE_SIDEWINDER_EAST_NORTH == 0);
    static_assert(BYUL_MAZE_SIDEWINDER_EAST_SOUTH == 1);
    static_assert(BYUL_MAZE_SIDEWINDER_WEST_NORTH == 2);
    static_assert(BYUL_MAZE_SIDEWINDER_WEST_SOUTH == 3);
    static_assert(sizeof(byul_maze_sidewinder_sweep_t) == 4);

    struct fixture_t {
        byul_maze_sidewinder_sweep_t sweep;
        uint32_t expected_hash;
    };
    const fixture_t fixtures[] = {
        {BYUL_MAZE_SIDEWINDER_EAST_NORTH, UINT32_C(915955447)},
        {BYUL_MAZE_SIDEWINDER_EAST_SOUTH, UINT32_C(907534649)},
        {BYUL_MAZE_SIDEWINDER_WEST_NORTH, UINT32_C(464412339)},
        {BYUL_MAZE_SIDEWINDER_WEST_SOUTH, UINT32_C(455990389)}
    };
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(1000000),
        UINT64_C(81),
        nullptr,
        nullptr
    };

    maze_t* mazes[4] = {nullptr, nullptr, nullptr, nullptr};
    for (size_t index = 0; index < 4; ++index) {
        CAPTURE(index);
        bool supported = false;
        REQUIRE(byul_maze_sidewinder_sweep_is_supported(
            fixtures[index].sweep, &supported) == NAVSYS_STATUS_OK);
        CHECK(supported);
        REQUIRE(byul_maze_generate_sidewinder(
            -5, 8, 9, 9, fixtures[index].sweep, &options, &mazes[index])
            == NAVSYS_STATUS_OK);
        REQUIRE(mazes[index] != nullptr);
        CHECK(maze_hash(mazes[index]) == fixtures[index].expected_hash);

        const maze_topology_t topology =
            analyze_logical_topology(mazes[index], -5, 8, 9, 9);
        CHECK(topology.queries_ok);
        CHECK(topology.border_blocked);
        CHECK(topology.logical_cells_open);
        CHECK(topology.connected);
        CHECK(topology.edge_count + 1 == topology.node_count);
    }

    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            bool east_north = true;
            bool east_south = true;
            bool west_north = true;
            bool west_south = true;
            REQUIRE(byul_maze_is_blocked(
                mazes[0], -5 + x, 8 + y, &east_north) == NAVSYS_STATUS_OK);
            REQUIRE(byul_maze_is_blocked(
                mazes[1], -5 + x, 8 + (8 - y), &east_south)
                == NAVSYS_STATUS_OK);
            REQUIRE(byul_maze_is_blocked(
                mazes[2], -5 + (8 - x), 8 + y, &west_north)
                == NAVSYS_STATUS_OK);
            REQUIRE(byul_maze_is_blocked(
                mazes[3], -5 + (8 - x), 8 + (8 - y), &west_south)
                == NAVSYS_STATUS_OK);
            CHECK(east_north == east_south);
            CHECK(east_north == west_north);
            CHECK(east_north == west_south);
        }
    }
    for (maze_t* maze : mazes) maze_destroy(maze);
}

TEST_CASE("Sidewinder all sweeps remain perfect across 100 seeds") {
    for (int sweep_value = BYUL_MAZE_SIDEWINDER_EAST_NORTH;
         sweep_value <= BYUL_MAZE_SIDEWINDER_WEST_SOUTH;
         ++sweep_value) {
        const auto sweep =
            static_cast<byul_maze_sidewinder_sweep_t>(sweep_value);
        const bool north = sweep == BYUL_MAZE_SIDEWINDER_EAST_NORTH
            || sweep == BYUL_MAZE_SIDEWINDER_WEST_NORTH;
        const int corridor_y = north ? 1 : 7;
        for (uint64_t seed = 0; seed < 100; ++seed) {
            CAPTURE(sweep_value);
            CAPTURE(seed);
            const byul_maze_generate_options_t options{
                sizeof(byul_maze_generate_options_t),
                BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
                seed,
                UINT64_C(1000000),
                UINT64_C(81),
                nullptr,
                nullptr
            };
            maze_t* maze = nullptr;
            REQUIRE(byul_maze_generate_sidewinder(
                13, -21, 9, 9, sweep, &options, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            for (int x = 1; x < 8; ++x) {
                bool blocked = true;
                REQUIRE(byul_maze_is_blocked(
                    maze, 13 + x, -21 + corridor_y, &blocked)
                    == NAVSYS_STATUS_OK);
                CHECK_FALSE(blocked);
            }
            const maze_topology_t topology =
                analyze_logical_topology(maze, 13, -21, 9, 9);
            CHECK(topology.queries_ok);
            CHECK(topology.border_blocked);
            CHECK(topology.logical_cells_open);
            CHECK(topology.connected);
            CHECK(topology.edge_count + 1 == topology.node_count);
            maze_destroy(maze);
        }
    }
}

TEST_CASE("Sidewinder run-close coin remains deterministically balanced") {
    constexpr uint64_t sample_count = UINT64_C(512);
    constexpr uint64_t decisions_per_sample = UINT64_C(9);
    constexpr uint64_t minimum_sweep_carves =
        sample_count * decisions_per_sample * UINT64_C(43) / UINT64_C(100);
    constexpr uint64_t maximum_sweep_carves =
        sample_count * decisions_per_sample * UINT64_C(57) / UINT64_C(100);

    for (int sweep_value = BYUL_MAZE_SIDEWINDER_EAST_NORTH;
         sweep_value <= BYUL_MAZE_SIDEWINDER_WEST_SOUTH;
         ++sweep_value) {
        const auto sweep =
            static_cast<byul_maze_sidewinder_sweep_t>(sweep_value);
        const bool north = sweep == BYUL_MAZE_SIDEWINDER_EAST_NORTH
            || sweep == BYUL_MAZE_SIDEWINDER_WEST_NORTH;
        uint64_t sweep_carves = 0;
        for (uint64_t seed = 0; seed < sample_count; ++seed) {
            const byul_maze_generate_options_t options{
                sizeof(byul_maze_generate_options_t),
                BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
                seed,
                UINT64_C(16),
                UINT64_C(81),
                nullptr,
                nullptr
            };
            maze_t* maze = nullptr;
            REQUIRE(byul_maze_generate_sidewinder(
                0, 0, 9, 9, sweep, &options, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            for (int row = 0; row < 3; ++row) {
                const int y = north ? 3 + row * 2 : 5 - row * 2;
                for (int x = 2; x < 8; x += 2) {
                    bool blocked = true;
                    REQUIRE(byul_maze_is_blocked(maze, x, y, &blocked)
                        == NAVSYS_STATUS_OK);
                    sweep_carves += blocked ? 0u : 1u;
                }
            }
            maze_destroy(maze);
        }
        CAPTURE(sweep_value);
        CAPTURE(sweep_carves);
        CHECK(sweep_carves >= minimum_sweep_carves);
        CHECK(sweep_carves <= maximum_sweep_carves);
    }
}

TEST_CASE("Sidewinder step work equals the logical cell count") {
    const uint32_t extents[][2] = {
        {3, 3}, {3, 31}, {31, 3}, {31, 31}
    };
    for (const auto& extent : extents) {
        const uint64_t logical_cells =
            static_cast<uint64_t>((extent[0] - 1) / 2)
            * ((extent[1] - 1) / 2);
        for (int sweep_value = BYUL_MAZE_SIDEWINDER_EAST_NORTH;
             sweep_value <= BYUL_MAZE_SIDEWINDER_WEST_SOUTH;
             ++sweep_value) {
            CAPTURE(extent[0]);
            CAPTURE(extent[1]);
            CAPTURE(sweep_value);
            byul_maze_generate_options_t options{
                sizeof(byul_maze_generate_options_t),
                BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
                UINT64_C(17),
                logical_cells,
                static_cast<uint64_t>(extent[0]) * extent[1],
                nullptr,
                nullptr
            };
            maze_t* maze = nullptr;
            REQUIRE(byul_maze_generate_sidewinder(
                7, -13, extent[0], extent[1],
                static_cast<byul_maze_sidewinder_sweep_t>(sweep_value),
                &options, &maze) == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            maze_destroy(maze);

            if (logical_cells <= 1) continue;
            options.max_steps = logical_cells - 1;
            maze = reinterpret_cast<maze_t*>(uintptr_t{1});
            CHECK(byul_maze_generate_sidewinder(
                7, -13, extent[0], extent[1],
                static_cast<byul_maze_sidewinder_sweep_t>(sweep_value),
                &options, &maze) == NAVSYS_STATUS_LIMIT_REACHED);
            CHECK(maze == nullptr);
        }
    }
}

TEST_CASE("Sidewinder checked API validates options and failure atomicity") {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(1000000),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 2, 3, BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 4, 5, BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        std::numeric_limits<int32_t>::max(), 0, 3, 3,
        BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        std::numeric_limits<int32_t>::min(), 0, UINT32_MAX, 3,
        BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 9, 9, static_cast<byul_maze_sidewinder_sweep_t>(4),
        &options, &output) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(output == nullptr);
    bool supported = true;
    CHECK(byul_maze_sidewinder_sweep_is_supported(
        static_cast<byul_maze_sidewinder_sweep_t>(-1), &supported)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK_FALSE(supported);
    CHECK(byul_maze_sidewinder_sweep_is_supported(
        BYUL_MAZE_SIDEWINDER_EAST_NORTH, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 9, 9, BYUL_MAZE_SIDEWINDER_EAST_NORTH, nullptr, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(80);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 9, 9, BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    options.max_cells = UINT64_C(81);
    options.max_steps = UINT64_C(15);
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 9, 9, BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);

    maze_cancel_fixture_t cancel_fixture{0, 83};
    options.max_steps = UINT64_C(1000000);
    options.cancel_func = cancel_maze_overlay;
    options.cancel_userdata = &cancel_fixture;
    output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_sidewinder(
        0, 0, 9, 9, BYUL_MAZE_SIDEWINDER_EAST_NORTH, &options, &output)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(output == nullptr);
    CHECK(cancel_fixture.calls == cancel_fixture.cancel_after);
}

TEST_CASE("Sidewinder checked EAST_NORTH preserves dispatcher output") {
    for (const uint64_t seed : {
             UINT64_C(0), UINT64_C(1), UINT64_C(17), UINT64_MAX}) {
        CAPTURE(seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            seed,
            UINT64_C(1000000),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* direct = nullptr;
        maze_t* dispatched = nullptr;
        REQUIRE(byul_maze_generate_sidewinder(
            -5, 8, 9, 9, BYUL_MAZE_SIDEWINDER_EAST_NORTH,
            &options, &direct) == NAVSYS_STATUS_OK);
        REQUIRE(byul_maze_generate(
            BYUL_MAZE_ALGORITHM_SIDEWINDER,
            -5, 8, 9, 9, &options, &dispatched) == NAVSYS_STATUS_OK);
        REQUIRE(direct != nullptr);
        REQUIRE(dispatched != nullptr);
        CHECK(maze_hash(direct) == maze_hash(dispatched));
        maze_destroy(dispatched);
        maze_destroy(direct);
    }
}

TEST_CASE("Eller direct dispatcher and legacy paths share topology") {
    const uint64_t seeds[] = {
        UINT64_C(0), UINT64_C(1), UINT64_C(17), UINT64_MAX
    };
    for (const uint64_t seed : seeds) {
        CAPTURE(seed);
        const byul_maze_generate_options_t options{
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            seed,
            UINT64_C(16),
            UINT64_C(81),
            nullptr,
            nullptr
        };
        maze_t* direct = nullptr;
        maze_t* dispatched = nullptr;
        REQUIRE(byul_maze_generate_eller(
            -5, 8, 9, 9, &options, &direct) == NAVSYS_STATUS_OK);
        REQUIRE(byul_maze_generate(
            BYUL_MAZE_ALGORITHM_ELLER,
            -5, 8, 9, 9, &options, &dispatched) == NAVSYS_STATUS_OK);
        REQUIRE(direct != nullptr);
        REQUIRE(dispatched != nullptr);
        CHECK(maze_hash(direct) == maze_hash(dispatched));
        maze_destroy(dispatched);
        maze_destroy(direct);
    }

    maze_t* legacy = maze_make_eller(-5, 8, 9, 9);
    REQUIRE(legacy != nullptr);
    const maze_topology_t topology =
        analyze_logical_topology(legacy, -5, 8, 9, 9);
    CHECK(topology.queries_ok);
    CHECK(topology.border_blocked);
    CHECK(topology.logical_cells_open);
    CHECK(topology.connected);
    CHECK(topology.edge_count + 1 == topology.node_count);
    maze_destroy(legacy);
}

TEST_CASE("all internal generators cancel without publishing partial output") {
    const maze_generator_t generators[] = {
        byul_maze_generate_recursive_internal,
        byul_maze_generate_prim_internal,
        byul_maze_generate_binary_internal,
        byul_maze_generate_eller_internal,
        byul_maze_generate_aldous_broder_internal,
        byul_maze_generate_wilson_internal,
        byul_maze_generate_hunt_and_kill_internal,
        byul_maze_generate_sidewinder_internal,
        byul_maze_generate_recursive_division_internal,
        byul_maze_generate_kruskal_internal,
        byul_maze_generate_room_blend_internal
    };

    for (const maze_generator_t generate : generators) {
        maze_cancel_fixture_t fixture{0, 3};
        byul_maze_generation_context cancelled_context(
            321, UINT64_C(1000000), cancel_maze_overlay, &fixture);
        maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
        CHECK(generate(2, -7, 9, 9, cancelled_context, &output)
            == NAVSYS_STATUS_CANCELLED);
        CHECK(output == nullptr);
        CHECK(fixture.calls == fixture.cancel_after);

        byul_maze_generation_context callback_failure_context(
            321, UINT64_C(1000000), throw_maze_generation_cancel, nullptr);
        output = reinterpret_cast<maze_t*>(uintptr_t{1});
        CHECK(generate(2, -7, 9, 9, callback_failure_context, &output)
            == NAVSYS_STATUS_CALLBACK_FAILED);
        CHECK(output == nullptr);
    }
}

TEST_CASE("Aldous-Broder and Wilson cover the 2x2 UST distribution") {
    const maze_generator_t generators[] = {
        byul_maze_generate_aldous_broder_internal,
        byul_maze_generate_wilson_internal
    };
    constexpr uint32_t sample_count = 4096;
    constexpr uint32_t minimum_bucket = 850;
    constexpr uint32_t maximum_bucket = 1200;

    for (const maze_generator_t generate : generators) {
        uint32_t missing_edge_counts[4] = {0, 0, 0, 0};
        for (uint64_t seed = 0; seed < sample_count; ++seed) {
            byul_maze_generation_context context(
                seed, UINT64_C(100000), nullptr, nullptr);
            maze_t* maze = nullptr;
            REQUIRE(generate(0, 0, 5, 5, context, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);

            const int edge_coordinates[4][2] = {
                {2, 1}, {2, 3}, {1, 2}, {3, 2}
            };
            uint32_t open_mask = 0;
            for (uint32_t edge = 0; edge < 4; ++edge) {
                bool blocked = true;
                REQUIRE(byul_maze_is_blocked(
                    maze,
                    edge_coordinates[edge][0],
                    edge_coordinates[edge][1],
                    &blocked) == NAVSYS_STATUS_OK);
                if (!blocked) open_mask |= UINT32_C(1) << edge;
            }
            const bool is_spanning_tree = open_mask == UINT32_C(7)
                || open_mask == UINT32_C(11)
                || open_mask == UINT32_C(13)
                || open_mask == UINT32_C(14);
            REQUIRE(is_spanning_tree);
            for (uint32_t edge = 0; edge < 4; ++edge) {
                if ((open_mask & (UINT32_C(1) << edge)) == 0) {
                    ++missing_edge_counts[edge];
                    break;
                }
            }
            maze_destroy(maze);
        }
        for (const uint32_t count : missing_edge_counts) {
            CHECK(count >= minimum_bucket);
            CHECK(count <= maximum_bucket);
        }
    }
}

TEST_CASE("contract default step budgets terminate a 64-seed corpus") {
    struct budget_case_t {
        maze_generator_t generate;
        uint64_t multiplier;
    };
    const budget_case_t generators[] = {
        {byul_maze_generate_recursive_internal, 8},
        {byul_maze_generate_prim_internal, 16},
        {byul_maze_generate_binary_internal, 4},
        {byul_maze_generate_eller_internal, 4},
        {byul_maze_generate_aldous_broder_internal, 256},
        {byul_maze_generate_wilson_internal, 256},
        {byul_maze_generate_hunt_and_kill_internal, 16},
        {byul_maze_generate_sidewinder_internal, 4},
        {byul_maze_generate_recursive_division_internal, 4},
        {byul_maze_generate_kruskal_internal, 8},
        {byul_maze_generate_room_blend_internal, 32}
    };

    for (const budget_case_t& generator : generators) {
        for (uint64_t seed = 0; seed < 64; ++seed) {
            CAPTURE(seed);
            byul_maze_generation_context context(
                seed, UINT64_C(81) * generator.multiplier, nullptr, nullptr);
            maze_t* maze = nullptr;
            REQUIRE(generator.generate(3, -4, 9, 9, context, &maze)
                == NAVSYS_STATUS_OK);
            REQUIRE(maze != nullptr);
            maze_destroy(maze);
        }
    }
}

TEST_CASE("Room Blend replays at its minimum extent and honors step limits") {
    byul_maze_generation_context first_context(789, 10000, nullptr, nullptr);
    byul_maze_generation_context second_context(789, 10000, nullptr, nullptr);
    maze_t* first = nullptr;
    maze_t* second = nullptr;
    REQUIRE(byul_maze_generate_room_blend_internal(
        -2, 5, 9, 9, first_context, &first) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_generate_room_blend_internal(
        -2, 5, 9, 9, second_context, &second) == NAVSYS_STATUS_OK);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    CHECK(maze_equal(first, second));
    CHECK(maze_hash(first) == maze_hash(second));
    maze_destroy(second);
    maze_destroy(first);

    byul_maze_generation_context limited_context(789, 1, nullptr, nullptr);
    maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_generate_room_blend_internal(
        0, 0, 9, 9, limited_context, &output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(output == nullptr);
    CHECK(limited_context.steps() == 1);
}
#endif

TEST_CASE("maze ABI gate accepts canonical and compatibility fingerprints") {
    CHECK(byul_maze_get_abi_version() == BYUL_MAZE_ABI_VERSION);
    CHECK(byul_maze_get_abi_fingerprint() == BYUL_MAZE_ABI_FINGERPRINT);
    CHECK(byul_maze_sizeof() == (sizeof(void*) == 8 ? 24 : 20));
    CHECK(byul_maze_alignof() == alignof(void*));

    byul_maze_abi_mismatch_t mismatch = BYUL_MAZE_ABI_VERSION_MISMATCH;
    REQUIRE(byul_maze_check_abi(
        BYUL_MAZE_ABI_VERSION, BYUL_MAZE_ABI_FINGERPRINT, &mismatch)
        == NAVSYS_STATUS_OK);
    CHECK(mismatch == BYUL_MAZE_ABI_MATCH);
    REQUIRE(byul_maze_check_abi(
        1, UINT64_C(0x4d415a4501000018), &mismatch) == NAVSYS_STATUS_OK);
    CHECK(mismatch == BYUL_MAZE_ABI_MATCH);

    CHECK(byul_maze_check_abi(99, 0, &mismatch)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == BYUL_MAZE_ABI_VERSION_MISMATCH);
    CHECK(byul_maze_check_abi(
        BYUL_MAZE_ABI_VERSION, BYUL_MAZE_ABI_FINGERPRINT + 1, &mismatch)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == BYUL_MAZE_ABI_FINGERPRINT_MISMATCH);
    CHECK(byul_maze_check_abi(
        BYUL_MAZE_ABI_VERSION, BYUL_MAZE_ABI_FINGERPRINT, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
}

TEST_CASE("maze_make: MAZE_TYPE_ALDOUS_BRODER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ALDOUS_BRODER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ALDOUS_BRODER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_BINARY") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_BINARY);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_BINARY.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_ELLER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ELLER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ELLER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_HUNT_AND_KILL") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_HUNT_AND_KILL);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_HUNT_AND_KILL.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_KRUSKAL") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_KRUSKAL);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_KRUSKAL.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_PRIM") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_PRIM);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_PRIM.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_RECURSIVE") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_RECURSIVE);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_RECURSIVE.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_RECURSIVE_DIVISION") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_RECURSIVE_DIVISION);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_RECURSIVE_DIVISION.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_ROOM_BLEND") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ROOM_BLEND);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ROOM_BLEND.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_SIDEWINDER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_SIDEWINDER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_SIDEWINDER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_WILSON") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_WILSON);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_WILSON.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze overlays preserve overlapping sources") {
    maze_t* first = maze_create_full(0, 0, 8, 8);
    maze_t* second = maze_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(grid != nullptr);

    const coord_t blocked{2, 5};
    bool changed = false;
    REQUIRE(byul_maze_set_blocked(
        first, blocked.x, blocked.y, true, &changed) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        second, blocked.x, blocked.y, true, &changed) == NAVSYS_STATUS_OK);

    maze_apply_to_navgrid(first, grid);
    maze_apply_to_navgrid(second, grid);
    CHECK(is_coord_blocked_navgrid(grid, 2, 5, nullptr));
    maze_remove_from_navgrid(first, grid);
    CHECK(is_coord_blocked_navgrid(grid, 2, 5, nullptr));
    maze_remove_from_navgrid(second, grid);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 5, nullptr));

    navgrid_destroy(grid);
    maze_destroy(second);
    maze_destroy(first);
}

TEST_CASE("maze core uses normalized half-open extents and translates world keys") {
    maze_t* maze = maze_create_full(10, 20, -4, -3);
    REQUIRE(maze != nullptr);

    const coord_t lower{6, 17};
    const coord_t upper{9, 19};
    bool changed = false;
    REQUIRE(byul_maze_set_blocked(
        maze, lower.x, lower.y, true, &changed) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        maze, upper.x, upper.y, true, &changed) == NAVSYS_STATUS_OK);
    maze_t* original = maze_copy(maze);
    REQUIRE(original != nullptr);

    CHECK(byul_maze_translate(maze, -7, 11) == NAVSYS_STATUS_OK);
    int origin_x = 0;
    int origin_y = 0;
    maze_get_origin(maze, &origin_x, &origin_y);
    CHECK(origin_x == 3);
    CHECK(origin_y == 31);
    CHECK(maze_get_width(maze) == -4);
    CHECK(maze_get_height(maze) == -3);
    const coord_t moved_lower{-1, 28};
    const coord_t moved_upper{2, 30};
    bool blocked = false;
    REQUIRE(byul_maze_is_blocked(
        maze, moved_lower.x, moved_lower.y, &blocked) == NAVSYS_STATUS_OK);
    CHECK(blocked);
    REQUIRE(byul_maze_is_blocked(
        maze, moved_upper.x, moved_upper.y, &blocked) == NAVSYS_STATUS_OK);
    CHECK(blocked);
    CHECK(byul_maze_is_blocked(maze, lower.x, lower.y, &blocked)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK_FALSE(maze_equal(maze, original));
    CHECK(maze_hash(maze) != maze_hash(original));

    CHECK(byul_maze_translate(maze, 7, -11) == NAVSYS_STATUS_OK);
    CHECK(maze_equal(maze, original));
    CHECK(maze_hash(maze) == maze_hash(original));

    maze_destroy(original);
    maze_destroy(maze);
}

TEST_CASE("maze origin translation is failure atomic at integer boundaries") {
    maze_t* maze = maze_create_full(0, 0, 4, 4);
    REQUIRE(maze != nullptr);
    const coord_t blocked{1, 1};
    bool changed = false;
    REQUIRE(byul_maze_set_blocked(
        maze, blocked.x, blocked.y, true, &changed) == NAVSYS_STATUS_OK);
    maze_t* snapshot = maze_copy(maze);
    REQUIRE(snapshot != nullptr);

    CHECK(byul_maze_translate(
        maze, std::numeric_limits<int32_t>::max(), 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(maze_equal(maze, snapshot));

    maze_set_origin(maze, 100, -200);
    const coord_t moved{101, -199};
    int origin_x = 0;
    int origin_y = 0;
    maze_get_origin(maze, &origin_x, &origin_y);
    CHECK(origin_x == 100);
    CHECK(origin_y == -200);
    bool moved_blocked = false;
    REQUIRE(byul_maze_is_blocked(
        maze, moved.x, moved.y, &moved_blocked) == NAVSYS_STATUS_OK);
    CHECK(moved_blocked);

    maze_destroy(snapshot);
    maze_destroy(maze);
}

TEST_CASE("maze checked mutation rejects out-of-extent coordinates") {
    maze_t* maze = maze_create_full(0, 0, 4, 4);
    REQUIRE(maze != nullptr);
    const coord_t outside{4, 3};
    maze_t* snapshot = maze_copy(maze);
    REQUIRE(snapshot != nullptr);

    bool changed = true;
    CHECK(byul_maze_set_blocked(
        maze, outside.x, outside.y, true, &changed) == NAVSYS_STATUS_NOT_FOUND);
    CHECK(changed);
    CHECK(maze_equal(maze, snapshot));

    maze_destroy(snapshot);
    maze_destroy(maze);
}

TEST_CASE("maze empty and even extents retain their signed core metadata") {
    maze_t* empty = maze_create_full(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::max(),
        0,
        8);
    REQUIRE(empty != nullptr);
    CHECK(maze_get_width(empty) == 0);
    CHECK(maze_get_height(empty) == 8);
    CHECK(byul_maze_translate(empty, 0, 0) == NAVSYS_STATUS_OK);
    size_t blocked_count = 1;
    REQUIRE(byul_maze_get_blocked_count(empty, &blocked_count)
        == NAVSYS_STATUS_OK);
    CHECK(blocked_count == 0);
    maze_destroy(empty);
}

TEST_CASE("maze checked lifecycle normalizes extent and preserves outputs on error") {
    const byul_maze_extent_t requested{-10, 20, 6, 8};
    maze_t* maze = nullptr;
    REQUIRE(byul_maze_create(&requested, &maze) == NAVSYS_STATUS_OK);
    REQUIRE(maze != nullptr);

    byul_maze_extent_t actual{1, 2, 3, 4};
    REQUIRE(byul_maze_get_extent(maze, &actual) == NAVSYS_STATUS_OK);
    CHECK(actual.origin_x == -10);
    CHECK(actual.origin_y == 20);
    CHECK(actual.width == 6);
    CHECK(actual.height == 8);

    maze_t* preserved = reinterpret_cast<maze_t*>(uintptr_t{1});
    CHECK(byul_maze_create(nullptr, &preserved)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved == reinterpret_cast<maze_t*>(uintptr_t{1}));
    CHECK(byul_maze_copy(nullptr, &preserved)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved == reinterpret_cast<maze_t*>(uintptr_t{1}));

    maze_t* copied = nullptr;
    REQUIRE(byul_maze_copy(maze, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(maze_equal(maze, copied));
    maze_destroy(copied);
    maze_destroy(maze);
}

TEST_CASE("maze legacy and checked constructors produce the same semantic maze") {
    maze_t* legacy = maze_create_full(-4, 6, 7, 5);
    const byul_maze_extent_t extent{-4, 6, 7, 5};
    maze_t* checked = nullptr;
    REQUIRE(legacy != nullptr);
    REQUIRE(byul_maze_create(&extent, &checked) == NAVSYS_STATUS_OK);

    bool changed = false;
    for (const coord_t coordinate : {coord_t{-4, 6}, coord_t{-1, 8}}) {
        REQUIRE(byul_maze_set_blocked(
            legacy, coordinate.x, coordinate.y, true, &changed)
            == NAVSYS_STATUS_OK);
        REQUIRE(byul_maze_set_blocked(
            checked, coordinate.x, coordinate.y, true, &changed)
            == NAVSYS_STATUS_OK);
    }
    CHECK(maze_equal(legacy, checked));
    CHECK(maze_hash(legacy) == maze_hash(checked));

    maze_destroy(checked);
    maze_destroy(legacy);
}

TEST_CASE("maze checked blocked enumeration supports count and undersized buffers") {
    maze_t* maze = maze_create_full(0, 0, 8, 8);
    REQUIRE(maze != nullptr);
    const coord_t first{1, 2};
    const coord_t second{6, 5};
    bool changed = false;
    REQUIRE(byul_maze_set_blocked(
        maze, first.x, first.y, true, &changed) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        maze, second.x, second.y, true, &changed) == NAVSYS_STATUS_OK);

    size_t count = 999;
    CHECK(byul_maze_get_blocked_count(maze, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 2);
    count = 999;
    CHECK(byul_maze_fetch_blocked(maze, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 2);

    coord_t undersized[1]{{77, 88}};
    count = 999;
    CHECK(byul_maze_fetch_blocked(maze, undersized, 1, &count)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(count == 2);
    CHECK(undersized[0].x == 77);
    CHECK(undersized[0].y == 88);

    coord_t exact[2]{};
    REQUIRE(byul_maze_fetch_blocked(maze, exact, 2, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 2);
    const bool has_first = (exact[0].x == first.x && exact[0].y == first.y)
        || (exact[1].x == first.x && exact[1].y == first.y);
    const bool has_second = (exact[0].x == second.x && exact[0].y == second.y)
        || (exact[1].x == second.x && exact[1].y == second.y);
    CHECK(has_first);
    CHECK(has_second);

    count = 123;
    CHECK(byul_maze_fetch_blocked(maze, nullptr, 1, &count)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(count == 123);
    maze_destroy(maze);
}

TEST_CASE("maze checked blocked access validates extent and reports changes") {
    const byul_maze_extent_t extent{-3, 7, 4, 3};
    maze_t* maze = nullptr;
    REQUIRE(byul_maze_create(&extent, &maze) == NAVSYS_STATUS_OK);

    bool changed = true;
    REQUIRE(byul_maze_set_blocked(maze, -2, 8, true, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);

    bool blocked = false;
    REQUIRE(byul_maze_is_blocked(maze, -2, 8, &blocked)
        == NAVSYS_STATUS_OK);
    CHECK(blocked);

    changed = true;
    REQUIRE(byul_maze_set_blocked(maze, -2, 8, true, &changed)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(changed);

    changed = true;
    CHECK(byul_maze_set_blocked(maze, 1, 8, true, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(changed);
    blocked = true;
    CHECK(byul_maze_is_blocked(maze, 1, 8, &blocked)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(blocked);

    REQUIRE(byul_maze_set_blocked(maze, -2, 8, false, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);
    REQUIRE(byul_maze_is_blocked(maze, -2, 8, &blocked)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(blocked);
    maze_destroy(maze);
}

TEST_CASE("maze checked extent reports normalized legacy signed extents") {
    maze_t* maze = maze_create_full(10, 20, -3, -2);
    REQUIRE(maze != nullptr);
    byul_maze_extent_t extent{};
    REQUIRE(byul_maze_get_extent(maze, &extent) == NAVSYS_STATUS_OK);
    CHECK(extent.origin_x == 7);
    CHECK(extent.origin_y == 18);
    CHECK(extent.width == 3);
    CHECK(extent.height == 2);
    maze_destroy(maze);
}

TEST_CASE("maze checked overlays preserve maze obstacle and base provenance") {
    maze_t* first = maze_create_full(1, 1, 7, 7);
    maze_t* second = maze_create_full(1, 1, 7, 7);
    obstacle_t* obstacle = obstacle_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(obstacle != nullptr);
    REQUIRE(grid != nullptr);

    const coord_t shared{2, 2};
    const coord_t first_only{3, 3};
    bool maze_changed = false;
    REQUIRE(byul_maze_set_blocked(
        first, shared.x, shared.y, true, &maze_changed) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        first, first_only.x, first_only.y, true, &maze_changed)
        == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        second, shared.x, shared.y, true, &maze_changed) == NAVSYS_STATUS_OK);
    bool obstacle_changed = false;
    REQUIRE(obstacle_set_blocked(
        obstacle, shared.x, shared.y, true, &obstacle_changed)
        == NAVSYS_STATUS_OK);
    REQUIRE(obstacle_changed);
    const navcell_t base{TERRAIN_TYPE_MOUNTAIN, 81};
    REQUIRE(navgrid_set_cell(grid, shared.x, shared.y, &base));

    byul_maze_navgrid_overlay_token_t first_token{};
    byul_maze_navgrid_overlay_token_t second_token{};
    obstacle_navgrid_overlay_token_t obstacle_token{};
    size_t changed = 999;
    REQUIRE(byul_maze_apply(first, grid, nullptr, &first_token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 2);
    REQUIRE(byul_maze_apply(second, grid, nullptr, &second_token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 0);
    REQUIRE(obstacle_apply_to_navgrid_checked(
        obstacle, grid, nullptr, &obstacle_token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 0);

    REQUIRE(byul_maze_remove_overlay(grid, &first_token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK(is_coord_blocked_navgrid(grid, shared.x, shared.y, nullptr));
    CHECK_FALSE(is_coord_blocked_navgrid(
        grid, first_only.x, first_only.y, nullptr));
    REQUIRE(obstacle_remove_from_navgrid_checked(
        grid, &obstacle_token, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 0);
    REQUIRE(byul_maze_remove_overlay(grid, &second_token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, shared.x, shared.y, nullptr));

    navcell_t fetched{};
    REQUIRE(navgrid_fetch_cell(grid, shared.x, shared.y, &fetched) == 0);
    CHECK(fetched.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(fetched.height == 81);

    navgrid_destroy(grid);
    obstacle_destroy(obstacle);
    maze_destroy(second);
    maze_destroy(first);
}

TEST_CASE("maze checked overlay token diagnoses owner mismatch and reuse") {
    maze_t* maze = maze_create_full(0, 0, 4, 4);
    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_4, nullptr);
    navgrid_t* other = navgrid_create_full(4, 4, NAVGRID_DIR_4, nullptr);
    REQUIRE(maze != nullptr);
    REQUIRE(grid != nullptr);
    REQUIRE(other != nullptr);
    const coord_t blocked{1, 1};
    bool maze_changed = false;
    REQUIRE(byul_maze_set_blocked(
        maze, blocked.x, blocked.y, true, &maze_changed) == NAVSYS_STATUS_OK);

    byul_maze_navgrid_overlay_token_t token{};
    size_t changed = 999;
    REQUIRE(byul_maze_apply(maze, grid, nullptr, &token, &changed)
        == NAVSYS_STATUS_OK);
    const byul_maze_navgrid_overlay_token_t preserved = token;
    changed = 777;
    CHECK(byul_maze_remove_overlay(other, &token, &changed)
        == NAVSYS_STATUS_INVALIDATED);
    CHECK(token.owner_cookie == preserved.owner_cookie);
    CHECK(token.overlay == preserved.overlay);
    CHECK(changed == 777);
    REQUIRE(byul_maze_remove_overlay(grid, &token, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK(token.owner_cookie == 0);
    CHECK(token.overlay == 0);
    CHECK(byul_maze_remove_overlay(grid, &token, &changed)
        == NAVSYS_STATUS_INVALIDATED);

    navgrid_destroy(other);
    navgrid_destroy(grid);
    maze_destroy(maze);
}

TEST_CASE("maze checked overlay cancellation bounds and options are failure atomic") {
    maze_t* maze = maze_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_8, nullptr);
    REQUIRE(maze != nullptr);
    REQUIRE(grid != nullptr);
    const coord_t inside{1, 1};
    const coord_t inside_second{2, 2};
    bool maze_changed = false;
    REQUIRE(byul_maze_set_blocked(
        maze, inside.x, inside.y, true, &maze_changed) == NAVSYS_STATUS_OK);
    REQUIRE(byul_maze_set_blocked(
        maze, inside_second.x, inside_second.y, true, &maze_changed)
        == NAVSYS_STATUS_OK);

    maze_cancel_fixture_t fixture{0, 2};
    byul_maze_navgrid_apply_options_t options{
        sizeof(byul_maze_navgrid_apply_options_t),
        BYUL_MAZE_NAVGRID_APPLY_OPTIONS_ABI_VERSION,
        BYUL_MAZE_NAVGRID_MERGE_PRESERVE_BASE,
        cancel_maze_overlay,
        &fixture
    };
    byul_maze_navgrid_overlay_token_t token{31, 32, 33, 34};
    size_t changed = 77;
    CHECK(byul_maze_apply(maze, grid, &options, &token, &changed)
        == NAVSYS_STATUS_CANCELLED);
    CHECK(token.owner_cookie == 33);
    CHECK(token.overlay == 34);
    CHECK(changed == 77);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, inside.x, inside.y, nullptr));
    CHECK_FALSE(is_coord_blocked_navgrid(
        grid, inside_second.x, inside_second.y, nullptr));

    options.cancel_func = throw_maze_overlay_cancel;
    CHECK(byul_maze_apply(maze, grid, &options, &token, &changed)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    options.cancel_func = nullptr;
    ++options.abi_version;
    CHECK(byul_maze_apply(maze, grid, &options, &token, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);
    --options.abi_version;
    options.merge_policy = 99u;
    CHECK(byul_maze_apply(maze, grid, &options, &token, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);

    options.merge_policy = BYUL_MAZE_NAVGRID_MERGE_PRESERVE_BASE;
    const coord_t outside{6, 6};
    REQUIRE(byul_maze_set_blocked(
        maze, outside.x, outside.y, true, &maze_changed) == NAVSYS_STATUS_OK);
    CHECK(byul_maze_apply(maze, grid, &options, &token, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(navgrid_get_width(grid) == 4);
    CHECK(navgrid_get_height(grid) == 4);
    CHECK(token.owner_cookie == 33);
    CHECK(changed == 77);

    navgrid_destroy(grid);
    maze_destroy(maze);
}
