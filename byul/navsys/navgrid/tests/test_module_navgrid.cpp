#include "doctest.h"
#include "navgrid.h"
#include "coord.h"

#include <cstddef>
#include <limits>
#include <type_traits>

static_assert(TERRAIN_TYPE_NORMAL == 0);
static_assert(TERRAIN_TYPE_WATER == 1);
static_assert(TERRAIN_TYPE_FOREST == 2);
static_assert(TERRAIN_TYPE_MOUNTAIN == 3);
static_assert(TERRAIN_TYPE_FORBIDDEN == 100);
static_assert(sizeof(terrain_type_t) == 4);
static_assert(std::is_standard_layout_v<navcell_t>);
static_assert(std::is_trivially_copyable_v<navcell_t>);
static_assert(sizeof(navcell_t) == 8);
static_assert(alignof(navcell_t) == 4);
static_assert(offsetof(navcell_t, terrain) == 0);
static_assert(offsetof(navcell_t, height) == 4);

TEST_CASE("navcell legacy value ABI baseline") {
    navcell_t zero{};
    CHECK(zero.terrain == TERRAIN_TYPE_NORMAL);
    CHECK(zero.height == 0);

    navcell_t boundaries{};
    CHECK(navcell_init_full(
        &boundaries,
        TERRAIN_TYPE_WATER,
        std::numeric_limits<int>::min()) == 0);
    CHECK(boundaries.terrain == TERRAIN_TYPE_WATER);
    CHECK(boundaries.height == std::numeric_limits<int>::min());

    navcell_t copied{};
    CHECK(navcell_assign(&copied, &boundaries) == 0);
    CHECK(copied.terrain == TERRAIN_TYPE_WATER);
    CHECK(copied.height == std::numeric_limits<int>::min());
    CHECK(navcell_init_full(
        &boundaries,
        static_cast<terrain_type_t>(101),
        std::numeric_limits<int>::max()) == 0);
    CHECK(static_cast<int>(boundaries.terrain) == 101);
    CHECK(boundaries.height == std::numeric_limits<int>::max());

    CHECK(navcell_init(nullptr) == -1);
    CHECK(navcell_init_full(nullptr, TERRAIN_TYPE_NORMAL, 0) == -1);
    CHECK(navcell_assign(nullptr, &boundaries) == -1);
    CHECK(navcell_assign(&boundaries, nullptr) == -1);
    CHECK(navcell_copy(nullptr) == nullptr);
    navcell_destroy(nullptr);
}

TEST_CASE("navgrid blocking and checking") {
    navgrid_t* m = navgrid_create();
    CHECK(navgrid_block_coord(m, 6, 6));
    CHECK(m->is_coord_blocked_fn(m, 6, 6, nullptr));
    CHECK_FALSE(m->is_coord_blocked_fn(m, 5, 5, nullptr));
    navgrid_destroy(m);
}

TEST_CASE("navgrid unblock") {
    navgrid_t* m = navgrid_create();
    CHECK(navgrid_block_coord(m, 4, 4));
    CHECK(m->is_coord_blocked_fn(m, 4, 4, nullptr));
    //CHECK(navgrid_unblock_coord(m, 4, 4));
    //CHECK_FALSE(m->is_coord_blocked_fn(m, 4, 4, nullptr));
    navgrid_destroy(m);
}

TEST_CASE("navgrid clear all") {
    navgrid_t* m = navgrid_create();
    for (int x = 0; x < 5; ++x)
        for (int y = 1; y < 10; ++y)
            navgrid_block_coord(m, x, y);

    CHECK(m->is_coord_blocked_fn(m, 2, 2, nullptr));
    navgrid_clear(m);
    CHECK_FALSE(m->is_coord_blocked_fn(m, 2, 2, nullptr));
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbors filtering") {
    navgrid_t* m = navgrid_create();
    navgrid_block_coord(m, 3, 2);
    navgrid_block_coord(m, 2, 3);

    coord_list_t* neighbors = navgrid_copy_neighbors(m, 2, 2);
    REQUIRE(neighbors);

    int expected = (navgrid_get_mode(m) == NAVGRID_DIR_8) ? 6 : 2;
    CHECK(coord_list_length(neighbors) == expected);

    bool has_21 = false, has_12 = false;
    for (int i = 0; i < coord_list_length(neighbors); ++i) {
        const coord_t* c = coord_list_get(neighbors, i);
        if (coord_get_x(c) == 2 && coord_get_y(c) == 1) has_21 = true;
        if (coord_get_x(c) == 1 && coord_get_y(c) == 2) has_12 = true;
    }
    CHECK(has_21);
    CHECK(has_12);
    coord_list_destroy(neighbors);
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbor at degree") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* c = navgrid_copy_neighbor_at_degree(m, 2, 2, 0.0);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 2);
    coord_destroy(c);
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbor at goal") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* center = coord_create_full(2, 2);
    coord_t* goal = coord_create_full(4, 1);
    coord_t* c = navgrid_copy_neighbor_at_goal(m, center, goal);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 1);
    coord_destroy(center);
    coord_destroy(goal);
    coord_destroy(c);
    navgrid_destroy(m);
}

TEST_CASE("navgrid cone neighbor range") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* center = coord_create_full(2, 2);
    coord_t* goal = coord_create_full(4, 2);

    coord_list_t* result = navgrid_copy_neighbors_at_degree_range(
        m, center, goal, -45.0, 45.0, 1);

    int count = coord_list_length(result);
    CHECK(count == 3);
    for (int i = 0; i < count; ++i) {
        const coord_t* c = coord_list_get(result, i);
        CHECK(navgrid_is_inside(m, coord_get_x(c), coord_get_y(c)));
    }

    coord_list_destroy(result);
    coord_destroy(center);
    coord_destroy(goal);
    navgrid_destroy(m);
}
