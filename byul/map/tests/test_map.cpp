#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "internal/map.h"
#include "internal/coord.h"

TEST_CASE("map blocking and checking") {
    map_t* m = map_new();
    CHECK(map_block_coord(m, 6, 6));
    CHECK(m->is_coord_blocked_fn(m, 6, 6, nullptr));
    CHECK_FALSE(m->is_coord_blocked_fn(m, 5, 5, nullptr));
    map_free(m);
}

TEST_CASE("map unblock") {
    map_t* m = map_new();
    CHECK(map_block_coord(m, 4, 4));
    CHECK(m->is_coord_blocked_fn(m, 4, 4, nullptr));
    CHECK(map_unblock_coord(m, 4, 4));
    CHECK_FALSE(m->is_coord_blocked_fn(m, 4, 4, nullptr));
    map_free(m);
}

TEST_CASE("map clear all") {
    map_t* m = map_new();
    for (int x = 0; x < 5; ++x)
        for (int y = 1; y < 10; ++y)
            map_block_coord(m, x, y);

    CHECK(m->is_coord_blocked_fn(m, 2, 2, nullptr));
    map_clear(m);
    CHECK_FALSE(m->is_coord_blocked_fn(m, 2, 2, nullptr));
    map_free(m);
}

TEST_CASE("map neighbors filtering") {
    map_t* m = map_new();
    map_block_coord(m, 3, 2);
    map_block_coord(m, 2, 3);

    coord_list_t* neighbors = map_make_neighbors(m, 2, 2);
    REQUIRE(neighbors);

    int expected = (map_get_mode(m) == MAP_NEIGHBOR_8) ? 6 : 2;
    CHECK(coord_list_length(neighbors) == expected);

    bool has_21 = false, has_12 = false;
    for (int i = 0; i < coord_list_length(neighbors); ++i) {
        const coord_t* c = coord_list_get(neighbors, i);
        if (coord_get_x(c) == 2 && coord_get_y(c) == 1) has_21 = true;
        if (coord_get_x(c) == 1 && coord_get_y(c) == 2) has_12 = true;
    }
    CHECK(has_21);
    CHECK(has_12);
    coord_list_free(neighbors);
    map_free(m);
}

TEST_CASE("map neighbor at degree") {
    map_t* m = map_new_full(5, 5, MAP_NEIGHBOR_8, is_coord_blocked_map);
    coord_t* c = map_make_neighbor_at_degree(m, 2, 2, 0.0);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 2);
    coord_free(c);
    map_free(m);
}

TEST_CASE("map neighbor at goal") {
    map_t* m = map_new_full(5, 5, MAP_NEIGHBOR_8, is_coord_blocked_map);
    coord_t* center = coord_new_full(2, 2);
    coord_t* goal = coord_new_full(4, 1);
    coord_t* c = map_make_neighbor_at_goal(m, center, goal);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 1);
    coord_free(center);
    coord_free(goal);
    coord_free(c);
    map_free(m);
}

TEST_CASE("map cone neighbor range") {
    map_t* m = map_new_full(5, 5, MAP_NEIGHBOR_8, is_coord_blocked_map);
    coord_t* center = coord_new_full(2, 2);
    coord_t* goal = coord_new_full(4, 2);

    coord_list_t* result = map_make_neighbors_at_degree_range(
        m, center, goal, -45.0, 45.0, 1);

    int count = coord_list_length(result);
    CHECK(count == 3);
    for (int i = 0; i < count; ++i) {
        const coord_t* c = coord_list_get(result, i);
        CHECK(map_is_inside(m, coord_get_x(c), coord_get_y(c)));
    }

    coord_list_free(result);
    coord_free(center);
    coord_free(goal);
    map_free(m);
}
