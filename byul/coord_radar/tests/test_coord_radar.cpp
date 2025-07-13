#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "internal/coord_radar.h"
#include "internal/coord.h"

static bool test_map[10][10];

static bool is_reachable_cb(const coord_t* c, void* user_data) {
    if (!c || coord_get_x(c) < 0 || coord_get_x(c) >= 10 ||
            coord_get_y(c) < 0 || coord_get_y(c) >= 10)
        return false;
    return test_map[coord_get_y(c)][coord_get_x(c)];
}

static void setup_map() {
    for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
            test_map[y][x] = false;

    test_map[4][4] = true;
    test_map[4][5] = true;
    test_map[5][4] = true;
    test_map[3][4] = true;
    test_map[4][3] = true;
}

TEST_CASE("find_goal_bfs finds correct target") {
    setup_map();
    coord_t* start = coord_new_full(2, 2);
    coord_t* result = coord_new_full(-1, -1);

    bool found = find_goal_bfs(start, is_reachable_cb, nullptr, 10, result);
    CHECK(found);
    CHECK(is_reachable_cb(result, nullptr));
    CHECK(coord_get_x(result) >= 3);
    CHECK(coord_get_y(result) >= 3);

    coord_free(start);
    coord_free(result);
}

TEST_CASE("find_goal_astar finds correct target") {
    setup_map();
    coord_t* start = coord_new_full(2, 2);
    coord_t* result = coord_new_full(-1, -1);

    bool found = find_goal_astar(start, is_reachable_cb, nullptr, 10, result);
    CHECK(found);
    CHECK(is_reachable_cb(result, nullptr));
    CHECK(coord_get_x(result) >= 3);
    CHECK(coord_get_y(result) >= 3);

    coord_free(start);
    coord_free(result);
}
