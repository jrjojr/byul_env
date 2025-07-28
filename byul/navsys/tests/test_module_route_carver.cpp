#include "doctest.h"
#include "route_carver.h"
#include "coord.h"
#include <locale.h>

#include "navgrid.h"
#include "obstacle.h"
#include "console.h"

TEST_CASE("route_carve_beam - straight line carve") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t start = { 1, 1 };
    coord_t goal = { 8, 8 };

    int removed = route_carve_beam(navgrid, &start, &goal, 0);
    CHECK(removed >= 1);

    navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
}

TEST_CASE("route_carve_beam - wide beam carve") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t start = { 2, 2 };
    coord_t goal = { 7, 7 };

    int removed = route_carve_beam(navgrid, &start, &goal, 1);  // 넓은 beam
    CHECK(removed >= 20);  // 충분한 영역 제거 예상

navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
}

TEST_CASE("route_carve_bomb - center explosion") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t center = { 5, 5 };
    int removed = route_carve_bomb(navgrid, &center, 2);  // 5x5 제거

    CHECK(removed == 25);  // 정확한 폭파 크기
navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
}
