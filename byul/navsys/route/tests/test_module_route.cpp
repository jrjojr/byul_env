#include "doctest.h"
#include "route.h"
#include "coord.h"

TEST_CASE("route creation and basic ops") {
    route_t* p = route_create();
    CHECK(route_get_cost(p) == doctest::Approx(0.0f));
    CHECK(route_get_success(p) == doctest::Approx(0.0f));

    coord_t* a = coord_create_full(1, 2);
    coord_t* b = coord_create_full(2, 2);
    coord_t* c = coord_create_full(3, 2);
    route_add_coord(p, a);
    route_add_coord(p, b);
    route_add_coord(p, c);

    const coord_list_t* coords = route_get_coords(p);
    CHECK(coord_list_length(coords) == 3);

    const coord_t* d = coord_list_get(coords, 0);
    const coord_t* e = coord_list_get(coords, 2);
    CHECK(coord_get_x(d) == 1);
    CHECK(coord_get_x(e) == 3);

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    route_destroy(p);
}

TEST_CASE("route exports coordinates with two-call buffer semantics") {
    route_t* route = route_create();
    REQUIRE(route != nullptr);
    const coord_t coords[] = {{1, 2}, {3, 4}, {5, 6}};
    for (const coord_t& coord : coords) {
        REQUIRE(route_add_coord(route, &coord) == 1);
    }

    size_t required = 99;
    CHECK(route_export_coords(route, nullptr, 0, &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == 3);

    coord_t short_output[2] = {{-1, -1}, {-1, -1}};
    required = 99;
    CHECK(route_export_coords(route, short_output, 2, &required)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(required == 3);
    CHECK(short_output[0].x == 1);
    CHECK(short_output[0].y == 2);
    CHECK(short_output[1].x == 3);
    CHECK(short_output[1].y == 4);

    coord_t exact_output[3] = {};
    required = 99;
    CHECK(route_export_coords(route, exact_output, 3, &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == 3);
    CHECK(exact_output[2].x == 5);
    CHECK(exact_output[2].y == 6);

    required = 77;
    CHECK(route_export_coords(nullptr, exact_output, 3, &required)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(required == 77);
    CHECK(route_export_coords(route, nullptr, 1, &required)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(required == 77);
    CHECK(route_export_coords(route, exact_output, 3, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    route_destroy(route);
}

TEST_CASE("route fetches coordinate values without exposing storage") {
    route_t* route = route_create();
    REQUIRE(route != nullptr);
    const coord_t first = {7, 8};
    const coord_t second = {9, 10};
    REQUIRE(route_add_coord(route, &first) == 1);
    REQUIRE(route_add_coord(route, &second) == 1);

    CHECK(route_get_coord_count(route) == 2);
    CHECK(route_get_coord_count(nullptr) == 0);

    coord_t output = {-1, -2};
    CHECK(route_fetch_coord(route, 1, &output) == NAVSYS_STATUS_OK);
    CHECK(output.x == 9);
    CHECK(output.y == 10);

    output = {-3, -4};
    CHECK(route_fetch_coord(route, 2, &output)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(output.x == -3);
    CHECK(output.y == -4);
    CHECK(route_fetch_coord(nullptr, 0, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output.x == -3);
    CHECK(output.y == -4);
    CHECK(route_fetch_coord(route, 0, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    route_destroy(route);
}

TEST_CASE("route visited tracking") {
    route_t* p = route_create();
    coord_t* a = coord_create_full(5, 5);
    coord_t* b = coord_create_full(6, 5);
    route_add_visited(p, a);
    route_add_visited(p, b);
    route_add_visited(p, a);

    const coord_hash_t* visited = route_get_visited_count(p);

    CHECK(*(int*)coord_hash_get(visited, a) == 2);
    CHECK(*(int*)coord_hash_get(visited, b) == 1);    

    const coord_list_t* order = route_get_visited_order(p);
    CHECK(coord_list_length(order) == 3);
    CHECK(coord_get_x(coord_list_get(order, 0)) == 5);
    CHECK(coord_get_x(coord_list_get(order, 2)) == 5);

    coord_destroy(a);
    coord_destroy(b);
    route_destroy(p);
}

TEST_CASE("route direction and angle") {
    route_t* p = route_create();
    coord_t* a = coord_create_full(1, 1);
    coord_t* b = coord_create_full(2, 1);
    coord_t* c = coord_create_full(3, 2);
    route_add_coord(p, a);
    route_add_coord(p, b);
    route_add_coord(p, c);

    coord_t* dir = route_make_direction(p, 0);
    CHECK(coord_get_x(dir) == 1);
    CHECK(coord_get_y(dir) == 0);

    CHECK(route_get_direction_by_dir_coord(dir) == ROUTE_DIR_RIGHT);
    CHECK(route_get_direction_by_index(p, 0) == ROUTE_DIR_RIGHT);

    coord_t* from = coord_create_full(2, 2);
    coord_t* to1  = coord_create_full(3, 2);
    coord_t* to2  = coord_create_full(2, 3);
    route_update_average_vector(p, from, to1);

    float angle = 0.0f;
    int changed = route_has_changed_with_angle(p, from, to2, 10.0f, &angle);
    CHECK(changed);
    CHECK(angle >= 89.0f);

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_destroy(from);
    coord_destroy(to1);
    coord_destroy(to2);
    route_destroy(p);
    coord_destroy(dir);    
}

TEST_CASE("route insert, remove, find") {
    route_t* r = route_create();
    coord_t* c1 = coord_create_full(1, 1);
    coord_t* c2 = coord_create_full(2, 2);
    coord_t* c3 = coord_create_full(3, 3);
    route_insert(r, 0, c1);
    route_insert(r, 1, c3);
    route_insert(r, 1, c2);

    CHECK(route_length(r) == 3);
    CHECK(route_find(r, c2) == 1);
    CHECK(route_contains(r, c3) == 1);

    route_remove_at(r, 1);
    CHECK(route_length(r) == 2);
    CHECK(route_contains(r, c2) == 0);

    route_remove_value(r, c3);
    CHECK(route_length(r) == 1);
    CHECK(route_find(r, c1) == 0);

    coord_destroy(c1);
    coord_destroy(c2);
    coord_destroy(c3);
    route_destroy(r);
}

TEST_CASE("route slice") {
    route_t* r = route_create();
    coord_t* tmp[5];
    for (int i = 0; i < 5; ++i) {
        tmp[i] = coord_create_full(i, i);
        route_add_coord(r, tmp[i]);
    }
    route_t* rs = route_slice(r, 1, 4);
    CHECK(route_length(rs) == 3);
    CHECK(coord_get_x(route_get_coord_at(rs, 0)) == 1);
    CHECK(coord_get_x(route_get_coord_at(rs, 2)) == 3);

    for (int i = 0; i < 5; ++i) coord_destroy(tmp[i]);
    route_destroy(r);

    route_destroy(rs);
}

TEST_CASE("route append and append_nodup") {
    coord_t* a = coord_create_full(0, 0);
    coord_t* b = coord_create_full(1, 0);
    coord_t* c = coord_create_full(2, 0);
    coord_t* d = coord_create_full(3, 0);

    // first route : (0,0) -> (1,0) -> (2,0)
    route_t* r1 = route_create();
    route_add_coord(r1, a);
    route_add_coord(r1, b);
    route_add_coord(r1, c);

    // second route: (2,0) -> (3,0)
    route_t* r2 = route_create();
    coord_t* c_dup = coord_create_full(2, 0);
    route_add_coord(r2, c_dup);
    route_add_coord(r2, d);

    SUBCASE("append with duplication") {
        route_t* merged = route_create();
        route_append(merged, r1);
        route_append(merged, r2);

        CHECK(route_length(merged) == 5);
        CHECK(coord_get_x(route_get_coord_at(merged, 0)) == 0);
        CHECK(coord_get_x(route_get_coord_at(merged, 4)) == 3);

        route_destroy(merged);
    }

    SUBCASE("append_nodup removes duplicated endpoint") {
        route_t* merged = route_create();
        route_append(merged, r1);
        route_append_nodup(merged, r2);  // (2,0) duplicated want remove

        CHECK(route_length(merged) == 4);  // duplicated 1개 remove
        CHECK(coord_get_x(route_get_coord_at(merged, 0)) == 0);
        CHECK(coord_get_x(route_get_coord_at(merged, 3)) == 3);


        coord_t* e = coord_create_full(1, 0);
        route_add_coord(r2, e);
        route_append_nodup(merged, r2); // (1,0) middle duplicated alive.
        CHECK(route_contains(merged, e) == 1);

        coord_destroy(e);
        route_destroy(merged);
    }

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_destroy(c_dup);
    coord_destroy(d);
    route_destroy(r1);
    route_destroy(r2);
}
