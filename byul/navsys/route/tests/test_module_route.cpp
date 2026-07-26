#include "doctest.h"
#include "route.h"
#include "coord.h"
#include "coord_hash.h"
#if defined(BYUL_ROUTE_INTERNAL_TESTING)
#include "../internal/route_internal.h"
#endif

#include <cstddef>

namespace {

void* copy_coord_for_hash(const void* value) {
    return coord_copy(static_cast<const coord_t*>(value));
}

void destroy_coord_for_hash(void* value) {
    coord_destroy(static_cast<coord_t*>(value));
}

}  // namespace

TEST_CASE("[ROUTE-ABI-001] route ABI 1 enum and public layout baseline") {
    CHECK(ROUTE_DIR_UNKNOWN == 0);
    CHECK(ROUTE_DIR_RIGHT == 1);
    CHECK(ROUTE_DIR_UP_RIGHT == 2);
    CHECK(ROUTE_DIR_UP == 3);
    CHECK(ROUTE_DIR_UP_LEFT == 4);
    CHECK(ROUTE_DIR_LEFT == 5);
    CHECK(ROUTE_DIR_DOWN_LEFT == 6);
    CHECK(ROUTE_DIR_DOWN == 7);
    CHECK(ROUTE_DIR_DOWN_RIGHT == 8);
    CHECK(ROUTE_DIR_COUNT == 9);
    CHECK(ROUTE_COMPLETION_NONE == 0);
    CHECK(ROUTE_COMPLETION_COMPLETE == 1);
    CHECK(ROUTE_COMPLETION_PARTIAL == 2);

    CHECK(sizeof(route_dir_t) == 4);
    CHECK(sizeof(route_completion_t) == 4);
    CHECK(sizeof(route_t) == (sizeof(void*) == 8 ? 48u : 36u));
    CHECK(alignof(route_t) == (sizeof(void*) == 8 ? 8u : 4u));
    CHECK(offsetof(route_t, coords) == 0);
    CHECK(offsetof(route_t, visited_order) == sizeof(void*));
    CHECK(offsetof(route_t, visited_count) == 2 * sizeof(void*));
    CHECK(offsetof(route_t, cost) == 3 * sizeof(void*));
    CHECK(offsetof(route_t, success) == 3 * sizeof(void*) + 4);
    CHECK(offsetof(route_t, total_retry_count) == 3 * sizeof(void*) + 8);
    CHECK(offsetof(route_t, avg_vec_x) == 3 * sizeof(void*) + 12);
    CHECK(offsetof(route_t, avg_vec_y) == 3 * sizeof(void*) + 16);
    CHECK(offsetof(route_t, vec_count) == 3 * sizeof(void*) + 20);
}

TEST_CASE("[ROUTE-LEGACY-001] route identity and NULL sentinel baseline") {
    route_t* route = route_create();
    REQUIRE(route != nullptr);
    route_t* copy = route_copy(route);
    REQUIRE(copy != nullptr);

    CHECK(route_equal(route, route) == 1);
    CHECK(route_equal(route, copy) == 0);
    CHECK(route_hash(route) == reinterpret_cast<uintptr_t>(route));
    CHECK(route_get_cost(nullptr) == doctest::Approx(0.0f));
    CHECK(route_get_success(nullptr) == 0);
    CHECK(route_length(nullptr) == 0);

    route_destroy(copy);
    route_destroy(route);
}

TEST_CASE("[ROUTE-LEGACY-002] slice and append copy coordinates only") {
    route_t* source = route_create();
    route_t* destination = route_create();
    REQUIRE(source != nullptr);
    REQUIRE(destination != nullptr);
    const coord_t first = {1, 2};
    const coord_t second = {3, 4};
    REQUIRE(route_add_coord(source, &first) == 1);
    REQUIRE(route_add_coord(source, &second) == 1);
    route_set_cost(source, 7.5f);
    route_set_success(source, 1);
    route_set_total_retry_count(source, 3);

    route_t* slice = route_slice(source, 0, 1);
    REQUIRE(slice != nullptr);
    CHECK(route_length(slice) == 1);
    CHECK(route_get_cost(slice) == doctest::Approx(0.0f));
    CHECK(route_get_success(slice) == 0);
    CHECK(route_get_total_retry_count(slice) == 0);

    route_set_cost(destination, 2.0f);
    route_set_total_retry_count(destination, 5);
    route_append(destination, source);
    CHECK(route_length(destination) == 2);
    CHECK(route_get_cost(destination) == doctest::Approx(2.0f));
    CHECK(route_get_success(destination) == 0);
    CHECK(route_get_total_retry_count(destination) == 5);

    route_destroy(slice);
    route_destroy(destination);
    route_destroy(source);
}

TEST_CASE("[ROUTE-P0-001] NULL legacy accessors and direction helpers do not crash") {
    CHECK(route_get_total_retry_count(nullptr) == 0);
    CHECK_NOTHROW(route_set_total_retry_count(nullptr, 7));
    CHECK(route_get_direction_by_dir_coord(nullptr) == ROUTE_DIR_UNKNOWN);
    CHECK(calc_direction(nullptr, nullptr) == ROUTE_DIR_UNKNOWN);

    route_t* route = route_create();
    REQUIRE(route != nullptr);
    const coord_t point = {1, 2};
    float angle = 17.0f;
    CHECK(route_has_changed(route, nullptr, &point, 10.0f) == 0);
    CHECK(route_has_changed_with_angle(
        route, &point, nullptr, 10.0f, &angle) == 0);
    CHECK(angle == doctest::Approx(17.0f));
    CHECK_NOTHROW(route_update_average_vector(route, nullptr, &point));

    route_destroy(route);
}

TEST_CASE("[ROUTE-P0-002] clone and slice companions preserve outputs") {
    route_t* source = route_create();
    REQUIRE(source != nullptr);
    const coord_t first = {3, 5};
    const coord_t second = {7, 11};
    REQUIRE(route_add_coord(source, &first) == 1);
    REQUIRE(route_add_coord(source, &second) == 1);
    route_set_cost(source, 13.0f);
    route_set_success(source, 1);
    route_set_total_retry_count(source, 17);

    route_t* clone = nullptr;
    CHECK(route_clone_ex(source, &clone) == NAVSYS_STATUS_OK);
    REQUIRE(clone != nullptr);
    CHECK(route_length(clone) == 2);
    CHECK(route_get_cost(clone) == doctest::Approx(13.0f));
    CHECK(route_get_success(clone) == 1);
    CHECK(route_get_total_retry_count(clone) == 17);

    route_t* const sentinel = reinterpret_cast<route_t*>(1);
    route_t* output = sentinel;
    CHECK(route_clone_ex(nullptr, &output) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == sentinel);
    CHECK(route_clone_ex(source, nullptr) == NAVSYS_STATUS_INVALID_ARGUMENT);

    output = sentinel;
    CHECK(route_slice_ex(source, 2, 1, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == sentinel);

    route_t* empty = nullptr;
    CHECK(route_slice_ex(source, 1, 1, &empty) == NAVSYS_STATUS_OK);
    REQUIRE(empty != nullptr);
    CHECK(route_length(empty) == 0);

    route_t* slice = nullptr;
    CHECK(route_slice_ex(source, 0, 1, &slice) == NAVSYS_STATUS_OK);
    REQUIRE(slice != nullptr);
    CHECK(route_length(slice) == 1);
    CHECK(route_get_cost(slice) == doctest::Approx(0.0f));
    CHECK(route_get_success(slice) == 0);
    CHECK(route_get_total_retry_count(slice) == 0);

    route_destroy(slice);
    route_destroy(empty);
    route_destroy(clone);
    route_destroy(source);
}

TEST_CASE("[ROUTE-P0-003] reconstruction rejects missing and cyclic predecessors atomically") {
    const coord_t marker = {-1, -1};
    const coord_t start = {0, 0};
    const coord_t middle = {1, 0};
    const coord_t goal = {2, 0};

    route_t* route = route_create();
    REQUIRE(route != nullptr);
    REQUIRE(route_add_coord(route, &marker) == 1);

    coord_hash_t* complete = coord_hash_create_full(
        copy_coord_for_hash,
        destroy_coord_for_hash);
    REQUIRE(complete != nullptr);
    REQUIRE(coord_hash_replace(complete, &goal, const_cast<coord_t*>(&middle)));
    REQUIRE(coord_hash_replace(complete, &middle, const_cast<coord_t*>(&start)));
    CHECK(route_reconstruct_ex(route, complete, &start, &goal)
        == NAVSYS_STATUS_OK);
    CHECK(route_length(route) == 4);
    CHECK(coord_equal(route_get_coord_at(route, 0), &marker));
    CHECK(coord_equal(route_get_coord_at(route, 1), &start));
    CHECK(coord_equal(route_get_coord_at(route, 2), &middle));
    CHECK(coord_equal(route_get_coord_at(route, 3), &goal));

    route_clear_coords(route);
    REQUIRE(route_add_coord(route, &marker) == 1);
    coord_hash_t* missing = coord_hash_create_full(
        copy_coord_for_hash,
        destroy_coord_for_hash);
    REQUIRE(missing != nullptr);
    CHECK(route_reconstruct_ex(route, missing, &start, &goal)
        == NAVSYS_STATUS_NO_PATH);
    CHECK(route_length(route) == 1);
    CHECK(coord_equal(route_get_coord_at(route, 0), &marker));

    coord_hash_t* cyclic = coord_hash_create_full(
        copy_coord_for_hash,
        destroy_coord_for_hash);
    REQUIRE(cyclic != nullptr);
    REQUIRE(coord_hash_replace(cyclic, &goal, const_cast<coord_t*>(&middle)));
    REQUIRE(coord_hash_replace(cyclic, &middle, const_cast<coord_t*>(&goal)));
    CHECK(route_reconstruct_ex(route, cyclic, &start, &goal)
        == NAVSYS_STATUS_CORRUPT_STATE);
    CHECK(route_length(route) == 1);
    CHECK(coord_equal(route_get_coord_at(route, 0), &marker));

    coord_hash_destroy(cyclic);
    coord_hash_destroy(missing);
    coord_hash_destroy(complete);
    route_destroy(route);
}

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
    CHECK(short_output[0].x == -1);
    CHECK(short_output[0].y == -1);
    CHECK(short_output[1].x == -1);
    CHECK(short_output[1].y == -1);

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

TEST_CASE("route builder edits transactionally and transfers one immutable result") {
    route_t* source = route_create();
    REQUIRE(source != nullptr);
    const coord_t first = {0, 0};
    const coord_t second = {1, 0};
    REQUIRE(route_add_coord(source, &first) == 1);
    REQUIRE(route_add_coord(source, &second) == 1);
    route_set_cost(source, 7.5f);
    route_set_success(source, 1);
    route_set_total_retry_count(source, 19);

    route_builder_t* builder = nullptr;
    REQUIRE(route_builder_create_from_route(source, &builder)
        == NAVSYS_STATUS_OK);
    REQUIRE(builder != nullptr);

    CHECK(route_builder_append(
        builder, source, ROUTE_JOIN_DEDUP_BOUNDARY) == NAVSYS_STATUS_OK);
    const coord_t inserted = {9, 9};
    CHECK(route_builder_insert_coord(builder, 1, &inserted)
        == NAVSYS_STATUS_OK);
    coord_t removed = {-1, -1};
    CHECK(route_builder_remove_coord(builder, 1, &removed)
        == NAVSYS_STATUS_OK);
    CHECK(removed.x == 9);
    CHECK(removed.y == 9);
    CHECK(route_builder_set_total_cost(builder, 12.25)
        == NAVSYS_STATUS_OK);
    CHECK(route_builder_set_completion(builder, ROUTE_COMPLETION_COMPLETE)
        == NAVSYS_STATUS_OK);

    route_t* result = nullptr;
    CHECK(route_builder_finish(builder, &result) == NAVSYS_STATUS_OK);
    REQUIRE(result != nullptr);
    CHECK(route_get_coord_count(result) == 4);
    coord_t output[4] = {};
    size_t required = 0;
    CHECK(route_export_coords(result, output, 4, &required)
        == NAVSYS_STATUS_OK);
    CHECK(required == 4);
    CHECK(output[0].x == 0);
    CHECK(output[1].x == 1);
    CHECK(output[2].x == 0);
    CHECK(output[3].x == 1);
    double cost = 0.0;
    route_completion_t completion = ROUTE_COMPLETION_NONE;
    CHECK(route_fetch_total_cost(result, &cost) == NAVSYS_STATUS_OK);
    CHECK(cost == doctest::Approx(12.25));
    CHECK(route_fetch_completion(result, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_COMPLETE);
    CHECK(route_get_total_retry_count(result) == 0);

    route_t* const sentinel = reinterpret_cast<route_t*>(1);
    route_t* invalid_output = sentinel;
    CHECK(route_builder_finish(builder, &invalid_output)
        == NAVSYS_STATUS_INVALIDATED);
    CHECK(invalid_output == sentinel);

    route_builder_destroy(builder);
    route_destroy(result);
    route_destroy(source);
}

TEST_CASE("route builder handles empty slices invalid ranges and large counts") {
    route_builder_t* builder = nullptr;
    REQUIRE(route_builder_create(&builder) == NAVSYS_STATUS_OK);
    for (int index = 0; index < 1024; ++index) {
        const coord_t coordinate = {index, -index};
        REQUIRE(route_builder_push_coord(builder, &coordinate)
            == NAVSYS_STATUS_OK);
    }
    route_t* large = nullptr;
    REQUIRE(route_builder_finish(builder, &large) == NAVSYS_STATUS_OK);
    CHECK(route_get_coord_count(large) == 1024);
    route_builder_destroy(builder);

    REQUIRE(route_builder_create(&builder) == NAVSYS_STATUS_OK);
    CHECK(route_builder_assign_slice(builder, large, 9, 8)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_builder_assign_slice(builder, large, 7, 7)
        == NAVSYS_STATUS_OK);
    CHECK(route_builder_set_completion(builder, ROUTE_COMPLETION_COMPLETE)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    route_t* empty = nullptr;
    REQUIRE(route_builder_finish(builder, &empty) == NAVSYS_STATUS_OK);
    CHECK(route_get_coord_count(empty) == 0);
    route_completion_t completion = ROUTE_COMPLETION_COMPLETE;
    CHECK(route_fetch_completion(empty, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_NONE);

    route_builder_destroy(builder);
    route_destroy(empty);
    route_destroy(large);
}

#if defined(BYUL_ROUTE_INTERNAL_TESTING)
TEST_CASE("search trace is an opaque owner with atomic ordered counts") {
    navsys_search_trace_t* trace = nullptr;
    REQUIRE(navsys_search_trace_create(&trace) == NAVSYS_STATUS_OK);
    REQUIRE(trace != nullptr);
    const coord_t first = {2, 3};
    const coord_t second = {4, 5};
    CHECK(navsys_search_trace_internal_record(trace, &first)
        == NAVSYS_STATUS_OK);
    CHECK(navsys_search_trace_internal_record(trace, &second)
        == NAVSYS_STATUS_OK);
    CHECK(navsys_search_trace_internal_record(trace, &first)
        == NAVSYS_STATUS_OK);
    CHECK(navsys_search_trace_get_visit_count(trace) == 3);

    size_t count = 99;
    CHECK(navsys_search_trace_fetch_coord_visit_count(
        trace, &first, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 2);
    const coord_t missing = {8, 9};
    count = 77;
    CHECK(navsys_search_trace_fetch_coord_visit_count(
        trace, &missing, &count) == NAVSYS_STATUS_NOT_FOUND);
    CHECK(count == 77);

    coord_t short_output[2] = {{-1, -1}, {-1, -1}};
    size_t required = 0;
    CHECK(navsys_search_trace_export_visits(
        trace, short_output, 2, &required) == NAVSYS_STATUS_INCOMPLETE);
    CHECK(required == 3);
    CHECK(short_output[0].x == -1);
    coord_t output[3] = {};
    CHECK(navsys_search_trace_export_visits(
        trace, output, 3, &required) == NAVSYS_STATUS_OK);
    CHECK(output[0].x == 2);
    CHECK(output[1].x == 4);
    CHECK(output[2].x == 2);

    navsys_search_trace_t* clone = nullptr;
    REQUIRE(navsys_search_trace_clone_ex(trace, &clone)
        == NAVSYS_STATUS_OK);
    CHECK(navsys_search_trace_get_visit_count(clone) == 3);
    CHECK(navsys_search_trace_internal_clear(trace) == NAVSYS_STATUS_OK);
    CHECK(navsys_search_trace_get_visit_count(trace) == 0);
    CHECK(navsys_search_trace_get_visit_count(clone) == 3);

    navsys_search_trace_destroy(clone);
    navsys_search_trace_destroy(trace);
}
#endif

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

TEST_CASE("route fetches cost and completion as result values") {
    route_t* route = route_create();
    REQUIRE(route != nullptr);

    double cost = -1.0;
    route_completion_t completion = ROUTE_COMPLETION_COMPLETE;
    CHECK(route_fetch_total_cost(route, &cost) == NAVSYS_STATUS_OK);
    CHECK(cost == doctest::Approx(0.0));
    CHECK(route_fetch_completion(route, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_NONE);

    route_set_cost(route, 12.5f);
    const coord_t coord = {3, 4};
    REQUIRE(route_add_coord(route, &coord) == 1);
    CHECK(route_fetch_total_cost(route, &cost) == NAVSYS_STATUS_OK);
    CHECK(cost == doctest::Approx(12.5));
    CHECK(route_fetch_completion(route, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_PARTIAL);

    route_set_success(route, 1);
    CHECK(route_fetch_completion(route, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_COMPLETE);

    cost = -2.0;
    completion = ROUTE_COMPLETION_PARTIAL;
    CHECK(route_fetch_total_cost(nullptr, &cost)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(cost == doctest::Approx(-2.0));
    CHECK(route_fetch_total_cost(route, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_fetch_completion(nullptr, &completion)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(completion == ROUTE_COMPLETION_PARTIAL);
    CHECK(route_fetch_completion(route, nullptr)
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
