#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include "internal/route.h"
#include "internal/coord.h"

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
    coord_destroy(dir);

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

    // 첫 번째 경로: (0,0) -> (1,0) -> (2,0)
    route_t* r1 = route_create();
    route_add_coord(r1, a);
    route_add_coord(r1, b);
    route_add_coord(r1, c);

    // 두 번째 경로: (2,0) -> (3,0)
    route_t* r2 = route_create();
    coord_t* c_dup = coord_create_full(2, 0); // r1의 끝과 동일 좌표
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
        route_append_nodup(merged, r2);  // (2,0) 중복 제거되어야 함

        CHECK(route_length(merged) == 4);  // 중복 1개 제거
        CHECK(coord_get_x(route_get_coord_at(merged, 0)) == 0);
        CHECK(coord_get_x(route_get_coord_at(merged, 3)) == 3);

        // 중간 중복이 제거되지 않는지 확인
        coord_t* e = coord_create_full(1, 0);
        route_add_coord(r2, e);
        route_append_nodup(merged, r2); // (1,0)은 중간 중복이므로 남아야 함
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

int main(int argc, char** argv) {
#ifdef _WIN32
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(65001);                          // UTF-8 출력용
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // UTF-8 로케일
#else
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // 리눅스/맥에서도 설정
#endif

    std::cout << u8"🌟 UTF-8 콘솔 코드페이지로 전환하고 테스트 시작!\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);
    int res = context.run();

    if (context.shouldExit()) {
        std::cout << u8"🌙 테스트 끝! 콘솔 코드페이지 원래대로 복구했습니다.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp);                // 원래 코드페이지 복원
        setlocale(LC_ALL, "");                          // 기본 로케일로 복귀
#endif
        return res;
    }

    std::cout << u8"🌙 테스트 종료. 콘솔 상태 복원 완료.\n";
#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, "");                              // 로케일 복원
#endif

    return res;
}
