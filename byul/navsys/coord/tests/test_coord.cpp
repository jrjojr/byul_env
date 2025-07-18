//test_coord.cpp

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/coord.h"
}

TEST_CASE("coord: create and free") {
    coord_t* c = coord_new_full(10, 20);
    REQUIRE(c != nullptr);
    CHECK(coord_get_x(c) == 10);
    CHECK(coord_get_y(c) == 20);
    coord_free(c);
}

TEST_CASE("coord: hash, equal, copy") {
    coord_t* c1 = coord_new_full(5, 5);
    coord_t* c2 = coord_copy(c1);
    REQUIRE(c2 != nullptr);
    CHECK(coord_equal(c1, c2));
    CHECK(coord_hash(c1) == coord_hash(c2));
    coord_free(c1);
    coord_free(c2);
}

TEST_CASE("coord: degree basic") {
    coord_t* a = coord_new_full(0, 0);
    coord_t* b = coord_new_full(1, 0);
    coord_t* c = coord_new_full(0, 1);
    coord_t* d = coord_new_full(-1, 0);
    coord_t* e = coord_new_full(0, -1);

    CHECK(coord_degree(a, b) == doctest::Approx(0.0));
    CHECK(coord_degree(a, c) == doctest::Approx(90.0));
    CHECK(coord_degree(a, d) == doctest::Approx(180.0));
    CHECK(coord_degree(a, e) == doctest::Approx(270.0));

    coord_free(a); coord_free(b); coord_free(c); coord_free(d); coord_free(e);
}

TEST_CASE("coord: degree diagonal") {
    coord_t* a = coord_new_full(0, 0);
    coord_t* b = coord_new_full(1, 1);
    coord_t* c = coord_new_full(-1, 1);
    coord_t* d = coord_new_full(-1, -1);
    coord_t* e = coord_new_full(1, -1);

    CHECK(coord_degree(a, b) == doctest::Approx(45.0).epsilon(0.002));
    CHECK(coord_degree(a, c) == doctest::Approx(135.0).epsilon(0.002));
    CHECK(coord_degree(a, d) == doctest::Approx(225.0).epsilon(0.002));
    CHECK(coord_degree(a, e) == doctest::Approx(315.0).epsilon(0.002));

    coord_free(a); coord_free(b); coord_free(c); coord_free(d); coord_free(e);
}

int main(int argc, char** argv) {
    // setlocale(LC_ALL, "ko_KR.UTF-8");  // 💥 별이아빠님 핵심

    std::cout << "🌟 UTF-8 로케일로 테스트를 시작합니다!\n";

    doctest::Context context;

    context.applyCommandLine(argc, argv);

    int res = context.run();  // 테스트 실행

    if (context.shouldExit()) {
        return res;  // early return if test-only mode
    }

    // 여기서 테스트 이후 추가 로직 가능

    return res;
}
