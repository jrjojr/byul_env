#include "doctest.h"

extern "C" {
#include "entity.h"
}

// ---------------------------------------------------------
// 테스트 케이스
// ---------------------------------------------------------

TEST_CASE("entity_init 기본값 확인") {
    entity_t e;
    entity_init(&e);

    CHECK(e.id == -1);
    CHECK(e.coord.x == 0);
    CHECK(e.coord.y == 0);
    CHECK(e.owner == nullptr);
    CHECK(e.age == doctest::Approx(0.0f));
    CHECK(e.lifetime == doctest::Approx(0.0f));
}

TEST_CASE("entity_init_full 사용자 지정 초기화") {
    entity_t e;
    coord_t c = {5, 7};
    // entity_init_full(&e, &c, 42, (void*)0x1234, 1.5f, 10.0f);
    entity_init_full(&e, &c, 42, (void*)0x1234, 1.5f, 10.0f, 0, 0, 1.0);

    CHECK(e.id == 42);
    CHECK(e.coord.x == 5);
    CHECK(e.coord.y == 7);
    CHECK(e.owner == (void*)0x1234);
    CHECK(e.age == doctest::Approx(1.5f));
    CHECK(e.lifetime == doctest::Approx(10.0f));
}

TEST_CASE("entity_init_full coord == NULL") {
    entity_t e;
    // entity_init_full(&e, nullptr, 99, nullptr, 0.0f, 5.0f);
    entity_init_full(&e, nullptr, 99, nullptr, 0.0f, 5.0f, 0, 0, 1.0f);

    CHECK(e.coord.x == 0);
    CHECK(e.coord.y == 0);
    CHECK(e.id == 99);
    CHECK(e.lifetime == doctest::Approx(5.0f));
}

TEST_CASE("entity_assign 복사 테스트") {
    entity_t src;
    coord_t c = {3, 4};
    // entity_init_full(&src, &c, 7, (void*)0x5678, 2.0f, 4.0f);
    entity_init_full(&src, &c, 7, (void*)0x5678, 2.0f, 4.0f, 0, 0, 1.0f);

    entity_t dst;
    entity_assign(&dst, &src);

    CHECK(dst.id == 7);
    CHECK(dst.coord.x == 3);
    CHECK(dst.coord.y == 4);
    CHECK(dst.owner == (void*)0x5678);
    CHECK(dst.age == doctest::Approx(2.0f));
    CHECK(dst.lifetime == doctest::Approx(4.0f));
}

TEST_CASE("entity_is_expired 및 tick") {
    entity_t e;
    // entity_init_full(&e, nullptr, 1, nullptr, 0.0f, 1.0f);
    entity_init_full(&e, nullptr, 1, nullptr, 0.0f, 1.0f, 0, 0, 1.0f);

    CHECK(entity_is_expired(&e) == false);

    bool expired = entity_tick(&e, 0.5f);
    CHECK(expired == false);
    CHECK(e.age == doctest::Approx(0.5f));

    expired = entity_tick(&e, 0.6f);
    CHECK(expired == true);
    CHECK(entity_is_expired(&e) == true);
}
