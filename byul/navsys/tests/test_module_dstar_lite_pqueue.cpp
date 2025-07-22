#include "doctest.h"

extern "C" {
#include "internal/dstar_lite_pqueue.h"
#include "internal/coord.h"
#include "internal/dstar_lite_key.h"
}

TEST_CASE("dstar_lite_pqueue multiple pushes with same key") {
    auto* q = dstar_lite_pqueue_create();

    dstar_lite_key_t* k = dstar_lite_key_create_full(1.5, 2.5);

    coord_t* c1 = coord_create_full(1, 1);
    coord_t* c2 = coord_create_full(2, 2);
    coord_t* c3 = coord_create_full(3, 3);

    // 같은 key로 여러 좌표 push
    dstar_lite_pqueue_push(q, k, c1);
    dstar_lite_pqueue_push(q, k, c2);
    dstar_lite_pqueue_push(q, k, c3);

    CHECK(dstar_lite_pqueue_contains(q, c1) == true);
    CHECK(dstar_lite_pqueue_contains(q, c2) == true);
    CHECK(dstar_lite_pqueue_contains(q, c3) == true);

    // pop 순서 확인 (추정상 삽입 순서일 가능성 높음)
    coord_t* p1 = dstar_lite_pqueue_pop(q);
    CHECK((coord_equal(p1, c1) || coord_equal(p1, c2) || coord_equal(p1, c3)));
    coord_destroy(p1);

    coord_t* p2 = dstar_lite_pqueue_pop(q);
    CHECK((coord_equal(p2, c1) || coord_equal(p2, c2) || coord_equal(p2, c3)));
    coord_destroy(p2);

    coord_t* p3 = dstar_lite_pqueue_pop(q);
    CHECK((coord_equal(p3, c1) || coord_equal(p3, c2) || coord_equal(p3, c3)));
    coord_destroy(p3);

    CHECK(dstar_lite_pqueue_is_empty(q));

    // clean-up
    coord_destroy(c1);
    coord_destroy(c2);
    coord_destroy(c3);
    dstar_lite_key_destroy(k);
    dstar_lite_pqueue_destroy(q);
}

TEST_CASE("dstar_lite_pqueue interleaved same key and different key") {
    auto* q = dstar_lite_pqueue_create();

    dstar_lite_key_t* k1 = dstar_lite_key_create_full(1.0, 2.0);
    dstar_lite_key_t* k2 = dstar_lite_key_create_full(0.5, 1.5); // 더 우선순위 높음

    coord_t* a1 = coord_create_full(10, 10);
    coord_t* a2 = coord_create_full(11, 11);
    coord_t* b1 = coord_create_full(5, 5);

    dstar_lite_pqueue_push(q, k1, a1);
    dstar_lite_pqueue_push(q, k1, a2);
    dstar_lite_pqueue_push(q, k2, b1);

    // peek하면 b1이 나와야 함 (key 우선순위)
    const coord_t* peek = dstar_lite_pqueue_peek(q);
    CHECK(coord_equal(peek, b1));

    // pop 순서도 우선순위 기준으로 나와야 함
    coord_t* p1 = dstar_lite_pqueue_pop(q);
    CHECK(coord_equal(p1, b1));
    coord_destroy(p1);

    // 이제 a1, a2 두 개 남음
    CHECK(dstar_lite_pqueue_contains(q, a1));
    CHECK(dstar_lite_pqueue_contains(q, a2));

    dstar_lite_pqueue_remove(q, a1);
    CHECK_FALSE(dstar_lite_pqueue_contains(q, a1));

    dstar_lite_pqueue_remove(q, a2);
    CHECK_FALSE(dstar_lite_pqueue_contains(q, a2));

    coord_destroy(a1);
    coord_destroy(a2);
    coord_destroy(b1);
    dstar_lite_key_destroy(k1);
    dstar_lite_key_destroy(k2);
    dstar_lite_pqueue_destroy(q);
}
