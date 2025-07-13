#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

extern "C" {
    #include "internal/cost_coord_pq.h"
    #include "internal/coord.h"
}

TEST_CASE("cost_coord_pq: 기본 삽입 및 pop") {
    cost_coord_pq_t* pq = cost_coord_pq_new();

    coord_t* c1 = coord_new_full(1, 1);
    coord_t* c2 = coord_new_full(2, 2);
    coord_t* c3 = coord_new_full(3, 3);

    cost_coord_pq_push(pq, 5.0f, c1);
    cost_coord_pq_push(pq, 2.0f, c2);
    cost_coord_pq_push(pq, 5.0f, c3);

    CHECK(cost_coord_pq_length(pq) == 3);
    CHECK(!cost_coord_pq_is_empty(pq));
    CHECK(cost_coord_pq_peek_cost(pq) == doctest::Approx(2.0f));

    coord_t* out = cost_coord_pq_pop(pq);
    CHECK(out->x == 2);
    CHECK(out->y == 2);
    coord_free(out);

    CHECK(cost_coord_pq_length(pq) == 2);
    CHECK(cost_coord_pq_peek_cost(pq) == doctest::Approx(5.0f));

    coord_free(c1);
    coord_free(c2);
    coord_free(c3);
    cost_coord_pq_free(pq);
}

TEST_CASE("cost_coord_pq: contains & remove & trim") {
    cost_coord_pq_t* pq = cost_coord_pq_new();

    coord_t* c1 = coord_new_full(1, 1);
    coord_t* c2 = coord_new_full(2, 2);
    coord_t* c3 = coord_new_full(3, 3);
    coord_t* c4 = coord_new_full(4, 4);

    cost_coord_pq_push(pq, 1.0f, c1);
    cost_coord_pq_push(pq, 1.0f, c2);
    cost_coord_pq_push(pq, 2.0f, c3);
    cost_coord_pq_push(pq, 3.0f, c4);

    CHECK(cost_coord_pq_contains(pq, c2));
    CHECK(cost_coord_pq_remove(pq, 1.0f, c2));
    CHECK(!cost_coord_pq_contains(pq, c2));
    CHECK(cost_coord_pq_length(pq) == 3);

    cost_coord_pq_trim_worst(pq, 2);
    CHECK(cost_coord_pq_length(pq) == 1);

    coord_t* remaining = cost_coord_pq_pop(pq);
    CHECK(remaining != nullptr);
    coord_free(remaining);

    CHECK(cost_coord_pq_is_empty(pq));

    coord_free(c1);
    coord_free(c2);
    coord_free(c3);
    coord_free(c4);
    cost_coord_pq_free(pq);
}
