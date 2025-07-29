#include "doctest.h"

extern "C" {
    #include "cost_coord_pq.h"
    #include "coord.h"
}

TEST_CASE("cost_coord_pq v1") {
    cost_coord_pq_t* pq = cost_coord_pq_create();

    coord_t* c1 = coord_create_full(1, 1);
    coord_t* c2 = coord_create_full(2, 2);
    coord_t* c3 = coord_create_full(3, 3);

    cost_coord_pq_push(pq, 5.0f, c1);
    cost_coord_pq_push(pq, 2.0f, c2);
    cost_coord_pq_push(pq, 5.0f, c3);

    CHECK(cost_coord_pq_length(pq) == 3);
    CHECK(!cost_coord_pq_is_empty(pq));
    CHECK(cost_coord_pq_peek_cost(pq) == doctest::Approx(2.0f));

    coord_t* out = cost_coord_pq_pop(pq);
    CHECK(out->x == 2);
    CHECK(out->y == 2);
    coord_destroy(out);

    CHECK(cost_coord_pq_length(pq) == 2);
    CHECK(cost_coord_pq_peek_cost(pq) == doctest::Approx(5.0f));

    coord_destroy(c1);
    coord_destroy(c2);
    coord_destroy(c3);
    cost_coord_pq_destroy(pq);
}

TEST_CASE("cost_coord_pq: contains & remove & trim") {
    cost_coord_pq_t* pq = cost_coord_pq_create();

    coord_t* c1 = coord_create_full(1, 1);
    coord_t* c2 = coord_create_full(2, 2);
    coord_t* c3 = coord_create_full(3, 3);
    coord_t* c4 = coord_create_full(4, 4);

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
    coord_destroy(remaining);

    CHECK(cost_coord_pq_is_empty(pq));

    coord_destroy(c1);
    coord_destroy(c2);
    coord_destroy(c3);
    coord_destroy(c4);
    cost_coord_pq_destroy(pq);
}
