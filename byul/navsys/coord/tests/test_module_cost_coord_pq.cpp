#include "doctest.h"

#include <functional>
#include <limits>

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

TEST_CASE("cost_coord_pq legacy null and empty contract") {
    coord_t* c = coord_create_full(7, 9);

    cost_coord_pq_destroy(nullptr);
    cost_coord_pq_push(nullptr, 1.0f, c);
    CHECK(cost_coord_pq_peek(nullptr) == nullptr);
    CHECK(cost_coord_pq_pop(nullptr) == nullptr);
    CHECK(cost_coord_pq_peek_cost(nullptr) == 0.0f);
    CHECK(cost_coord_pq_is_empty(nullptr));
    CHECK(!cost_coord_pq_contains(nullptr, c));
    CHECK(!cost_coord_pq_remove(nullptr, 1.0f, c));
    CHECK(cost_coord_pq_length(nullptr) == 0);
    cost_coord_pq_trim_worst(nullptr, 1);

    cost_coord_pq_t* pq = cost_coord_pq_create();
    REQUIRE(pq != nullptr);
    cost_coord_pq_push(pq, 1.0f, nullptr);
    CHECK(cost_coord_pq_length(pq) == 0);
    CHECK(cost_coord_pq_peek(pq) == nullptr);
    CHECK(cost_coord_pq_pop(pq) == nullptr);
    CHECK(cost_coord_pq_peek_cost(pq) == 0.0f);
    CHECK(!cost_coord_pq_contains(pq, nullptr));
    CHECK(!cost_coord_pq_remove(pq, 1.0f, nullptr));

    coord_destroy(c);
    cost_coord_pq_destroy(pq);
}

TEST_CASE("cost_coord_pq legacy ties are LIFO and pop transfers peeked pointer") {
    cost_coord_pq_t* pq = cost_coord_pq_create();
    coord_t* first = coord_create_full(1, 1);
    coord_t* second = coord_create_full(2, 2);
    REQUIRE(pq != nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    cost_coord_pq_push(pq, 4.0f, first);
    cost_coord_pq_push(pq, 4.0f, second);

    coord_t* borrowed = cost_coord_pq_peek(pq);
    REQUIRE(borrowed != nullptr);
    CHECK(borrowed->x == 2);
    CHECK(borrowed->y == 2);

    coord_t* owned = cost_coord_pq_pop(pq);
    CHECK(owned == borrowed);
    coord_destroy(owned);

    owned = cost_coord_pq_pop(pq);
    REQUIRE(owned != nullptr);
    CHECK(owned->x == 1);
    CHECK(owned->y == 1);
    coord_destroy(owned);

    coord_destroy(first);
    coord_destroy(second);
    cost_coord_pq_destroy(pq);
}

TEST_CASE("cost_coord_pq legacy remove deletes all exact-cost duplicates") {
    cost_coord_pq_t* pq = cost_coord_pq_create();
    coord_t* same = coord_create_full(3, 5);
    REQUIRE(pq != nullptr);
    REQUIRE(same != nullptr);

    cost_coord_pq_push(pq, 0.0f, same);
    cost_coord_pq_push(pq, -0.0f, same);
    cost_coord_pq_push(pq, 2.0f, same);
    CHECK(cost_coord_pq_length(pq) == 3);

    CHECK(cost_coord_pq_remove(pq, -0.0f, same));
    CHECK(cost_coord_pq_length(pq) == 1);
    CHECK(cost_coord_pq_contains(pq, same));
    CHECK(!cost_coord_pq_remove(pq, 1.0f, same));

    coord_t* owned = cost_coord_pq_pop(pq);
    REQUIRE(owned != nullptr);
    CHECK(owned->x == 3);
    CHECK(owned->y == 5);
    coord_destroy(owned);

    coord_destroy(same);
    cost_coord_pq_destroy(pq);
}

TEST_CASE("cost_coord_pq legacy trim argument is a remove count") {
    cost_coord_pq_t* pq = cost_coord_pq_create();
    coord_t* c = coord_create_full(8, 13);
    REQUIRE(pq != nullptr);
    REQUIRE(c != nullptr);

    for (int cost = 1; cost <= 4; ++cost) {
        cost_coord_pq_push(pq, static_cast<float>(cost), c);
    }
    cost_coord_pq_trim_worst(pq, 0);
    cost_coord_pq_trim_worst(pq, -1);
    CHECK(cost_coord_pq_length(pq) == 4);

    cost_coord_pq_trim_worst(pq, 2);
    CHECK(cost_coord_pq_length(pq) == 2);
    CHECK(cost_coord_pq_peek_cost(pq) == 1.0f);

    cost_coord_pq_trim_worst(pq, 99);
    CHECK(cost_coord_pq_is_empty(pq));

    coord_destroy(c);
    cost_coord_pq_destroy(pq);
}

TEST_CASE("cost_coord_pq legacy float comparator cannot order NaN") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::less<float> less;

    CHECK(!less(1.0f, nan));
    CHECK(!less(nan, 1.0f));
    CHECK(!less(2.0f, nan));
    CHECK(!less(nan, 2.0f));
    CHECK(less(1.0f, 2.0f));
}
