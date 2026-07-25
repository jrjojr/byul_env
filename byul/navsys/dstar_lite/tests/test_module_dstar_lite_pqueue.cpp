#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <limits>

extern "C" {
#include "dstar_lite_pqueue.h"
#include "coord.h"
#include "dstar_lite_key.h"
}

namespace {

int reference_lexicographic_compare(
    const dstar_lite_key_t& lhs,
    const dstar_lite_key_t& rhs) {
    if (lhs.k1 < rhs.k1) return -1;
    if (lhs.k1 > rhs.k1) return 1;
    if (lhs.k2 < rhs.k2) return -1;
    if (lhs.k2 > rhs.k2) return 1;
    return 0;
}

} // namespace

TEST_CASE("dstar_lite_key legacy layout and allocation ABI") {
    static_assert(sizeof(dstar_lite_key_t) == 8);
    static_assert(alignof(dstar_lite_key_t) == 4);
    static_assert(offsetof(dstar_lite_key_t, k1) == 0);
    static_assert(offsetof(dstar_lite_key_t, k2) == 4);

    dstar_lite_key_t* zero = dstar_lite_key_create();
    REQUIRE(zero != nullptr);
    CHECK(zero->k1 == 0.0f);
    CHECK(zero->k2 == 0.0f);

    dstar_lite_key_t* full = dstar_lite_key_create_full(-0.0f, 7.0f);
    REQUIRE(full != nullptr);
    dstar_lite_key_t* copied = dstar_lite_key_copy(full);
    REQUIRE(copied != nullptr);
    CHECK(copied->k1 == full->k1);
    CHECK(copied->k2 == full->k2);
    CHECK(dstar_lite_key_copy(nullptr) == nullptr);
    CHECK_FALSE(dstar_lite_key_equal(nullptr, full));
    CHECK(dstar_lite_key_hash(nullptr) == 0U);

    dstar_lite_key_destroy(copied);
    dstar_lite_key_destroy(full);
    dstar_lite_key_destroy(zero);
    dstar_lite_key_destroy(nullptr);
}

TEST_CASE("dstar_lite_key legacy approximate relation conflicts with hash") {
    const dstar_lite_key_t exact = {1.0f, 2.0f};
    const dstar_lite_key_t close = {1.000005f, 2.0f};
    CHECK(dstar_lite_key_equal(&exact, &close));
    CHECK(dstar_lite_key_compare(&exact, &close) == 0);
    CHECK(dstar_lite_key_hash(&exact) != dstar_lite_key_hash(&close));

    const dstar_lite_key_t a = {1.0f, 0.0f};
    const dstar_lite_key_t b = {1.000009f, 0.0f};
    const dstar_lite_key_t c = {1.000018f, 0.0f};
    CHECK(dstar_lite_key_equal(&a, &b));
    CHECK(dstar_lite_key_equal(&b, &c));
    CHECK_FALSE(dstar_lite_key_equal(&a, &c));
    CHECK(dstar_lite_key_compare(&a, &b) == 0);
    CHECK(dstar_lite_key_compare(&b, &c) == 0);
    CHECK(dstar_lite_key_compare(&a, &c) < 0);

    CHECK(reference_lexicographic_compare(exact, close) < 0);
    CHECK(reference_lexicographic_compare(a, b) < 0);
    CHECK(reference_lexicographic_compare(b, c) < 0);
}

TEST_CASE("dstar_lite_key legacy special float behavior") {
    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const dstar_lite_key_t positive_zero = {0.0f, 0.0f};
    const dstar_lite_key_t negative_zero = {-0.0f, 0.0f};
    CHECK(dstar_lite_key_equal(&positive_zero, &negative_zero));
    CHECK(dstar_lite_key_compare(&positive_zero, &negative_zero) == 0);
    CHECK(
        dstar_lite_key_hash(&positive_zero)
        != dstar_lite_key_hash(&negative_zero));

    const dstar_lite_key_t finite = {3.0f, 0.0f};
    const dstar_lite_key_t positive_infinity = {infinity, 0.0f};
    const dstar_lite_key_t negative_infinity = {-infinity, 0.0f};
    CHECK(dstar_lite_key_equal(&positive_infinity, &positive_infinity));
    CHECK(dstar_lite_key_equal(&finite, &positive_infinity));
    CHECK(dstar_lite_key_equal(&negative_infinity, &finite));
    CHECK(dstar_lite_key_compare(&finite, &positive_infinity) == 0);
    CHECK(dstar_lite_key_compare(&negative_infinity, &finite) == 0);

    const dstar_lite_key_t nan_key = {nan, 0.0f};
    CHECK_FALSE(dstar_lite_key_equal(&nan_key, &nan_key));
    CHECK(dstar_lite_key_compare(&nan_key, &finite) == 0);
    CHECK(dstar_lite_key_compare(&finite, &nan_key) == 0);

    dstar_lite_key_t* accepted_nan = dstar_lite_key_create_full(nan, 1.0f);
    dstar_lite_key_t* accepted_positive_infinity =
        dstar_lite_key_create_full(infinity, 1.0f);
    dstar_lite_key_t* accepted_negative_infinity =
        dstar_lite_key_create_full(-infinity, 1.0f);
    REQUIRE(accepted_nan != nullptr);
    REQUIRE(accepted_positive_infinity != nullptr);
    REQUIRE(accepted_negative_infinity != nullptr);
    dstar_lite_key_destroy(accepted_nan);
    dstar_lite_key_destroy(accepted_positive_infinity);
    dstar_lite_key_destroy(accepted_negative_infinity);
}

TEST_CASE("dstar_lite_pqueue multiple pushes with same key") {
    auto* q = dstar_lite_pqueue_create();

    dstar_lite_key_t* k = dstar_lite_key_create_full(1.5, 2.5);

    coord_t* c1 = coord_create_full(1, 1);
    coord_t* c2 = coord_create_full(2, 2);
    coord_t* c3 = coord_create_full(3, 3);

    dstar_lite_pqueue_push(q, k, c1);
    dstar_lite_pqueue_push(q, k, c2);
    dstar_lite_pqueue_push(q, k, c3);

    CHECK(dstar_lite_pqueue_contains(q, c1) == true);
    CHECK(dstar_lite_pqueue_contains(q, c2) == true);
    CHECK(dstar_lite_pqueue_contains(q, c3) == true);

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
    dstar_lite_key_t* k2 = dstar_lite_key_create_full(0.5, 1.5);

    coord_t* a1 = coord_create_full(10, 10);
    coord_t* a2 = coord_create_full(11, 11);
    coord_t* b1 = coord_create_full(5, 5);

    dstar_lite_pqueue_push(q, k1, a1);
    dstar_lite_pqueue_push(q, k1, a2);
    dstar_lite_pqueue_push(q, k2, b1);

    const coord_t* peek = dstar_lite_pqueue_peek(q);
    CHECK(coord_equal(peek, b1));

    coord_t* p1 = dstar_lite_pqueue_pop(q);
    CHECK(coord_equal(p1, b1));
    coord_destroy(p1);

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
