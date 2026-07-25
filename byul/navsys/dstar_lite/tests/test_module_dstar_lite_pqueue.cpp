#include "doctest.h"
#include "internal/dstar_lite_key_ops.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <unordered_set>

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

struct ExactKeyHash {
    std::size_t operator()(const dstar_lite_key_t& key) const {
        return dstar_lite_key_hash_exact(&key);
    }
};

struct ExactKeyEqual {
    bool operator()(
        const dstar_lite_key_t& lhs,
        const dstar_lite_key_t& rhs) const {
        return dstar_lite_key_equal_exact(&lhs, &rhs);
    }
};

} // namespace

bool dstar_lite_key_ops_odr_a(
    const dstar_lite_key_t& lhs,
    const dstar_lite_key_t& rhs);
bool dstar_lite_key_ops_odr_b(
    const dstar_lite_key_t& lhs,
    const dstar_lite_key_t& rhs);

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
    CHECK_FALSE(std::signbit(full->k1));
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

TEST_CASE("dstar_lite_key explicit closeness is not container identity") {
    const dstar_lite_key_t exact = {1.0f, 2.0f};
    const dstar_lite_key_t close = {1.000005f, 2.0f};
    CHECK_FALSE(dstar_lite_key_equal(&exact, &close));
    CHECK(dstar_lite_key_compare(&exact, &close) < 0);
    CHECK(dstar_lite_key_hash(&exact) != dstar_lite_key_hash(&close));

    bool is_close = false;
    CHECK(dstar_lite_key_is_close(
        &exact, &close, 0.0f, 1e-5f, &is_close) == NAVSYS_STATUS_OK);
    CHECK(is_close);

    const dstar_lite_key_t a = {1.0f, 0.0f};
    const dstar_lite_key_t b = {1.000009f, 0.0f};
    const dstar_lite_key_t c = {1.000018f, 0.0f};
    CHECK(dstar_lite_key_is_close(
        &a, &b, 0.0f, 1e-5f, &is_close) == NAVSYS_STATUS_OK);
    CHECK(is_close);
    CHECK(dstar_lite_key_is_close(
        &b, &c, 0.0f, 1e-5f, &is_close) == NAVSYS_STATUS_OK);
    CHECK(is_close);
    CHECK(dstar_lite_key_is_close(
        &a, &c, 0.0f, 1e-5f, &is_close) == NAVSYS_STATUS_OK);
    CHECK_FALSE(is_close);

    CHECK(reference_lexicographic_compare(exact, close) < 0);
    CHECK(reference_lexicographic_compare(a, b) < 0);
    CHECK(reference_lexicographic_compare(b, c) < 0);

    is_close = true;
    CHECK(dstar_lite_key_is_close(
        &exact, &close, 0.0f, 0.0f, &is_close) == NAVSYS_STATUS_OK);
    CHECK_FALSE(is_close);
    is_close = true;
    CHECK(dstar_lite_key_is_close(
        &exact, &close, -1.0f, 0.0f, &is_close)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(is_close);
    CHECK(dstar_lite_key_is_close(
        &exact, &close, 0.0f,
        std::numeric_limits<float>::infinity(), &is_close)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(is_close);
}

TEST_CASE("dstar_lite_key legacy API forwards canonical semantics") {
    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const dstar_lite_key_t positive_zero = {0.0f, 0.0f};
    const dstar_lite_key_t negative_zero = {-0.0f, 0.0f};
    CHECK(dstar_lite_key_equal(&positive_zero, &negative_zero));
    CHECK(dstar_lite_key_compare(&positive_zero, &negative_zero) == 0);
    CHECK(
        dstar_lite_key_hash(&positive_zero)
        == dstar_lite_key_hash(&negative_zero));

    const dstar_lite_key_t finite = {3.0f, 0.0f};
    const dstar_lite_key_t positive_infinity = {infinity, 0.0f};
    const dstar_lite_key_t negative_infinity = {-infinity, 0.0f};
    CHECK(dstar_lite_key_equal(&positive_infinity, &positive_infinity));
    CHECK_FALSE(dstar_lite_key_equal(&finite, &positive_infinity));
    CHECK_FALSE(dstar_lite_key_equal(&negative_infinity, &finite));
    CHECK(dstar_lite_key_compare(&finite, &positive_infinity) < 0);
    CHECK(dstar_lite_key_compare(&negative_infinity, &finite) == 0);

    bool is_close = false;
    CHECK(dstar_lite_key_is_close(
        &positive_infinity, &positive_infinity, 0.0f, 0.0f, &is_close)
        == NAVSYS_STATUS_OK);
    CHECK(is_close);
    CHECK(dstar_lite_key_is_close(
        &finite, &positive_infinity, infinity, 0.0f, &is_close)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(is_close);

    const dstar_lite_key_t nan_key = {nan, 0.0f};
    CHECK_FALSE(dstar_lite_key_equal(&nan_key, &nan_key));
    CHECK(dstar_lite_key_compare(&nan_key, &finite) == 0);
    CHECK(dstar_lite_key_compare(&finite, &nan_key) == 0);

    dstar_lite_key_t* accepted_nan = dstar_lite_key_create_full(nan, 1.0f);
    dstar_lite_key_t* accepted_positive_infinity =
        dstar_lite_key_create_full(infinity, 1.0f);
    dstar_lite_key_t* accepted_negative_infinity =
        dstar_lite_key_create_full(-infinity, 1.0f);
    CHECK(accepted_nan == nullptr);
    REQUIRE(accepted_positive_infinity != nullptr);
    CHECK(accepted_negative_infinity == nullptr);
    dstar_lite_key_destroy(accepted_nan);
    dstar_lite_key_destroy(accepted_positive_infinity);
    dstar_lite_key_destroy(accepted_negative_infinity);
}

TEST_CASE("dstar_lite_key canonical value and layout API") {
    CHECK(dstar_lite_key_sizeof() == sizeof(dstar_lite_key_t));
    CHECK(dstar_lite_key_alignof() == alignof(dstar_lite_key_t));
    CHECK(dstar_lite_key_offsetof_k1() == offsetof(dstar_lite_key_t, k1));
    CHECK(dstar_lite_key_offsetof_k2() == offsetof(dstar_lite_key_t, k2));

    dstar_lite_key_t key = {17.0f, 19.0f};
    CHECK(dstar_lite_key_init(&key, -0.0f, 2.0f) == NAVSYS_STATUS_OK);
    CHECK(key.k1 == 0.0f);
    CHECK_FALSE(std::signbit(key.k1));
    CHECK(key.k2 == 2.0f);

    const float infinity = std::numeric_limits<float>::infinity();
    CHECK(dstar_lite_key_init(&key, infinity, infinity) == NAVSYS_STATUS_OK);
    CHECK(key.k1 == infinity);
    CHECK(key.k2 == infinity);

    const dstar_lite_key_t preserved = {17.0f, 19.0f};
    key = preserved;
    CHECK(dstar_lite_key_init(
        &key, std::numeric_limits<float>::quiet_NaN(), 0.0f)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(key.k1 == preserved.k1);
    CHECK(key.k2 == preserved.k2);
    CHECK(dstar_lite_key_init(&key, -infinity, 0.0f)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(key.k1 == preserved.k1);
    CHECK(key.k2 == preserved.k2);
    CHECK(dstar_lite_key_init(nullptr, 0.0f, 0.0f)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
}

TEST_CASE("dstar_lite_key checked allocation preserves outputs") {
    dstar_lite_key_t* const sentinel =
        reinterpret_cast<dstar_lite_key_t*>(std::uintptr_t{1});
    dstar_lite_key_t* created = sentinel;
    CHECK(dstar_lite_key_create_ex(-0.0f, 3.0f, &created)
        == NAVSYS_STATUS_OK);
    REQUIRE(created != nullptr);
    CHECK(created != sentinel);
    CHECK(created->k1 == 0.0f);
    CHECK_FALSE(std::signbit(created->k1));
    CHECK(created->k2 == 3.0f);

    dstar_lite_key_t* copied = sentinel;
    CHECK(dstar_lite_key_copy_ex(created, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(copied != sentinel);
    CHECK(dstar_lite_key_equal_exact(created, copied));
    dstar_lite_key_destroy(copied);
    dstar_lite_key_destroy(created);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    created = sentinel;
    CHECK(dstar_lite_key_create_ex(nan, 0.0f, &created)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(created == sentinel);
    CHECK(dstar_lite_key_create_ex(0.0f, 0.0f, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    const dstar_lite_key_t invalid = {nan, 0.0f};
    copied = sentinel;
    CHECK(dstar_lite_key_copy_ex(&invalid, &copied)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(copied == sentinel);
    CHECK(dstar_lite_key_copy_ex(nullptr, &copied)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(copied == sentinel);
}

TEST_CASE("dstar_lite_key exact relation orders canonical corpus") {
    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float adjacent = std::nextafter(1.0f, 2.0f);
    const std::array<dstar_lite_key_t, 8> keys = {{
        {nan, 0.0f}, // Invalid sentinel: excluded from the valid corpus.
        {-3.0f, 5.0f},
        {-0.0f, 0.0f},
        {0.0f, 0.0f},
        {1.0f, -2.0f},
        {1.0f, -1.0f},
        {adjacent, -1.0f},
        {infinity, infinity}
    }};

    CHECK_FALSE(dstar_lite_key_equal_exact(&keys[0], &keys[0]));
    CHECK(dstar_lite_key_hash_exact(&keys[0]) == 0U);
    int preserved_compare = 23;
    CHECK(dstar_lite_key_compare_exact(
        &keys[0], &keys[1], &preserved_compare)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved_compare == 23);
    CHECK(dstar_lite_key_compare_exact(
        &keys[1], &keys[2], nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    for (std::size_t i = 1; i < keys.size(); ++i) {
        for (std::size_t j = 1; j < keys.size(); ++j) {
            int forward = 7;
            int reverse = 7;
            REQUIRE(dstar_lite_key_compare_exact(
                &keys[i], &keys[j], &forward) == NAVSYS_STATUS_OK);
            REQUIRE(dstar_lite_key_compare_exact(
                &keys[j], &keys[i], &reverse) == NAVSYS_STATUS_OK);
            CHECK(forward == -reverse);
            CHECK((forward == 0)
                == dstar_lite_key_equal_exact(&keys[i], &keys[j]));
            if (forward == 0) {
                CHECK(dstar_lite_key_hash_exact(&keys[i])
                    == dstar_lite_key_hash_exact(&keys[j]));
            }
        }
    }

    for (std::size_t i = 1; i < keys.size(); ++i) {
        for (std::size_t j = 1; j < keys.size(); ++j) {
            for (std::size_t k = 1; k < keys.size(); ++k) {
                int ij = 0;
                int jk = 0;
                int ik = 0;
                REQUIRE(dstar_lite_key_compare_exact(
                    &keys[i], &keys[j], &ij) == NAVSYS_STATUS_OK);
                REQUIRE(dstar_lite_key_compare_exact(
                    &keys[j], &keys[k], &jk) == NAVSYS_STATUS_OK);
                REQUIRE(dstar_lite_key_compare_exact(
                    &keys[i], &keys[k], &ik) == NAVSYS_STATUS_OK);
                if (ij < 0 && jk < 0) CHECK(ik < 0);
            }
        }
    }

    CHECK(dstar_lite_key_equal_exact(&keys[2], &keys[3]));
    CHECK(dstar_lite_key_hash_exact(&keys[2])
        == dstar_lite_key_hash_exact(&keys[3]));
    CHECK_FALSE(dstar_lite_key_equal_exact(&keys[4], &keys[6]));

    using byul::navsys::dstar_lite_detail::key_less;
    static_assert(noexcept(
        key_less{}(dstar_lite_key_t{}, dstar_lite_key_t{})));
    const key_less less;
    for (std::size_t i = 1; i < keys.size(); ++i) {
        CHECK_FALSE(less(keys[i], keys[i]));
        for (std::size_t j = 1; j < keys.size(); ++j) {
            const int expected = reference_lexicographic_compare(
                keys[i], keys[j]);
            CHECK(less(keys[i], keys[j]) == (expected < 0));
            if (less(keys[i], keys[j])) {
                CHECK_FALSE(less(keys[j], keys[i]));
            }
        }
    }
    for (std::size_t i = 1; i < keys.size(); ++i) {
        for (std::size_t j = 1; j < keys.size(); ++j) {
            for (std::size_t k = 1; k < keys.size(); ++k) {
                const bool ij_equivalent =
                    !less(keys[i], keys[j]) && !less(keys[j], keys[i]);
                const bool jk_equivalent =
                    !less(keys[j], keys[k]) && !less(keys[k], keys[j]);
                if (less(keys[i], keys[j]) && less(keys[j], keys[k])) {
                    CHECK(less(keys[i], keys[k]));
                }
                if (ij_equivalent && jk_equivalent) {
                    CHECK_FALSE(less(keys[i], keys[k]));
                    CHECK_FALSE(less(keys[k], keys[i]));
                }
            }
        }
    }

    CHECK(dstar_lite_key_ops_odr_a(keys[4], keys[6]));
    CHECK(dstar_lite_key_ops_odr_b(keys[4], keys[6]));
    CHECK_FALSE(dstar_lite_key_ops_odr_a(keys[6], keys[4]));
    CHECK_FALSE(dstar_lite_key_ops_odr_b(keys[6], keys[4]));

    std::set<dstar_lite_key_t, key_less> ordered;
    std::unordered_set<dstar_lite_key_t, ExactKeyHash, ExactKeyEqual> hashed;
    for (std::size_t i = 1; i < keys.size(); ++i) {
        ordered.insert(keys[i]);
        hashed.insert(keys[i]);
    }
    CHECK(ordered.size() == 6);
    CHECK(hashed.size() == 6);
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
