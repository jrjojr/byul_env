#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

extern "C" {
    #include "internal/core.h"
}

TEST_CASE("float_equal: similar and dissimilar values") {
    CHECK(float_equal(1.000001f, 1.000002f));
    CHECK(float_equal(1.000001f, 1.000003f));
    CHECK(float_equal(1.000001f, 1.000009f));

    CHECK(float_equal(1.00001f, 1.000019f));
    CHECK(float_equal(1.00001f, 1.000001f));

    CHECK_FALSE(float_equal(1.00001f, 1.000020f));
    CHECK_FALSE(float_equal(1.00001f, 1.000000f));

    CHECK_FALSE(float_equal(1.0f, 1.1f));
}

TEST_CASE("float_compare: ordering (value-based)") {
    CHECK(float_compare(1.0f, 2.0f, nullptr) < 0);
    CHECK(float_compare(2.0f, 1.0f, nullptr) > 0);
    CHECK(float_compare(1.0f, 1.0f, nullptr) == 0);
}

TEST_CASE("int_compare: ordering (value-based)") {
    CHECK(int_compare(3, 7, nullptr) < 0);
    CHECK(int_compare(10, 5, nullptr) > 0);
    CHECK(int_compare(42, 42, nullptr) == 0);
}

TEST_CASE("hashset: basic operations") {
    hashset_t* set = hashset_new();
    REQUIRE(set);

    CHECK(hashset_add(set, (hashkey)"apple"));
    CHECK(hashset_contains(set, (hashkey)"apple"));
    CHECK_FALSE(hashset_add(set, (hashkey)"apple"));  // 중복
    CHECK(hashset_size(set) == 1);
    CHECK(hashset_remove(set, (hashkey)"apple"));
    CHECK(hashset_size(set) == 0);

    hashset_free(set);
}

TEST_CASE("hashset: copy and equality") {
    hashset_t* a = hashset_new();
    REQUIRE(a);
    hashset_add(a, (hashkey)"x");
    hashset_add(a, (hashkey)"y");

    hashset_t* b = hashset_copy(a);
    REQUIRE(b);
    CHECK(hashset_equal(a, b));

    hashset_add(b, (hashkey)"z");
    CHECK_FALSE(hashset_equal(a, b));

    hashset_free(a);
    hashset_free(b);
}
