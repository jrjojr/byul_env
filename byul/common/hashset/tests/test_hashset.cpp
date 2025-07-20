#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

extern "C" {
    #include "internal/hashset.h"
}

TEST_CASE("hashset: basic operations") {
    hashset_t* set = hashset_create();
    REQUIRE(set);

    CHECK(hashset_add(set, (hashkey)"apple"));
    CHECK(hashset_contains(set, (hashkey)"apple"));
    CHECK_FALSE(hashset_add(set, (hashkey)"apple"));  // 중복
    CHECK(hashset_size(set) == 1);
    CHECK(hashset_remove(set, (hashkey)"apple"));
    CHECK(hashset_size(set) == 0);

    hashset_destroy(set);
}

TEST_CASE("hashset: copy and equality") {
    hashset_t* a = hashset_create();
    REQUIRE(a);
    hashset_add(a, (hashkey)"x");
    hashset_add(a, (hashkey)"y");

    hashset_t* b = hashset_copy(a);
    REQUIRE(b);
    CHECK(hashset_equal(a, b));

    hashset_add(b, (hashkey)"z");
    CHECK_FALSE(hashset_equal(a, b));

    hashset_destroy(a);
    hashset_destroy(b);
}
