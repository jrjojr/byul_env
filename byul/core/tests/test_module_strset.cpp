#include "doctest.h"
extern "C" {
    #include "strset.h"
}

TEST_CASE("strset: basic operations") {
    strset_t* set = strset_create();
    REQUIRE(set);

    CHECK(strset_add(set, "apple"));
    CHECK(strset_contains(set, "apple"));
    CHECK_FALSE(strset_add(set, "apple")); // 중복 추가 실패
    CHECK(strset_size(set) == 1);
    CHECK(strset_remove(set, "apple"));
    CHECK(strset_size(set) == 0);

    strset_destroy(set);
}

TEST_CASE("strset: copy and equality") {
    strset_t* a = strset_create();
    strset_add(a, "x");
    strset_add(a, "y");

    strset_t* b = strset_copy(a);
    CHECK(strset_equal(a, b));

    strset_add(b, "z");
    CHECK_FALSE(strset_equal(a, b));

    strset_destroy(a);
    strset_destroy(b);
}
