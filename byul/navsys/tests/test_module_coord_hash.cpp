#include "doctest.h"

extern "C" {
#include "coord.h"
#include "coord_hash.h"
}

int* make_int(int value) {
    return new int(value);
}

TEST_CASE("coord_hash: replace with int values (new/delete)") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);
    coord_t* c = coord_create_full(1, 1);

    int* v1 = make_int(100);
    int* v2 = make_int(200);
    int* v3 = make_int(300);

    CHECK(coord_hash_replace(hash, c, v1));
    int* found1 = (int*)coord_hash_get(hash, c);
    REQUIRE(found1 != nullptr);
    CHECK(*found1 == 100);

    CHECK(coord_hash_replace(hash, c, v2));
    int* found2 = (int*)coord_hash_get(hash, c);
    REQUIRE(found2 != nullptr);
    CHECK(*found2 == 200);

    CHECK(coord_hash_replace(hash, c, v3));
    int* found3 = (int*)coord_hash_get(hash, c);
    REQUIRE(found3 != nullptr);
    CHECK(*found3 == 300);

    coord_destroy(c);
    delete v1;
    delete v2;
    delete v3;
    coord_hash_destroy(hash);
}
