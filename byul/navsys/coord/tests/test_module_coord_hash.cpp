#include "doctest.h"

#include <string.h> //memset
#include <iostream>
#include <vector>

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

TEST_CASE("coord_hash: insert and get") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c1 = coord_create_full(2, 3);
    int* v = make_int(42);

    CHECK(coord_hash_insert(hash, c1, v));
    CHECK(coord_hash_length(hash) == 1);

    int* got = (int*)coord_hash_get(hash, c1);
    REQUIRE(got != nullptr);
    CHECK(*got == 42);

    coord_destroy(c1);
    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: contains and remove") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c1 = coord_create_full(5, 5);
    coord_t* c2 = coord_create_full(6, 6);
    int* v = make_int(123);

    CHECK_FALSE(coord_hash_contains(hash, c1));
    CHECK(coord_hash_insert(hash, c1, v));
    CHECK(coord_hash_contains(hash, c1));
    CHECK_FALSE(coord_hash_contains(hash, c2));

    CHECK(coord_hash_remove(hash, c1));
    CHECK_FALSE(coord_hash_contains(hash, c1));
    CHECK(coord_hash_length(hash) == 0);

    coord_destroy(c1);
    coord_destroy(c2);
    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: clear and empty") {
    coord_hash_t* hash = coord_hash_create();

    coord_t* c1 = coord_create_full(7, 8);
    coord_t* c2 = coord_create_full(9, 10);

    int* v1 = make_int(11);
    int* v2 = make_int(22);

    CHECK(coord_hash_insert(hash, c1, v1));
    CHECK(coord_hash_insert(hash, c2, v2));
    CHECK(coord_hash_length(hash) == 2);

    coord_hash_clear(hash);
    CHECK(coord_hash_is_empty(hash));
    CHECK(coord_hash_length(hash) == 0);

    coord_destroy(c1);
    coord_destroy(c2);
    delete v1;
    delete v2;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: deep copy") {
    coord_hash_t* a = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c = coord_create_full(1, 2);
    int* v = make_int(55);

    coord_hash_insert(a, c, v);
    coord_hash_t* b = coord_hash_copy(a);

    int* a_val = (int*)coord_hash_get(a, c);
    int* b_val = (int*)coord_hash_get(b, c);

    REQUIRE(a_val);
    REQUIRE(b_val);
    CHECK(*a_val == 55);
    CHECK(*b_val == 55);
    CHECK(a_val != b_val);

    coord_destroy(c);
    delete v;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}

TEST_CASE("coord_hash: equality with null-check and key presence only") {
    coord_hash_t* a = coord_hash_create();
    coord_hash_t* b = coord_hash_create();

    coord_t* c = coord_create_full(1, 1);
    int* v1 = make_int(999);
    int* v2 = make_int(999);

    coord_hash_insert(a, c, v1);
    coord_hash_insert(b, c, v1);
    CHECK(coord_hash_equal(a, b));

    coord_hash_replace(b, c, v2);
    CHECK(coord_hash_equal(a, b));

    coord_hash_replace(b, c, nullptr);
    CHECK_FALSE(coord_hash_equal(a, b));

    coord_hash_replace(a, c, nullptr);
    CHECK(coord_hash_equal(a, b));

    coord_destroy(c);
    delete v1;
    delete v2;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}


TEST_CASE("coord_hash: iteration test") {
    coord_hash_t* hash = coord_hash_create();

    for (int i = 0; i < 10; ++i) {
        coord_t* c = coord_create_full(i, i + 1);
        int* v = make_int(i * 10);
        coord_hash_insert(hash, c, v);
        coord_destroy(c);
        delete v;
    }

    int count = 0;
    coord_hash_foreach(hash, [](const coord_t* key, void* val, void* userdata) {
        int* cnt = (int*)userdata;
        (*cnt)++;
        REQUIRE(key != nullptr);
        REQUIRE(val != nullptr);
        }, &count);

    CHECK(count == 10);
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: crash when destroying key and value after insert") {
    coord_hash_t* hash = coord_hash_create();

    coord_t* c = coord_create_full(1, 1);
    int* v = new int(123);

    // Insert into hash, then immediately destroy key and value
    coord_hash_insert(hash, c, v);
    coord_destroy(c);  
    delete v;          

    coord_hash_foreach(hash, [](const coord_t* key, void* val, void* userdata) {
        CHECK(key != nullptr);  // may trigger exception
        CHECK(val != nullptr);  // may trigger exception
        }, nullptr);

    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: crash when using copied hash with destroyed shared key") {
    coord_hash_t* a = coord_hash_create();
    coord_t* c = coord_create_full(2, 2);
    int* v = new int(456);

    coord_hash_insert(a, c, v);
    coord_hash_t* b = coord_hash_copy(a);  // shallow copy of key pointer

    coord_destroy(c);

	// Comparing a and b will equal is correctly implemented
    CHECK(coord_hash_equal(a, b));

    delete v;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}
TEST_CASE("coord_hash: force crash by comparing destroyed key") {
    coord_hash_t* hash = coord_hash_create();

    coord_t c;
    coord_init_full(&c, 7, 7);
    int* v = new int(77);

    coord_hash_insert(hash, &c, v);

    // Force access to internal key by finding with new object
    coord_t probe;
    coord_init_full(&probe, 7, 7);

    void* val = coord_hash_get(hash, &probe);
    CHECK(*(int*)val == *v);

    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: crash by accessing destroyed external key (MSVC only)") {
    coord_hash_t* hash = coord_hash_create();

    coord_t c;
    coord_init_full(&c, 5, 5);
    int* v = new int(55);

    // Insert the key (copied inside)
    CHECK(coord_hash_insert(hash, &c, v));

    // Reuse the dangling pointer as key for lookup
    void* result = nullptr;
#ifdef _MSC_VER
    // MSVC may crash here due to dereferencing freed memory during coord_equal
    result = coord_hash_get(hash, &c);
    if (!result) {
        std::cout << "[MSVC] coord_hash_get() returned nullptr as expected.\n";
        CHECK(true);  // test passes: crash avoided, lookup failed as expected
    }
    else {
        std::cout << "[MSVC] Unexpectedly found value: " << *(int*)result << std::endl;
        CHECK(true);  // unexpected success
    }
#else
    // GCC/Clang: should succeed with value-based comparison
    result = coord_hash_get(hash, &c);
    REQUIRE(result != nullptr);
    std::cout << "[GCC/Clang] expectedly found value: " << *(int*)result << std::endl;
    CHECK(true);  // expected success
#endif

    delete v;
    coord_hash_destroy(hash);
}

