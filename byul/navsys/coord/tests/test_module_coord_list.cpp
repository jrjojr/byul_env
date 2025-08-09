#include "doctest.h"

extern "C" {
#include "coord_list.h"
#include "coord.h"
}

TEST_CASE("coord_list") {
    coord_list_t* list = coord_list_create();
    REQUIRE(list != nullptr);
    CHECK(coord_list_length(list) == 0);
    CHECK(coord_list_empty(list));

    coord_list_destroy(list);
}

TEST_CASE("coord_list: push_back, get, front, back") {
    coord_list_t* list = coord_list_create();

    coord_t* a = coord_create_full(1, 2);
    coord_t* b = coord_create_full(3, 4);
    coord_t* c = coord_create_full(5, 6);

    coord_list_push_back(list, a);
    coord_list_push_back(list, b);
    coord_list_push_back(list, c);

    CHECK(coord_list_length(list) == 3);
    CHECK(coord_equal(coord_list_get(list, 0), a));
    CHECK(coord_equal(coord_list_get(list, 1), b));
    CHECK(coord_equal(coord_list_get(list, 2), c));
    CHECK(coord_equal(coord_list_front(list), a));
    CHECK(coord_equal(coord_list_back(list), c));

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_list_destroy(list);
}

TEST_CASE("coord_list: pop_back, pop_front") {
    coord_list_t* list = coord_list_create();

    coord_t* a = coord_create_full(10, 10);
    coord_t* b = coord_create_full(20, 20);
    coord_t* c = coord_create_full(30, 30);

    coord_list_push_back(list, a);
    coord_list_push_back(list, b);
    coord_list_push_back(list, c);

    coord_t back = coord_list_pop_back(list);
    CHECK(coord_equal(&back, c));

    coord_t front = coord_list_pop_front(list);
    CHECK(coord_equal(&front, a));

    CHECK(coord_list_length(list) == 1);
    CHECK(coord_equal(coord_list_front(list), b));

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_list_destroy(list);
}

TEST_CASE("coord_list: insert, remove_at, remove_value") {
    coord_list_t* list = coord_list_create();

    coord_t* a = coord_create_full(1, 1);
    coord_t* b = coord_create_full(2, 2);
    coord_t* c = coord_create_full(3, 3);
    coord_list_push_back(list, a);
    coord_list_push_back(list, c);

    coord_list_insert(list, 1, b);
    CHECK(coord_list_length(list) == 3);
    CHECK(coord_equal(coord_list_get(list, 1), b));

    coord_list_remove_at(list, 1);
    CHECK(coord_list_length(list) == 2);
    CHECK(coord_equal(coord_list_get(list, 1), c));

    coord_list_remove_value(list, c);
    CHECK(coord_list_length(list) == 1);
    CHECK(coord_equal(coord_list_get(list, 0), a));

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_list_destroy(list);
}

TEST_CASE("coord_list: clear, reverse, copy") {
    coord_list_t* list = coord_list_create();

    coord_t* a = coord_create_full(1, 1);
    coord_t* b = coord_create_full(2, 2);
    coord_t* c = coord_create_full(3, 3);

    coord_list_push_back(list, a);
    coord_list_push_back(list, b);
    coord_list_push_back(list, c);

    coord_list_reverse(list);
    CHECK(coord_equal(coord_list_get(list, 0), c));
    CHECK(coord_equal(coord_list_get(list, 2), a));

    coord_list_t* copy = coord_list_copy(list);
    CHECK(coord_list_equals(list, copy));

    coord_list_clear(list);
    CHECK(coord_list_empty(list));

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_list_destroy(list);
    coord_list_destroy(copy);
}

TEST_CASE("coord_list: contains, find, sublist") {
    coord_list_t* list = coord_list_create();

    coord_t* a = coord_create_full(10, 10);
    coord_t* b = coord_create_full(20, 20);
    coord_t* c = coord_create_full(30, 30);

    coord_list_push_back(list, a);
    coord_list_push_back(list, b);
    coord_list_push_back(list, c);

    CHECK(coord_list_contains(list, b));
    CHECK(coord_list_find(list, b) == 1);
    coord_t tmp = {999, 999};
    CHECK(coord_list_find(list, &tmp) == -1);

    coord_list_t* sub = coord_list_sublist(list, 1, 3);
    CHECK(coord_list_length(sub) == 2);
    CHECK(coord_equal(coord_list_get(sub, 0), b));
    CHECK(coord_equal(coord_list_get(sub, 1), c));

    coord_destroy(a);
    coord_destroy(b);
    coord_destroy(c);
    coord_list_destroy(list);
    coord_list_destroy(sub);
}
