#include "doctest.h"

extern "C" {
#include "coord_list.h"
#include "coord.h"
}

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

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

TEST_CASE("coord_list: legacy boundary characterization") {
    coord_t zero = {0, 0};
    coord_t other = {7, 9};

    CHECK(coord_list_length(nullptr) == 0);
    CHECK(coord_list_empty(nullptr));
    CHECK(coord_list_get(nullptr, 0) == nullptr);
    CHECK(coord_list_front(nullptr) == nullptr);
    CHECK(coord_list_back(nullptr) == nullptr);
    CHECK(coord_list_push_back(nullptr, &zero) == 0);
    CHECK(coord_list_insert(nullptr, 0, &zero) == 0);
    coord_t null_back = coord_list_pop_back(nullptr);
    coord_t null_front = coord_list_pop_front(nullptr);
    CHECK(coord_equal(&null_back, &zero));
    CHECK(coord_equal(&null_front, &zero));
    CHECK(coord_list_contains(nullptr, &zero) == 0);
    CHECK(coord_list_find(nullptr, &zero) == -1);
    CHECK(coord_list_sublist(nullptr, 0, 0) == nullptr);
    CHECK_FALSE(coord_list_equals(nullptr, nullptr));

    coord_list_t* list = coord_list_create();
    REQUIRE(list != nullptr);

    coord_t popped = coord_list_pop_back(list);
    CHECK(coord_equal(&popped, &zero));
    CHECK(coord_list_empty(list));

    REQUIRE(coord_list_push_back(list, &zero) == 1);
    REQUIRE(coord_list_push_back(list, &other) == 1);
    REQUIRE(coord_list_insert(list, coord_list_length(list), &zero) == 1);
    CHECK(coord_list_length(list) == 3);
    CHECK(coord_list_get(list, -1) == nullptr);
    CHECK(coord_list_get(list, coord_list_length(list)) == nullptr);
    CHECK(coord_list_insert(list, -1, &zero) == 0);
    CHECK(coord_list_insert(list, coord_list_length(list) + 1, &zero) == 0);

    coord_list_remove_at(list, -1);
    coord_list_remove_at(list, coord_list_length(list));
    CHECK(coord_list_length(list) == 3);

    coord_list_remove_value(list, &zero);
    CHECK(coord_list_length(list) == 2);
    CHECK(coord_equal(coord_list_front(list), &other));
    CHECK(coord_list_find(list, &zero) == 1);

    coord_list_t* full =
        coord_list_sublist(list, 0, coord_list_length(list));
    REQUIRE(full != nullptr);
    CHECK(coord_list_equals(list, full));
    CHECK(coord_list_sublist(list, 0, 0) == nullptr);
    CHECK(coord_list_sublist(list, 1, 1) == nullptr);
    CHECK(coord_list_sublist(list, 2, 1) == nullptr);
    CHECK(coord_list_sublist(list, -1, 1) == nullptr);
    CHECK(coord_list_sublist(list, 0, 3) == nullptr);

    coord_list_destroy(full);
    coord_list_destroy(list);
}

TEST_CASE("coord_list: checked status preserves outputs and supports aliases") {
    coord_list_t* const pointer_sentinel =
        reinterpret_cast<coord_list_t*>(1);
    coord_list_t* created = pointer_sentinel;
    CHECK(
        coord_list_create_ex(nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(created == pointer_sentinel);
    REQUIRE(coord_list_create_ex(&created) == NAVSYS_STATUS_OK);
    REQUIRE(created != nullptr);

    coord_t output = {91, 92};
    CHECK(
        coord_list_fetch(created, 0, &output)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(output.x == 91);
    CHECK(output.y == 92);
    CHECK(
        coord_list_try_pop_front(created, &output)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(output.x == 91);
    CHECK(output.y == 92);
    CHECK(
        coord_list_fetch(nullptr, 0, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output.x == 91);
    CHECK(output.y == 92);

    coord_t first = {1, 2};
    coord_t second = {3, 4};
    REQUIRE(
        coord_list_push_back_ex(created, &first)
        == NAVSYS_STATUS_OK);
    REQUIRE(
        coord_list_push_back_ex(created, &second)
        == NAVSYS_STATUS_OK);

    const coord_t* borrowed = coord_list_get(created, 0);
    REQUIRE(borrowed != nullptr);
    REQUIRE(
        coord_list_push_back_ex(created, borrowed)
        == NAVSYS_STATUS_OK);
    CHECK(coord_list_size(created) == 3);
    REQUIRE(
        coord_list_insert_ex(created, 1, coord_list_get(created, 2))
        == NAVSYS_STATUS_OK);
    CHECK(coord_list_size(created) == 4);

    bool removed = false;
    REQUIRE(
        coord_list_remove_value_ex(
            created, coord_list_get(created, 0), &removed)
        == NAVSYS_STATUS_OK);
    CHECK(removed);
    CHECK(coord_list_size(created) == 3);

    size_t found_index = 99;
    bool found = false;
    REQUIRE(
        coord_list_find_ex(
            created, &first, &found_index, &found)
        == NAVSYS_STATUS_OK);
    CHECK(found);
    CHECK(found_index == 0);

    coord_t missing = {8, 8};
    found_index = 77;
    found = true;
    REQUIRE(
        coord_list_find_ex(
            created, &missing, &found_index, &found)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(found);
    CHECK(found_index == 77);

    coord_list_t* copied = pointer_sentinel;
    REQUIRE(coord_list_copy_ex(created, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(copied != created);
    CHECK(coord_list_size(copied) == coord_list_size(created));

    coord_t removed_coord = {0, 0};
    REQUIRE(
        coord_list_remove_at_ex(created, 0, &removed_coord)
        == NAVSYS_STATUS_OK);
    CHECK(coord_equal(&removed_coord, &first));
    CHECK(coord_list_size(created) + 1 == coord_list_size(copied));

    coord_list_destroy(copied);
    coord_list_destroy(created);
}

TEST_CASE("coord_list: checked mutations match deterministic vector reference") {
    coord_list_t* list = nullptr;
    REQUIRE(coord_list_create_ex(&list) == NAVSYS_STATUS_OK);
    std::vector<coord_t> reference;
    std::uint32_t state = UINT32_C(0x5a17c9e3);

    for (size_t step = 0; step < 128; ++step) {
        state = state * UINT32_C(1664525) + UINT32_C(1013904223);
        const unsigned operation = state % 5U;
        coord_t value = {
            static_cast<int>(state & UINT32_C(0x7fff)),
            static_cast<int>((state >> 16) & UINT32_C(0x7fff)),
        };

        if (operation == 0 || reference.empty()) {
            REQUIRE(
                coord_list_push_back_ex(list, &value)
                == NAVSYS_STATUS_OK);
            reference.push_back(value);
        } else if (operation == 1) {
            const size_t index = state % (reference.size() + 1);
            REQUIRE(
                coord_list_insert_ex(list, index, &value)
                == NAVSYS_STATUS_OK);
            reference.insert(
                reference.begin() + static_cast<std::ptrdiff_t>(index),
                value);
        } else if (operation == 2) {
            coord_t actual = {-1, -1};
            REQUIRE(
                coord_list_try_pop_back(list, &actual)
                == NAVSYS_STATUS_OK);
            CHECK(coord_equal(&actual, &reference.back()));
            reference.pop_back();
        } else if (operation == 3) {
            const size_t index = state % reference.size();
            coord_t actual = {-1, -1};
            REQUIRE(
                coord_list_remove_at_ex(list, index, &actual)
                == NAVSYS_STATUS_OK);
            CHECK(coord_equal(&actual, &reference[index]));
            reference.erase(
                reference.begin() + static_cast<std::ptrdiff_t>(index));
        } else {
            const size_t index = state % reference.size();
            const coord_t target = reference[index];
            bool actual_removed = false;
            REQUIRE(
                coord_list_remove_value_ex(
                    list, &target, &actual_removed)
                == NAVSYS_STATUS_OK);
            CHECK(actual_removed);
            const auto expected = std::find_if(
                reference.begin(),
                reference.end(),
                [&](const coord_t& item) {
                    return item.x == target.x && item.y == target.y;
                });
            REQUIRE(expected != reference.end());
            reference.erase(expected);
        }

        REQUIRE(coord_list_size(list) == reference.size());
        for (size_t index = 0; index < reference.size(); ++index) {
            coord_t actual = {-1, -1};
            REQUIRE(
                coord_list_fetch(list, index, &actual)
                == NAVSYS_STATUS_OK);
            CHECK(coord_equal(&actual, &reference[index]));
        }
    }

    coord_list_destroy(list);
}

TEST_CASE("coord_list: canonical slice and equality include empty ranges") {
    coord_list_t* list = nullptr;
    REQUIRE(coord_list_create_ex(&list) == NAVSYS_STATUS_OK);
    const coord_t values[] = {{1, 2}, {3, 4}, {5, 6}};
    for (const coord_t& value : values) {
        REQUIRE(
            coord_list_push_back_ex(list, &value)
            == NAVSYS_STATUS_OK);
    }

    coord_list_t* const sentinel = reinterpret_cast<coord_list_t*>(1);
    coord_list_t* slice = sentinel;
    CHECK(
        coord_list_create_slice(list, 2, 1, &slice)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(slice == sentinel);
    CHECK(
        coord_list_create_slice(list, 0, 4, &slice)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(slice == sentinel);

    REQUIRE(
        coord_list_create_slice(list, 1, 1, &slice)
        == NAVSYS_STATUS_OK);
    REQUIRE(slice != nullptr);
    CHECK(coord_list_size(slice) == 0);
    bool equal = true;
    REQUIRE(
        coord_list_equal(list, slice, &equal)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(equal);
    coord_list_destroy(slice);

    REQUIRE(
        coord_list_create_slice(list, 1, 3, &slice)
        == NAVSYS_STATUS_OK);
    REQUIRE(coord_list_size(slice) == 2);
    coord_t fetched = {-1, -1};
    REQUIRE(coord_list_fetch(slice, 0, &fetched) == NAVSYS_STATUS_OK);
    CHECK(coord_equal(&fetched, &values[1]));

    coord_list_t* copied = nullptr;
    REQUIRE(coord_list_copy_ex(slice, &copied) == NAVSYS_STATUS_OK);
    equal = false;
    REQUIRE(
        coord_list_equal(slice, copied, &equal)
        == NAVSYS_STATUS_OK);
    CHECK(equal);

    equal = true;
    CHECK(
        coord_list_equal(nullptr, copied, &equal)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(equal);

    coord_list_destroy(copied);
    coord_list_destroy(slice);
    coord_list_destroy(list);
}

TEST_CASE("coord_list: canonical export preserves short buffers") {
    coord_list_t* list = nullptr;
    REQUIRE(coord_list_create_ex(&list) == NAVSYS_STATUS_OK);
    const coord_t values[] = {{7, 8}, {9, 10}, {11, 12}};
    for (const coord_t& value : values) {
        REQUIRE(
            coord_list_push_back_ex(list, &value)
            == NAVSYS_STATUS_OK);
    }
    REQUIRE(coord_list_reserve(list, 8) == NAVSYS_STATUS_OK);

    size_t count = 99;
    REQUIRE(
        coord_list_export(list, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 3);

    coord_t short_buffer[] = {{91, 92}, {93, 94}};
    count = 0;
    CHECK(
        coord_list_export(list, short_buffer, 2, &count)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(count == 3);
    CHECK(short_buffer[0].x == 91);
    CHECK(short_buffer[0].y == 92);
    CHECK(short_buffer[1].x == 93);
    CHECK(short_buffer[1].y == 94);

    coord_t exact_buffer[3] = {};
    REQUIRE(
        coord_list_export(list, exact_buffer, 3, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 3);
    for (size_t index = 0; index < count; ++index) {
        CHECK(coord_equal(&exact_buffer[index], &values[index]));
    }

    coord_t oversized_buffer[5] = {};
    REQUIRE(
        coord_list_export(list, oversized_buffer, 5, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 3);
    CHECK(
        coord_list_export(list, oversized_buffer, 0, &count)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    coord_list_clear(list);
    count = 99;
    REQUIRE(
        coord_list_export(list, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    CHECK(count == 0);

    coord_list_destroy(list);
}
