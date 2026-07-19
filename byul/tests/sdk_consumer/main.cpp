#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "coord.h"
#include "coord_hash.h"

struct coord_hash_callback_counts {
    int copies{};
    int destroys{};
    int equals{};
};

static navsys_status_t copy_coord_hash_int(
    const void* source,
    void** out_copy,
    void* userdata) {
    auto* counts = static_cast<coord_hash_callback_counts*>(userdata);
    if (!source || !out_copy || !counts) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    void* copied = coord_hash_int_copy(source);
    if (!copied) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    ++counts->copies;
    *out_copy = copied;
    return NAVSYS_STATUS_OK;
}

static void destroy_coord_hash_int(void* value, void* userdata) {
    auto* counts = static_cast<coord_hash_callback_counts*>(userdata);
    if (counts) {
        ++counts->destroys;
    }
    coord_hash_int_destroy(value);
}

static bool equal_coord_hash_int(
    const void* lhs,
    const void* rhs,
    void* userdata) {
    auto* counts = static_cast<coord_hash_callback_counts*>(userdata);
    if (!lhs || !rhs || !counts) {
        return false;
    }
    ++counts->equals;
    return *static_cast<const int*>(lhs) == *static_cast<const int*>(rhs);
}

int main() {
    static_assert(std::is_standard_layout_v<coord_t>);
    static_assert(std::is_standard_layout_v<coord_hash_create_info_t>);
    static_assert(std::is_standard_layout_v<coord_hash_entry_view_t>);
    static_assert(
        std::is_same_v<
            decltype(&copy_coord_hash_int),
            coord_hash_value_copy_func_ex>);
    static_assert(
        std::is_same_v<
            decltype(&destroy_coord_hash_int),
            coord_hash_value_destroy_func_ex>);
    static_assert(
        std::is_same_v<
            decltype(&equal_coord_hash_int),
            coord_hash_value_equal_func_ex>);
    static_assert(sizeof(void*) == 8);
    static_assert(sizeof(coord_hash_create_info_t) == 40);
    static_assert(alignof(coord_hash_create_info_t) == 8);
    static_assert(offsetof(coord_hash_create_info_t, struct_size) == 0);
    static_assert(offsetof(coord_hash_create_info_t, abi_version) == 4);
    static_assert(offsetof(coord_hash_create_info_t, copy_value) == 8);
    static_assert(offsetof(coord_hash_create_info_t, destroy_value) == 16);
    static_assert(offsetof(coord_hash_create_info_t, equal_value) == 24);
    static_assert(offsetof(coord_hash_create_info_t, userdata) == 32);
    static_assert(sizeof(coord_hash_entry_view_t) == 16);
    static_assert(alignof(coord_hash_entry_view_t) == 8);
    static_assert(offsetof(coord_hash_entry_view_t, key) == 0);
    static_assert(offsetof(coord_hash_entry_view_t, value) == 8);

    assert(sizeof(coord_t) == coord_sizeof());
    assert(alignof(coord_t) == coord_alignof());
    assert(offsetof(coord_t, x) == coord_offsetof_x());
    assert(offsetof(coord_t, y) == coord_offsetof_y());

    coord_t value{};
    assert(coord_init_checked(&value, -3, 7) == NAVSYS_STATUS_OK);

    std::size_t required = 0;
    assert(coord_format(&value, nullptr, 0, &required) == NAVSYS_STATUS_OK);
    assert(required == sizeof("(-3, 7)"));

    char buffer[sizeof("(-3, 7)")]{};
    assert(
        coord_format(&value, buffer, sizeof(buffer), &required)
        == NAVSYS_STATUS_OK);
    assert(std::strcmp(buffer, "(-3, 7)") == 0);

    coord_hash_callback_counts counts{};
    coord_hash_create_info_t info{
        static_cast<std::uint32_t>(sizeof(coord_hash_create_info_t)),
        BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION,
        copy_coord_hash_int,
        destroy_coord_hash_int,
        equal_coord_hash_int,
        &counts,
    };
    coord_hash_t* hash = nullptr;
    assert(coord_hash_create_ex(&info, &hash) == NAVSYS_STATUS_OK);
    assert(hash != nullptr);

    coord_t key{5, 6};
    int stored = 42;
    bool inserted = false;
    assert(
        coord_hash_upsert_copy(hash, &key, &stored, &inserted)
        == NAVSYS_STATUS_OK);
    assert(inserted);

    coord_hash_t* copied = nullptr;
    assert(coord_hash_copy_ex(hash, &copied) == NAVSYS_STATUS_OK);
    assert(copied != nullptr);
    bool equal = false;
    assert(coord_hash_equal_full(hash, copied, &equal) == NAVSYS_STATUS_OK);
    assert(equal);

    coord_hash_destroy(copied);
    coord_hash_destroy(hash);
    assert(counts.copies == 2);
    assert(counts.destroys == 2);
    assert(counts.equals == 1);
    return 0;
}
