#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "coord.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "navcell.h"
#include "navgrid.h"
#include "route.h"

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
    static_assert(std::is_standard_layout_v<cost_coord_pq_create_info_t>);
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
    static_assert(TERRAIN_TYPE_NORMAL == 0);
    static_assert(TERRAIN_TYPE_WATER == 1);
    static_assert(TERRAIN_TYPE_FOREST == 2);
    static_assert(TERRAIN_TYPE_MOUNTAIN == 3);
    static_assert(TERRAIN_TYPE_FORBIDDEN == 100);
    static_assert(sizeof(terrain_type_t) == 4);
    static_assert(alignof(terrain_type_t) == 4);
    static_assert(std::is_standard_layout_v<navcell_t>);
    static_assert(std::is_trivially_copyable_v<navcell_t>);
    static_assert(sizeof(navcell_t) == 8);
    static_assert(alignof(navcell_t) == 4);
    static_assert(offsetof(navcell_t, terrain) == 0);
    static_assert(offsetof(navcell_t, height) == 4);
    static_assert(NAVGRID_DIR_4 == 0);
    static_assert(NAVGRID_DIR_8 == 1);
    static_assert(sizeof(navgrid_dir_mode_t) == 4);
    static_assert(std::is_same_v<
        is_coord_blocked_func,
        bool (*)(const void*, int, int, void*)>);
    static_assert(ROUTE_DIR_UNKNOWN == 0);
    static_assert(ROUTE_DIR_DOWN_RIGHT == 8);
    static_assert(ROUTE_DIR_COUNT == 9);
    static_assert(ROUTE_COMPLETION_NONE == 0);
    static_assert(ROUTE_COMPLETION_COMPLETE == 1);
    static_assert(ROUTE_COMPLETION_PARTIAL == 2);
    static_assert(std::is_standard_layout_v<route_t>);
    static_assert(sizeof(route_t) == 48);
    static_assert(alignof(route_t) == 8);
    static_assert(offsetof(route_t, coords) == 0);
    static_assert(offsetof(route_t, visited_order) == 8);
    static_assert(offsetof(route_t, visited_count) == 16);
    static_assert(offsetof(route_t, cost) == 24);
    static_assert(offsetof(route_t, success) == 28);
    static_assert(offsetof(route_t, total_retry_count) == 32);
    static_assert(offsetof(route_t, avg_vec_x) == 36);
    static_assert(offsetof(route_t, avg_vec_y) == 40);
    static_assert(offsetof(route_t, vec_count) == 44);
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
    static_assert(sizeof(cost_coord_pq_create_info_t) == 12);
    static_assert(alignof(cost_coord_pq_create_info_t) == 4);
    static_assert(offsetof(cost_coord_pq_create_info_t, struct_size) == 0);
    static_assert(offsetof(cost_coord_pq_create_info_t, abi_version) == 4);
    static_assert(offsetof(cost_coord_pq_create_info_t, flags) == 8);

    navcell_t zero_cell{};
    const navcell_t value_cell{TERRAIN_TYPE_MOUNTAIN, -1};
    assert(zero_cell.terrain == TERRAIN_TYPE_NORMAL);
    assert(zero_cell.height == 0);
    assert(value_cell.terrain == TERRAIN_TYPE_MOUNTAIN);
    assert(value_cell.height == -1);

    bool terrain_supported = false;
    navcell_t checked_cell{};
    navcell_t assigned_cell{};
    navcell_t* allocated_cell = nullptr;
    navcell_t* copied_cell = nullptr;
    assert(navcell_is_terrain_supported(
        TERRAIN_TYPE_MOUNTAIN, &terrain_supported) == NAVSYS_STATUS_OK);
    assert(terrain_supported);
    assert(navcell_init_checked(
        &checked_cell, TERRAIN_TYPE_MOUNTAIN, INT32_MIN) == NAVSYS_STATUS_OK);
    assert(navcell_validate(&checked_cell) == NAVSYS_STATUS_OK);
    assert(navcell_assign_checked(
        &assigned_cell, &checked_cell) == NAVSYS_STATUS_OK);
    assert(navcell_create_checked(
        assigned_cell.terrain, assigned_cell.height,
        &allocated_cell) == NAVSYS_STATUS_OK);
    assert(navcell_copy_checked(
        allocated_cell, &copied_cell) == NAVSYS_STATUS_OK);
    assert(copied_cell != nullptr);
    assert(copied_cell->terrain == TERRAIN_TYPE_MOUNTAIN);
    assert(copied_cell->height == INT32_MIN);
    navcell_destroy(copied_cell);
    navcell_destroy(allocated_cell);

    navgrid_t* public_grid = navgrid_create_full(
        7, 9, NAVGRID_DIR_4, nullptr);
    assert(public_grid != nullptr);
    assert(navgrid_get_width(public_grid) == 7);
    assert(navgrid_get_height(public_grid) == 9);
    assert(navgrid_get_mode(public_grid) == NAVGRID_DIR_4);
    assert(navgrid_get_abi_version() == BYUL_NAVGRID_ABI_VERSION);
    assert(navgrid_get_abi_fingerprint() == BYUL_NAVGRID_ABI_FINGERPRINT);
    navgrid_abi_mismatch_t public_mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    assert(navgrid_check_abi(
        BYUL_NAVGRID_ABI_VERSION,
        BYUL_NAVGRID_ABI_FINGERPRINT,
        &public_mismatch) == NAVSYS_STATUS_OK);
    assert(public_mismatch == NAVGRID_ABI_MATCH);
    is_coord_blocked_func public_blocked_fn = nullptr;
    void* public_blocked_userdata = reinterpret_cast<void*>(1);
    assert(navgrid_fetch_is_coord_blocked_binding(
        public_grid, &public_blocked_fn, &public_blocked_userdata)
        == NAVSYS_STATUS_OK);
    assert(public_blocked_fn == is_coord_blocked_navgrid);
    assert(public_blocked_userdata == nullptr);
    navcell_t public_cell{TERRAIN_TYPE_FOREST, 23};
    navcell_t public_prior{TERRAIN_TYPE_MOUNTAIN, -1};
    bool public_had_prior = true;
    bool public_changed = false;
    assert(navgrid_set_cell_ex(
        public_grid, 2, 3, &public_cell, &public_prior,
        &public_had_prior, &public_changed) == NAVSYS_STATUS_OK);
    assert(!public_had_prior);
    assert(public_changed);
    navcell_t public_fetched{};
    bool public_present = false;
    assert(navgrid_fetch_cell_ex(
        public_grid, 2, 3, &public_fetched, &public_present)
        == NAVSYS_STATUS_OK);
    assert(public_present);
    assert(public_fetched.terrain == TERRAIN_TYPE_FOREST);
    const coord_t public_overlay_coords[]{{2, 3}, {2, 3}};
    navgrid_overlay_id_t public_overlay = 0;
    std::size_t public_changed_count = 0;
    assert(navgrid_apply_blocked_overlay(
        public_grid, public_overlay_coords, 2, &public_overlay,
        &public_changed_count) == NAVSYS_STATUS_OK);
    assert(public_overlay != 0);
    assert(public_changed_count == 1);
    assert(navgrid_remove_blocked_overlay(
        public_grid, public_overlay, &public_changed_count)
        == NAVSYS_STATUS_OK);
    assert(public_changed_count == 1);
    static_assert(sizeof(navgrid_cell_entry_t) == 20);
    static_assert(alignof(navgrid_cell_entry_t) == 4);
    std::size_t public_neighbor_count = 0;
    assert(navgrid_export_neighbors(
        public_grid, 2, 3, false, nullptr, 0, &public_neighbor_count)
        == NAVSYS_STATUS_OK);
    assert(public_neighbor_count == 4);
    coord_t public_neighbors[4]{};
    assert(navgrid_export_neighbors(
        public_grid, 2, 3, false, public_neighbors, 4,
        &public_neighbor_count) == NAVSYS_STATUS_OK);
    assert(public_neighbors[0].x == 3);
    assert(public_neighbors[0].y == 3);
    std::size_t public_entry_count = 0;
    assert(navgrid_export_cells(
        public_grid, nullptr, 0, &public_entry_count) == NAVSYS_STATUS_OK);
    assert(public_entry_count == 1);
    navgrid_cell_entry_t public_entry{};
    assert(navgrid_export_cells(
        public_grid, &public_entry, 1, &public_entry_count)
        == NAVSYS_STATUS_OK);
    assert(public_entry.present);
    assert(!public_entry.blocked);
    assert(public_entry.cell.terrain == TERRAIN_TYPE_FOREST);
    navgrid_destroy(public_grid);

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

    const cost_coord_pq_create_info_t pq_info{
        static_cast<std::uint32_t>(sizeof(cost_coord_pq_create_info_t)),
        BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION,
        0,
    };
    cost_coord_pq_t* queue = nullptr;
    assert(cost_coord_pq_create_ex(&pq_info, &queue) == NAVSYS_STATUS_OK);
    assert(queue != nullptr);
    const coord_t first{1, 2};
    const coord_t second{3, 4};
    assert(cost_coord_pq_push_ex(queue, 1.0f, &first) == NAVSYS_STATUS_OK);
    assert(cost_coord_pq_push_ex(queue, 1.0f, &second) == NAVSYS_STATUS_OK);
    float popped_cost = -1.0f;
    coord_t popped_coord{-1, -1};
    assert(
        cost_coord_pq_pop_min(queue, &popped_cost, &popped_coord)
        == NAVSYS_STATUS_OK);
    assert(popped_cost == 1.0f);
    assert(popped_coord.x == first.x);
    assert(popped_coord.y == first.y);
    assert(cost_coord_pq_size(queue) == 1);
    bool removed = false;
    assert(
        cost_coord_pq_remove_one(queue, 1.0f, &second, &removed)
        == NAVSYS_STATUS_OK);
    assert(removed);
    assert(cost_coord_pq_empty(queue));
    assert(cost_coord_pq_push_ex(queue, 3.0f, &first) == NAVSYS_STATUS_OK);
    assert(cost_coord_pq_push_ex(queue, 4.0f, &first) == NAVSYS_STATUS_OK);
    std::size_t removed_count = 0;
    assert(
        cost_coord_pq_remove_all(queue, &first, &removed_count)
        == NAVSYS_STATUS_OK);
    assert(removed_count == 2);
    cost_coord_pq_clear(queue);
    cost_coord_pq_destroy(queue);
    return 0;
}
