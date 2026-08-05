#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "coord.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "maze_core.h"
#include "maze_eller.h"
#include "navcell.h"
#include "navgrid.h"
#include "obstacle.h"
#include "obstacle_core.h"
#include "route.h"
#include "route_carver.h"

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
    static_assert(std::is_same_v<
        decltype(&byul_maze_translate),
        navsys_status_t (*)(maze_t*, int32_t, int32_t)>);
    static_assert(std::is_same_v<
        decltype(&byul_maze_create),
        navsys_status_t (*)(const byul_maze_extent_t*, maze_t**)>);
    static_assert(std::is_standard_layout_v<byul_maze_extent_t>);
    static_assert(sizeof(byul_maze_extent_t) == 16);
    static_assert(alignof(byul_maze_extent_t) == 4);
    static_assert(offsetof(byul_maze_extent_t, origin_x) == 0);
    static_assert(offsetof(byul_maze_extent_t, origin_y) == 4);
    static_assert(offsetof(byul_maze_extent_t, width) == 8);
    static_assert(offsetof(byul_maze_extent_t, height) == 12);
    static_assert(std::is_standard_layout_v<byul_maze_navgrid_apply_options_t>);
    static_assert(sizeof(byul_maze_navgrid_apply_options_t) == 32);
    static_assert(alignof(byul_maze_navgrid_apply_options_t) == 8);
    static_assert(offsetof(
        byul_maze_navgrid_apply_options_t, cancel_func) == 16);
    static_assert(offsetof(
        byul_maze_navgrid_apply_options_t, cancel_userdata) == 24);
    static_assert(std::is_standard_layout_v<byul_maze_navgrid_overlay_token_t>);
    static_assert(sizeof(byul_maze_navgrid_overlay_token_t) == 24);
    static_assert(alignof(byul_maze_navgrid_overlay_token_t) == 8);
    static_assert(offsetof(
        byul_maze_navgrid_overlay_token_t, owner_cookie) == 8);
    static_assert(offsetof(
        byul_maze_navgrid_overlay_token_t, overlay) == 16);
    static_assert(std::is_same_v<
        decltype(&byul_maze_apply),
        navsys_status_t (*)(
            const maze_t*, navgrid_t*,
            const byul_maze_navgrid_apply_options_t*,
            byul_maze_navgrid_overlay_token_t*, size_t*)>);
    static_assert(std::is_same_v<
        decltype(&byul_maze_set_blocked),
        navsys_status_t (*)(maze_t*, int32_t, int32_t, bool, bool*)>);
    static_assert(std::is_same_v<
        decltype(&byul_maze_is_blocked),
        navsys_status_t (*)(const maze_t*, int32_t, int32_t, bool*)>);
    static_assert(std::is_same_v<
        decltype(&byul_maze_check_abi),
        navsys_status_t (*)(
            uint32_t, uint64_t, byul_maze_abi_mismatch_t*)>);
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

    obstacle_t* obstacle = obstacle_create_full(0, 0, 3, 3);
    assert(obstacle != nullptr);
    assert(obstacle_block_coord(obstacle, 0, 0));
    assert(obstacle_is_coord_blocked(obstacle, 0, 0));
    coord_list_t* obstacle_neighbors =
        obstacle_clone_neighbors(obstacle, 1, 1);
    assert(obstacle_neighbors != nullptr);
    assert(coord_list_size(obstacle_neighbors) == 7);
    coord_list_destroy(obstacle_neighbors);
    obstacle_destroy(obstacle);

    obstacle_t* checked_obstacle = nullptr;
    obstacle_t* checked_copy = nullptr;
    bool obstacle_changed = false;
    std::size_t obstacle_count = 0;
    coord_t obstacle_coords[1]{};
    assert(obstacle_create_checked(0, 0, 3, 3, &checked_obstacle)
        == NAVSYS_STATUS_OK);
    assert(obstacle_set_blocked(
        checked_obstacle, 1, 2, true, &obstacle_changed)
        == NAVSYS_STATUS_OK);
    assert(obstacle_changed);
    assert(obstacle_export_blocked(
        checked_obstacle, nullptr, 0, &obstacle_count)
        == NAVSYS_STATUS_OK);
    assert(obstacle_count == 1);
    assert(obstacle_export_blocked(
        checked_obstacle, obstacle_coords, 1, &obstacle_count)
        == NAVSYS_STATUS_OK);
    assert(obstacle_coords[0].x == 1);
    assert(obstacle_coords[0].y == 2);
    assert(obstacle_copy_checked(checked_obstacle, &checked_copy)
        == NAVSYS_STATUS_OK);
    assert(obstacle_equal(checked_obstacle, checked_copy));

    coord_t neighbor_coords[8]{};
    coord_t selected_neighbor{};
    std::size_t raster_changed = 0;
    coord_list_t* canonical_neighbors = obstacle_create_neighbors(
        checked_obstacle, 1, 1);
    assert(canonical_neighbors != nullptr);
    assert(coord_list_size(canonical_neighbors) == 7);
    assert(obstacle_export_neighbors(
        checked_obstacle, 1, 1, false,
        neighbor_coords, 8, &obstacle_count) == NAVSYS_STATUS_OK);
    assert(obstacle_count == 8);
    assert(obstacle_fetch_neighbor_at_degree(
        checked_obstacle, 1, 1, 0.0, &selected_neighbor)
        == NAVSYS_STATUS_OK);
    assert(selected_neighbor.x == 2);
    assert(selected_neighbor.y == 1);
    assert(obstacle_block_square(
        checked_obstacle, 1, 1, 0, &raster_changed)
        == NAVSYS_STATUS_OK);
    assert(raster_changed == 1);
    assert(obstacle_block_line(
        checked_obstacle, 0, 0, 2, 0, 0, &raster_changed)
        == NAVSYS_STATUS_OK);
    assert(raster_changed == 3);
    navgrid_t* obstacle_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, nullptr);
    assert(obstacle_grid != nullptr);
    obstacle_navgrid_apply_options_t overlay_options{
        sizeof(obstacle_navgrid_apply_options_t),
        OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION,
        OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE,
        nullptr,
        nullptr
    };
    obstacle_navgrid_overlay_token_t obstacle_overlay{};
    std::size_t overlay_changed = 0;
    assert(obstacle_apply_to_navgrid_checked(
        checked_obstacle,
        obstacle_grid,
        &overlay_options,
        &obstacle_overlay,
        &overlay_changed) == NAVSYS_STATUS_OK);
    assert(overlay_changed == 5);
    assert(obstacle_remove_from_navgrid_checked(
        obstacle_grid, &obstacle_overlay, &overlay_changed)
        == NAVSYS_STATUS_OK);
    assert(overlay_changed == 5);
    assert(obstacle_overlay.owner_cookie == 0);
    assert(obstacle_overlay.overlay == 0);
    navgrid_destroy(obstacle_grid);
    coord_list_destroy(canonical_neighbors);
    obstacle_destroy(checked_copy);
    obstacle_destroy(checked_obstacle);

    const byul_maze_extent_t maze_extent{-2, 4, 5, 3};
    byul_maze_abi_mismatch_t maze_mismatch =
        BYUL_MAZE_ABI_VERSION_MISMATCH;
    assert(byul_maze_check_abi(
        BYUL_MAZE_ABI_VERSION,
        BYUL_MAZE_ABI_FINGERPRINT,
        &maze_mismatch) == NAVSYS_STATUS_OK);
    assert(maze_mismatch == BYUL_MAZE_ABI_MATCH);
    maze_t* checked_maze = nullptr;
    assert(byul_maze_create(&maze_extent, &checked_maze)
        == NAVSYS_STATUS_OK);
    bool maze_changed = false;
    assert(byul_maze_set_blocked(
        checked_maze, -1, 5, true, &maze_changed) == NAVSYS_STATUS_OK);
    assert(maze_changed);
    bool maze_blocked = false;
    assert(byul_maze_is_blocked(
        checked_maze, -1, 5, &maze_blocked) == NAVSYS_STATUS_OK);
    assert(maze_blocked);
    std::size_t maze_blocked_count = 0;
    assert(byul_maze_get_blocked_count(
        checked_maze, &maze_blocked_count) == NAVSYS_STATUS_OK);
    assert(maze_blocked_count == 1);
    maze_destroy(checked_maze);

    const byul_maze_generate_options_t eller_options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(0),
        UINT64_C(16),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    maze_t* eller = nullptr;
    assert(byul_maze_generate_eller(
        -5, 8, 9, 9, &eller_options, &eller) == NAVSYS_STATUS_OK);
    assert(eller != nullptr);
    assert(maze_hash(eller) == UINT32_C(789167229));
    maze_destroy(eller);

    navgrid_t* legacy_carver_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, nullptr);
    assert(legacy_carver_grid != nullptr);
    const coord_t legacy_carver_start{0, 1};
    const coord_t legacy_carver_goal{2, 1};
    assert(navgrid_block_coord(legacy_carver_grid, 0, 1));
    assert(navgrid_block_coord(legacy_carver_grid, 1, 1));
    assert(navgrid_block_coord(legacy_carver_grid, 2, 1));
    assert(route_carve_beam(
        legacy_carver_grid,
        &legacy_carver_start,
        &legacy_carver_goal,
        0) == 2);
    assert(route_carve_bomb(
        legacy_carver_grid, &legacy_carver_start, 0) == 1);
    navgrid_destroy(legacy_carver_grid);

    navgrid_t* checked_carver_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, nullptr);
    assert(checked_carver_grid != nullptr);
    const navgrid_carve_options_t carve_options{
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_INCLUDE_START | NAVGRID_CARVE_INCLUDE_END
            | NAVGRID_CARVE_ATOMIC,
        0,
        16,
        nullptr,
        nullptr
    };
    assert(navgrid_block_coord(checked_carver_grid, 0, 1));
    assert(navgrid_block_coord(checked_carver_grid, 1, 1));
    assert(navgrid_block_coord(checked_carver_grid, 2, 1));
    std::size_t carved_count = 99;
    assert(navgrid_carve_line(
        checked_carver_grid,
        &legacy_carver_start,
        &legacy_carver_goal,
        &carve_options,
        &carved_count) == NAVSYS_STATUS_OK);
    assert(carved_count == 3);
    navgrid_destroy(checked_carver_grid);
    return 0;
}
