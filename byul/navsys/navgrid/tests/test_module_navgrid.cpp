#include "doctest.h"
#include "navgrid.h"
#include "coord.h"

#include <cstddef>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace {

struct terrain_cost_policy_t {
    float water;
    float forest;
    float mountain;
};

bool block_selected_terrain(
    const void* context, int x, int y, void* userdata) {
    const auto* grid = static_cast<const navgrid_t*>(context);
    const auto selected = *static_cast<const terrain_type_t*>(userdata);
    navcell_t cell{};
    return navgrid_fetch_cell(grid, x, y, &cell) == 0
        && cell.terrain == selected;
}

bool throw_from_blocked_callback(
    const void*, int, int, void*) {
    throw std::runtime_error("navgrid callback failure");
}

struct reentrant_callback_context_t {
    navgrid_t* grid;
    int calls;
    navsys_status_t bind_status;
    navsys_status_t unbind_status;
    navgrid_t* copied;
};

bool exercise_reentrant_callback(
    const void*, int, int, void* userdata) {
    auto* state = static_cast<reentrant_callback_context_t*>(userdata);
    ++state->calls;
    if (state->calls == 1) {
        state->bind_status = navgrid_bind_is_coord_blocked_func(
            state->grid, exercise_reentrant_callback, state);
        state->unbind_status = navgrid_unbind_is_coord_blocked_func(
            state->grid);
        state->copied = navgrid_copy(state->grid);
    }
    return false;
}

float terrain_cost(
    const navgrid_t* grid, const coord_t*, const coord_t* goal,
    void* userdata) {
    const auto* policy = static_cast<const terrain_cost_policy_t*>(userdata);
    navcell_t cell{};
    if (navgrid_fetch_cell(grid, goal->x, goal->y, &cell) != 0) return 1.0f;
    switch (cell.terrain) {
    case TERRAIN_TYPE_WATER: return policy->water;
    case TERRAIN_TYPE_FOREST: return policy->forest;
    case TERRAIN_TYPE_MOUNTAIN: return policy->mountain;
    default: return 1.0f;
    }
}

void check_coord_sequence(
    const coord_list_t* list,
    std::initializer_list<coord_t> expected) {
    REQUIRE(list != nullptr);
    REQUIRE(coord_list_size(list) == expected.size());
    std::size_t index = 0;
    for (const coord_t& value : expected) {
        coord_t actual{};
        REQUIRE(coord_list_fetch(list, index, &actual) == NAVSYS_STATUS_OK);
        CHECK(actual.x == value.x);
        CHECK(actual.y == value.y);
        ++index;
    }
}

void check_navgrid_binding(
    const navgrid_t* grid,
    is_coord_blocked_func expected_fn,
    void* expected_userdata) {
    is_coord_blocked_func actual_fn = nullptr;
    void* actual_userdata = nullptr;
    REQUIRE(navgrid_fetch_is_coord_blocked_binding(
        grid, &actual_fn, &actual_userdata) == NAVSYS_STATUS_OK);
    CHECK(actual_fn == expected_fn);
    CHECK(actual_userdata == expected_userdata);
}

} // namespace

static_assert(TERRAIN_TYPE_NORMAL == 0);
static_assert(TERRAIN_TYPE_WATER == 1);
static_assert(TERRAIN_TYPE_FOREST == 2);
static_assert(TERRAIN_TYPE_MOUNTAIN == 3);
static_assert(TERRAIN_TYPE_FORBIDDEN == 100);
static_assert(sizeof(terrain_type_t) == 4);
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
static_assert(std::is_standard_layout_v<navgrid_cell_entry_t>);
static_assert(sizeof(navgrid_cell_entry_t) == 20);
static_assert(alignof(navgrid_cell_entry_t) == 4);
static_assert(offsetof(navgrid_cell_entry_t, coord) == 0);
static_assert(offsetof(navgrid_cell_entry_t, cell) == 8);
static_assert(offsetof(navgrid_cell_entry_t, present) == 16);
static_assert(offsetof(navgrid_cell_entry_t, blocked) == 17);

TEST_CASE("navcell legacy value ABI baseline") {
    navcell_t zero{};
    CHECK(zero.terrain == TERRAIN_TYPE_NORMAL);
    CHECK(zero.height == 0);

    navcell_t boundaries{};
    CHECK(navcell_init_full(
        &boundaries,
        TERRAIN_TYPE_WATER,
        std::numeric_limits<int>::min()) == 0);
    CHECK(boundaries.terrain == TERRAIN_TYPE_WATER);
    CHECK(boundaries.height == std::numeric_limits<int>::min());

    navcell_t copied{};
    CHECK(navcell_assign(&copied, &boundaries) == 0);
    CHECK(copied.terrain == TERRAIN_TYPE_WATER);
    CHECK(copied.height == std::numeric_limits<int>::min());
    CHECK(navcell_init_full(
        &boundaries,
        TERRAIN_TYPE_MOUNTAIN,
        std::numeric_limits<int>::max()) == 0);
    CHECK(boundaries.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(boundaries.height == std::numeric_limits<int>::max());

    CHECK(navcell_init(nullptr) == -1);
    CHECK(navcell_init_full(nullptr, TERRAIN_TYPE_NORMAL, 0) == -1);
    CHECK(navcell_assign(nullptr, &boundaries) == -1);
    CHECK(navcell_assign(&boundaries, nullptr) == -1);
    CHECK(navcell_copy(nullptr) == nullptr);
    navcell_destroy(nullptr);
}

TEST_CASE("navcell checked terrain query and validation") {
    for (const terrain_type_t terrain : {
             TERRAIN_TYPE_NORMAL,
             TERRAIN_TYPE_WATER,
             TERRAIN_TYPE_FOREST,
             TERRAIN_TYPE_MOUNTAIN,
             TERRAIN_TYPE_FORBIDDEN}) {
        bool supported = false;
        CHECK(navcell_is_terrain_supported(terrain, &supported)
            == NAVSYS_STATUS_OK);
        CHECK(supported);

        const navcell_t cell{terrain, 0};
        CHECK(navcell_validate(&cell) == NAVSYS_STATUS_OK);
    }

    for (const int terrain_value : {-1, 4, 99, 101, INT32_MAX}) {
        const auto terrain = static_cast<terrain_type_t>(terrain_value);
        bool supported = true;
        CHECK(navcell_is_terrain_supported(terrain, &supported)
            == NAVSYS_STATUS_OK);
        CHECK_FALSE(supported);

        const navcell_t cell{terrain, 0};
        CHECK(navcell_validate(&cell) == NAVSYS_STATUS_UNSUPPORTED);
    }

    bool preserved = true;
    CHECK(navcell_is_terrain_supported(TERRAIN_TYPE_NORMAL, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(preserved);
    CHECK(navcell_validate(nullptr) == NAVSYS_STATUS_INVALID_ARGUMENT);
}

TEST_CASE("navcell checked value operations are failure atomic") {
    navcell_t value{TERRAIN_TYPE_WATER, 71};
    CHECK(navcell_init_checked(
        &value, TERRAIN_TYPE_MOUNTAIN, INT32_MIN) == NAVSYS_STATUS_OK);
    CHECK(value.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(value.height == INT32_MIN);

    const navcell_t preserved = value;
    CHECK(navcell_init_checked(
        &value, static_cast<terrain_type_t>(101), INT32_MAX)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(value.terrain == preserved.terrain);
    CHECK(value.height == preserved.height);
    CHECK(navcell_init_checked(nullptr, TERRAIN_TYPE_NORMAL, 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    navcell_t invalid_source{static_cast<terrain_type_t>(-1), 99};
    CHECK(navcell_assign_checked(&value, &invalid_source)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(value.terrain == preserved.terrain);
    CHECK(value.height == preserved.height);
    CHECK(navcell_assign_checked(&value, &value) == NAVSYS_STATUS_OK);
    CHECK(navcell_assign_checked(nullptr, &value)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(navcell_assign_checked(&value, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    navcell_t* created = reinterpret_cast<navcell_t*>(1);
    CHECK(navcell_create_checked(
        static_cast<terrain_type_t>(101), INT32_MAX, &created)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(created == reinterpret_cast<navcell_t*>(1));
    CHECK(navcell_create_checked(TERRAIN_TYPE_FOREST, INT32_MAX, &created)
        == NAVSYS_STATUS_OK);
    REQUIRE(created != nullptr);
    CHECK(created->terrain == TERRAIN_TYPE_FOREST);
    CHECK(created->height == INT32_MAX);

    navcell_t* copied = reinterpret_cast<navcell_t*>(1);
    CHECK(navcell_copy_checked(created, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    CHECK(copied != created);
    CHECK(copied->terrain == created->terrain);
    CHECK(copied->height == created->height);

    navcell_t* pointer_preserved = reinterpret_cast<navcell_t*>(1);
    CHECK(navcell_copy_checked(&invalid_source, &pointer_preserved)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(pointer_preserved == reinterpret_cast<navcell_t*>(1));
    CHECK(navcell_create_checked(TERRAIN_TYPE_NORMAL, 0, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(navcell_copy_checked(nullptr, &pointer_preserved)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(navcell_copy_checked(created, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    navcell_destroy(copied);
    navcell_destroy(created);
}

TEST_CASE("navcell legacy functions forward to checked contracts") {
    navcell_t value{TERRAIN_TYPE_FOREST, 41};
    const navcell_t preserved = value;
    CHECK(navcell_init_full(
        &value, static_cast<terrain_type_t>(101), 42)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(value.terrain == preserved.terrain);
    CHECK(value.height == preserved.height);
    CHECK(navcell_create_full(static_cast<terrain_type_t>(101), 0)
        == nullptr);

    const navcell_t invalid_source{static_cast<terrain_type_t>(101), 0};
    CHECK(navcell_assign(&value, &invalid_source)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(value.terrain == preserved.terrain);
    CHECK(value.height == preserved.height);
    CHECK(navcell_copy(&invalid_source) == nullptr);
}

TEST_CASE("navgrid opaque ABI and legacy binding characterization") {
    CHECK(navgrid_get_abi_version() == BYUL_NAVGRID_ABI_VERSION);
    CHECK(navgrid_get_abi_fingerprint() == BYUL_NAVGRID_ABI_FINGERPRINT);
    navgrid_abi_mismatch_t mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    CHECK(navgrid_check_abi(
        BYUL_NAVGRID_ABI_VERSION,
        BYUL_NAVGRID_ABI_FINGERPRINT,
        &mismatch) == NAVSYS_STATUS_OK);
    CHECK(mismatch == NAVGRID_ABI_MATCH);
    CHECK(navgrid_check_abi(
        UINT32_C(1), UINT64_C(0x4e47524944010028), &mismatch)
        == NAVSYS_STATUS_OK);
    CHECK(mismatch == NAVGRID_ABI_MATCH);
    CHECK(navgrid_check_abi(
        BYUL_NAVGRID_ABI_VERSION,
        BYUL_NAVGRID_ABI_FINGERPRINT ^ UINT64_C(1),
        &mismatch) == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == NAVGRID_ABI_FINGERPRINT_MISMATCH);
    CHECK(navgrid_check_abi(
        UINT32_C(99), UINT64_C(0), &mismatch)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(mismatch == NAVGRID_ABI_VERSION_MISMATCH);
    CHECK(navgrid_check_abi(
        BYUL_NAVGRID_ABI_VERSION,
        BYUL_NAVGRID_ABI_FINGERPRINT,
        nullptr) == NAVSYS_STATUS_INVALID_ARGUMENT);

    navgrid_t* grid = navgrid_create_full(
        7, 9, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);
    CHECK(navgrid_get_width(grid) == 7);
    CHECK(navgrid_get_height(grid) == 9);
    CHECK(navgrid_get_mode(grid) == NAVGRID_DIR_4);
    check_navgrid_binding(grid, is_coord_blocked_navgrid, nullptr);

    terrain_type_t blocked_terrain = TERRAIN_TYPE_WATER;
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, block_selected_terrain, &blocked_terrain) == NAVSYS_STATUS_OK);
    check_navgrid_binding(grid, block_selected_terrain, &blocked_terrain);

    const navcell_t source_cell{TERRAIN_TYPE_FOREST, 41};
    REQUIRE(navgrid_set_cell(grid, 2, 3, &source_cell));

    navgrid_t* copy = navgrid_copy(grid);
    REQUIRE(copy != nullptr);
    CHECK(copy != grid);
    CHECK(navgrid_get_width(copy) == navgrid_get_width(grid));
    CHECK(navgrid_get_height(copy) == navgrid_get_height(grid));
    CHECK(navgrid_get_mode(copy) == navgrid_get_mode(grid));
    check_navgrid_binding(copy, block_selected_terrain, &blocked_terrain);

    const navcell_t replacement{TERRAIN_TYPE_MOUNTAIN, 99};
    REQUIRE(navgrid_set_cell(grid, 2, 3, &replacement));
    navcell_t copied_cell{};
    REQUIRE(navgrid_fetch_cell(copy, 2, 3, &copied_cell) == 0);
    CHECK(copied_cell.terrain == TERRAIN_TYPE_FOREST);
    CHECK(copied_cell.height == 41);
    navgrid_destroy(copy);

    navgrid_set_is_coord_blocked_func(grid, nullptr);
    check_navgrid_binding(grid, nullptr, &blocked_terrain);
    coord_list_t* neighbors = navgrid_copy_neighbors(grid, 3, 4);
    REQUIRE(neighbors != nullptr);
    CHECK(coord_list_length(neighbors) == 4);
    coord_list_destroy(neighbors);
    navgrid_destroy(grid);
}

TEST_CASE("navgrid callback binding is atomic and exception safe") {
    navgrid_t* grid = navgrid_create_full(3, 3, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);

    int userdata = 17;
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, block_selected_terrain, &userdata) == NAVSYS_STATUS_OK);
    CHECK(navgrid_bind_is_coord_blocked_func(grid, nullptr, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    check_navgrid_binding(grid, block_selected_terrain, &userdata);

    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, throw_from_blocked_callback, &userdata) == NAVSYS_STATUS_OK);
    coord_list_t* failed = reinterpret_cast<coord_list_t*>(1);
    CHECK_NOTHROW(failed = navgrid_copy_neighbors(grid, 1, 1));
    CHECK(failed == nullptr);

    reentrant_callback_context_t state{
        grid, 0, NAVSYS_STATUS_OK, NAVSYS_STATUS_OK, nullptr};
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, exercise_reentrant_callback, &state) == NAVSYS_STATUS_OK);
    coord_list_t* neighbors = navgrid_copy_neighbors(grid, 1, 1);
    REQUIRE(neighbors != nullptr);
    CHECK(coord_list_length(neighbors) == 4);
    CHECK(state.calls == 4);
    CHECK(state.bind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(state.unbind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(state.copied == nullptr);
    check_navgrid_binding(grid, exercise_reentrant_callback, &state);
    coord_list_destroy(neighbors);

    CHECK(navgrid_unbind_is_coord_blocked_func(grid) == NAVSYS_STATUS_OK);
    check_navgrid_binding(grid, nullptr, nullptr);
    navgrid_destroy(grid);
}

TEST_CASE("navgrid checked mutation preserves base cells and overlay provenance") {
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);

    const navcell_t forest{TERRAIN_TYPE_FOREST, 73};
    navcell_t prior{TERRAIN_TYPE_MOUNTAIN, -1};
    bool had_prior = true;
    bool changed = false;
    CHECK(navgrid_set_cell_ex(
        grid, 3, 4, &forest, &prior, &had_prior, &changed)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(had_prior);
    CHECK(changed);
    CHECK(prior.terrain == TERRAIN_TYPE_NORMAL);
    CHECK(prior.height == 0);

    navcell_t fetched{};
    bool present = false;
    CHECK(navgrid_fetch_cell_ex(grid, 3, 4, &fetched, &present)
        == NAVSYS_STATUS_OK);
    CHECK(present);
    CHECK(fetched.terrain == TERRAIN_TYPE_FOREST);
    CHECK(fetched.height == 73);

    CHECK(navgrid_block_coord_ex(grid, 3, 4, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    CHECK(navgrid_fetch_cell_ex(grid, 3, 4, &fetched, &present)
        == NAVSYS_STATUS_OK);
    CHECK(fetched.terrain == TERRAIN_TYPE_FOREST);
    CHECK(fetched.height == 73);

    CHECK(navgrid_unblock_coord_ex(grid, 3, 4, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    CHECK(navgrid_fetch_cell_ex(grid, 3, 4, &fetched, &present)
        == NAVSYS_STATUS_OK);
    CHECK(present);
    CHECK(fetched.terrain == TERRAIN_TYPE_FOREST);
    CHECK(fetched.height == 73);

    const coord_t coords[] = {{3, 4}, {3, 4}, {5, 6}};
    navgrid_overlay_id_t first = 0;
    navgrid_overlay_id_t second = 0;
    size_t changed_count = 99;
    CHECK(navgrid_apply_blocked_overlay(
        grid, coords, 3, &first, &changed_count) == NAVSYS_STATUS_OK);
    CHECK(first != 0);
    CHECK(changed_count == 2);
    CHECK(navgrid_apply_blocked_overlay(
        grid, coords, 3, &second, &changed_count) == NAVSYS_STATUS_OK);
    CHECK(second != 0);
    CHECK(second != first);
    CHECK(changed_count == 0);

    navgrid_t* copied = navgrid_copy(grid);
    REQUIRE(copied != nullptr);
    CHECK(is_coord_blocked_navgrid(copied, 3, 4, nullptr));
    CHECK(navgrid_remove_blocked_overlay(copied, first, &changed_count)
        == NAVSYS_STATUS_OK);
    CHECK(changed_count == 0);
    CHECK(navgrid_remove_blocked_overlay(copied, second, &changed_count)
        == NAVSYS_STATUS_OK);
    CHECK(changed_count == 2);
    CHECK_FALSE(is_coord_blocked_navgrid(copied, 3, 4, nullptr));
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    navgrid_destroy(copied);

    CHECK(navgrid_remove_blocked_overlay(grid, first, &changed_count)
        == NAVSYS_STATUS_OK);
    CHECK(changed_count == 0);
    CHECK(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    CHECK(navgrid_remove_blocked_overlay(grid, second, &changed_count)
        == NAVSYS_STATUS_OK);
    CHECK(changed_count == 2);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 3, 4, nullptr));
    CHECK(navgrid_remove_blocked_overlay(grid, second, &changed_count)
        == NAVSYS_STATUS_NOT_FOUND);

    fetched = navcell_t{TERRAIN_TYPE_MOUNTAIN, 91};
    present = true;
    CHECK(navgrid_fetch_cell_ex(grid, 8, 0, &fetched, &present)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(fetched.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(fetched.height == 91);
    CHECK(present);

    prior = navcell_t{TERRAIN_TYPE_MOUNTAIN, 92};
    had_prior = true;
    changed = true;
    CHECK(navgrid_set_cell_ex(
        grid, 8, 0, &forest, &prior, &had_prior, &changed)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(prior.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(prior.height == 92);
    CHECK(had_prior);
    CHECK(changed);
    const coord_t outside[] = {{7, 7}, {8, 0}};
    navgrid_overlay_id_t outside_overlay = 123;
    changed_count = 77;
    CHECK(navgrid_apply_blocked_overlay(
        grid, outside, 2, &outside_overlay, &changed_count)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(outside_overlay == 123);
    CHECK(changed_count == 77);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 7, 7, nullptr));

    CHECK(navgrid_clear_ex(grid, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed);
    CHECK(navgrid_fetch_cell_ex(grid, 3, 4, &fetched, &present)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(present);
    CHECK(fetched.terrain == TERRAIN_TYPE_NORMAL);
    CHECK(fetched.height == 0);
    CHECK(navgrid_clear_ex(grid, &changed) == NAVSYS_STATUS_OK);
    CHECK_FALSE(changed);
    navgrid_destroy(grid);
}

TEST_CASE("navgrid legacy extent and resize characterization") {
    navgrid_t* negative = navgrid_create_full(
        -3, -2, NAVGRID_DIR_4, nullptr);
    REQUIRE(negative != nullptr);
    CHECK(navgrid_is_inside(negative, -3, -2));
    CHECK(navgrid_is_inside(negative, -1, -1));
    CHECK_FALSE(navgrid_is_inside(negative, 0, -1));
    CHECK_FALSE(navgrid_is_inside(negative, -1, 0));
    CHECK_FALSE(navgrid_is_inside(negative, -4, -1));
    navgrid_destroy(negative);

    navgrid_t* mixed = navgrid_create_full(
        2, 0, NAVGRID_DIR_4, nullptr);
    REQUIRE(mixed != nullptr);
    CHECK(navgrid_is_inside(mixed, 0, std::numeric_limits<int>::min()));
    CHECK(navgrid_is_inside(mixed, 1, std::numeric_limits<int>::max()));
    CHECK_FALSE(navgrid_is_inside(mixed, 2, 0));

    const navcell_t retained{TERRAIN_TYPE_FOREST, 29};
    REQUIRE(navgrid_set_cell(mixed, 1, 100, &retained));
    navgrid_set_width(mixed, 1);
    CHECK_FALSE(navgrid_is_inside(mixed, 1, 100));
    navcell_t fetched{};
    CHECK(navgrid_fetch_cell(mixed, 1, 100, &fetched) == 0);
    CHECK(fetched.terrain == TERRAIN_TYPE_FOREST);
    CHECK(fetched.height == 29);
    navgrid_set_width(mixed, 2);
    CHECK(navgrid_is_inside(mixed, 1, 100));
    CHECK(navgrid_fetch_cell(mixed, 1, 100, &fetched) == 0);
    navgrid_destroy(mixed);
}

TEST_CASE("navgrid legacy neighbor order is deterministic and four-way is cardinal") {
    navgrid_t* grid = navgrid_create_full(
        0, 0, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);

    coord_list_t* four = navgrid_copy_neighbors_all(grid, 10, 20);
    check_coord_sequence(four, {
        {10, 19}, {9, 20}, {11, 20}, {10, 21},
    });
    coord_list_destroy(four);

    navgrid_set_mode(grid, NAVGRID_DIR_8);
    coord_list_t* eight = navgrid_copy_neighbors_all(grid, 10, 20);
    check_coord_sequence(eight, {
        {10, 19}, {9, 20}, {11, 20}, {10, 21},
        {9, 19}, {9, 21}, {11, 19}, {11, 21},
    });
    coord_list_destroy(eight);

    navgrid_set_mode(grid, NAVGRID_DIR_4);
    coord_t* legacy_degree = navgrid_copy_neighbor_at_degree(
        grid, 10, 20, 90.0);
    REQUIRE(legacy_degree != nullptr);
    CHECK(legacy_degree->x == 10);
    CHECK(legacy_degree->y == 21);
    coord_destroy(legacy_degree);
    navgrid_destroy(grid);
}

TEST_CASE("navgrid caller-buffer queries are deterministic and atomic") {
    navgrid_t* grid = navgrid_create_full(7, 7, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);

    size_t count = 99;
    CHECK(navgrid_export_neighbors(
        grid, 3, 3, false, nullptr, 0, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 4);

    coord_t short_buffer[3] = {{91, 92}, {93, 94}, {95, 96}};
    count = 99;
    CHECK(navgrid_export_neighbors(
        grid, 3, 3, false, short_buffer, 3, &count)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(count == 4);
    CHECK(short_buffer[0].x == 91);
    CHECK(short_buffer[2].y == 96);

    coord_t neighbors[4]{};
    CHECK(navgrid_export_neighbors(
        grid, 3, 3, false, neighbors, 4, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 4);
    CHECK(neighbors[0].x == 4); CHECK(neighbors[0].y == 3);
    CHECK(neighbors[1].x == 3); CHECK(neighbors[1].y == 4);
    CHECK(neighbors[2].x == 2); CHECK(neighbors[2].y == 3);
    CHECK(neighbors[3].x == 3); CHECK(neighbors[3].y == 2);

    bool changed = false;
    REQUIRE(navgrid_block_coord_ex(grid, 4, 3, &changed) == NAVSYS_STATUS_OK);
    REQUIRE(changed);
    CHECK(navgrid_export_neighbors(
        grid, 3, 3, true, nullptr, 0, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 3);

    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, throw_from_blocked_callback, nullptr) == NAVSYS_STATUS_OK);
    count = 71;
    neighbors[0] = {72, 73};
    CHECK(navgrid_export_neighbors(
        grid, 3, 3, true, neighbors, 4, &count)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(count == 71);
    CHECK(neighbors[0].x == 72);
    CHECK(neighbors[0].y == 73);
    REQUIRE(navgrid_unbind_is_coord_blocked_func(grid) == NAVSYS_STATUS_OK);

    coord_t selected{81, 82};
    CHECK(navgrid_fetch_neighbor_at_degree(grid, 3, 3, 90.0, &selected)
        == NAVSYS_STATUS_OK);
    CHECK(selected.x == 3); CHECK(selected.y == 4);
    const coord_t center{3, 3};
    const coord_t same_goal{3, 3};
    selected = {81, 82};
    CHECK(navgrid_fetch_neighbor_at_goal(
        grid, &center, &same_goal, &selected)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(selected.x == 81); CHECK(selected.y == 82);

    const coord_t east_goal{6, 3};
    CHECK(navgrid_export_neighbors_at_degree_range(
        grid, &center, &east_goal, -45.0, 45.0, 1,
        nullptr, 0, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 3);
    coord_t cone[3]{};
    CHECK(navgrid_export_neighbors_at_degree_range(
        grid, &center, &east_goal, -45.0, 45.0, 1,
        cone, 3, &count) == NAVSYS_STATUS_OK);
    CHECK(cone[0].x == 4); CHECK(cone[0].y == 2);
    CHECK(cone[1].x == 4); CHECK(cone[1].y == 3);
    CHECK(cone[2].x == 4); CHECK(cone[2].y == 4);

    navgrid_destroy(grid);
}

TEST_CASE("navgrid cell export merges base and overlay snapshots") {
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);
    const navcell_t forest{TERRAIN_TYPE_FOREST, 17};
    const navcell_t mountain{TERRAIN_TYPE_MOUNTAIN, 29};
    REQUIRE(navgrid_set_cell(grid, 5, 1, &forest));
    REQUIRE(navgrid_set_cell(grid, 1, 5, &mountain));
    const coord_t overlay_coords[] = {{5, 1}, {3, 3}, {3, 3}};
    navgrid_overlay_id_t overlay = 0;
    size_t changed_count = 0;
    REQUIRE(navgrid_apply_blocked_overlay(
        grid, overlay_coords, 3, &overlay, &changed_count)
        == NAVSYS_STATUS_OK);
    CHECK(changed_count == 2);

    size_t count = 88;
    CHECK(navgrid_export_cells(grid, nullptr, 0, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 3);
    navgrid_cell_entry_t short_entry{{91, 92}, {TERRAIN_TYPE_WATER, 93}, true, false};
    count = 88;
    CHECK(navgrid_export_cells(grid, &short_entry, 1, &count)
        == NAVSYS_STATUS_INCOMPLETE);
    CHECK(count == 3);
    CHECK(short_entry.coord.x == 91);
    CHECK(short_entry.cell.height == 93);

    navgrid_cell_entry_t entries[3]{};
    CHECK(navgrid_export_cells(grid, entries, 3, &count) == NAVSYS_STATUS_OK);
    CHECK(count == 3);
    CHECK(entries[0].coord.x == 1); CHECK(entries[0].coord.y == 5);
    CHECK(entries[0].present); CHECK_FALSE(entries[0].blocked);
    CHECK(entries[0].cell.terrain == TERRAIN_TYPE_MOUNTAIN);
    CHECK(entries[0].cell.height == 29);
    CHECK(entries[1].coord.x == 3); CHECK(entries[1].coord.y == 3);
    CHECK_FALSE(entries[1].present); CHECK(entries[1].blocked);
    CHECK(entries[1].cell.terrain == TERRAIN_TYPE_NORMAL);
    CHECK(entries[1].cell.height == 0);
    CHECK(entries[2].coord.x == 5); CHECK(entries[2].coord.y == 1);
    CHECK(entries[2].present); CHECK(entries[2].blocked);
    CHECK(entries[2].cell.terrain == TERRAIN_TYPE_FOREST);
    CHECK(entries[2].cell.height == 17);

    navgrid_destroy(grid);
}

TEST_CASE("navgrid blocking and checking") {
    navgrid_t* m = navgrid_create();
    CHECK(navgrid_block_coord(m, 6, 6));
    CHECK(is_coord_blocked_navgrid(m, 6, 6, nullptr));
    CHECK_FALSE(is_coord_blocked_navgrid(m, 5, 5, nullptr));
    navgrid_destroy(m);
}

TEST_CASE("navgrid unblock") {
    navgrid_t* m = navgrid_create();
    CHECK(navgrid_block_coord(m, 4, 4));
    CHECK(is_coord_blocked_navgrid(m, 4, 4, nullptr));
    CHECK(navgrid_unblock_coord(m, 4, 4));
    CHECK_FALSE(is_coord_blocked_navgrid(m, 4, 4, nullptr));

    navcell_t absent{};
    CHECK(navgrid_fetch_cell(m, 4, 4, &absent) == -1);
    CHECK_FALSE(navgrid_unblock_coord(m, 4, 4));
    navgrid_destroy(m);
}

TEST_CASE("navgrid block and unblock preserve user terrain and height") {
    navgrid_t* m = navgrid_create();
    navcell_t user_cell{TERRAIN_TYPE_FOREST, 73};
    REQUIRE(navgrid_set_cell(m, 4, 4, &user_cell));

    CHECK_FALSE(navgrid_block_coord(m, 4, 4));
    CHECK_FALSE(navgrid_unblock_coord(m, 4, 4));

    navcell_t fetched{};
    REQUIRE(navgrid_fetch_cell(m, 4, 4, &fetched) == 0);
    CHECK(fetched.terrain == TERRAIN_TYPE_FOREST);
    CHECK(fetched.height == 73);
    CHECK_FALSE(is_coord_blocked_navgrid(m, 4, 4, nullptr));

    const navcell_t unknown{static_cast<terrain_type_t>(101), 11};
    CHECK_FALSE(navgrid_set_cell(m, 5, 5, &unknown));
    CHECK(navgrid_fetch_cell(m, 5, 5, &fetched) == -1);
    CHECK(navgrid_fetch_cell(m, 4, 4, nullptr) == -1);
    navgrid_destroy(m);
}

TEST_CASE("navgrid terrain interpretation belongs to explicit policies") {
    navgrid_t* m = navgrid_create();
    const terrain_type_t terrains[] = {
        TERRAIN_TYPE_NORMAL,
        TERRAIN_TYPE_WATER,
        TERRAIN_TYPE_FOREST,
        TERRAIN_TYPE_MOUNTAIN,
        TERRAIN_TYPE_FORBIDDEN,
    };
    for (int i = 0; i < 5; ++i) {
        const navcell_t cell{terrains[i], 100 + i};
        REQUIRE(navgrid_set_cell(m, i, 0, &cell));
        CHECK(is_coord_blocked_navgrid(m, i, 0, nullptr)
            == (terrains[i] == TERRAIN_TYPE_FORBIDDEN));
    }

    const coord_t corrupted_coord{0, 0};
    auto* corrupted = static_cast<navcell_t*>(
        coord_hash_get(
            const_cast<coord_hash_t*>(navgrid_get_cell_map(m)),
            &corrupted_coord));
    REQUIRE(corrupted != nullptr);
    corrupted->terrain = static_cast<terrain_type_t>(101);
    CHECK(is_coord_blocked_navgrid(m, 0, 0, nullptr));
    corrupted->terrain = TERRAIN_TYPE_NORMAL;

    terrain_type_t selected = TERRAIN_TYPE_WATER;
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        m, block_selected_terrain, &selected) == NAVSYS_STATUS_OK);
    CHECK(block_selected_terrain(m, 1, 0, &selected));
    CHECK_FALSE(block_selected_terrain(m, 2, 0, &selected));
    coord_list_t* neighbors = navgrid_copy_neighbors(m, 0, 0);
    REQUIRE(neighbors != nullptr);
    bool includes_water = false;
    for (int i = 0; i < coord_list_length(neighbors); ++i) {
        const coord_t* neighbor = coord_list_get(neighbors, i);
        includes_water |= neighbor->x == 1 && neighbor->y == 0;
    }
    CHECK_FALSE(includes_water);
    coord_list_destroy(neighbors);

    terrain_cost_policy_t costs{2.0f, 3.0f, 5.0f};
    const coord_t start{0, 0};
    for (int i = 0; i < 4; ++i) {
        const coord_t goal{i, 0};
        const float expected[] = {1.0f, 2.0f, 3.0f, 5.0f};
        CHECK(terrain_cost(m, &start, &goal, &costs) == expected[i]);
    }
    navgrid_destroy(m);
}

TEST_CASE("navgrid clear all") {
    navgrid_t* m = navgrid_create();
    for (int x = 0; x < 5; ++x)
        for (int y = 1; y < 10; ++y)
            navgrid_block_coord(m, x, y);

    CHECK(is_coord_blocked_navgrid(m, 2, 2, nullptr));
    navgrid_clear(m);
    CHECK_FALSE(is_coord_blocked_navgrid(m, 2, 2, nullptr));
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbors filtering") {
    navgrid_t* m = navgrid_create();
    navgrid_block_coord(m, 3, 2);
    navgrid_block_coord(m, 2, 3);

    coord_list_t* neighbors = navgrid_copy_neighbors(m, 2, 2);
    REQUIRE(neighbors);

    int expected = (navgrid_get_mode(m) == NAVGRID_DIR_8) ? 6 : 2;
    CHECK(coord_list_length(neighbors) == expected);

    bool has_21 = false, has_12 = false;
    for (int i = 0; i < coord_list_length(neighbors); ++i) {
        const coord_t* c = coord_list_get(neighbors, i);
        if (coord_get_x(c) == 2 && coord_get_y(c) == 1) has_21 = true;
        if (coord_get_x(c) == 1 && coord_get_y(c) == 2) has_12 = true;
    }
    CHECK(has_21);
    CHECK(has_12);
    coord_list_destroy(neighbors);
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbor at degree") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* c = navgrid_copy_neighbor_at_degree(m, 2, 2, 0.0);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 2);
    coord_destroy(c);
    navgrid_destroy(m);
}

TEST_CASE("navgrid neighbor at goal") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* center = coord_create_full(2, 2);
    coord_t* goal = coord_create_full(4, 1);
    coord_t* c = navgrid_copy_neighbor_at_goal(m, center, goal);
    REQUIRE(c);
    CHECK(coord_get_x(c) == 3);
    CHECK(coord_get_y(c) == 1);
    coord_destroy(center);
    coord_destroy(goal);
    coord_destroy(c);
    navgrid_destroy(m);
}

TEST_CASE("navgrid cone neighbor range") {
    navgrid_t* m = navgrid_create_full(
        5, 5, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* center = coord_create_full(2, 2);
    coord_t* goal = coord_create_full(4, 2);

    coord_list_t* result = navgrid_copy_neighbors_at_degree_range(
        m, center, goal, -45.0, 45.0, 1);

    int count = coord_list_length(result);
    CHECK(count == 3);
    for (int i = 0; i < count; ++i) {
        const coord_t* c = coord_list_get(result, i);
        CHECK(navgrid_is_inside(m, coord_get_x(c), coord_get_y(c)));
    }

    coord_list_destroy(result);
    coord_destroy(center);
    coord_destroy(goal);
    navgrid_destroy(m);
}
