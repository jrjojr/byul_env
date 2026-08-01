#include "doctest.h"

#include "../route_carver/route_carver.h"
#include "../route_carver/internal/route_carver_mutation.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace {

namespace mutation = byul::navsys::route_carver::internal;

std::vector<navgrid_cell_entry_t> snapshot_cells(const navgrid_t* navgrid) {
    std::size_t count = 0;
    REQUIRE(navgrid_export_cells(navgrid, nullptr, 0, &count)
        == NAVSYS_STATUS_OK);
    std::vector<navgrid_cell_entry_t> entries(count);
    if (count != 0) {
        REQUIRE(navgrid_export_cells(
            navgrid, entries.data(), entries.size(), &count)
            == NAVSYS_STATUS_OK);
    }
    return entries;
}

bool same_snapshot(
    const std::vector<navgrid_cell_entry_t>& lhs,
    const std::vector<navgrid_cell_entry_t>& rhs) {
    if (lhs.size() != rhs.size()) return false;
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        if (lhs[index].coord.x != rhs[index].coord.x
            || lhs[index].coord.y != rhs[index].coord.y
            || lhs[index].cell.terrain != rhs[index].cell.terrain
            || lhs[index].cell.height != rhs[index].cell.height
            || lhs[index].present != rhs[index].present
            || lhs[index].blocked != rhs[index].blocked) {
            return false;
        }
    }
    return true;
}

struct callback_state {
    int calls = 0;
    bool blocked = true;
    bool throws = false;
};

struct cancel_state {
    std::size_t calls = 0;
    std::size_t cancel_on = 0;
    bool throws = false;
};

bool cancel_carve(void* userdata) {
    auto& state = *static_cast<cancel_state*>(userdata);
    ++state.calls;
    if (state.throws) throw 1;
    return state.cancel_on != 0 && state.calls == state.cancel_on;
}

navgrid_carve_options_t area_options(cancel_state* cancel = nullptr) {
    return {
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        8,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_ATOMIC,
        0,
        1024,
        cancel ? cancel_carve : nullptr,
        cancel
    };
}

navgrid_t* create_blocked_grid(int side) {
    navgrid_t* grid = navgrid_create_full(side, side, NAVGRID_DIR_8, nullptr);
    if (!grid) return nullptr;
    for (int y = 0; y < side; ++y) {
        for (int x = 0; x < side; ++x) {
            if (!navgrid_block_coord(grid, x, y)) {
                navgrid_destroy(grid);
                return nullptr;
            }
        }
    }
    return grid;
}

std::size_t count_open_cells(const navgrid_t* grid, int side) {
    std::size_t result = 0;
    for (int y = 0; y < side; ++y)
        for (int x = 0; x < side; ++x)
            if (!is_coord_blocked_navgrid(grid, x, y, nullptr)) ++result;
    return result;
}

bool virtual_block_callback(
    const void*, int, int, void* userdata) {
    auto& state = *static_cast<callback_state*>(userdata);
    ++state.calls;
    if (state.throws) throw std::runtime_error("callback failure");
    return state.blocked;
}

} // namespace

TEST_CASE("route carver atomic open preserves terrain and blocked overlay provenance") {
    navgrid_t* grid = navgrid_create_full(5, 5, NAVGRID_DIR_8, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t target{2, 2};
    const navcell_t forest{TERRAIN_TYPE_FOREST, 7};
    navcell_t prior{};
    bool had_prior = false;
    bool changed = false;
    REQUIRE(navgrid_set_cell_ex(
        grid, target.x, target.y, &forest,
        &prior, &had_prior, &changed) == NAVSYS_STATUS_OK);

    navgrid_overlay_id_t first = 0;
    navgrid_overlay_id_t second = 0;
    std::size_t overlay_changed = 0;
    REQUIRE(navgrid_apply_blocked_overlay(
        grid, &target, 1, &first, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 1);
    REQUIRE(navgrid_apply_blocked_overlay(
        grid, &target, 1, &second, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 0);
    CHECK(is_coord_blocked_navgrid(grid, 2, 2, nullptr));

    std::size_t carve_changed = 99;
    REQUIRE(mutation::mutate_candidates_atomic(
        grid, &target, 1, mutation::carve_match::effective_blocked,
        false, true, nullptr, nullptr, &carve_changed) == NAVSYS_STATUS_OK);
    CHECK(carve_changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 2, nullptr));

    navcell_t preserved{};
    bool present = false;
    REQUIRE(navgrid_fetch_cell_ex(
        grid, 2, 2, &preserved, &present) == NAVSYS_STATUS_OK);
    CHECK(present);
    CHECK(preserved.terrain == TERRAIN_TYPE_FOREST);
    CHECK(preserved.height == 7);

    REQUIRE(navgrid_remove_blocked_overlay(
        grid, first, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 0);
    REQUIRE(navgrid_remove_blocked_overlay(
        grid, second, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 0);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 2, nullptr));

    navgrid_overlay_id_t later = 0;
    REQUIRE(navgrid_apply_blocked_overlay(
        grid, &target, 1, &later, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 1);
    CHECK(is_coord_blocked_navgrid(grid, 2, 2, nullptr));
    REQUIRE(navgrid_remove_blocked_overlay(
        grid, later, &overlay_changed) == NAVSYS_STATUS_OK);
    CHECK(overlay_changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 2, nullptr));
    navgrid_destroy(grid);
}

TEST_CASE("route carver stored match opens without overwriting forbidden base") {
    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t target{1, 1};
    const navcell_t forbidden{TERRAIN_TYPE_FORBIDDEN, 9};
    navcell_t prior{};
    bool had_prior = false;
    bool changed = false;
    REQUIRE(navgrid_set_cell_ex(
        grid, 1, 1, &forbidden, &prior, &had_prior, &changed)
        == NAVSYS_STATUS_OK);

    std::size_t carve_changed = 0;
    REQUIRE(mutation::mutate_candidates_atomic(
        grid, &target, 1,
        mutation::carve_match::stored_forbidden_only,
        false, true, nullptr, nullptr, &carve_changed) == NAVSYS_STATUS_OK);
    CHECK(carve_changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 1, 1, nullptr));

    navcell_t preserved{};
    bool present = false;
    REQUIRE(navgrid_fetch_cell_ex(
        grid, 1, 1, &preserved, &present) == NAVSYS_STATUS_OK);
    CHECK(present);
    CHECK(preserved.terrain == TERRAIN_TYPE_FORBIDDEN);
    CHECK(preserved.height == 9);
    navgrid_destroy(grid);
}

TEST_CASE("route carver dry run counts the same change without mutation") {
    navgrid_t* grid = navgrid_create_full(3, 3, NAVGRID_DIR_8, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t target{1, 1};
    REQUIRE(navgrid_block_coord(grid, 1, 1));
    const auto before = snapshot_cells(grid);

    std::size_t changed = 0;
    REQUIRE(mutation::mutate_candidates_atomic(
        grid, &target, 1, mutation::carve_match::effective_blocked,
        true, true, nullptr, nullptr, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK(is_coord_blocked_navgrid(grid, 1, 1, nullptr));
    CHECK(same_snapshot(before, snapshot_cells(grid)));

    REQUIRE(mutation::mutate_candidates_atomic(
        grid, &target, 1, mutation::carve_match::effective_blocked,
        false, true, nullptr, nullptr, &changed) == NAVSYS_STATUS_OK);
    CHECK(changed == 1);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 1, 1, nullptr));
    navgrid_destroy(grid);
}

TEST_CASE("route carver effective match does not claim query-only callback blocks") {
    navgrid_t* grid = navgrid_create_full(3, 3, NAVGRID_DIR_8, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t target{1, 1};
    REQUIRE(navgrid_block_coord(grid, 1, 1));
    callback_state callback{};
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, virtual_block_callback, &callback) == NAVSYS_STATUS_OK);
    const auto before = snapshot_cells(grid);

    std::size_t changed = 77;
    REQUIRE(mutation::mutate_candidates_atomic(
        grid, &target, 1, mutation::carve_match::effective_blocked,
        false, true, nullptr, nullptr, &changed) == NAVSYS_STATUS_OK);
    CHECK(callback.calls == 1);
    CHECK(changed == 0);
    CHECK(same_snapshot(before, snapshot_cells(grid)));
    CHECK(is_coord_blocked_navgrid(grid, 1, 1, nullptr));
    navgrid_destroy(grid);
}

TEST_CASE("route carver callback and validation failures preserve grid and output") {
    navgrid_t* grid = navgrid_create_full(3, 3, NAVGRID_DIR_8, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t target{1, 1};
    REQUIRE(navgrid_block_coord(grid, 1, 1));
    callback_state callback{};
    callback.throws = true;
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        grid, virtual_block_callback, &callback) == NAVSYS_STATUS_OK);
    const auto before = snapshot_cells(grid);

    std::size_t changed = 77;
    CHECK(mutation::mutate_candidates_atomic(
        grid, &target, 1, mutation::carve_match::effective_blocked,
        false, true, nullptr, nullptr, &changed)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(changed == 77);
    CHECK(same_snapshot(before, snapshot_cells(grid)));

    CHECK(mutation::mutate_candidates_atomic(
        grid, &target, 1, static_cast<mutation::carve_match>(99),
        false, true, nullptr, nullptr, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(changed == 77);
    CHECK(same_snapshot(before, snapshot_cells(grid)));
    navgrid_destroy(grid);
}

TEST_CASE("route carver checked status separates no change and failures") {
    navgrid_t* grid = navgrid_create_full(5, 5, NAVGRID_DIR_4, nullptr);
    REQUIRE(grid != nullptr);
    const coord_t center{2, 2};
    auto options = area_options();
    options.radius_cells = 0;

    std::size_t changed = 77;
    CHECK(navgrid_carve_area(grid, &center, &options, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 0);

    options.metric = 99;
    changed = 77;
    CHECK(navgrid_carve_area(grid, &center, &options, &changed)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(changed == 77);

    options.metric = NAVGRID_CARVE_CHEBYSHEV_SQUARE;
    options.max_cells = 0;
    CHECK(navgrid_carve_area(grid, &center, &options, &changed)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(changed == 77);

    options.max_cells = 1;
    options.radius_cells = 2;
    CHECK(navgrid_carve_area(grid, &center, &options, &changed)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(changed == 77);
    navgrid_destroy(grid);
}

TEST_CASE("route carver atomic cancellation at every poll preserves state") {
    constexpr int side = 17;
    const coord_t center{8, 8};
    cancel_state baseline_cancel{};
    auto baseline_options = area_options(&baseline_cancel);
    baseline_options.flags |= NAVGRID_CARVE_DRY_RUN;
    navgrid_t* baseline_grid = create_blocked_grid(side);
    REQUIRE(baseline_grid != nullptr);
    std::size_t changed = 0;
    REQUIRE(navgrid_carve_area(
        baseline_grid, &center, &baseline_options, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 289);
    const std::size_t poll_count = baseline_cancel.calls;
    CHECK(poll_count > 8);
    navgrid_destroy(baseline_grid);

    for (std::size_t poll = 1; poll <= poll_count; ++poll) {
        navgrid_t* grid = create_blocked_grid(side);
        REQUIRE(grid != nullptr);
        cancel_state cancel{0, poll, false};
        auto options = area_options(&cancel);
        std::size_t output = 77;
        CHECK(navgrid_carve_area(grid, &center, &options, &output)
            == NAVSYS_STATUS_CANCELLED);
        CHECK(output == 77);
        CHECK(count_open_cells(grid, side) == 0);
        navgrid_destroy(grid);
    }
}

TEST_CASE("route carver non atomic cancellation reports deterministic partial count") {
    constexpr int side = 17;
    const coord_t center{8, 8};
    navgrid_t* baseline_grid = create_blocked_grid(side);
    REQUIRE(baseline_grid != nullptr);
    cancel_state baseline_cancel{};
    auto baseline_options = area_options(&baseline_cancel);
    baseline_options.flags &= ~NAVGRID_CARVE_ATOMIC;
    std::size_t changed = 0;
    REQUIRE(navgrid_carve_area(
        baseline_grid, &center, &baseline_options, &changed)
        == NAVSYS_STATUS_OK);
    CHECK(changed == 289);
    const std::size_t poll_count = baseline_cancel.calls;
    navgrid_destroy(baseline_grid);

    bool observed_partial = false;
    for (std::size_t poll = 1; poll <= poll_count; ++poll) {
        navgrid_t* grid = create_blocked_grid(side);
        REQUIRE(grid != nullptr);
        cancel_state cancel{0, poll, false};
        auto options = area_options(&cancel);
        options.flags &= ~NAVGRID_CARVE_ATOMIC;
        std::size_t output = 77;
        const navsys_status_t status = navgrid_carve_area(
            grid, &center, &options, &output);
        if (status == NAVSYS_STATUS_CANCELLED) {
            const std::size_t opened = count_open_cells(grid, side);
            if (opened != 0) {
                observed_partial = true;
                CHECK(output == opened);

                cancel_state retry_cancel{};
                options.cancel_userdata = &retry_cancel;
                std::size_t retry_changed = 0;
                CHECK(navgrid_carve_area(grid, &center, &options, &retry_changed)
                    == NAVSYS_STATUS_OK);
                CHECK(retry_changed + opened == 289);
            } else {
                CHECK((output == 0 || output == 77));
            }
        } else {
            CHECK(status == NAVSYS_STATUS_OK);
            CHECK(output == 289);
        }
        navgrid_destroy(grid);
    }
    CHECK(observed_partial);

    navgrid_t* throwing_grid = create_blocked_grid(side);
    REQUIRE(throwing_grid != nullptr);
    cancel_state throwing{0, 0, true};
    auto throwing_options = area_options(&throwing);
    std::size_t output = 91;
    CHECK(navgrid_carve_area(
        throwing_grid, &center, &throwing_options, &output)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(output == 91);
    CHECK(count_open_cells(throwing_grid, side) == 0);
    navgrid_destroy(throwing_grid);
}
