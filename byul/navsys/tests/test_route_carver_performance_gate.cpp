/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "route_carver.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

namespace {

struct alignas(std::max_align_t) allocation_header {
    std::size_t payload;
    bool tracked;
};

thread_local bool tracking = false;
thread_local std::size_t allocation_count = 0;
thread_local std::size_t allocated_bytes = 0;
thread_local std::size_t live_bytes = 0;
thread_local std::size_t peak_bytes = 0;

void* allocate(std::size_t size) {
    const std::size_t payload = size == 0 ? 1 : size;
    auto* header = static_cast<allocation_header*>(
        std::malloc(sizeof(allocation_header) + payload));
    if (!header) throw std::bad_alloc();
    header->payload = payload;
    header->tracked = tracking;
    if (header->tracked) {
        ++allocation_count;
        allocated_bytes += payload;
        live_bytes += payload;
        peak_bytes = std::max(peak_bytes, live_bytes);
    }
    return header + 1;
}

void deallocate(void* pointer) noexcept {
    if (!pointer) return;
    auto* header = static_cast<allocation_header*>(pointer) - 1;
    if (header->tracked) live_bytes -= header->payload;
    std::free(header);
}

struct measurement {
    navsys_status_t status = NAVSYS_STATUS_CORRUPT_STATE;
    std::size_t changed = 0;
    std::size_t callback_calls = 0;
    std::size_t allocations = 0;
    std::size_t bytes = 0;
    std::size_t peak = 0;
    std::size_t live_after = 0;
    std::uint64_t elapsed_ns = 0;
};

void begin_measurement() {
    allocation_count = 0;
    allocated_bytes = 0;
    live_bytes = 0;
    peak_bytes = 0;
    tracking = true;
}

void finish_measurement(measurement& result) {
    tracking = false;
    result.allocations = allocation_count;
    result.bytes = allocated_bytes;
    result.peak = peak_bytes;
    result.live_after = live_bytes;
}

bool count_cancel_polls(void* userdata) {
    ++*static_cast<std::size_t*>(userdata);
    return false;
}

navgrid_t* create_forbidden_grid(
    int width, int height, navgrid_dir_mode_t mode) {
    navgrid_t* grid = navgrid_create_full(width, height, mode, nullptr);
    if (!grid) return nullptr;
    std::vector<coord_t> coords;
    coords.reserve(static_cast<std::size_t>(width)
        * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) coords.push_back({x, y});
    }
    navgrid_overlay_id_t overlay = 0;
    std::size_t changed = 0;
    if (navgrid_apply_blocked_overlay(
            grid, coords.data(), coords.size(), &overlay, &changed)
            != NAVSYS_STATUS_OK
        || overlay == 0 || changed != coords.size()) {
        navgrid_destroy(grid);
        return nullptr;
    }
    return grid;
}

navgrid_carve_options_t line_options(
    std::uint32_t radius,
    navgrid_carve_metric_t metric,
    std::size_t* callback_calls) {
    return {
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        radius,
        static_cast<std::uint32_t>(metric),
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_INCLUDE_START | NAVGRID_CARVE_INCLUDE_END
            | NAVGRID_CARVE_CLIP_TO_EXTENT | NAVGRID_CARVE_ATOMIC
            | NAVGRID_CARVE_DRY_RUN,
        0,
        NAVGRID_CARVE_MAX_CELLS,
        callback_calls ? count_cancel_polls : nullptr,
        callback_calls
    };
}

measurement measure_canonical_line() {
    navgrid_t* grid = create_forbidden_grid(640, 64, NAVGRID_DIR_8);
    if (!grid) return {};
    const coord_t start{32, 32};
    const coord_t goal{607, 32};
    measurement result{};
    auto options = line_options(
        8, NAVGRID_CARVE_CHEBYSHEV_SQUARE, &result.callback_calls);

    begin_measurement();
    const auto before = std::chrono::steady_clock::now();
    result.status = navgrid_carve_line(
        grid, &start, &goal, &options, &result.changed);
    const auto after = std::chrono::steady_clock::now();
    result.elapsed_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(after - before)
            .count());
    finish_measurement(result);
    navgrid_destroy(grid);
    return result;
}

measurement measure_legacy_neighbor_union() {
    navgrid_t* grid = create_forbidden_grid(192, 32, NAVGRID_DIR_8);
    if (!grid) return {};
    const coord_t start{32, 16};
    const coord_t goal{159, 16};
    measurement result{};

    begin_measurement();
    const auto before = std::chrono::steady_clock::now();
    const int changed = route_carve_beam(grid, &start, &goal, 2);
    const auto after = std::chrono::steady_clock::now();
    result.status = changed >= 0 ? NAVSYS_STATUS_OK : NAVSYS_STATUS_CORRUPT_STATE;
    result.changed = changed >= 0 ? static_cast<std::size_t>(changed) : 0;
    result.elapsed_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(after - before)
            .count());
    finish_measurement(result);
    navgrid_destroy(grid);
    return result;
}

bool verify_metric_counts() {
    navgrid_t* grid = create_forbidden_grid(96, 96, NAVGRID_DIR_4);
    if (!grid) return false;
    const coord_t center{48, 48};
    const struct {
        navgrid_carve_metric_t metric;
        std::size_t expected;
    } cases[] = {
        {NAVGRID_CARVE_CHEBYSHEV_SQUARE, 4225},
        {NAVGRID_CARVE_MANHATTAN_DIAMOND, 2113},
        {NAVGRID_CARVE_EUCLIDEAN_DISK, 3209}
    };
    for (const auto& item : cases) {
        auto options = line_options(32, item.metric, nullptr);
        options.flags &= ~(NAVGRID_CARVE_INCLUDE_START | NAVGRID_CARVE_INCLUDE_END);
        std::size_t changed = 0;
        if (navgrid_carve_area(grid, &center, &options, &changed)
                != NAVSYS_STATUS_OK
            || changed != item.expected) {
            navgrid_destroy(grid);
            return false;
        }
    }
    navgrid_destroy(grid);
    return true;
}

bool verify_topology_and_unbounded() {
    const coord_t start{4, 8};
    const coord_t goal{59, 23};
    std::size_t counts[2]{};
    const navgrid_dir_mode_t modes[] = {NAVGRID_DIR_4, NAVGRID_DIR_8};
    for (std::size_t index = 0; index < 2; ++index) {
        navgrid_t* grid = create_forbidden_grid(64, 32, modes[index]);
        if (!grid) return false;
        auto options = line_options(3, NAVGRID_CARVE_EUCLIDEAN_DISK, nullptr);
        if (navgrid_carve_line(
                grid, &start, &goal, &options, &counts[index])
            != NAVSYS_STATUS_OK) {
            navgrid_destroy(grid);
            return false;
        }
        navgrid_destroy(grid);
    }
    if (counts[0] != counts[1]) return false;

    navgrid_t* unbounded = navgrid_create();
    if (!unbounded) return false;
    const coord_t outside_start{-128, -64};
    const coord_t outside_goal{128, 64};
    auto options = line_options(2, NAVGRID_CARVE_MANHATTAN_DIAMOND, nullptr);
    options.flags &= ~NAVGRID_CARVE_CLIP_TO_EXTENT;
    std::size_t changed = 99;
    const navsys_status_t status = navgrid_carve_line(
        unbounded, &outside_start, &outside_goal, &options, &changed);
    navgrid_destroy(unbounded);
    return status == NAVSYS_STATUS_OK && changed == 0;
}

double throughput(const measurement& value) {
    if (value.elapsed_ns == 0) return 0.0;
    return static_cast<double>(value.changed) * 1.0e9
        / static_cast<double>(value.elapsed_ns);
}

} // namespace

void* operator new(std::size_t size) {
    return allocate(size);
}

void* operator new[](std::size_t size) {
    return allocate(size);
}

void operator delete(void* pointer) noexcept {
    deallocate(pointer);
}

void operator delete[](void* pointer) noexcept {
    deallocate(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept {
    deallocate(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept {
    deallocate(pointer);
}

int main() {
    constexpr std::uint64_t maximum_elapsed_ns = UINT64_C(5000000000);
    constexpr std::size_t maximum_allocations_per_candidate = 16;
    constexpr std::size_t maximum_canonical_peak_bytes = 64 * 1024 * 1024;
    constexpr double minimum_unique_cells_per_second = 1000.0;

    const measurement canonical = measure_canonical_line();
    const measurement legacy = measure_legacy_neighbor_union();
    const double canonical_throughput = throughput(canonical);
    const double legacy_throughput = throughput(legacy);

    const bool valid = canonical.status == NAVSYS_STATUS_OK
        && canonical.changed == 10064
        && canonical.callback_calls > 0
        && canonical.callback_calls <= canonical.changed * 2
        && canonical.elapsed_ns <= maximum_elapsed_ns
        && canonical.allocations
            <= canonical.changed * maximum_allocations_per_candidate
        && canonical.peak <= maximum_canonical_peak_bytes
        && canonical.live_after == 0
        && canonical_throughput >= minimum_unique_cells_per_second
        && legacy.status == NAVSYS_STATUS_OK
        && legacy.changed > 0
        && canonical.allocations < legacy.allocations
        && verify_metric_counts()
        && verify_topology_and_unbounded();

    std::printf(
        "{\"schema_version\":1,\"seed\":\"not-applicable\","
        "\"canonical\":{\"line_length_cells\":576,\"radius\":8,"
        "\"metric\":\"chebyshev_square\",\"callback\":\"count-only\","
        "\"unique_candidates\":%zu,\"callback_calls\":%zu,"
        "\"allocations\":%zu,\"allocated_bytes\":%zu,"
        "\"peak_bytes\":%zu,\"live_bytes_after\":%zu,"
        "\"elapsed_ns\":%llu,\"unique_cells_per_second\":%.3f},"
        "\"legacy_neighbor_union\":{\"line_length_cells\":128,"
        "\"range\":2,\"changed\":%zu,\"allocations\":%zu,"
        "\"allocated_bytes\":%zu,\"peak_bytes\":%zu,"
        "\"elapsed_ns\":%llu,\"changed_cells_per_second\":%.3f},"
        "\"thresholds\":{\"maximum_elapsed_ns\":%llu,"
        "\"maximum_allocations_per_candidate\":%zu,"
        "\"maximum_canonical_peak_bytes\":%zu,"
        "\"minimum_unique_cells_per_second\":%.3f},"
        "\"metric_radius\":32,\"topology_modes\":[4,8],"
        "\"bounded_and_unbounded\":true,\"passed\":%s}\n",
        canonical.changed,
        canonical.callback_calls,
        canonical.allocations,
        canonical.bytes,
        canonical.peak,
        canonical.live_after,
        static_cast<unsigned long long>(canonical.elapsed_ns),
        canonical_throughput,
        legacy.changed,
        legacy.allocations,
        legacy.bytes,
        legacy.peak,
        static_cast<unsigned long long>(legacy.elapsed_ns),
        legacy_throughput,
        static_cast<unsigned long long>(maximum_elapsed_ns),
        maximum_allocations_per_candidate,
        maximum_canonical_peak_bytes,
        minimum_unique_cells_per_second,
        valid ? "true" : "false");
    return valid ? 0 : 1;
}
