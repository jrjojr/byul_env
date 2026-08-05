/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "navsys.h"
#include "maze_binary.h"
#include "maze_eller.h"
#include "maze_hunt_and_kill.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

namespace {

struct alignas(std::max_align_t) allocation_header {
    bool tracked;
};

thread_local bool track_allocations = false;
thread_local std::ptrdiff_t fail_after = -1;
thread_local std::size_t tracked_live_allocations = 0;

void* allocate_for_test(std::size_t size) {
    if (fail_after == 0) {
        fail_after = -1;
        throw std::bad_alloc();
    }
    if (fail_after > 0) --fail_after;

    const std::size_t payload = size == 0 ? 1 : size;
    auto* header = static_cast<allocation_header*>(
        std::malloc(sizeof(allocation_header) + payload));
    if (!header) throw std::bad_alloc();
    header->tracked = track_allocations;
    if (header->tracked) ++tracked_live_allocations;
    return header + 1;
}

void deallocate_for_test(void* pointer) noexcept {
    if (!pointer) return;
    auto* header = static_cast<allocation_header*>(pointer) - 1;
    if (header->tracked) --tracked_live_allocations;
    std::free(header);
}

navgrid_t* dependency_navgrid = nullptr;

navgrid_t* create_navgrid() {
    return navgrid_create();
}

void destroy_navgrid(navgrid_t* navgrid) {
    navgrid_destroy(navgrid);
}

route_finder_t* create_route_finder() {
    return route_finder_create(dependency_navgrid);
}

void destroy_route_finder(route_finder_t* finder) {
    route_finder_destroy(finder);
}

dstar_lite_t* create_dstar_lite() {
    return dstar_lite_create(dependency_navgrid);
}

void destroy_dstar_lite(dstar_lite_t* dsl) {
    dstar_lite_destroy(dsl);
}

route_t* create_route() {
    return route_create();
}

void destroy_route(route_t* route) {
    route_destroy(route);
}

route_builder_t* create_route_builder() {
    route_builder_t* builder = nullptr;
    return route_builder_create(&builder) == NAVSYS_STATUS_OK
        ? builder
        : nullptr;
}

void destroy_route_builder(route_builder_t* builder) {
    route_builder_destroy(builder);
}

route_heading_tracker_t* create_route_heading_tracker() {
    route_heading_tracker_t* tracker = nullptr;
    return route_heading_tracker_create(&tracker) == NAVSYS_STATUS_OK
        ? tracker
        : nullptr;
}

void destroy_route_heading_tracker(route_heading_tracker_t* tracker) {
    route_heading_tracker_destroy(tracker);
}

navsys_search_trace_t* create_navsys_search_trace() {
    navsys_search_trace_t* trace = nullptr;
    return navsys_search_trace_create(&trace) == NAVSYS_STATUS_OK
        ? trace
        : nullptr;
}

void destroy_navsys_search_trace(navsys_search_trace_t* trace) {
    navsys_search_trace_destroy(trace);
}

void* copy_coord_for_hash(const void* value) {
    return coord_copy(static_cast<const coord_t*>(value));
}

void destroy_coord_for_hash(void* value) {
    coord_destroy(static_cast<coord_t*>(value));
}

coord_t* create_checked_coord() {
    coord_t* coord = nullptr;
    return coord_create_checked(7, 9, &coord) == NAVSYS_STATUS_OK
        ? coord
        : nullptr;
}

navcell_t* create_checked_navcell() {
    navcell_t* cell = nullptr;
    return navcell_create_checked(TERRAIN_TYPE_FOREST, 17, &cell)
            == NAVSYS_STATUS_OK
        ? cell
        : nullptr;
}

bool verify_navcell_checked_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    const navcell_t source = {TERRAIN_TYPE_WATER, 19};
    navcell_t* const sentinel = reinterpret_cast<navcell_t*>(1);

    navcell_t* created = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status = navcell_create_checked(
        source.terrain, source.height, &created);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || created != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "navcell_create_checked did not preserve output on allocation failure\n");
        return false;
    }

    navcell_t* copied = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t copy_status =
        navcell_copy_checked(&source, &copied);
    fail_after = -1;
    track_allocations = false;
    if (copy_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || copied != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "navcell_copy_checked did not preserve output on allocation failure\n");
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    navcell_t* legacy_created =
        navcell_create_full(source.terrain, source.height);
    fail_after = -1;
    track_allocations = false;
    if (legacy_created != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "navcell_create_full leaked an allocation failure\n");
        navcell_destroy(legacy_created);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    navcell_t* legacy_copied = navcell_copy(&source);
    fail_after = -1;
    track_allocations = false;
    if (legacy_copied != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "navcell_copy leaked an allocation failure\n");
        navcell_destroy(legacy_copied);
        return false;
    }

    return true;
}

bool verify_coord_checked_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    coord_t source = {7, 9};
    coord_t* const sentinel = reinterpret_cast<coord_t*>(1);

    coord_t* created = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status =
        coord_create_checked(source.x, source.y, &created);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || created != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_create_checked did not preserve output on allocation failure\n");
        return false;
    }

    coord_t* copied = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t copy_status =
        coord_copy_checked(&source, &copied);
    fail_after = -1;
    track_allocations = false;
    if (copy_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || copied != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_copy_checked did not preserve output on allocation failure\n");
        return false;
    }

    return true;
}

bool verify_dstar_lite_key_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    const dstar_lite_key_t source = {7.0f, 9.0f};
    dstar_lite_key_t* const sentinel =
        reinterpret_cast<dstar_lite_key_t*>(1);

    dstar_lite_key_t* created = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status =
        dstar_lite_key_create_ex(source.k1, source.k2, &created);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || created != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "dstar_lite_key_create_ex did not preserve output on allocation failure\n");
        return false;
    }

    dstar_lite_key_t* copied = sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t copy_status =
        dstar_lite_key_copy_ex(&source, &copied);
    fail_after = -1;
    track_allocations = false;
    if (copy_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || copied != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "dstar_lite_key_copy_ex did not preserve output on allocation failure\n");
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    dstar_lite_key_t* legacy_created = dstar_lite_key_create();
    fail_after = -1;
    track_allocations = false;
    if (legacy_created != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "dstar_lite_key_create leaked an allocation failure\n");
        dstar_lite_key_destroy(legacy_created);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    legacy_created = dstar_lite_key_create_full(source.k1, source.k2);
    fail_after = -1;
    track_allocations = false;
    if (legacy_created != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "dstar_lite_key_create_full leaked an allocation failure\n");
        dstar_lite_key_destroy(legacy_created);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    dstar_lite_key_t* legacy_copied = dstar_lite_key_copy(&source);
    fail_after = -1;
    track_allocations = false;
    if (legacy_copied != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "dstar_lite_key_copy leaked an allocation failure\n");
        dstar_lite_key_destroy(legacy_copied);
        return false;
    }

    return true;
}

bool verify_route_checked_allocation_failure() {
    constexpr std::ptrdiff_t max_allocations = 128;
    const coord_t first = {1, 2};
    const coord_t second = {3, 4};
    route_t* source = route_create();
    if (!source
        || !route_add_coord(source, &first)
        || !route_add_coord(source, &second)) {
        route_destroy(source);
        return false;
    }

    const auto verify_route_output = [source](bool slice) {
        constexpr std::ptrdiff_t limit = 128;
        for (std::ptrdiff_t index = 0; index < limit; ++index) {
            const std::size_t baseline = tracked_live_allocations;
            route_t* const sentinel = reinterpret_cast<route_t*>(1);
            route_t* output = sentinel;
            track_allocations = true;
            fail_after = index;
            const navsys_status_t status = slice
                ? route_slice_ex(source, 0, 2, &output)
                : route_clone_ex(source, &output);
            fail_after = -1;

            if (status == NAVSYS_STATUS_OK) {
                if (!output || output == sentinel || route_length(output) != 2) {
                    track_allocations = false;
                    return false;
                }
                route_destroy(output);
                track_allocations = false;
                return tracked_live_allocations == baseline;
            }
            track_allocations = false;
            if (status != NAVSYS_STATUS_OUT_OF_MEMORY
                || output != sentinel
                || tracked_live_allocations != baseline) {
                std::fprintf(
                    stderr,
                    "route_%s_ex was not failure-atomic at allocation %td\n",
                    slice ? "slice" : "clone",
                    index);
                return false;
            }
        }
        return false;
    };

    if (!verify_route_output(false) || !verify_route_output(true)) {
        route_destroy(source);
        return false;
    }

    bool builder_append_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        route_builder_t* builder = nullptr;
        if (route_builder_create_from_route(source, &builder)
            != NAVSYS_STATUS_OK) {
            route_destroy(source);
            return false;
        }

        const std::size_t baseline = tracked_live_allocations;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = route_builder_append(
            builder, source, ROUTE_JOIN_KEEP_ALL);
        fail_after = -1;

        route_t* result = nullptr;
        const navsys_status_t finish_status =
            route_builder_finish(builder, &result);
        route_builder_destroy(builder);
        const bool valid = finish_status == NAVSYS_STATUS_OK
            && result
            && route_get_coord_count(result)
                == (status == NAVSYS_STATUS_OK ? 4u : 2u)
            && (status == NAVSYS_STATUS_OK
                || status == NAVSYS_STATUS_OUT_OF_MEMORY)
            && route_get_coord_count(source) == 2u;
        builder_append_succeeded = status == NAVSYS_STATUS_OK;
        route_destroy(result);
        track_allocations = false;
        if (!valid || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "route_builder_append was not failure-atomic at allocation %td\n",
                index);
            route_destroy(source);
            return false;
        }
        if (builder_append_succeeded) break;
    }
    if (!builder_append_succeeded) {
        route_destroy(source);
        return false;
    }

    coord_hash_t* predecessors = coord_hash_create_full(
        copy_coord_for_hash,
        destroy_coord_for_hash);
    if (!predecessors
        || !coord_hash_replace(
            predecessors, &second, const_cast<coord_t*>(&first))) {
        coord_hash_destroy(predecessors);
        route_destroy(source);
        return false;
    }

    bool reconstruct_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        route_t* destination = route_create();
        const coord_t marker = {-1, -1};
        if (!destination || !route_add_coord(destination, &marker)) {
            route_destroy(destination);
            break;
        }

        const std::size_t baseline = tracked_live_allocations;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = route_reconstruct_ex(
            destination, predecessors, &first, &second);
        fail_after = -1;

        bool valid = false;
        if (status == NAVSYS_STATUS_OK) {
            valid = route_length(destination) == 3;
            reconstruct_succeeded = valid;
        } else {
            const coord_t* preserved = route_get_coord_at(destination, 0);
            valid = status == NAVSYS_STATUS_OUT_OF_MEMORY
                && route_length(destination) == 1
                && preserved
                && preserved->x == marker.x
                && preserved->y == marker.y;
        }
        route_destroy(destination);
        track_allocations = false;
        if (!valid || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "route_reconstruct_ex was not failure-atomic at allocation %td\n",
                index);
            coord_hash_destroy(predecessors);
            route_destroy(source);
            return false;
        }
        if (reconstruct_succeeded) break;
    }

    bool visited_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        route_t* destination = route_create();
        if (!destination) break;
        const std::size_t baseline = tracked_live_allocations;
        track_allocations = true;
        fail_after = index;
        const int added = route_add_visited(destination, &first);
        fail_after = -1;
        const bool valid = added
            ? coord_list_size(route_get_visited_order(destination)) == 1
                && coord_hash_size(route_get_visited_count(destination)) == 1
            : coord_list_size(route_get_visited_order(destination)) == 0
                && coord_hash_size(route_get_visited_count(destination)) == 0;
        visited_succeeded = added != 0;
        route_destroy(destination);
        track_allocations = false;
        if (!valid || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "route_add_visited was not failure-atomic at allocation %td\n",
                index);
            coord_hash_destroy(predecessors);
            route_destroy(source);
            return false;
        }
        if (visited_succeeded) break;
    }

    coord_hash_destroy(predecessors);
    route_destroy(source);
    return reconstruct_succeeded && visited_succeeded;
}

#if !defined(_MSC_VER)
bool verify_coord_list_checked_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    coord_list_t* const pointer_sentinel =
        reinterpret_cast<coord_list_t*>(1);

    coord_list_t* created = pointer_sentinel;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status =
        coord_list_create_ex(&created);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || created != pointer_sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_list_create_ex did not preserve output on allocation failure\n");
        return false;
    }

    coord_list_t* list = nullptr;
    if (coord_list_create_ex(&list) != NAVSYS_STATUS_OK || !list) {
        return false;
    }
    coord_t first = {1, 2};
    coord_t second = {3, 4};

    track_allocations = true;
    fail_after = 0;
    const navsys_status_t push_status =
        coord_list_push_back_ex(list, &first);
    fail_after = -1;
    track_allocations = false;
    if (push_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || coord_list_size(list) != 0
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_list_push_back_ex was not failure-atomic\n");
        coord_list_destroy(list);
        return false;
    }

    if (coord_list_push_back_ex(list, &first) != NAVSYS_STATUS_OK) {
        coord_list_destroy(list);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    const navsys_status_t insert_status =
        coord_list_insert_ex(list, 0, &second);
    fail_after = -1;
    track_allocations = false;
    coord_t preserved = {-1, -1};
    if (insert_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || coord_list_size(list) != 1
        || coord_list_fetch(list, 0, &preserved) != NAVSYS_STATUS_OK
        || preserved.x != first.x
        || preserved.y != first.y
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_list_insert_ex was not failure-atomic\n");
        coord_list_destroy(list);
        return false;
    }

    const std::ptrdiff_t copy_failure_points[] = {0, 1};
    for (const std::ptrdiff_t allocation : copy_failure_points) {
        coord_list_t* copied = pointer_sentinel;
        track_allocations = true;
        fail_after = allocation;
        const navsys_status_t copy_status =
            coord_list_copy_ex(list, &copied);
        fail_after = -1;
        track_allocations = false;
        if (copy_status != NAVSYS_STATUS_OUT_OF_MEMORY
            || copied != pointer_sentinel
            || coord_list_size(list) != 1
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "coord_list_copy_ex was not failure-atomic at allocation %td\n",
                allocation);
            coord_list_destroy(list);
            return false;
        }
    }

    coord_list_destroy(list);
    return true;
}

bool verify_cost_coord_pq_checked_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    const cost_coord_pq_create_info_t info = {
        static_cast<std::uint32_t>(sizeof(cost_coord_pq_create_info_t)),
        BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION,
        0
    };
    cost_coord_pq_t* const sentinel =
        reinterpret_cast<cost_coord_pq_t*>(1);
    cost_coord_pq_t* created = sentinel;

    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status =
        cost_coord_pq_create_ex(&info, &created);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || created != sentinel
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "cost_coord_pq_create_ex did not preserve output on allocation failure\n");
        return false;
    }

    cost_coord_pq_t* queue = nullptr;
    if (cost_coord_pq_create_ex(&info, &queue) != NAVSYS_STATUS_OK
        || !queue) {
        return false;
    }
    const coord_t coord = {11, 13};
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t push_status =
        cost_coord_pq_push_ex(queue, 2.0f, &coord);
    fail_after = -1;
    track_allocations = false;
    float output_cost = 17.0f;
    coord_t output_coord = {19, 23};
    if (push_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || cost_coord_pq_peek_min(
            queue, &output_cost, &output_coord) != NAVSYS_STATUS_NOT_FOUND
        || output_cost != 17.0f
        || output_coord.x != 19
        || output_coord.y != 23
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "cost_coord_pq_push_ex was not failure-atomic\n");
        cost_coord_pq_destroy(queue);
        return false;
    }

    if (cost_coord_pq_push_ex(queue, 2.0f, &coord)
        != NAVSYS_STATUS_OK) {
        cost_coord_pq_destroy(queue);
        return false;
    }
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t pop_status =
        cost_coord_pq_pop_min(queue, &output_cost, &output_coord);
    fail_after = -1;
    track_allocations = false;
    if (pop_status != NAVSYS_STATUS_OK
        || output_cost != 2.0f
        || output_coord.x != coord.x
        || output_coord.y != coord.y
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "cost_coord_pq_pop_min unexpectedly allocated output storage\n");
        cost_coord_pq_destroy(queue);
        return false;
    }

    cost_coord_pq_push(queue, 3.0f, &coord);
    track_allocations = true;
    coord_t* legacy_output = cost_coord_pq_pop(queue);
    track_allocations = false;
    if (!legacy_output
        || legacy_output->x != coord.x
        || legacy_output->y != coord.y
        || tracked_live_allocations != baseline + 1) {
        std::fprintf(
            stderr,
            "cost_coord_pq_pop did not return one caller-owned allocation\n");
        coord_destroy(legacy_output);
        cost_coord_pq_destroy(queue);
        return false;
    }
    coord_destroy(legacy_output);
    if (tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "coord_destroy did not release legacy cost_coord_pq_pop output\n");
        cost_coord_pq_destroy(queue);
        return false;
    }

    cost_coord_pq_destroy(queue);
    return true;
}
#endif

template <typename T>
bool verify_failure_atomic_create(
    const char* family,
    T* (*create)(),
    void (*destroy)(T*),
    bool inject_leak = false) {
    constexpr std::ptrdiff_t max_allocations = 256;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        const std::size_t baseline = tracked_live_allocations;
        track_allocations = true;
        fail_after = index;

        T* value = nullptr;
        bool exception_escaped = false;
        try {
            value = create();
        } catch (...) {
            exception_escaped = true;
        }
        fail_after = -1;
        if (value) destroy(value);

        void* injected_leak = nullptr;
        if (inject_leak && value) {
            injected_leak = allocate_for_test(1);
        }
        track_allocations = false;

        if (exception_escaped) {
            std::fprintf(
                stderr, "%s create leaked a C++ exception at allocation %td\n",
                family, index);
            return false;
        }
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "%s create leaked allocations at allocation %td: %zu -> %zu\n",
                family,
                index,
                baseline,
                tracked_live_allocations);
            deallocate_for_test(injected_leak);
            return false;
        }
        if (value) return true;
    }

    std::fprintf(
        stderr, "%s create exceeded the allocation fixture limit\n", family);
    return false;
}

bool verify_navgrid_copy_allocation_failure() {
    navgrid_t* source = navgrid_create();
    if (!source || !navgrid_block_coord(source, 7, 9)) {
        navgrid_destroy(source);
        return false;
    }

    constexpr std::ptrdiff_t max_allocations = 256;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        const std::size_t baseline = tracked_live_allocations;
        track_allocations = true;
        fail_after = index;

        navgrid_t* copy = nullptr;
        bool exception_escaped = false;
        try {
            copy = navgrid_copy(source);
        } catch (...) {
            exception_escaped = true;
        }
        fail_after = -1;
        if (copy) navgrid_destroy(copy);
        track_allocations = false;

        if (exception_escaped) {
            std::fprintf(
                stderr,
                "navgrid copy leaked a C++ exception at allocation %td\n",
                index);
            navgrid_destroy(source);
            return false;
        }
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "navgrid copy leaked allocations at allocation %td: %zu -> %zu\n",
                index,
                baseline,
                tracked_live_allocations);
            navgrid_destroy(source);
            return false;
        }
        if (copy) {
            navgrid_destroy(source);
            return true;
        }
    }

    std::fprintf(stderr, "navgrid copy exceeded the allocation fixture limit\n");
    navgrid_destroy(source);
    return false;
}

bool verify_navgrid_checked_mutation_allocation_failure() {
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    if (!grid) return false;

    const navcell_t cell{TERRAIN_TYPE_FOREST, 37};
    navcell_t prior{TERRAIN_TYPE_MOUNTAIN, 91};
    bool had_prior = true;
    bool changed = true;
    const std::size_t baseline = tracked_live_allocations;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t set_status = navgrid_set_cell_ex(
        grid, 2, 3, &cell, &prior, &had_prior, &changed);
    fail_after = -1;
    track_allocations = false;
    if (set_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || prior.terrain != TERRAIN_TYPE_MOUNTAIN
        || prior.height != 91
        || !had_prior
        || !changed
        || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "navgrid_set_cell_ex was not failure atomic\n");
        navgrid_destroy(grid);
        return false;
    }

    navcell_t fetched{};
    bool present = true;
    if (navgrid_fetch_cell_ex(grid, 2, 3, &fetched, &present)
            != NAVSYS_STATUS_OK
        || present) {
        std::fprintf(stderr, "navgrid_set_cell_ex mutated the grid on OOM\n");
        navgrid_destroy(grid);
        return false;
    }

    const coord_t coords[] = {{2, 3}, {4, 5}};
    navgrid_overlay_id_t overlay = 77;
    std::size_t changed_count = 88;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t overlay_status = navgrid_apply_blocked_overlay(
        grid, coords, 2, &overlay, &changed_count);
    fail_after = -1;
    track_allocations = false;
    if (overlay_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || overlay != 77
        || changed_count != 88
        || tracked_live_allocations != baseline
        || is_coord_blocked_navgrid(grid, 2, 3, nullptr)) {
        std::fprintf(stderr, "navgrid overlay apply was not failure atomic\n");
        navgrid_destroy(grid);
        return false;
    }

    navgrid_destroy(grid);
    return true;
}

bool verify_navgrid_caller_buffer_queries_do_not_allocate() {
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    if (!grid) return false;
    const navcell_t forest{TERRAIN_TYPE_FOREST, 37};
    if (!navgrid_set_cell(grid, 2, 3, &forest)) {
        navgrid_destroy(grid);
        return false;
    }
    const coord_t overlay_coords[] = {{2, 3}, {4, 5}};
    navgrid_overlay_id_t overlay = 0;
    std::size_t changed_count = 0;
    if (navgrid_apply_blocked_overlay(
            grid, overlay_coords, 2, &overlay, &changed_count)
        != NAVSYS_STATUS_OK) {
        navgrid_destroy(grid);
        return false;
    }

    coord_t neighbors[4]{};
    navgrid_cell_entry_t entries[2]{};
    std::size_t count = 0;
    const std::size_t baseline = tracked_live_allocations;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t neighbor_status = navgrid_export_neighbors(
        grid, 3, 3, false, neighbors, 4, &count);
    const navsys_status_t cell_status = navgrid_export_cells(
        grid, entries, 2, &count);
    const bool no_allocation_attempt = fail_after == 0;
    fail_after = -1;
    track_allocations = false;

    const bool valid = neighbor_status == NAVSYS_STATUS_OK
        && cell_status == NAVSYS_STATUS_OK
        && no_allocation_attempt
        && tracked_live_allocations == baseline;
    if (!valid) {
        std::fprintf(stderr, "navgrid caller-buffer query allocated memory\n");
    }
    navgrid_destroy(grid);
    return valid;
}

bool verify_obstacle_checked_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;

    constexpr std::ptrdiff_t max_allocations = 64;
    bool create_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        obstacle_t* output = reinterpret_cast<obstacle_t*>(uintptr_t{1});
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = obstacle_create_checked(
            0, 0, 4, 4, &output);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OK) {
            const bool output_valid = output
                && output != reinterpret_cast<obstacle_t*>(uintptr_t{1})
                && obstacle_get_blocked_coords(output);
            obstacle_destroy(output);
            if (!output_valid || tracked_live_allocations != baseline) {
                std::fprintf(stderr, "obstacle checked create committed invalid output\n");
                return false;
            }
            create_succeeded = true;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || output != reinterpret_cast<obstacle_t*>(uintptr_t{1})
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "obstacle checked create was not failure-atomic at allocation %td\n",
                index);
            return false;
        }
    }
    if (!create_succeeded) {
        std::fprintf(stderr, "obstacle checked create exceeded fault fixture limit\n");
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    obstacle_t* legacy_created = obstacle_create_full(0, 0, 4, 4);
    fail_after = -1;
    track_allocations = false;
    if (legacy_created || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "legacy obstacle create did not forward checked failure\n");
        obstacle_destroy(legacy_created);
        return false;
    }

    obstacle_t* source = obstacle_create_full(0, 0, 4, 4);
    if (!source || !obstacle_block_coord(source, 1, 1)) {
        obstacle_destroy(source);
        return false;
    }

    bool copy_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        obstacle_t* output = reinterpret_cast<obstacle_t*>(uintptr_t{1});
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = obstacle_copy_checked(source, &output);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OK) {
            const bool output_valid = output
                && output != reinterpret_cast<obstacle_t*>(uintptr_t{1})
                && obstacle_equal(source, output)
                && obstacle_get_blocked_coords(output)
                    != obstacle_get_blocked_coords(source);
            obstacle_destroy(output);
            if (!output_valid || tracked_live_allocations != baseline) {
                std::fprintf(stderr, "obstacle checked copy committed invalid output\n");
                obstacle_destroy(source);
                return false;
            }
            copy_succeeded = true;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || output != reinterpret_cast<obstacle_t*>(uintptr_t{1})
            || !obstacle_is_coord_blocked(source, 1, 1)
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "obstacle checked copy was not failure-atomic at allocation %td\n",
                index);
            obstacle_destroy(source);
            return false;
        }
    }
    if (!copy_succeeded) {
        std::fprintf(stderr, "obstacle checked copy exceeded fault fixture limit\n");
        obstacle_destroy(source);
        return false;
    }

    bool changed = true;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t block_status = obstacle_set_blocked(
        source, 2, 2, true, &changed);
    fail_after = -1;
    track_allocations = false;
    if (block_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || !changed
        || obstacle_is_coord_blocked(source, 2, 2)
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "obstacle checked mutation did not preserve failure outputs\n");
        obstacle_destroy(source);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    const bool legacy_blocked = obstacle_block_coord(source, 3, 3);
    fail_after = -1;
    track_allocations = false;
    if (legacy_blocked
        || obstacle_is_coord_blocked(source, 3, 3)
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "legacy obstacle mutation did not forward checked failure\n");
        obstacle_destroy(source);
        return false;
    }

    changed = false;
    if (obstacle_set_blocked(source, 2, 2, true, &changed)
            != NAVSYS_STATUS_OK
        || !changed) {
        obstacle_destroy(source);
        return false;
    }
    track_allocations = true;
    fail_after = 0;
    changed = true;
    const navsys_status_t unchanged_status = obstacle_set_blocked(
        source, 2, 2, true, &changed);
    fail_after = -1;
    track_allocations = false;
    if (unchanged_status != NAVSYS_STATUS_OK
        || changed
        || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "unchanged obstacle mutation allocated memory\n");
        obstacle_destroy(source);
        return false;
    }

    size_t required = 999;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t query_status = obstacle_export_blocked(
        source, nullptr, 0, &required);
    fail_after = -1;
    track_allocations = false;
    if (query_status != NAVSYS_STATUS_OK
        || required != 2
        || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "obstacle count query allocated or returned wrong count\n");
        obstacle_destroy(source);
        return false;
    }

    coord_t exported[2] = {};
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t export_status = obstacle_export_blocked(
        source, exported, 2, &required);
    fail_after = -1;
    track_allocations = false;
    if (export_status != NAVSYS_STATUS_OK
        || required != 2
        || tracked_live_allocations != baseline) {
        std::fprintf(stderr, "obstacle exact export allocated memory\n");
        obstacle_destroy(source);
        return false;
    }

    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_8, nullptr);
    if (!grid) {
        obstacle_destroy(source);
        return false;
    }
    track_allocations = true;
    fail_after = 0;
    obstacle_apply_to_navgrid(source, grid);
    fail_after = -1;
    track_allocations = false;
    const bool valid = !is_coord_blocked_navgrid(grid, 1, 1, nullptr)
        && tracked_live_allocations == baseline;
    if (!valid) {
        std::fprintf(
            stderr,
            "obstacle apply allocation-failure baseline changed\n");
    }
    navgrid_destroy(grid);
    obstacle_destroy(source);
    return valid;
}

bool verify_obstacle_extent_mutation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    obstacle_t* obstacle = obstacle_create_full(0, 0, 4, 4);
    if (!obstacle || !obstacle_block_coord(obstacle, 1, 1)) {
        obstacle_destroy(obstacle);
        return false;
    }
    obstacle_t* snapshot = obstacle_copy(obstacle);
    if (!snapshot) {
        obstacle_destroy(obstacle);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    obstacle_set_origin(obstacle, 10, 20);
    fail_after = -1;
    track_allocations = false;
    if (!obstacle_equal(obstacle, snapshot)
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "obstacle origin translation did not preserve state on allocation failure\n");
        obstacle_destroy(snapshot);
        obstacle_destroy(obstacle);
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    obstacle_set_width(obstacle, 1);
    fail_after = -1;
    track_allocations = false;
    const bool valid = obstacle_equal(obstacle, snapshot)
        && tracked_live_allocations == baseline;
    if (!valid) {
        std::fprintf(
            stderr,
            "obstacle resize did not preserve state on allocation failure\n");
    }

    obstacle_destroy(snapshot);
    obstacle_destroy(obstacle);
    return valid;
}

bool verify_obstacle_geometry_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    constexpr std::ptrdiff_t max_allocations = 128;

    auto verify_operation = [&](bool line) {
        obstacle_t* obstacle = obstacle_create_full(0, 0, 7, 7);
        obstacle_t* snapshot = obstacle_copy(obstacle);
        if (!obstacle || !snapshot) {
            obstacle_destroy(snapshot);
            obstacle_destroy(obstacle);
            return false;
        }

        bool succeeded = false;
        for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
            std::size_t changed = 999;
            track_allocations = true;
            fail_after = index;
            const navsys_status_t status = line
                ? obstacle_block_line(
                    obstacle, 1, 1, 5, 3, 1, &changed)
                : obstacle_block_square(
                    obstacle, 3, 3, 1, &changed);
            fail_after = -1;
            track_allocations = false;

            if (status == NAVSYS_STATUS_OK) {
                const bool result_valid = changed > 0
                    && !obstacle_equal(obstacle, snapshot);
                succeeded = result_valid;
                break;
            }
            if (status != NAVSYS_STATUS_OUT_OF_MEMORY
                || changed != 999
                || !obstacle_equal(obstacle, snapshot)
                || tracked_live_allocations != baseline) {
                std::fprintf(
                    stderr,
                    "obstacle checked geometry was not failure-atomic at allocation %td\n",
                    index);
                obstacle_destroy(snapshot);
                obstacle_destroy(obstacle);
                return false;
            }
        }

        obstacle_destroy(snapshot);
        obstacle_destroy(obstacle);
        if (!succeeded || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "obstacle checked geometry exceeded fault fixture limit\n");
            return false;
        }
        return true;
    };

    return verify_operation(false) && verify_operation(true);
}

bool verify_obstacle_generator_initial_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    const coord_t a = {0, 0};
    const coord_t b = {2, 0};
    const coord_t c = {0, 2};
    const coord_t d = {2, 2};
    coord_list_t* polygon = coord_list_create();
    if (!polygon
        || !coord_list_push_back(polygon, &a)
        || !coord_list_push_back(polygon, &b)
        || !coord_list_push_back(polygon, &c)) {
        coord_list_destroy(polygon);
        return false;
    }

    const auto verify = [&](const char* name, const auto& generator) {
        track_allocations = true;
        fail_after = 0;
        obstacle_t* obstacle = generator();
        fail_after = -1;
        track_allocations = false;
        const bool valid = obstacle == nullptr
            && tracked_live_allocations == baseline;
        if (!valid) {
            std::fprintf(
                stderr,
                "%s did not preserve the initial allocation baseline\n",
                name);
        }
        obstacle_destroy(obstacle);
        return valid;
    };

    const bool valid =
        verify("obstacle_make_rect_all_blocked", [&] {
            return obstacle_make_rect_all_blocked(0, 0, 3, 3);
        })
        && verify("obstacle_make_rect_random_blocked", [&] {
            return obstacle_make_rect_random_blocked(0, 0, 3, 3, 1.0f);
        })
        && verify("obstacle_make_beam", [&] {
            return obstacle_make_beam(&a, &c, 0);
        })
        && verify("obstacle_make_torus", [&] {
            return obstacle_make_torus(&a, &d, 1);
        })
        && verify("obstacle_make_enclosure", [&] {
            return obstacle_make_enclosure(
                &a, &c, 1, ENCLOSURE_OPEN_UNKNOWN);
        })
        && verify("obstacle_make_cross", [&] {
            return obstacle_make_cross(&a, 1, 0);
        })
        && verify("obstacle_make_spiral", [&] {
            return obstacle_make_spiral(
                &a, 1, 1, 0, 0, SPIRAL_CLOCKWISE);
        })
        && verify("obstacle_make_triangle", [&] {
            return obstacle_make_triangle(&a, &b, &c);
        })
        && verify("obstacle_make_triangle_torus", [&] {
            return obstacle_make_triangle_torus(&a, &b, &c, 0);
        })
        && verify("obstacle_make_polygon", [&] {
            return obstacle_make_polygon(polygon);
        })
        && verify("obstacle_make_polygon_torus", [&] {
            return obstacle_make_polygon_torus(polygon, 0);
        });

    coord_list_destroy(polygon);
    return valid;
}

bool verify_obstacle_checked_generator_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    constexpr std::ptrdiff_t max_allocations = 256;
    const coord_t a = {0, 0};
    const coord_t b = {3, 1};
    const coord_t polygon[] = {{0, 0}, {3, 0}, {0, 3}};

    const auto verify = [&](const char* name, const auto& generator) {
        bool succeeded = false;
        for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
            obstacle_t* output = reinterpret_cast<obstacle_t*>(uintptr_t{1});
            track_allocations = true;
            fail_after = index;
            const navsys_status_t status = generator(&output);
            fail_after = -1;
            track_allocations = false;

            if (status == NAVSYS_STATUS_OK) {
                const bool valid = output != nullptr;
                obstacle_destroy(output);
                succeeded = valid && tracked_live_allocations == baseline;
                break;
            }
            if (status != NAVSYS_STATUS_OUT_OF_MEMORY
                || output != nullptr
                || tracked_live_allocations != baseline) {
                std::fprintf(
                    stderr,
                    "%s was not failure-atomic at allocation %td\n",
                    name, index);
                obstacle_destroy(output);
                return false;
            }
        }
        if (!succeeded) {
            std::fprintf(
                stderr,
                "%s exceeded the checked generator fault limit\n",
                name);
        }
        return succeeded;
    };

    obstacle_generate_options_t options{};
    if (obstacle_generate_options_init(&options) != NAVSYS_STATUS_OK)
        return false;
    options.seed = 17;
    obstacle_enclosure_desc_t enclosure{};
    obstacle_cross_desc_t cross{};
    obstacle_spiral_desc_t spiral{};
    if (obstacle_enclosure_desc_init(&enclosure) != NAVSYS_STATUS_OK
        || obstacle_cross_desc_init(&cross) != NAVSYS_STATUS_OK
        || obstacle_spiral_desc_init(&spiral) != NAVSYS_STATUS_OK) {
        return false;
    }
    enclosure.width = 5;
    enclosure.height = 5;
    enclosure.open_side = OBSTACLE_ENCLOSURE_OPEN_LEFT;
    enclosure.aperture_offset_cells = 1;
    enclosure.aperture_length_cells = 3;
    cross.arm_length_cells = 2;
    cross.radius_cells = 1;
    spiral.max_radius_cells = 2;
    spiral.pitch_cells = 2;
    spiral.path_radius_cells = 1;
    const bool center_valid = verify(
        "obstacle_generate_filled_rect", [&](obstacle_t** output) {
        return obstacle_generate_filled_rect(
            0, 0, 3, 3, &options, output);
    }) && verify("obstacle_generate_rect_outline", [&](obstacle_t** output) {
        return obstacle_generate_rect_outline(
            0, 0, 5, 5, 1, &options, output);
    }) && verify("obstacle_generate_random_rect", [&](obstacle_t** output) {
        return obstacle_generate_random_rect(
            0, 0, 3, 3, 0.5, &options, output);
    }) && verify("obstacle_generate_line", [&](obstacle_t** output) {
        return obstacle_generate_line(&a, &b, 1, &options, output);
    }) && verify("obstacle_generate_polygon", [&](obstacle_t** output) {
        return obstacle_generate_polygon(
            polygon, 3, OBSTACLE_POLYGON_EVEN_ODD, &options, output);
    }) && verify("obstacle_generate_polygon_outline", [&](obstacle_t** output) {
        return obstacle_generate_polygon_outline(
            polygon, 3, 1, &options, output);
    }) && verify("obstacle_generate_triangle", [&](obstacle_t** output) {
        return obstacle_generate_triangle(
            &polygon[0], &polygon[1], &polygon[2],
            OBSTACLE_POLYGON_EVEN_ODD, &options, output);
    }) && verify("obstacle_generate_triangle_outline", [&](obstacle_t** output) {
        return obstacle_generate_triangle_outline(
            &polygon[0], &polygon[1], &polygon[2], 1, &options, output);
    }) && verify("obstacle_generate_enclosure", [&](obstacle_t** output) {
        return obstacle_generate_enclosure(&enclosure, &options, output);
    }) && verify("obstacle_generate_cross", [&](obstacle_t** output) {
        return obstacle_generate_cross(&cross, &options, output);
    }) && verify("obstacle_generate_spiral", [&](obstacle_t** output) {
        return obstacle_generate_spiral(&spiral, &options, output);
    });
    if (!center_valid) return false;

    options.raster_rule = OBSTACLE_RASTER_ALL_TOUCHED;
    return verify("obstacle_generate_line/all_touched", [&](obstacle_t** output) {
        return obstacle_generate_line(&a, &b, 1, &options, output);
    }) && verify(
        "obstacle_generate_polygon/all_touched", [&](obstacle_t** output) {
        return obstacle_generate_polygon(
            polygon, 3, OBSTACLE_POLYGON_EVEN_ODD, &options, output);
    }) && verify(
        "obstacle_generate_polygon_outline/all_touched",
        [&](obstacle_t** output) {
        return obstacle_generate_polygon_outline(
            polygon, 3, 1, &options, output);
    });
}

bool verify_obstacle_overlay_allocation_failure() {
    const std::size_t baseline = tracked_live_allocations;
    constexpr std::ptrdiff_t max_allocations = 128;
    obstacle_t* obstacle = obstacle_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_8, nullptr);
    if (!obstacle || !grid
        || !obstacle_block_coord(obstacle, 1, 1)
        || !obstacle_block_coord(obstacle, 2, 2)
        || !obstacle_block_coord(obstacle, 3, 3)) {
        navgrid_destroy(grid);
        obstacle_destroy(obstacle);
        return false;
    }

    obstacle_navgrid_overlay_token_t token{};
    bool apply_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        token = obstacle_navgrid_overlay_token_t{31, 32, 33, 34};
        std::size_t changed = 77;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = obstacle_apply_to_navgrid_checked(
            obstacle, grid, nullptr, &token, &changed);
        fail_after = -1;
        track_allocations = false;
        if (status == NAVSYS_STATUS_OK) {
            apply_succeeded = changed == 3
                && token.owner_cookie != 0
                && token.overlay != 0;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || token.struct_size != 31
            || token.abi_version != 32
            || token.owner_cookie != 33
            || token.overlay != 34
            || changed != 77
            || is_coord_blocked_navgrid(grid, 1, 1, nullptr)
            || is_coord_blocked_navgrid(grid, 2, 2, nullptr)
            || is_coord_blocked_navgrid(grid, 3, 3, nullptr)
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "obstacle overlay apply was not failure-atomic at allocation %td\n",
                index);
            navgrid_destroy(grid);
            obstacle_destroy(obstacle);
            return false;
        }
    }
    if (!apply_succeeded) {
        std::fprintf(stderr, "obstacle overlay apply exceeded fault fixture limit\n");
        navgrid_destroy(grid);
        obstacle_destroy(obstacle);
        return false;
    }

    const std::size_t applied_live = tracked_live_allocations;
    bool remove_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        const obstacle_navgrid_overlay_token_t before = token;
        std::size_t changed = 88;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = obstacle_remove_from_navgrid_checked(
            grid, &token, &changed);
        fail_after = -1;
        track_allocations = false;
        if (status == NAVSYS_STATUS_OK) {
            remove_succeeded = changed == 3
                && token.owner_cookie == 0
                && token.overlay == 0;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || token.owner_cookie != before.owner_cookie
            || token.overlay != before.overlay
            || changed != 88
            || !is_coord_blocked_navgrid(grid, 1, 1, nullptr)
            || !is_coord_blocked_navgrid(grid, 2, 2, nullptr)
            || !is_coord_blocked_navgrid(grid, 3, 3, nullptr)
            || tracked_live_allocations != applied_live) {
            std::fprintf(
                stderr,
                "obstacle overlay remove was not failure-atomic at allocation %td\n",
                index);
            navgrid_destroy(grid);
            obstacle_destroy(obstacle);
            return false;
        }
    }

    const bool valid = remove_succeeded
        && !is_coord_blocked_navgrid(grid, 1, 1, nullptr)
        && tracked_live_allocations == baseline;
    if (!valid) {
        std::fprintf(stderr, "obstacle overlay remove exceeded fault fixture limit\n");
    }
    navgrid_destroy(grid);
    obstacle_destroy(obstacle);
    return valid;
}

bool verify_route_carver_mutation_allocation_failure() {
    constexpr std::ptrdiff_t max_allocations = 128;
    const coord_t candidate{1, 1};
    const navgrid_carve_options_t options{
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_ATOMIC,
        0,
        16,
        nullptr,
        nullptr
    };

    // Prime implementation/runtime caches before allocation tracking.  This
    // fixture can run in isolation, where a first library call may otherwise
    // attribute process-lifetime runtime allocations to the carve operation.
    navgrid_t* warm_grid = navgrid_create_full(
        4, 4, NAVGRID_DIR_4, nullptr);
    std::size_t warm_changed = 0;
    if (!warm_grid
        || !navgrid_block_coord(warm_grid, candidate.x, candidate.y)
        || navgrid_carve_area(
            warm_grid, &candidate, &options, &warm_changed)
            != NAVSYS_STATUS_OK
        || warm_changed != 1) {
        navgrid_destroy(warm_grid);
        return false;
    }
    navgrid_destroy(warm_grid);

    navgrid_t* grid = navgrid_create_full(4, 4, NAVGRID_DIR_4, nullptr);
    if (!grid || !navgrid_block_coord(grid, candidate.x, candidate.y)) {
        navgrid_destroy(grid);
        return false;
    }

    const std::size_t baseline = tracked_live_allocations;
    bool succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        std::size_t changed = 77;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = navgrid_carve_area(
            grid, &candidate, &options, &changed);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OK) {
            succeeded = changed == 1
                && !is_coord_blocked_navgrid(
                    grid, candidate.x, candidate.y, nullptr);
            break;
        }
        const bool still_blocked = is_coord_blocked_navgrid(
            grid, candidate.x, candidate.y, nullptr);
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || changed != 77
            || !still_blocked
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "route carver mutation was not failure-atomic at allocation "
                "%td (status=%d, changed=%zu, blocked=%d, live=%zu, "
                "baseline=%zu)\n",
                index,
                static_cast<int>(status),
                changed,
                still_blocked ? 1 : 0,
                tracked_live_allocations,
                baseline);
            navgrid_destroy(grid);
            return false;
        }
    }

    if (!succeeded) {
        std::fprintf(
            stderr,
            "route carver mutation exceeded fault fixture limit\n");
    }
    navgrid_destroy(grid);
    if (tracked_live_allocations != baseline) {
        std::fprintf(stderr, "route carver mutation leaked allocations\n");
        return false;
    }
    return succeeded;
}

bool verify_maze_lifecycle_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;

    track_allocations = true;
    fail_after = 0;
    maze_t* failed_create = maze_create_full(1, 2, 7, 9);
    fail_after = -1;
    track_allocations = false;
    if (failed_create != nullptr || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "maze create allocation failure was not atomic "
            "(maze=%p, live=%zu, baseline=%zu)\n",
            static_cast<void*>(failed_create),
            tracked_live_allocations,
            baseline);
        maze_destroy(failed_create);
        return false;
    }

    const byul_maze_extent_t extent{1, 2, 7, 9};
    maze_t* checked_create = reinterpret_cast<maze_t*>(uintptr_t{1});
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t create_status =
        byul_maze_create(&extent, &checked_create);
    fail_after = -1;
    track_allocations = false;
    if (create_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || checked_create != reinterpret_cast<maze_t*>(uintptr_t{1})
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "maze checked create changed output on allocation failure "
            "(status=%d, output=%p, live=%zu, baseline=%zu)\n",
            static_cast<int>(create_status),
            static_cast<void*>(checked_create),
            tracked_live_allocations,
            baseline);
        return false;
    }

    maze_t* source = maze_create_full(-3, 5, 11, 13);
    if (!source) {
        maze_destroy(source);
        std::fprintf(stderr, "maze legacy copy baseline source failed\n");
        return false;
    }

    maze_t* checked_copy = reinterpret_cast<maze_t*>(uintptr_t{1});
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t copy_status = byul_maze_copy(source, &checked_copy);
    fail_after = -1;
    track_allocations = false;
    if (copy_status != NAVSYS_STATUS_OUT_OF_MEMORY
        || checked_copy != reinterpret_cast<maze_t*>(uintptr_t{1})
        || tracked_live_allocations != baseline) {
        std::fprintf(
            stderr,
            "maze checked copy changed output on allocation failure "
            "(status=%d, output=%p, live=%zu, baseline=%zu)\n",
            static_cast<int>(copy_status),
            static_cast<void*>(checked_copy),
            tracked_live_allocations,
            baseline);
        maze_destroy(source);
        return false;
    }

    bool reproduced_copy_failure = false;
    for (std::ptrdiff_t index = 0; index < 64; ++index) {
        track_allocations = true;
        fail_after = index;
        maze_t* failed_copy = maze_copy(source);
        fail_after = -1;
        track_allocations = false;

        if (!failed_copy) {
            reproduced_copy_failure = true;
        }
        maze_destroy(failed_copy);
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "maze legacy copy baseline leaked at allocation %td "
                "(live=%zu, baseline=%zu)\n",
                index,
                tracked_live_allocations,
                baseline);
            maze_destroy(source);
            return false;
        }
        if (reproduced_copy_failure) break;
    }

    maze_destroy(source);
    if (!reproduced_copy_failure) {
        std::fprintf(
            stderr,
            "maze copy allocation failure was not reproduced\n");
        return false;
    }
    return tracked_live_allocations == baseline;
}

bool verify_maze_binary_generation_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(17),
        UINT64_C(10000),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    bool reproduced_failure = false;
    bool completed = false;

    for (std::ptrdiff_t index = 0; index < 128; ++index) {
        maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = byul_maze_generate_binary_tree(
            -4,
            6,
            9,
            9,
            BYUL_MAZE_BINARY_BIAS_NORTH_WEST,
            &options,
            &output);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OUT_OF_MEMORY) {
            reproduced_failure = true;
            if (output != nullptr) {
                std::fprintf(
                    stderr,
                    "maze binary published output at failed allocation %td\n",
                    index);
                maze_destroy(output);
                return false;
            }
        } else if (status == NAVSYS_STATUS_OK) {
            if (!output) {
                std::fprintf(stderr, "maze binary succeeded without output\n");
                return false;
            }
            maze_destroy(output);
            completed = true;
        } else {
            std::fprintf(
                stderr,
                "maze binary returned unexpected allocation status %d at %td\n",
                static_cast<int>(status),
                index);
            maze_destroy(output);
            return false;
        }
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "maze binary leaked at failed allocation %td "
                "(live=%zu, baseline=%zu)\n",
                index,
                tracked_live_allocations,
                baseline);
            return false;
        }
        if (completed) break;
    }
    return reproduced_failure && completed
        && tracked_live_allocations == baseline;
}

bool verify_maze_eller_generation_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(17),
        UINT64_C(10000),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    bool reproduced_failure = false;
    bool completed = false;

    for (std::ptrdiff_t index = 0; index < 128; ++index) {
        maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = byul_maze_generate_eller(
            -4, 6, 9, 9, &options, &output);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OUT_OF_MEMORY) {
            reproduced_failure = true;
            if (output != nullptr) {
                std::fprintf(
                    stderr,
                    "maze Eller published output at failed allocation %td\n",
                    index);
                maze_destroy(output);
                return false;
            }
        } else if (status == NAVSYS_STATUS_OK) {
            if (!output) {
                std::fprintf(stderr, "maze Eller succeeded without output\n");
                return false;
            }
            maze_destroy(output);
            completed = true;
        } else {
            std::fprintf(
                stderr,
                "maze Eller returned unexpected allocation status %d at %td\n",
                static_cast<int>(status),
                index);
            maze_destroy(output);
            return false;
        }
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "maze Eller leaked at failed allocation %td "
                "(live=%zu, baseline=%zu)\n",
                index,
                tracked_live_allocations,
                baseline);
            return false;
        }
        if (reproduced_failure && completed) break;
    }
    if (!reproduced_failure || !completed) {
        std::fprintf(
            stderr,
            "maze Eller allocation sweep did not cover failure and success\n");
        return false;
    }
    return tracked_live_allocations == baseline;
}

bool verify_maze_hunt_and_kill_generation_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(17),
        UINT64_C(10000),
        UINT64_C(81),
        nullptr,
        nullptr
    };
    bool reproduced_failure = false;
    bool completed = false;

    for (std::ptrdiff_t index = 0; index < 128; ++index) {
        maze_t* output = reinterpret_cast<maze_t*>(uintptr_t{1});
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status = byul_maze_generate_hunt_and_kill(
            -4, 6, 9, 9, &options, &output);
        fail_after = -1;
        track_allocations = false;

        if (status == NAVSYS_STATUS_OUT_OF_MEMORY) {
            reproduced_failure = true;
            if (output != nullptr) {
                std::fprintf(
                    stderr,
                    "maze Hunt-and-Kill published output at failed allocation %td\n",
                    index);
                maze_destroy(output);
                return false;
            }
        } else if (status == NAVSYS_STATUS_OK) {
            if (!output) {
                std::fprintf(
                    stderr, "maze Hunt-and-Kill succeeded without output\n");
                return false;
            }
            maze_destroy(output);
            completed = true;
        } else {
            std::fprintf(
                stderr,
                "maze Hunt-and-Kill returned unexpected allocation status %d at %td\n",
                static_cast<int>(status),
                index);
            maze_destroy(output);
            return false;
        }
        if (tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "maze Hunt-and-Kill leaked at failed allocation %td "
                "(live=%zu, baseline=%zu)\n",
                index,
                tracked_live_allocations,
                baseline);
            return false;
        }
        if (reproduced_failure && completed) break;
    }
    if (!reproduced_failure || !completed) {
        std::fprintf(
            stderr,
            "maze Hunt-and-Kill allocation sweep did not cover failure and success\n");
        return false;
    }
    return tracked_live_allocations == baseline;
}

bool verify_maze_translate_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    maze_t* maze = maze_create_full(-5, 7, 9, 11);
    if (!maze) {
        maze_destroy(maze);
        std::fprintf(stderr, "maze translate source creation failed\n");
        return false;
    }
    const coord_t blocked{-3, 9};
    bool maze_changed = false;
    if (byul_maze_set_blocked(
            maze, blocked.x, blocked.y, true, &maze_changed)
        != NAVSYS_STATUS_OK) {
        maze_destroy(maze);
        std::fprintf(stderr, "maze translate source population failed\n");
        return false;
    }
    maze_t* snapshot = maze_copy(maze);
    if (!snapshot) {
        maze_destroy(maze);
        std::fprintf(stderr, "maze translate snapshot creation failed\n");
        return false;
    }

    track_allocations = true;
    fail_after = 0;
    const navsys_status_t status = byul_maze_translate(maze, 13, -17);
    fail_after = -1;
    track_allocations = false;

    const bool preserved = status == NAVSYS_STATUS_OUT_OF_MEMORY
        && maze_equal(maze, snapshot)
        && maze_hash(maze) == maze_hash(snapshot)
        && tracked_live_allocations == baseline;
    if (!preserved) {
        std::fprintf(
            stderr,
            "maze translate allocation failure was not atomic "
            "(status=%d, live=%zu, baseline=%zu)\n",
            static_cast<int>(status),
            tracked_live_allocations,
            baseline);
    }
    maze_destroy(snapshot);
    maze_destroy(maze);
    return preserved && tracked_live_allocations == baseline;
}

bool verify_maze_blocked_mutation_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    maze_t* maze = maze_create_full(0, 0, 8, 8);
    maze_t* snapshot = maze_copy(maze);
    if (!maze || !snapshot) {
        maze_destroy(snapshot);
        maze_destroy(maze);
        return false;
    }

    bool changed = true;
    track_allocations = true;
    fail_after = 0;
    const navsys_status_t status =
        byul_maze_set_blocked(maze, 2, 3, true, &changed);
    fail_after = -1;
    track_allocations = false;

    const bool preserved = status == NAVSYS_STATUS_OUT_OF_MEMORY
        && changed
        && maze_equal(maze, snapshot)
        && maze_hash(maze) == maze_hash(snapshot);
    if (!preserved) {
        std::fprintf(
            stderr,
            "maze blocked mutation allocation failure was not atomic "
            "(status=%d, changed=%d)\n",
            static_cast<int>(status),
            changed ? 1 : 0);
    }
    maze_destroy(snapshot);
    maze_destroy(maze);
    return preserved && tracked_live_allocations == baseline;
}

bool verify_maze_overlay_allocation_failure_atomic() {
    const std::size_t baseline = tracked_live_allocations;
    constexpr std::ptrdiff_t max_allocations = 128;
    maze_t* maze = maze_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_8, nullptr);
    const coord_t a{1, 1};
    const coord_t b{2, 2};
    const coord_t c{3, 3};
    bool maze_changed = false;
    if (!maze || !grid
        || byul_maze_set_blocked(maze, a.x, a.y, true, &maze_changed)
            != NAVSYS_STATUS_OK
        || byul_maze_set_blocked(maze, b.x, b.y, true, &maze_changed)
            != NAVSYS_STATUS_OK
        || byul_maze_set_blocked(maze, c.x, c.y, true, &maze_changed)
            != NAVSYS_STATUS_OK) {
        navgrid_destroy(grid);
        maze_destroy(maze);
        return false;
    }

    byul_maze_navgrid_overlay_token_t token{};
    bool apply_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        token = byul_maze_navgrid_overlay_token_t{31, 32, 33, 34};
        std::size_t changed = 77;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status =
            byul_maze_apply(maze, grid, nullptr, &token, &changed);
        fail_after = -1;
        track_allocations = false;
        if (status == NAVSYS_STATUS_OK) {
            apply_succeeded = changed == 3
                && token.owner_cookie != 0
                && token.overlay != 0;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || token.struct_size != 31
            || token.abi_version != 32
            || token.owner_cookie != 33
            || token.overlay != 34
            || changed != 77
            || is_coord_blocked_navgrid(grid, 1, 1, nullptr)
            || is_coord_blocked_navgrid(grid, 2, 2, nullptr)
            || is_coord_blocked_navgrid(grid, 3, 3, nullptr)
            || tracked_live_allocations != baseline) {
            std::fprintf(
                stderr,
                "maze overlay apply was not failure-atomic at allocation %td\n",
                index);
            navgrid_destroy(grid);
            maze_destroy(maze);
            return false;
        }
    }
    if (!apply_succeeded) {
        std::fprintf(stderr, "maze overlay apply exceeded fault fixture limit\n");
        navgrid_destroy(grid);
        maze_destroy(maze);
        return false;
    }

    const std::size_t applied_live = tracked_live_allocations;
    bool remove_succeeded = false;
    for (std::ptrdiff_t index = 0; index < max_allocations; ++index) {
        const byul_maze_navgrid_overlay_token_t before = token;
        std::size_t changed = 88;
        track_allocations = true;
        fail_after = index;
        const navsys_status_t status =
            byul_maze_remove_overlay(grid, &token, &changed);
        fail_after = -1;
        track_allocations = false;
        if (status == NAVSYS_STATUS_OK) {
            remove_succeeded = changed == 3
                && token.owner_cookie == 0
                && token.overlay == 0;
            break;
        }
        if (status != NAVSYS_STATUS_OUT_OF_MEMORY
            || token.owner_cookie != before.owner_cookie
            || token.overlay != before.overlay
            || changed != 88
            || !is_coord_blocked_navgrid(grid, 1, 1, nullptr)
            || !is_coord_blocked_navgrid(grid, 2, 2, nullptr)
            || !is_coord_blocked_navgrid(grid, 3, 3, nullptr)
            || tracked_live_allocations != applied_live) {
            std::fprintf(
                stderr,
                "maze overlay remove was not failure-atomic at allocation %td\n",
                index);
            navgrid_destroy(grid);
            maze_destroy(maze);
            return false;
        }
    }

    const bool valid = remove_succeeded
        && !is_coord_blocked_navgrid(grid, 1, 1, nullptr)
        && tracked_live_allocations == baseline;
    if (!valid) {
        std::fprintf(stderr, "maze overlay remove exceeded fault fixture limit\n");
    }
    navgrid_destroy(grid);
    maze_destroy(maze);
    return valid;
}

} // namespace

void* operator new(std::size_t size) {
    return allocate_for_test(size);
}

void* operator new[](std::size_t size) {
    return allocate_for_test(size);
}

void operator delete(void* pointer) noexcept {
    deallocate_for_test(pointer);
}

void operator delete[](void* pointer) noexcept {
    deallocate_for_test(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept {
    deallocate_for_test(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept {
    deallocate_for_test(pointer);
}

int main(int argc, char** argv) {
    bool inject_route_finder_leak = false;
    if (argc == 2
        && std::strcmp(argv[1], "--route-carver-only") == 0) {
        return verify_route_carver_mutation_allocation_failure() ? 0 : 26;
    }
    if (argc == 2
        && std::strcmp(argv[1], "--obstacle-generator-only") == 0) {
        return verify_obstacle_generator_initial_allocation_failure()
            && verify_obstacle_checked_generator_allocation_failure()
            ? 0 : 25;
    }
    if (argc == 2
        && std::strcmp(argv[1], "--inject-route-finder-leak") == 0) {
        inject_route_finder_leak = true;
    } else if (argc != 1) {
        std::fprintf(stderr, "unknown allocation fixture argument\n");
        return 64;
    }

    if (!verify_coord_checked_allocation_failure()) {
        return 1;
    }
    if (!verify_navcell_checked_allocation_failure()) {
        return 15;
    }
    if (!verify_dstar_lite_key_allocation_failure()) {
        return 9;
    }
    if (!verify_route_checked_allocation_failure()) {
        return 10;
    }
#if !defined(_MSC_VER)
    // MSVC's STL uses iterator-proxy allocation that cannot be safely
    // failure-injected through the executable's global operator new.
    if (!verify_coord_list_checked_allocation_failure()) {
        return 7;
    }
    if (!verify_cost_coord_pq_checked_allocation_failure()) {
        return 8;
    }
#endif

    if (!verify_failure_atomic_create(
            "coord", create_checked_coord, coord_destroy)) {
        return 2;
    }

    if (!verify_failure_atomic_create(
            "navcell", create_checked_navcell, navcell_destroy)) {
        return 16;
    }

    if (!verify_failure_atomic_create(
            "route", create_route, destroy_route)) {
        return 11;
    }

    if (!verify_failure_atomic_create(
            "route_builder", create_route_builder, destroy_route_builder)) {
        return 12;
    }

    if (!verify_failure_atomic_create(
            "route_heading_tracker",
            create_route_heading_tracker,
            destroy_route_heading_tracker)) {
        return 14;
    }

    if (!verify_failure_atomic_create(
            "navsys_search_trace",
            create_navsys_search_trace,
            destroy_navsys_search_trace)) {
        return 13;
    }

    if (!verify_failure_atomic_create(
            "navgrid", create_navgrid, destroy_navgrid)) {
        return 3;
    }
    if (!verify_navgrid_copy_allocation_failure()) {
        return 17;
    }
    if (!verify_navgrid_checked_mutation_allocation_failure()) {
        return 18;
    }
    if (!verify_navgrid_caller_buffer_queries_do_not_allocate()) {
        return 19;
    }
    if (!verify_obstacle_checked_allocation_failure()) {
        return 20;
    }
    if (!verify_obstacle_extent_mutation_failure_atomic()) {
        return 21;
    }
    if (!verify_obstacle_geometry_allocation_failure()) {
        return 22;
    }
    if (!verify_obstacle_generator_initial_allocation_failure()) {
        return 23;
    }
    if (!verify_obstacle_checked_generator_allocation_failure()) {
        return 25;
    }
    if (!verify_obstacle_overlay_allocation_failure()) {
        return 24;
    }
    if (!verify_route_carver_mutation_allocation_failure()) {
        return 26;
    }
    if (!verify_maze_lifecycle_allocation_failure_atomic()) {
        return 27;
    }
    if (!verify_maze_binary_generation_allocation_failure_atomic()) {
        return 31;
    }
    if (!verify_maze_eller_generation_allocation_failure_atomic()) {
        return 32;
    }
    if (!verify_maze_hunt_and_kill_generation_allocation_failure_atomic()) {
        return 33;
    }
    if (!verify_maze_translate_allocation_failure_atomic()) {
        return 28;
    }
    if (!verify_maze_blocked_mutation_allocation_failure_atomic()) {
        return 30;
    }
    if (!verify_maze_overlay_allocation_failure_atomic()) {
        return 29;
    }

    dependency_navgrid = navgrid_create();
    if (!dependency_navgrid) return 4;

    if (!verify_failure_atomic_create(
            "route_finder",
            create_route_finder,
            destroy_route_finder,
            inject_route_finder_leak)) {
        navgrid_destroy(dependency_navgrid);
        return 5;
    }
    if (!verify_failure_atomic_create(
            "dstar_lite", create_dstar_lite, destroy_dstar_lite)) {
        navgrid_destroy(dependency_navgrid);
        return 6;
    }

    navgrid_destroy(dependency_navgrid);
    dependency_navgrid = nullptr;
    return 0;
}
