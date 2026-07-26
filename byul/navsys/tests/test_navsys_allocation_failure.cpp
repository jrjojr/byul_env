/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "navsys.h"

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

coord_t* create_checked_coord() {
    coord_t* coord = nullptr;
    return coord_create_checked(7, 9, &coord) == NAVSYS_STATUS_OK
        ? coord
        : nullptr;
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

    coord_hash_t* predecessors = coord_hash_create_full(
        reinterpret_cast<coord_hash_copy_func>(coord_copy),
        reinterpret_cast<coord_hash_destroy_func>(coord_destroy));
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
        && std::strcmp(argv[1], "--inject-route-finder-leak") == 0) {
        inject_route_finder_leak = true;
    } else if (argc != 1) {
        std::fprintf(stderr, "unknown allocation fixture argument\n");
        return 64;
    }

    if (!verify_coord_checked_allocation_failure()) {
        return 1;
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
            "route", create_route, destroy_route)) {
        return 11;
    }

    if (!verify_failure_atomic_create(
            "navgrid", create_navgrid, destroy_navgrid)) {
        return 3;
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
