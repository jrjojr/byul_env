/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "navsys.h"

#include <cstddef>
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

    if (!verify_failure_atomic_create(
            "coord", create_checked_coord, coord_destroy)) {
        return 2;
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
