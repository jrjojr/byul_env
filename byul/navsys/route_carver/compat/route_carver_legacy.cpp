/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "route_carver_legacy.hpp"

#include "../../navgrid/internal/navgrid_overlay.hpp"

#include <cstddef>

namespace byul::navsys::route_carver::compat {
namespace {

bool clear_if_blocked(navgrid_t* navgrid, int x, int y, int& removed) {
    if (!is_coord_blocked_navgrid(navgrid, x, y, nullptr)) return true;
    bool changed = false;
    const navsys_status_t status =
        byul::navsys::internal::navgrid_clear_blocked_at_coord(
            navgrid, x, y, &changed);
    if (status != NAVSYS_STATUS_OK) return false;
    if (changed) ++removed;
    return true;
}

} // namespace

int carve_beam_abi1(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    int range) noexcept {
    if (!navgrid || !start || !goal) return 0;

    try {
        int removed = 0;
        coord_t current = *start;

        while (!coord_equal(&current, goal)) {
            coord_t next{};
            if (coord_step_toward(&current, goal, &next) != NAVSYS_STATUS_OK) {
                return 0;
            }

            if (range <= 0) {
                if (!clear_if_blocked(navgrid, next.x, next.y, removed)) return 0;
            } else {
                coord_list_t* neighbors = navgrid_copy_neighbors_all_range(
                    navgrid, next.x, next.y, range - 1);
                if (!neighbors) return 0;
                const std::size_t count = coord_list_size(neighbors);
                for (std::size_t index = 0; index < count; ++index) {
                    coord_t candidate{};
                    if (coord_list_fetch(neighbors, index, &candidate)
                            != NAVSYS_STATUS_OK
                        || !clear_if_blocked(
                            navgrid, candidate.x, candidate.y, removed)) {
                        coord_list_destroy(neighbors);
                        return 0;
                    }
                }
                coord_list_destroy(neighbors);
            }

            current = next;
        }

        return removed;
    } catch (...) {
        return 0;
    }
}

int carve_bomb_abi1(
    navgrid_t* navgrid,
    const coord_t* center,
    int range) noexcept {
    if (!navgrid || !center) return 0;

    try {
        int removed = 0;
        if (!clear_if_blocked(navgrid, center->x, center->y, removed)) {
            return 0;
        }
        if (range <= 0) return removed;

        coord_list_t* neighbors = navgrid_copy_neighbors_all_range(
            navgrid, center->x, center->y, range - 1);
        if (!neighbors) return 0;

        const std::size_t count = coord_list_size(neighbors);
        for (std::size_t index = 0; index < count; ++index) {
            coord_t candidate{};
            if (coord_list_fetch(neighbors, index, &candidate)
                    != NAVSYS_STATUS_OK
                || !clear_if_blocked(
                    navgrid, candidate.x, candidate.y, removed)) {
                coord_list_destroy(neighbors);
                return 0;
            }
        }
        coord_list_destroy(neighbors);
        return removed;
    } catch (...) {
        return 0;
    }
}

} // namespace byul::navsys::route_carver::compat
