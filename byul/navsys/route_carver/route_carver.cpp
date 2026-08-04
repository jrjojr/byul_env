/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "route_carver.h"

#include "compat/route_carver_legacy.hpp"

int route_carve_beam(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    int range) {
    return byul::navsys::route_carver::compat::carve_beam_abi1(
        navgrid, start, goal, range);
}

int route_carve_bomb(
    navgrid_t* navgrid,
    const coord_t* center,
    int range) {
    return byul::navsys::route_carver::compat::carve_bomb_abi1(
        navgrid, center, range);
}
