/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_ROUTE_CARVER_COMPAT_ROUTE_CARVER_LEGACY_HPP
#define BYUL_ROUTE_CARVER_COMPAT_ROUTE_CARVER_LEGACY_HPP

#include "../route_carver.h"

namespace byul::navsys::route_carver::compat {

int carve_beam_abi1(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    int range) noexcept;

int carve_bomb_abi1(
    navgrid_t* navgrid,
    const coord_t* center,
    int range) noexcept;

} // namespace byul::navsys::route_carver::compat

#endif // BYUL_ROUTE_CARVER_COMPAT_ROUTE_CARVER_LEGACY_HPP
