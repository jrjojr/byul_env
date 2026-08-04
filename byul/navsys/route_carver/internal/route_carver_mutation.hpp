/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_ROUTE_CARVER_MUTATION_INTERNAL_HPP
#define BYUL_ROUTE_CARVER_MUTATION_INTERNAL_HPP

#include "coord.h"
#include "navgrid.h"
#include "navsys_status.h"
#include "route_carver_geometry.hpp"

#include <cstddef>

namespace byul::navsys::route_carver::internal {

enum class carve_match {
    stored_forbidden_only = 0,
    effective_blocked = 1
};

navsys_status_t mutate_candidates_atomic(
    navgrid_t* navgrid,
    const coord_t* candidates,
    size_t count,
    carve_match match,
    bool dry_run,
    bool atomic,
    carve_cancel_func cancel_func,
    void* cancel_userdata,
    size_t* out_changed_count);

} // namespace byul::navsys::route_carver::internal

#endif /* BYUL_ROUTE_CARVER_MUTATION_INTERNAL_HPP */
