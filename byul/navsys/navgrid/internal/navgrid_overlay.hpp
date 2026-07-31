/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_NAVGRID_OVERLAY_INTERNAL_HPP
#define BYUL_NAVGRID_OVERLAY_INTERNAL_HPP

#include "../navgrid.h"

namespace byul::navsys::internal {

enum class navgrid_overlay_source_kind : uint32_t {
    maze = 1,
    obstacle = 2
};

navsys_status_t navgrid_replace_blocked_overlay_source(
    navgrid_t* navgrid,
    navgrid_overlay_source_kind kind,
    const void* source,
    const coord_t* coords,
    size_t count,
    size_t* out_changed_count);

navsys_status_t navgrid_remove_blocked_overlay_source(
    navgrid_t* navgrid,
    navgrid_overlay_source_kind kind,
    const void* source,
    size_t* out_changed_count);

navsys_status_t navgrid_clear_blocked_at_coord(
    navgrid_t* navgrid, int x, int y, bool* out_changed);

} // namespace byul::navsys::internal

#endif /* BYUL_NAVGRID_OVERLAY_INTERNAL_HPP */
