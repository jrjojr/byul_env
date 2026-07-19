/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_NAVGRID_CALLBACK_INTERNAL_HPP
#define BYUL_NAVGRID_CALLBACK_INTERNAL_HPP

#include "../navgrid.h"

namespace byul::navsys::internal {

inline thread_local const navgrid_t* active_callback_navgrid = nullptr;

class navgrid_callback_scope {
public:
    explicit navgrid_callback_scope(const navgrid_t* navgrid)
        : previous_(active_callback_navgrid) {
        active_callback_navgrid = navgrid;
    }

    ~navgrid_callback_scope() {
        active_callback_navgrid = previous_;
    }

private:
    const navgrid_t* previous_;
};

inline bool navgrid_callback_is_active(const navgrid_t* navgrid) {
    return active_callback_navgrid == navgrid;
}

inline bool navgrid_invoke_is_coord_blocked(
    const navgrid_t* navgrid, int x, int y) {
    if (!navgrid || !navgrid->is_coord_blocked_fn) return false;
    navgrid_callback_scope scope(navgrid);
    return navgrid->is_coord_blocked_fn(
        navgrid, x, y, navgrid->is_coord_blocked_fn_userdata);
}

} // namespace byul::navsys::internal

#endif /* BYUL_NAVGRID_CALLBACK_INTERNAL_HPP */
