/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_NAVGRID_CALLBACK_INTERNAL_HPP
#define BYUL_NAVGRID_CALLBACK_INTERNAL_HPP

#include "navgrid_private.hpp"

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

inline navsys_status_t navgrid_invoke_is_coord_blocked_checked(
    const navgrid_t* navgrid, int x, int y, bool* out_blocked) {
    if (!navgrid || !out_blocked) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!navgrid->is_coord_blocked_fn) {
        *out_blocked = false;
        return NAVSYS_STATUS_OK;
    }
    if (navgrid_callback_is_active(navgrid)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }

    try {
        navgrid_callback_scope scope(navgrid);
        *out_blocked = navgrid->is_coord_blocked_fn(
            navgrid, x, y, navgrid->is_coord_blocked_fn_userdata);
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

inline bool navgrid_invoke_is_coord_blocked(
    const navgrid_t* navgrid, int x, int y) {
    bool blocked = false;
    return navgrid_invoke_is_coord_blocked_checked(
        navgrid, x, y, &blocked) == NAVSYS_STATUS_OK
        ? blocked
        : true;
}

} // namespace byul::navsys::internal

#endif /* BYUL_NAVGRID_CALLBACK_INTERNAL_HPP */
