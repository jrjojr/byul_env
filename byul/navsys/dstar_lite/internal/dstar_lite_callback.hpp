/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_DSTAR_LITE_CALLBACK_INTERNAL_HPP
#define BYUL_DSTAR_LITE_CALLBACK_INTERNAL_HPP

#include "../dstar_lite.h"

#include <cfloat>

namespace byul::navsys::internal {

inline thread_local const dstar_lite_t* active_callback_dstar_lite = nullptr;

class dstar_lite_callback_scope {
public:
    explicit dstar_lite_callback_scope(const dstar_lite_t* dsl)
        : previous_(active_callback_dstar_lite) {
        active_callback_dstar_lite = dsl;
    }

    ~dstar_lite_callback_scope() {
        active_callback_dstar_lite = previous_;
    }

private:
    const dstar_lite_t* previous_;
};

inline bool dstar_lite_callback_is_active(const dstar_lite_t* dsl) {
    return active_callback_dstar_lite == dsl;
}

inline float dstar_lite_invoke_cost(
    const dstar_lite_t* dsl,
    const navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal) {
    if (!dsl || !dsl->cost_fn) return FLT_MAX;
    dstar_lite_callback_scope scope(dsl);
    return dsl->cost_fn(
        navgrid, start, goal, dsl->cost_fn_userdata);
}

inline float dstar_lite_invoke_heuristic(
    const dstar_lite_t* dsl,
    const coord_t* start,
    const coord_t* goal) {
    if (!dsl || !dsl->heuristic_fn) return FLT_MAX;
    dstar_lite_callback_scope scope(dsl);
    return dsl->heuristic_fn(
        start, goal, dsl->heuristic_fn_userdata);
}

inline void dstar_lite_invoke_move(
    const dstar_lite_t* dsl, const coord_t* coord) {
    if (!dsl || !dsl->move_fn) return;
    dstar_lite_callback_scope scope(dsl);
    dsl->move_fn(coord, dsl->move_fn_userdata);
}

inline coord_list_t* dstar_lite_invoke_changed_coords(
    const dstar_lite_t* dsl) {
    if (!dsl || !dsl->changed_coords_fn) return nullptr;
    dstar_lite_callback_scope scope(dsl);
    return dsl->changed_coords_fn(dsl->changed_coords_fn_userdata);
}

} // namespace byul::navsys::internal

#endif /* BYUL_DSTAR_LITE_CALLBACK_INTERNAL_HPP */
