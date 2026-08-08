/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file dstar_lite_planner.h
 * @brief Status-based incremental D* Lite planner API.
 *
 * The planner borrows its grid and callback userdata. Routes returned by
 * dstar_lite_replan are caller-owned. A planner is externally synchronized;
 * cancellation is the sole operation allowed concurrently with replan.
 */

#ifndef BYUL_DSTAR_LITE_PLANNER_H
#define BYUL_DSTAR_LITE_PLANNER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_navgrid navgrid_t;
typedef struct s_route route_t;
typedef struct s_dstar_lite dstar_lite_t;

/** Current public create-info layout version. */
#define DSTAR_LITE_CREATE_INFO_VERSION 1u
/** Current public replan-options layout version. */
#define DSTAR_LITE_REPLAN_OPTIONS_VERSION 1u

/** Checked directed-edge evaluator. */
typedef navsys_status_t (*dstar_lite_cost_callback_t)(
    const navgrid_t* grid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost,
    void* userdata);

/** Checked heuristic evaluator. */
typedef navsys_status_t (*dstar_lite_heuristic_callback_t)(
    const coord_t* from,
    const coord_t* to,
    float* out_estimate,
    void* userdata);

/** Optional cooperative cancellation callback. */
typedef bool (*dstar_lite_cancel_func)(void* userdata);

/** A directed edge-cost change. Positive infinity means blocked. */
typedef struct s_dstar_lite_edge_update {
    coord_t from;
    coord_t to;
    float old_cost;
    float new_cost;
} dstar_lite_edge_update_t;

/** Versioned planner construction input. */
typedef struct s_dstar_lite_create_info {
    size_t struct_size;
    uint32_t version;
    navgrid_t* navgrid;
    coord_t start;
    coord_t goal;
    dstar_lite_cost_callback_t cost_callback;
    void* cost_userdata;
    dstar_lite_heuristic_callback_t heuristic_callback;
    void* heuristic_userdata;
} dstar_lite_create_info_t;

/** Versioned per-replan limits and cancellation hook. */
typedef struct s_dstar_lite_replan_options {
    size_t struct_size;
    uint32_t version;
    size_t max_expansions;
    size_t max_route_steps;
    dstar_lite_cancel_func cancel_func;
    void* cancel_userdata;
} dstar_lite_replan_options_t;

/** Snapshot of cumulative and last-replan diagnostics. */
typedef struct s_dstar_lite_stats {
    size_t expansions;
    size_t vertex_updates;
    size_t edge_changes;
    size_t replans;
    size_t route_steps;
    float km;
    navsys_status_t last_status;
} dstar_lite_stats_t;

/**
 * @brief Initializes planner construction information with stable defaults.
 * @param[out] out_info Destination construction information.
 * @param[in] navgrid Borrowed navigation grid.
 * @param[in] start Initial start coordinate.
 * @param[in] goal Initial goal coordinate.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable out_info false
 * @byul.nullable navgrid false
 * @byul.nullable start false
 * @byul.nullable goal false
 */
BYUL_API navsys_status_t dstar_lite_create_info_init(
    dstar_lite_create_info_t* out_info, navgrid_t* navgrid,
    const coord_t* start, const coord_t* goal);

/**
 * @brief Initializes replan options with unlimited work defaults.
 * @param[out] out_options Destination options.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable out_options false
 */
BYUL_API navsys_status_t dstar_lite_replan_options_init(
    dstar_lite_replan_options_t* out_options);

/**
 * @brief Creates a planner that borrows its grid and callback userdata.
 * @param[in] info Valid versioned construction information.
 * @param[out] out_planner Receives the caller-owned planner on success.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable info false
 * @byul.nullable out_planner false
 */
BYUL_API navsys_status_t dstar_lite_create_ex(
    const dstar_lite_create_info_t* info, dstar_lite_t** out_planner);

/**
 * @brief Resets all incremental state around a new start and goal.
 * @param[in,out] planner Planner to reset.
 * @param[in] start New start coordinate.
 * @param[in] goal New goal coordinate.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable start false
 * @byul.nullable goal false
 */
BYUL_API navsys_status_t dstar_lite_reset_ex(
    dstar_lite_t* planner, const coord_t* start, const coord_t* goal);

/**
 * @brief Binds a checked cost callback or restores the default evaluator.
 * @param[in,out] planner Planner whose evaluator is changed.
 * @param[in] callback Callback, or NULL to use the grid evaluator.
 * @param[in] userdata Borrowed callback context; may be NULL.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable userdata true
 */
BYUL_API navsys_status_t dstar_lite_bind_cost_callback(
    dstar_lite_t* planner, dstar_lite_cost_callback_t callback,
    void* userdata);

/**
 * @brief Binds a checked heuristic or restores Euclidean distance.
 * @param[in,out] planner Planner whose heuristic is changed.
 * @param[in] callback Callback, or NULL to use the default heuristic.
 * @param[in] userdata Borrowed callback context; may be NULL.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable userdata true
 */
BYUL_API navsys_status_t dstar_lite_bind_heuristic_callback(
    dstar_lite_t* planner, dstar_lite_heuristic_callback_t callback,
    void* userdata);

/**
 * @brief Advances the current start and updates the D* Lite km term.
 * @param[in,out] planner Planner to update.
 * @param[in] start New current start coordinate.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable start false
 */
BYUL_API navsys_status_t dstar_lite_set_current_start(
    dstar_lite_t* planner, const coord_t* start);

/**
 * @brief Changes the goal and resets goal-dependent incremental state.
 * @param[in,out] planner Planner to update.
 * @param[in] goal New goal coordinate.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable goal false
 */
BYUL_API navsys_status_t dstar_lite_set_goal_ex(
    dstar_lite_t* planner, const coord_t* goal);

/**
 * @brief Applies a batch of directed edge-cost changes.
 * @param[in,out] planner Planner to update.
 * @param[in] updates Change array, or NULL when update_count is zero.
 * @param[in] update_count Number of records in updates.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable updates true
 */
BYUL_API navsys_status_t dstar_lite_notify_edge_changes(
    dstar_lite_t* planner, const dstar_lite_edge_update_t* updates,
    size_t update_count);

/**
 * @brief Computes or repairs a path and returns a complete route.
 * @param[in,out] planner Planner to execute.
 * @param[in] options Optional versioned limits and cancellation callback.
 * @param[out] out_route Receives a caller-owned route on success only.
 * @param[out] out_stats Optional destination for this replan's statistics.
 * @return A status code; NAVSYS_STATUS_OK only for a complete route.
 * @byul.nullable planner false
 * @byul.nullable options true
 * @byul.nullable out_route false
 * @byul.nullable out_stats true
 */
BYUL_API navsys_status_t dstar_lite_replan(
    dstar_lite_t* planner, const dstar_lite_replan_options_t* options,
    route_t** out_route, dstar_lite_stats_t* out_stats);

/**
 * @brief Copies the latest planner statistics.
 * @param[in] planner Planner to inspect.
 * @param[out] out_stats Destination statistics snapshot.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 * @byul.nullable out_stats false
 */
BYUL_API navsys_status_t dstar_lite_get_stats(
    const dstar_lite_t* planner, dstar_lite_stats_t* out_stats);

/**
 * @brief Requests cooperative cancellation of current or future work.
 * @param[in,out] planner Planner to cancel; safe during replan.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable planner false
 */
BYUL_API navsys_status_t dstar_lite_request_cancel(dstar_lite_t* planner);

/**
 * @brief Reports the sticky cancellation flag, which reset clears.
 * @param[in] planner Planner to inspect.
 * @return true when cancellation was requested; false otherwise.
 * @byul.nullable planner false
 */
BYUL_API bool dstar_lite_is_cancel_requested(const dstar_lite_t* planner);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_DSTAR_LITE_PLANNER_H */
