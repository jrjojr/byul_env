/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_sidewinder.h
 * @brief Declares the deterministic Sidewinder Maze public C ABI.
 */

#ifndef BYUL_MAZE_SIDEWINDER_H
#define BYUL_MAZE_SIDEWINDER_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Selects the Sidewinder run and previous-row directions. */
typedef enum e_byul_maze_sidewinder_sweep {
    BYUL_MAZE_SIDEWINDER_EAST_NORTH = 0, /**< Runs east, connects north. */
    BYUL_MAZE_SIDEWINDER_EAST_SOUTH = 1, /**< Runs east, connects south. */
    BYUL_MAZE_SIDEWINDER_WEST_NORTH = 2, /**< Runs west, connects north. */
    BYUL_MAZE_SIDEWINDER_WEST_SOUTH = 3  /**< Runs west, connects south. */
} byul_maze_sidewinder_sweep_t;

/**
 * @brief Reports whether a Sidewinder sweep value is supported.
 * @param[in] sweep Sweep value to query.
 * @param[out] out_supported Receives true for a known sweep.
 * @return A common Navsys status value.
 * @retval NAVSYS_STATUS_OK The sweep is known.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_supported is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED sweep is unknown.
 * @byul.nullable out_supported false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_supported-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t byul_maze_sidewinder_sweep_is_supported(
    byul_maze_sidewinder_sweep_t sweep, bool* out_supported);

/**
 * @brief Generates a deterministic perfect Sidewinder Maze.
 *
 * Local X increases east and local Y increases south. The boundary logical row
 * nearest the connection direction is a forced corridor. Every run in each
 * later row opens exactly one connection toward the preceding logical row.
 * Equal extents, sweep, and seed produce an equal raster. On failure,
 * *out_maze is NULL.
 *
 * @param[in] origin_x Minimum X coordinate of the raster extent.
 * @param[in] origin_y Minimum Y coordinate of the raster extent.
 * @param[in] width Odd raster width of at least 3.
 * @param[in] height Odd raster height of at least 3.
 * @param[in] sweep Run and previous-row direction pair.
 * @param[in] options Required generation controls.
 * @param[out] out_maze Receives a caller-owned Maze on success.
 * @return A common Navsys status value.
 * @retval NAVSYS_STATUS_OK Generation succeeded.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer or extent is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The option version, dimensions, or sweep are unsupported.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Allocation failed.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED The cancellation callback threw.
 * @retval NAVSYS_STATUS_CANCELLED The callback requested cancellation.
 * @retval NAVSYS_STATUS_LIMIT_REACHED A resource limit was reached.
 * @byul.enum_support sweep query:byul_maze_sidewinder_sweep_is_supported
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity O(width*height) time and O(width) temporary run storage, in
 * addition to the O(width*height) Maze storage.
 */
BYUL_API navsys_status_t byul_maze_generate_sidewinder(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_sidewinder_sweep_t sweep,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

/**
 * @brief Generates a Sidewinder Maze using legacy process-local seeding.
 *
 * This ABI 1 adapter selects BYUL_MAZE_SIDEWINDER_EAST_NORTH. Use the checked
 * API for replay, direction control, cancellation, and resource budgets.
 *
 * @deprecated Use byul_maze_generate_sidewinder. Removal requires ABI 2 or later.
 * @param[in] x0 Maze origin X coordinate.
 * @param[in] y0 Maze origin Y coordinate.
 * @param[in] width Odd raster width of at least 3.
 * @param[in] height Odd raster height of at least 3.
 * @return A caller-owned Maze, or NULL for invalid input or failure.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_DEPRECATED(
    "Use byul_maze_generate_sidewinder; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_sidewinder(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_SIDEWINDER_H */
