/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_recursive_division.h
 * @brief Declares the deterministic Recursive Division Maze public C ABI.
 */

#ifndef BYUL_MAZE_RECURSIVE_DIVISION_H
#define BYUL_MAZE_RECURSIVE_DIVISION_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generates a deterministic perfect Recursive Division Maze.
 *
 * The generator begins with an open interior and a blocked one-cell border.
 * Each active rectangular region is divided across its longer axis by an
 * even-coordinate wall with exactly one odd-coordinate passage. Square ties
 * use one random draw. Lower-coordinate child regions are processed first.
 * Equal extents and seeds produce an equal raster. The logical passage graph
 * is connected and acyclic. On failure, *out_maze is NULL.
 *
 * @param[in] origin_x Minimum X coordinate of the raster extent.
 * @param[in] origin_y Minimum Y coordinate of the raster extent.
 * @param[in] width Odd raster width of at least 3.
 * @param[in] height Odd raster height of at least 3.
 * @param[in] options Required generation controls.
 * @param[out] out_maze Receives a caller-owned Maze on success.
 * @return A common Navsys status value.
 * @retval NAVSYS_STATUS_OK Generation succeeded.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer or extent is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The option version or dimensions are unsupported.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Allocation failed.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED The cancellation callback threw.
 * @retval NAVSYS_STATUS_CANCELLED The callback requested cancellation.
 * @retval NAVSYS_STATUS_LIMIT_REACHED A resource limit was reached.
 * @retval NAVSYS_STATUS_CORRUPT_STATE An unexpected internal exception occurred.
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity O(width*height) time and O(width*height) Maze/grid storage, with
 * O(width+height) worst-case pending-region stack storage.
 */
BYUL_API navsys_status_t byul_maze_generate_recursive_division(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

/**
 * @brief Generates a Recursive Division Maze with legacy process-local seeding.
 *
 * Use the checked API for deterministic replay, cancellation, and resource
 * budgets.
 *
 * @deprecated Use byul_maze_generate_recursive_division. Removal requires ABI 2 or later.
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
    "Use byul_maze_generate_recursive_division; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_recursive_division(
    int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_RECURSIVE_DIVISION_H */
