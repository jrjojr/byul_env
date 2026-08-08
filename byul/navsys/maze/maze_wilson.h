/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_wilson.h
 * @brief Wilson Maze 생성기의 레거시 public C ABI를 선언한다.
 */

#ifndef BYUL_MAZE_WILSON_H
#define BYUL_MAZE_WILSON_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generates a replayable uniform-spanning-tree Maze with Wilson's algorithm.
 *
 * Loop-erased random walks are committed only after reaching the existing tree.
 * The same extent and seed produce the same raster. On failure, `*out_maze` is
 * NULL and no partial Maze is published.
 *
 * @param[in] origin_x Minimum X coordinate of the raster extent.
 * @param[in] origin_y Minimum Y coordinate of the raster extent.
 * @param[in] width Odd raster width of at least 3.
 * @param[in] height Odd raster height of at least 3.
 * @param[in] options Required generation controls and seed.
 * @param[out] out_maze Receives a caller-owned Maze on success.
 * @return A common Navsys status value.
 * @retval NAVSYS_STATUS_OK Generation succeeded.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer or extent is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The option version or dimensions are unsupported.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Allocation failed.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED The cancellation callback threw an exception.
 * @retval NAVSYS_STATUS_CANCELLED The callback requested cancellation.
 * @retval NAVSYS_STATUS_LIMIT_REACHED A resource limit was reached.
 * @retval NAVSYS_STATUS_CORRUPT_STATE An internal invariant failed.
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity Expected O(width*height) time with unbounded random-walk variance,
 * and O(width*height) temporary and output storage.
 */
BYUL_API navsys_status_t byul_maze_generate_wilson(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

/**
 * @brief 비결정적 Wilson loop-erased random walk로 uniform spanning-tree Maze를 생성한다.
 * @deprecated Use byul_maze_generate_wilson. Removal requires ABI 2 or later.
 * @param[in] x0 Maze 원점의 X 좌표다.
 * @param[in] y0 Maze 원점의 Y 좌표다.
 * @param[in] width 3 이상인 홀수 너비다.
 * @param[in] height 3 이상인 홀수 높이다.
 * @return caller-owned Maze이며 입력 또는 allocation 실패 시 NULL이다.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_DEPRECATED("Use byul_maze_generate_wilson; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_wilson(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_WILSON_H */
