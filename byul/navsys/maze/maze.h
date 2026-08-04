/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze.h
 * @brief Maze 생성 알고리즘 선택과 checked dispatcher C ABI를 선언한다.
 *
 * 결정적 seed, 실행 예산, 협력적 취소를 지원하는 공통 생성 진입점과 ABI 1
 * 레거시 생성 진입점을 함께 제공한다.
 */

#ifndef BYUL_MAZE_H
#define BYUL_MAZE_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ABI 1 레거시 Maze 알고리즘 식별자다. */
typedef enum e_maze_type {
    MAZE_TYPE_RECURSIVE = 0,
    MAZE_TYPE_PRIM = 1,
    MAZE_TYPE_BINARY = 2,
    MAZE_TYPE_ELLER = 3,

    MAZE_TYPE_ALDOUS_BRODER = 4,
    MAZE_TYPE_WILSON = 5,
    MAZE_TYPE_HUNT_AND_KILL = 6,
    MAZE_TYPE_SIDEWINDER = 7,
    
    MAZE_TYPE_RECURSIVE_DIVISION = 8,
    MAZE_TYPE_KRUSKAL = 9,
    MAZE_TYPE_ROOM_BLEND = 10
} maze_type_t;

/** @brief Checked Maze 생성 API의 안정적인 알고리즘 식별자다. */
typedef enum e_byul_maze_algorithm {
    BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER = 0,
    BYUL_MAZE_ALGORITHM_RANDOMIZED_PRIM = 1,
    BYUL_MAZE_ALGORITHM_BINARY_TREE = 2,
    BYUL_MAZE_ALGORITHM_ELLER = 3,
    BYUL_MAZE_ALGORITHM_ALDOUS_BRODER = 4,
    BYUL_MAZE_ALGORITHM_WILSON = 5,
    BYUL_MAZE_ALGORITHM_HUNT_AND_KILL = 6,
    BYUL_MAZE_ALGORITHM_SIDEWINDER = 7,
    BYUL_MAZE_ALGORITHM_RECURSIVE_DIVISION = 8,
    BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL = 9,
    BYUL_MAZE_ALGORITHM_ROOM_BLEND = 10
} byul_maze_algorithm_t;

#define BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION UINT32_C(1)

/** @brief 호출 스레드에서 Maze 생성을 협력적으로 취소할지 조회한다. */
typedef bool (*byul_maze_generate_cancel_func)(void* userdata);

/**
 * @brief 버전이 지정된 결정적 Maze 생성 제어값이다.
 *
 * struct_size는 sizeof(byul_maze_generate_options_t), abi_version은
 * BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION이어야 한다. seed는 그대로 PCG32
 * 입력으로 사용된다. max_steps와 max_cells가 0이면 구현의 안전 기본값을 쓴다.
 * cancel_func와 cancel_userdata는 호출 중에만 빌려 쓰며 보관하지 않는다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_byul_maze_generate_options {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t seed;
    uint64_t max_steps;
    uint64_t max_cells;
    byul_maze_generate_cancel_func cancel_func;
    void* cancel_userdata;
} byul_maze_generate_options_t;

/**
 * @brief Reports whether an algorithm passed its current correctness gate.
 * @param[in] algorithm Algorithm identifier to query.
 * @param[out] out_supported Receives true only for an advertised algorithm.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The known algorithm was queried.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_supported is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED algorithm is not a known value.
 * @byul.nullable out_supported false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_supported-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t byul_maze_algorithm_is_supported(
    byul_maze_algorithm_t algorithm, bool* out_supported);

/**
 * @brief Generates a deterministic Maze through the checked dispatcher.
 * @param[in] algorithm Versioned Maze algorithm identifier.
 * @param[in] origin_x Minimum X coordinate of the half-open extent.
 * @param[in] origin_y Minimum Y coordinate of the half-open extent.
 * @param[in] width Number of cells on the X axis.
 * @param[in] height Number of cells on the Y axis.
 * @param[in] options Required versioned generation controls.
 * @param[out] out_maze Receives a caller-owned Maze only on success.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK Generation completed.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer, extent, or option size is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The version, algorithm, or dimension is unsupported.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Generation allocation failed.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED The cancellation callback threw an exception.
 * @retval NAVSYS_STATUS_CANCELLED The callback requested cancellation.
 * @retval NAVSYS_STATUS_LIMIT_REACHED A configured resource limit was reached.
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API navsys_status_t byul_maze_generate(
    byul_maze_algorithm_t algorithm,
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

BYUL_DEPRECATED(
    "Use byul_maze_generate; removal requires ABI 2 or later.")
/**
 * @brief ABI 1의 비결정적 레거시 계약으로 Maze를 생성한다.
 * @param[in] x0 Maze 원점의 X 좌표다.
 * @param[in] y0 Maze 원점의 Y 좌표다.
 * @param[in] width 양수인 홀수 너비다.
 * @param[in] height 양수인 홀수 높이다.
 * @param[in] type 레거시 알고리즘 식별자다.
 * @return caller-owned Maze이며 실패하면 NULL이다.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API maze_t* maze_make(
    int x0, int y0, int width, int height, maze_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_H */
