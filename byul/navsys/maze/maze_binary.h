/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_binary.h
 * @brief 방향 bias를 지정할 수 있는 Binary Tree Maze 생성 C ABI를 선언한다.
 *
 * 결정적 seed, generation budget과 협력적 취소를 사용하는 checked 생성 API와
 * ABI 1 호환용 비결정적 legacy adapter를 함께 제공한다.
 */

#ifndef BYUL_MAZE_BINARY_H
#define BYUL_MAZE_BINARY_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Binary Tree가 각 logical cell을 연결할 직교 방향 조합이다.
 *
 * 첫 번째 방향은 root corridor의 Y축 끝을, 두 번째 방향은 X축 끝을 나타낸다.
 */
typedef enum e_byul_maze_binary_bias {
    BYUL_MAZE_BINARY_BIAS_NORTH_WEST = 0, /**< 북쪽과 서쪽을 후보로 사용한다. */
    BYUL_MAZE_BINARY_BIAS_NORTH_EAST = 1, /**< 북쪽과 동쪽을 후보로 사용한다. */
    BYUL_MAZE_BINARY_BIAS_SOUTH_WEST = 2, /**< 남쪽과 서쪽을 후보로 사용한다. */
    BYUL_MAZE_BINARY_BIAS_SOUTH_EAST = 3  /**< 남쪽과 동쪽을 후보로 사용한다. */
} byul_maze_binary_bias_t;

/**
 * @brief Binary Tree bias가 현재 구현에서 지원되는지 조회한다.
 * @param[in] bias 조회할 방향 조합이다.
 * @param[out] out_supported 알려진 bias이면 true를 받는다.
 * @return 공통 Navsys 상태값이다.
 * @retval NAVSYS_STATUS_OK 알려진 bias를 조회했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_supported가 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED bias가 알려진 값이 아니다.
 * @byul.nullable out_supported false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_supported-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t byul_maze_binary_bias_is_supported(
    byul_maze_binary_bias_t bias, bool* out_supported);

/**
 * @brief 지정한 diagonal bias와 seed로 perfect Binary Tree Maze를 생성한다.
 *
 * X는 동쪽으로, Y는 남쪽으로 증가한다. 경계에서 한 방향만 가능하면 RNG를
 * 소비하지 않고 그 방향을 강제한다. 두 방향이 가능할 때만 PCG32 draw 하나를
 * 소비한다. 같은 extent, bias, options seed는 같은 raster를 생성한다.
 *
 * 결과는 logical cell N=((width-1)/2)*((height-1)/2)개와 passage N-1개인
 * connected acyclic perfect Maze다. Uniform spanning tree는 아니며 선택한 bias의
 * root row와 root column에 강제 corridor가 생긴다. 실패하면 *out_maze는 NULL이다.
 *
 * @param[in] origin_x 반열린 extent의 최소 X 좌표다.
 * @param[in] origin_y 반열린 extent의 최소 Y 좌표다.
 * @param[in] width 3 이상인 홀수 너비다.
 * @param[in] height 3 이상인 홀수 높이다.
 * @param[in] bias logical cell 연결 방향 조합이다.
 * @param[in] options 필수 generation 제어값이다.
 * @param[out] out_maze 성공 시 caller-owned Maze를 받는다.
 * @return 공통 Navsys 상태값이다.
 * @retval NAVSYS_STATUS_OK 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 extent가 유효하지 않다.
 * @retval NAVSYS_STATUS_UNSUPPORTED option version, dimension 또는 bias가 지원되지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED 취소 callback이 예외를 던졌다.
 * @retval NAVSYS_STATUS_CANCELLED callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED resource limit에 도달했다.
 * @byul.enum_support bias query:byul_maze_binary_bias_is_supported
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity O(width*height) 시간과 O(width*height) Maze 저장공간을 사용한다.
 */
BYUL_API navsys_status_t byul_maze_generate_binary_tree(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_binary_bias_t bias,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

/**
 * @brief 비결정적 Binary Tree 방식으로 perfect Maze를 생성한다.
 *
 * BYUL_MAZE_BINARY_BIAS_SOUTH_EAST와 process-local 비결정 seed를 사용하는
 * ABI 1 adapter다. Replay와 명시적 resource budget이 필요하면 checked API를 사용한다.
 *
 * @deprecated byul_maze_generate_binary_tree를 사용한다. ABI 2 이후 제거할 수 있다.
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
BYUL_DEPRECATED(
    "Use byul_maze_generate_binary_tree; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_binary(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_BINARY_H */
