/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_hunt_and_kill.h
 * @brief Replay 가능한 Hunt-and-Kill Maze 생성 C ABI를 선언한다.
 *
 * 결정적 checked 생성 API와 ABI 1 호환 레거시 adapter를 함께 제공한다.
 */

#ifndef BYUL_MAZE_HUNT_AND_KILL_H
#define BYUL_MAZE_HUNT_AND_KILL_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 지정한 seed로 connected acyclic Hunt-and-Kill Maze를 생성한다.
 *
 * Kill phase는 현재 cell의 unvisited neighbor 순서를 무작위화해 첫 cell로
 * 진행한다. Dead end에서는 odd/odd logical cell을 row-major로 hunt하고,
 * 첫 eligible cell을 기존 tree의 무작위 인접 cell 하나에 연결한다. 같은
 * extent와 options seed는 같은 raster를 생성한다. 실패하면 *out_maze는
 * NULL이다.
 *
 * @param[in] origin_x 반열린 extent의 최소 X 좌표다.
 * @param[in] origin_y 반열린 extent의 최소 Y 좌표다.
 * @param[in] width 3 이상의 홀수 너비다.
 * @param[in] height 3 이상의 홀수 높이다.
 * @param[in] options 필수 generation 제어값이다.
 * @param[out] out_maze 성공 시 caller-owned Maze를 받는다.
 * @return 공통 Navsys 상태값이다.
 * @retval NAVSYS_STATUS_OK 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 extent가 유효하지 않다.
 * @retval NAVSYS_STATUS_UNSUPPORTED option version 또는 dimension이 지원되지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED 취소 callback이 예외를 던졌다.
 * @retval NAVSYS_STATUS_CANCELLED callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED resource limit에 도달했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 모든 logical cell을 tree에 연결하지 못했다.
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity 현재 row-major hunt 구현은 최악 O(V^2) 시간이며,
 * O(width*height) Maze와 visited 저장 공간을 사용한다.
 */
BYUL_API navsys_status_t byul_maze_generate_hunt_and_kill(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze);

/**
 * @brief 비결정적 Hunt-and-Kill 방식으로 perfect Maze를 생성한다.
 *
 * Process-local seed와 ROW_MAJOR_FIRST hunt 정책을 사용하는 ABI 1
 * adapter다. Replay와 명시적 resource budget이 필요하면 checked API를
 * 사용한다.
 *
 * @deprecated byul_maze_generate_hunt_and_kill을 사용한다. ABI 2 이후 제거할 수 있다.
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
    "Use byul_maze_generate_hunt_and_kill; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_hunt_and_kill(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_HUNT_AND_KILL_H */
