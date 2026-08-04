/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_sidewinder.h
 * @brief Sidewinder Maze 생성기의 레거시 public C ABI를 선언한다.
 */

#ifndef BYUL_MAZE_SIDEWINDER_H
#define BYUL_MAZE_SIDEWINDER_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 비결정적 Sidewinder 방식으로 perfect Maze를 생성한다.
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
BYUL_API maze_t* maze_make_sidewinder(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_SIDEWINDER_H */
