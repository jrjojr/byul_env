/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_room_blend.h
 * @brief 결정적 Room Blend Maze 생성기의 public C ABI를 선언한다.
 *
 * 방 배치 범위와 간격, seed, 실행 예산 및 협력적 취소를 명시하는 checked
 * 생성 API와 ABI 1.x 레거시 진입점을 제공한다.
 */

#ifndef BYUL_MAZE_ROOM_BLEND_H
#define BYUL_MAZE_ROOM_BLEND_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ABI 1 호환용 직사각형 방 값이다.
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_room {
    int x; /**< 왼쪽 위 X 좌표다. */
    int y; /**< 왼쪽 위 Y 좌표다. */
    int w; /**< 방 너비다. */
    int h; /**< 방 높이다. */
} room_t;

#define BYUL_ROOM_BLEND_OPTIONS_ABI_VERSION UINT32_C(1)

/**
 * @brief 버전이 지정된 Room Blend 생성 제어값이다.
 *
 * 방 크기는 3 이상의 홀수이고 min 값은 대응 max 값 이하여야 한다.
 * room_padding은 방 사이에 유지할 blocked cell 수이며 외곽 경계에는 적용하지
 * 않는다. room_attempts가 0이면 방 없이 depth-first maze fill만 수행한다.
 * room_attempts는 raster cell 수의 32배 이하여야 한다.
 * max_steps와 max_cells가 0이면 구현의 안전 기본값을 사용한다.
 * cancel_func와 cancel_userdata는 호출 중에만 빌려 쓰며 보관하지 않는다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_byul_room_blend_options {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t seed;
    uint64_t max_steps;
    uint64_t max_cells;
    uint32_t room_attempts;
    uint32_t min_room_width;
    uint32_t min_room_height;
    uint32_t max_room_width;
    uint32_t max_room_height;
    uint32_t room_padding;
    byul_maze_generate_cancel_func cancel_func;
    void* cancel_userdata;
} byul_room_blend_options_t;

/**
 * @brief 명시된 정책으로 결정적 Room Blend Maze를 생성한다.
 *
 * 각 배치 시도는 홀수 크기와 홀수 local origin을 균등 선택하고 padding을
 * 포함해 기존 방과 겹치지 않을 때만 채택한다. 채택 순서의 인접한 방 중심은
 * seeded RNG draw가 선택한 horizontal-first 또는 vertical-first L corridor로 연결한다.
 * corridor는 반경 0인 한 cell 너비의 centerline이며 두 room center에서 시작해
 * room boundary를 통과한다. 모든 홀수 logical cell을 (1, 1)에서 시작하는 하나의
 * depth-first tree로 채우므로 room, corridor와 fill passage 전체가 하나의
 * orthogonally connected network를 이룬다. 방은 cycle을 허용하지만 room_attempts가
 * 0이면 fill은 perfect maze다. raster 외곽은 항상 blocked다. 같은 extent,
 * options와 seed는 같은 raster를 만든다. 실패하면 *out_maze는 NULL이다.
 *
 * @param[in] origin_x raster extent의 최소 X 좌표다.
 * @param[in] origin_y raster extent의 최소 Y 좌표다.
 * @param[in] width 9 이상인 raster 너비다.
 * @param[in] height 9 이상인 raster 높이다.
 * @param[in] options 필수 Room Blend 생성 제어값이다.
 * @param[out] out_maze 성공 시 caller-owned Maze를 받는다.
 * @return 공통 Navsys 상태 값이다.
 * @retval NAVSYS_STATUS_OK 생성에 성공했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer, extent 또는 option 값이 잘못됐다.
 * @retval NAVSYS_STATUS_UNSUPPORTED option version 또는 dimension을 지원하지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED 취소 callback이 예외를 던졌다.
 * @retval NAVSYS_STATUS_CANCELLED callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED resource limit에 도달했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 예상하지 못한 내부 예외가 발생했다.
 * @byul.nullable options false
 * @byul.nullable out_maze false
 * @byul.lifetime options call-only
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant false
 * @complexity O(room_attempts*R + R*(width+height) + width*height) time and
 * O(width*height + R) storage, where R is the number of accepted rooms.
 */
BYUL_API navsys_status_t byul_maze_generate_room_blend(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_room_blend_options_t* options,
    maze_t** out_maze);

/**
 * @brief 비결정적 Room Blend 방식으로 연결된 순환 허용 Maze를 생성한다.
 *
 * 30회 배치, 3..7 홀수 방 크기와 padding 0을 사용하는 ABI 1 adapter다.
 * Replay, 명시 room policy와 resource budget이 필요하면 checked API를 사용한다.
 *
 * @deprecated byul_maze_generate_room_blend를 사용한다. ABI 2 이후 제거할 수 있다.
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
    "Use byul_maze_generate_room_blend; removal requires ABI 2 or later.")
BYUL_API maze_t* maze_make_room_blend(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_ROOM_BLEND_H */
