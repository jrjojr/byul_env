/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file obstacle.h
 * @brief Obstacle 이웃 조회, mutation, 기하 생성 public C API를 선언한다.
 */

#ifndef BYUL_OBSTACLE_H
#define BYUL_OBSTACLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord_list.h"
#include "obstacle_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 인접 좌표를 legacy-compatible 순서로 caller buffer에 내보낸다.
 *
 * 순서는 N/W/E/S/NW/SW/NE/SE이며 traversable_only가 true이면 blocked 좌표를
 * 제외한다. NULL/0은 count query이고 짧은 buffer는 보존하면서 전체 필요 수와
 * NAVSYS_STATUS_INCOMPLETE를 반환한다.
 *
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] traversable_only blocked 좌표 제외 여부.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element 용량.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값. 실패하면 출력 buffer를 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.buffer out_coords
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_export_neighbors(
    const obstacle_t* obstacle, int32_t x, int32_t y,
    bool traversable_only, coord_t* out_coords,
    size_t capacity, size_t* out_count);

/**
 * @brief Legacy all-range topology 좌표를 X/Y 오름차순으로 내보낸다.
 *
 * range 0은 중심을 제외한 즉시 이웃이고, 양수 range는 중심을 포함하는
 * Chebyshev radius range+1 영역이다. Extent 밖 좌표는 제외한다.
 *
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] range 0 이상의 legacy range 값.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element 용량.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값. 실패하면 출력 buffer를 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.buffer out_coords
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_export_neighbors_range(
    const obstacle_t* obstacle, int32_t x, int32_t y, int32_t range,
    coord_t* out_coords, size_t capacity, size_t* out_count);

/**
 * @brief 지정 각도에 가장 가까운 extent 내부 인접 좌표를 복사한다.
 *
 * 0도는 +X이고 양의 각도는 +Y 방향이다. Tie는 normalized angle이 작은 좌표를
 * 선택한다.
 *
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] degree 유한한 degree 각도.
 * @param[out] out_coord 선택 좌표를 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 out_coord를 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_fetch_neighbor_at_degree(
    const obstacle_t* obstacle, int32_t x, int32_t y,
    double degree, coord_t* out_coord);

/**
 * @brief Goal 방향에 가장 가까운 extent 내부 인접 좌표를 복사한다.
 *
 * center와 goal이 같으면 NAVSYS_STATUS_INVALID_ARGUMENT를 반환한다.
 *
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 방향을 정할 목표 좌표.
 * @param[out] out_coord 선택 좌표를 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 out_coord를 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_fetch_neighbor_at_goal(
    const obstacle_t* obstacle, const coord_t* center,
    const coord_t* goal, coord_t* out_coord);

/**
 * @brief Goal 상대 각도 구간과 square range 안의 좌표를 내보낸다.
 *
 * 결과는 X/Y 오름차순이며 center를 제외한다. 각도 구간의 양 끝은 포함한다.
 *
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 기준 방향을 정할 목표 좌표.
 * @param[in] start_deg 포함되는 시작 상대 각도.
 * @param[in] end_deg 포함되는 끝 상대 각도.
 * @param[in] range 0 이상의 Chebyshev radius.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element 용량.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값. 실패하면 출력 buffer를 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.buffer out_coords
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_export_neighbors_at_degree_range(
    const obstacle_t* obstacle,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg, int32_t range,
    coord_t* out_coords, size_t capacity, size_t* out_count);

/**
 * @brief Blocked 좌표를 제외한 즉시 이웃 list를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @return Caller가 coord_list_destroy()로 해제할 list 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_list_t* obstacle_create_neighbors(
    const obstacle_t* obstacle, int32_t x, int32_t y);

/**
 * @brief Blocked 여부와 무관한 즉시 이웃 list를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @return Caller가 coord_list_destroy()로 해제할 list 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_list_t* obstacle_create_neighbors_all(
    const obstacle_t* obstacle, int32_t x, int32_t y);

/**
 * @brief Legacy all-range topology 좌표 list를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] range 0 이상의 legacy range 값.
 * @return Caller가 coord_list_destroy()로 해제할 list 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_list_t* obstacle_create_neighbors_all_range(
    const obstacle_t* obstacle, int32_t x, int32_t y, int32_t range);

/**
 * @brief 지정 각도에 가장 가까운 인접 좌표를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] degree 유한한 degree 각도.
 * @return Caller가 coord_destroy()로 해제할 좌표 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_t* obstacle_create_neighbor_at_degree(
    const obstacle_t* obstacle, int32_t x, int32_t y, double degree);

/**
 * @brief Goal 방향에 가장 가까운 인접 좌표를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 방향을 정할 목표 좌표.
 * @return Caller가 coord_destroy()로 해제할 좌표 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_t* obstacle_create_neighbor_at_goal(
    const obstacle_t* obstacle, const coord_t* center, const coord_t* goal);

/**
 * @brief Goal 상대 각도 구간의 좌표 list를 생성한다.
 * @param[in] obstacle 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 기준 방향을 정할 목표 좌표.
 * @param[in] start_deg 포함되는 시작 상대 각도.
 * @param[in] end_deg 포함되는 끝 상대 각도.
 * @param[in] range 0 이상의 Chebyshev radius.
 * @return Caller가 coord_list_destroy()로 해제할 list 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API coord_list_t* obstacle_create_neighbors_at_degree_range(
    const obstacle_t* obstacle,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg, int32_t range);

/**
 * @brief 중심 주위 Chebyshev square를 failure-atomic하게 block한다.
 * @param[in,out] obstacle 변경할 obstacle.
 * @param[in] x 중심 X 좌표(grid cell).
 * @param[in] y 중심 Y 좌표(grid cell).
 * @param[in] radius 0 이상의 Chebyshev radius.
 * @param[out] out_changed_count 새로 block된 좌표 수.
 * @return 공통 Navsys 상태 값. 실패하면 obstacle과 output을 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:obstacle,out_changed_count-on-success
 * @byul.invalidates block_square all-internal-pointers
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_block_square(
    obstacle_t* obstacle, int32_t x, int32_t y, int32_t radius,
    size_t* out_changed_count);

/**
 * @brief Inclusive Bresenham line과 Chebyshev thickness를 failure-atomic하게 block한다.
 * @param[in,out] obstacle 변경할 obstacle.
 * @param[in] x0 시작 X 좌표(grid cell).
 * @param[in] y0 시작 Y 좌표(grid cell).
 * @param[in] x1 끝 X 좌표(grid cell).
 * @param[in] y1 끝 Y 좌표(grid cell).
 * @param[in] radius 0 이상의 Chebyshev thickness radius.
 * @param[out] out_changed_count 새로 block된 고유 좌표 수.
 * @return 공통 Navsys 상태 값. 실패하면 obstacle과 output을 보존한다.
 * @byul.nullable obstacle false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:obstacle,out_changed_count-on-success
 * @byul.invalidates block_line all-internal-pointers
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_block_line(
    obstacle_t* obstacle,
    int32_t x0, int32_t y0, int32_t x1, int32_t y1,
    int32_t radius, size_t* out_changed_count);

/** @brief Obstacle generator raster selection policy. */
typedef enum e_obstacle_raster_rule {
    OBSTACLE_RASTER_CELL_CENTER = 0,
    OBSTACLE_RASTER_ALL_TOUCHED = 1
} obstacle_raster_rule_t;

/** @brief Polygon interior fill policy. */
typedef enum e_obstacle_polygon_fill_rule {
    OBSTACLE_POLYGON_EVEN_ODD = 0,
    OBSTACLE_POLYGON_NON_ZERO = 1
} obstacle_polygon_fill_rule_t;

/**
 * @brief Generator cooperative cancellation callback.
 * @return true to cancel the current generator call.
 * @byul.nullable userdata true
 * @byul.lifetime userdata call-scoped
 * @byul.thread_safety externally-synchronized
 */
typedef bool (*obstacle_generate_cancel_func)(void* userdata);

#define OBSTACLE_GENERATE_OPTIONS_ABI_VERSION 1u

/**
 * @brief Versioned call-scoped controls for checked obstacle generators.
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_generate_options {
    uint32_t struct_size;
    uint32_t abi_version;
    obstacle_raster_rule_t raster_rule;
    uint32_t reserved0;
    uint64_t seed;
    uint64_t max_cells;
    obstacle_generate_cancel_func cancel_func;
    void* cancel_userdata;
} obstacle_generate_options_t;

/** @brief Enclosure aperture side. */
typedef enum e_obstacle_enclosure_side {
    OBSTACLE_ENCLOSURE_CLOSED = 0,
    OBSTACLE_ENCLOSURE_OPEN_RIGHT = 1,
    OBSTACLE_ENCLOSURE_OPEN_UP = 2,
    OBSTACLE_ENCLOSURE_OPEN_LEFT = 3,
    OBSTACLE_ENCLOSURE_OPEN_DOWN = 4
} obstacle_enclosure_side_t;

/** @brief Square spiral rotation direction in the +Y-down grid. */
typedef enum e_obstacle_spiral_direction {
    OBSTACLE_SPIRAL_CLOCKWISE = 0,
    OBSTACLE_SPIRAL_COUNTER_CLOCKWISE = 1
} obstacle_spiral_direction_t;

/** @brief Square spiral max-radius clipping policy. */
typedef enum e_obstacle_spiral_clip_rule {
    OBSTACLE_SPIRAL_CLIP_PATH_ONLY = 0,
    OBSTACLE_SPIRAL_CLIP_OUTPUT = 1
} obstacle_spiral_clip_rule_t;

#define OBSTACLE_ENCLOSURE_DESC_ABI_VERSION UINT32_C(1)
#define OBSTACLE_CROSS_DESC_ABI_VERSION UINT32_C(1)
#define OBSTACLE_SPIRAL_DESC_ABI_VERSION UINT32_C(1)

/**
 * @brief Versioned rectangular enclosure descriptor.
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_enclosure_desc {
    uint32_t struct_size;
    uint32_t abi_version;
    int32_t x0;
    int32_t y0;
    int32_t width;
    int32_t height;
    uint32_t wall_thickness_cells;
    obstacle_enclosure_side_t open_side;
    uint32_t aperture_offset_cells;
    uint32_t aperture_length_cells;
} obstacle_enclosure_desc_t;

/**
 * @brief Versioned four-arm cross descriptor.
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_cross_desc {
    uint32_t struct_size;
    uint32_t abi_version;
    coord_t center;
    uint32_t arm_length_cells;
    uint32_t radius_cells;
    uint32_t reserved0;
    uint32_t reserved1;
} obstacle_cross_desc_t;

/**
 * @brief Versioned square spiral descriptor.
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_spiral_desc {
    uint32_t struct_size;
    uint32_t abi_version;
    coord_t center;
    uint32_t max_radius_cells;
    uint32_t pitch_cells;
    uint32_t path_radius_cells;
    obstacle_spiral_direction_t direction;
    obstacle_spiral_clip_rule_t clip_rule;
    uint32_t reserved0;
} obstacle_spiral_desc_t;

/**
 * @brief Generator options를 ABI v1 default로 초기화한다.
 * @param[out] options 초기화할 caller storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable options false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:options-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_generate_options_init(
    obstacle_generate_options_t* options);

/**
 * @brief Half-open rectangle의 모든 cell을 block한 obstacle을 생성한다.
 *
 * width 또는 height 0은 valid empty다. 모든 실패에서 *out_obstacle은 NULL이다.
 * @param[in] x0 Rectangle origin X cell.
 * @param[in] y0 Rectangle origin Y cell.
 * @param[in] width 0 이상의 X cell 수.
 * @param[in] height 0 이상의 Y cell 수.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_filled_rect(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Half-open rectangle 안쪽에 일정 두께의 closed outline을 생성한다.
 *
 * wall_thickness_cells는 1 이상이며 각 축에는 wall 양쪽과 interior 1칸 이상이
 * 있어야 한다. Raster rule은 explicit rectangle cell wall에는 영향을 주지 않는다.
 * 모든 실패에서 *out_obstacle은 NULL이다.
 * @param[in] x0 Rectangle origin X cell.
 * @param[in] y0 Rectangle origin Y cell.
 * @param[in] width 양쪽 wall과 interior를 포함하는 X cell 수.
 * @param[in] height 양쪽 wall과 interior를 포함하는 Y cell 수.
 * @param[in] wall_thickness_cells 안쪽으로 쌓는 wall 두께.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_rect_outline(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    uint32_t wall_thickness_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief SplitMix64-v1 seed로 random half-open rectangle을 생성한다.
 * @param[in] x0 Rectangle origin X cell.
 * @param[in] y0 Rectangle origin Y cell.
 * @param[in] width 0 이상의 X cell 수.
 * @param[in] height 0 이상의 Y cell 수.
 * @param[in] blocked_probability [0,1]의 finite binary64 probability.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_random_rect(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    double blocked_probability,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Inclusive symmetric centerline과 Chebyshev radius를 rasterize한다.
 * @param[in] start 포함되는 시작 cell.
 * @param[in] end 포함되는 끝 cell.
 * @param[in] radius_cells Chebyshev dilation radius.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable start false
 * @byul.nullable end false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_line(
    const coord_t* start, const coord_t* end, uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Contiguous ring을 explicit fill rule로 rasterize한다.
 * @param[in] vertices 마지막 vertex를 반복하지 않는 ring storage.
 * @param[in] vertex_count vertices element 수.
 * @param[in] fill_rule Even-odd 또는 non-zero winding rule.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable vertices false
 * @byul.buffer vertices
 * @byul.count vertex_count vertices
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_polygon(
    const coord_t* vertices, size_t vertex_count,
    obstacle_polygon_fill_rule_t fill_rule,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Contiguous ring boundary를 symmetric line과 Chebyshev radius로 rasterize한다.
 * @param[in] vertices 마지막 vertex를 반복하지 않는 ring storage.
 * @param[in] vertex_count vertices element 수.
 * @param[in] radius_cells Boundary Chebyshev dilation radius.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable vertices false
 * @byul.buffer vertices
 * @byul.count vertex_count vertices
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_polygon_outline(
    const coord_t* vertices, size_t vertex_count, uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief 세 vertex를 explicit fill rule로 채우는 polygon convenience API다.
 * @param[in] a 첫 vertex.
 * @param[in] b 둘째 vertex.
 * @param[in] c 셋째 vertex.
 * @param[in] fill_rule Even-odd 또는 non-zero winding rule.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값. Collinear triangle은 invalid다.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable c false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_triangle(
    const coord_t* a, const coord_t* b, const coord_t* c,
    obstacle_polygon_fill_rule_t fill_rule,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief 세 vertex boundary를 symmetric line과 Chebyshev radius로 rasterize한다.
 * @param[in] a 첫 vertex.
 * @param[in] b 둘째 vertex.
 * @param[in] c 셋째 vertex.
 * @param[in] radius_cells Boundary Chebyshev dilation radius.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값. Collinear triangle은 invalid다.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable c false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_triangle_outline(
    const coord_t* a, const coord_t* b, const coord_t* c,
    uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Enclosure descriptor를 ABI v1 closed 3x3 enclosure로 초기화한다.
 * @param[out] desc 초기화할 caller storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:desc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_enclosure_desc_init(
    obstacle_enclosure_desc_t* desc);

/**
 * @brief Cross descriptor를 ABI v1 center-only cross로 초기화한다.
 * @param[out] desc 초기화할 caller storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:desc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_cross_desc_init(
    obstacle_cross_desc_t* desc);

/**
 * @brief Spiral descriptor를 ABI v1 center-only clockwise spiral로 초기화한다.
 * @param[out] desc 초기화할 caller storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:desc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_spiral_desc_init(
    obstacle_spiral_desc_t* desc);

/**
 * @brief Inward wall과 optional aperture를 가진 enclosure를 생성한다.
 *
 * UP/DOWN aperture offset은 +X, LEFT/RIGHT offset은 +Y 기준이다. Aperture는
 * corner overlap보다 우선한다. 모든 실패에서 *out_obstacle은 NULL이다.
 * @param[in] desc Versioned enclosure descriptor.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_enclosure(
    const obstacle_enclosure_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief 두 inclusive axis line의 합집합인 cross를 생성한다.
 * @param[in] desc Versioned cross descriptor.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_cross(
    const obstacle_cross_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief Max radius와 pitch가 명시된 square spiral을 생성한다.
 * @param[in] desc Versioned spiral descriptor.
 * @param[in] options Optional call controls 또는 NULL.
 * @param[out] out_obstacle caller-owned 결과를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable desc false
 * @byul.nullable options true
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-always
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 */
BYUL_API navsys_status_t obstacle_generate_spiral(
    const obstacle_spiral_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle);

/**
 * @brief 직사각형 extent 전체가 blocked인 obstacle을 생성한다.
 * @param[in] x0 origin X 좌표.
 * @param[in] y0 origin Y 좌표.
 * @param[in] width X축 extent.
 * @param[in] height Y축 extent.
 * @return caller-owned obstacle 또는 실패 시 NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_filled_rect; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_rect_all_blocked(
    int x0, int y0, int width, int height);

/**
 * @brief 지정 비율의 좌표를 무작위로 block한 직사각형 obstacle을 생성한다.
 * @param[in] x0 origin X 좌표.
 * @param[in] y0 origin Y 좌표.
 * @param[in] width X축 extent.
 * @param[in] height Y축 extent.
 * @param[in] ratio blocked 비율(0.0~1.0).
 * @return caller-owned obstacle 또는 실패 시 NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_random_rect after selecting an explicit seed; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio);

/**
 * @brief start와 goal 사이의 선을 주어진 range 두께로 block한다.
 * @param[in] start 시작 좌표.
 * @param[in] goal 끝 좌표.
 * @param[in] range 선 주위의 두께 범위.
 * @return caller-owned obstacle 또는 실패 시 NULL.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_line after reviewing the radius contract; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range);

/**
 * @brief Creates a torus (donut)-shaped obstacle.
 *
 * This function creates a ring-shaped (torus) obstacle within a rectangular
 * area, leaving the inner part empty while blocking the outer boundary.
 * The inner and outer areas are completely separated with no path.
 * 
 * @note Minimum size:
 *       - width >= thickness × 2 + 1
 *       - height >= thickness × 2 + 1
 *       If smaller, there will be no inner space to form a donut shape.
 *
 * @note If `thickness` is too large and no inner space can be formed,
 *       the function returns NULL.
 *
 * @param[in] start      One corner coordinate of the rectangular area
 * @param[in] goal       Opposite corner coordinate
 * @param[in] thickness  Thickness of the boundary wall (>=1)
 *                   A larger value creates a thicker outer ring
 *
 * @return A pointer to the torus-shaped obstacle on success, or NULL on failure
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_rect_outline; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_torus(
    const coord_t* start, const coord_t* goal, int thickness);

/**
 * @deprecated Use obstacle_enclosure_side_t and OBSTACLE_ENCLOSURE_* names.
 * Numeric values remain source-compatible; removal requires a future ABI-major.
 */
typedef enum e_enclosure_open_dir {
    ENCLOSURE_OPEN_UNKNOWN,
    ENCLOSURE_OPEN_RIGHT,
    ENCLOSURE_OPEN_UP,
    ENCLOSURE_OPEN_LEFT,
    ENCLOSURE_OPEN_DOWN,
} enclosure_open_dir_t;

/**
 * @brief Creates a rectangular enclosure obstacle with one open side.
 *
 * This function creates a "pot" or "U-shaped" obstacle by leaving one of
 * the four sides of a rectangular boundary open. Useful for creating 
 * structures where a player or NPC can enter or exit.
 *
 * @note Minimum size:
 *       - width >= thickness × 2 + 1
 *       - height >= thickness × 2 + 1
 *       If there is no space for walls, creation may fail.
 * 
 * The `open` argument specifies the open direction.
 * If ENCLOSURE_OPEN_UNKNOWN is passed, all sides are closed.
 *
 * @param[in] start      One corner coordinate of the rectangular area
 * @param[in] goal       Opposite corner coordinate
 * @param[in] thickness  Wall thickness (>=1)
 *                   Larger values create thicker walls
 * @param[in] open       Direction to be left open (up, down, left, right)
 *
 * @return A pointer to the enclosure obstacle on success, or NULL on failure
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_enclosure after reviewing the aperture contract; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_enclosure(
    const coord_t* start, const coord_t* goal, int thickness, 
    enclosure_open_dir_t open);

/**
 * @brief Creates a cross (+)-shaped obstacle centered at a given coordinate.
 *
 * This function creates a cross shape extending `length` units
 * from `center` in four directions. The thickness of each arm is 
 * defined by `range`.
 *
 * @param[in] center    Center coordinate of the cross
 * @param[in] length    Length of each arm (0 means only the center point)
 * @param[in] range     Width of each arm (0 means only center, >=1 includes neighbors)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable center false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_cross; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_cross(
    const coord_t* center, int length, int range);

/**
 * @deprecated Use obstacle_spiral_direction_t and OBSTACLE_SPIRAL_* names.
 * Numeric values remain source-compatible; removal requires a future ABI-major.
 */
typedef enum e_spiral_dir {
    SPIRAL_CLOCKWISE,        ///< Clockwise (default)
    SPIRAL_COUNTER_CLOCKWISE ///< Counterclockwise
} spiral_dir_t;

/**
 * @brief Creates a spiral-shaped obstacle centered around a coordinate.
 *
 * This function creates a grid-based square spiral structure, blocking
 * cells along a path that rotates clockwise or counterclockwise.
 * The number of rotations (`turns`) determines the total spiral length.
 * Setting `gap` adds spacing between rotations to create open areas.
 *
 * `range` defines the thickness of blocked cells around the spiral path.
 *
 * @param[in] center    Center coordinate of the spiral
 * @param[in] radius    Maximum spiral radius (in grid distance)
 * @param[in] turns     Total number of rotations (1 rotation = 4 directional turns)
 * @param[in] range     Path radius (0 blocks only the path center, >=1 includes area)
 * @param[in] gap       Spacing between rotations (0 means continuous)
 * @param[in] direction Rotation direction (SPIRAL_CLOCKWISE or SPIRAL_COUNTER_CLOCKWISE)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable center false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 *
 * @note Larger `gap` creates more spacing between rotations,
 *       larger `range` creates thicker obstacles.
 */
BYUL_DEPRECATED("Use obstacle_generate_spiral after reviewing pitch and clipping; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_spiral(
    const coord_t* center,
    int radius,
    int turns,
    int range,
    int gap,
    spiral_dir_t direction
);

/**
 * @brief Creates a triangle-shaped obstacle that blocks the area inside.
 *
 * This function blocks the grid area defined by the three vertices `a`, `b`, `c`.
 *
 * @param[in] a  First vertex
 * @param[in] b  Second vertex
 * @param[in] c  Third vertex
 *
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable c false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_triangle after selecting an explicit fill rule; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_triangle(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c);

/**
 * @brief Creates a triangle torus obstacle, blocking only the outer boundary.
 *
 * This function follows the edges of the triangle defined by `a`, `b`, `c`
 * and blocks the boundary line, leaving the interior unblocked.
 * If `thickness` >= 1, the boundary lines are thickened accordingly.
 *
 * @param[in] a         Triangle vertex A
 * @param[in] b         Triangle vertex B
 * @param[in] c         Triangle vertex C
 * @param[in] thickness Boundary line thickness (0 = line only, >=1 includes area)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable c false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_triangle_outline after reviewing radius semantics; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_triangle_torus(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c,
    int thickness);

/**
 * @brief Creates a polygon-shaped obstacle that blocks the inside area.
 *
 * This function connects the coordinates in the list to form a closed shape
 * and blocks the inside area. At least 3 coordinates are required.
 * The polygon is considered closed automatically (last -> first).
 *
 * @param[in] list  Polygon vertex list (coord_list_t*)
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_polygon with contiguous vertices and an explicit fill rule; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_polygon(coord_list_t* list);

/**
 * @brief Creates a polygon torus obstacle, blocking only the boundary.
 *
 * This function connects the coordinates in the list to form a closed shape,
 * and blocks along the boundary lines only, leaving the inside unblocked.
 * If `thickness` >= 1, the boundary thickness can be expanded.
 *
 * @param[in] list       Polygon vertex list (at least 3 points)
 * @param[in] thickness  Boundary thickness (0 = line only, >=1 includes area)
 * @return A pointer to the created obstacle, or NULL on failure
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_generate_polygon_outline after reviewing radius semantics; removal requires a future ABI-major.")
BYUL_API obstacle_t* obstacle_make_polygon_torus(
    coord_list_t* list, int thickness);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_OBSTACLE_H */
