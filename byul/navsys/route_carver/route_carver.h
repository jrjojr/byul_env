/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file route_carver.h
 * @brief Navgrid 통행 가능 영역을 만드는 public C ABI를 선언한다.
 *
 * 결정적인 line/area cell geometry, blocked 선택 policy, cooperative cancellation과
 * atomic 또는 chunked mutation 결과를 status와 changed count로 제공한다.
 */

#ifndef BYUL_ROUTE_CARVER_H
#define BYUL_ROUTE_CARVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "navgrid.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Carve radius의 cell metric이다. */
typedef enum e_navgrid_carve_metric {
    NAVGRID_CARVE_CHEBYSHEV_SQUARE = 0,
    NAVGRID_CARVE_MANHATTAN_DIAMOND = 1,
    NAVGRID_CARVE_EUCLIDEAN_DISK = 2
} navgrid_carve_metric_t;

/** @brief Line이 통과한다고 판정할 cell coverage다. */
typedef enum e_navgrid_line_coverage {
    NAVGRID_LINE_CENTER_CELLS = 0,
    NAVGRID_LINE_SUPERCOVER_CELLS = 1
} navgrid_line_coverage_t;

/** @brief Mutation 대상으로 선택할 blocked 상태다. */
typedef enum e_navgrid_carve_match {
    NAVGRID_CARVE_STORED_FORBIDDEN_ONLY = 0,
    NAVGRID_CARVE_EFFECTIVE_BLOCKED = 1
} navgrid_carve_match_t;

#define NAVGRID_CARVE_INCLUDE_START UINT32_C(0x00000001)
#define NAVGRID_CARVE_INCLUDE_END UINT32_C(0x00000002)
#define NAVGRID_CARVE_CLIP_TO_EXTENT UINT32_C(0x00000004)
#define NAVGRID_CARVE_ATOMIC UINT32_C(0x00000008)
#define NAVGRID_CARVE_DRY_RUN UINT32_C(0x00000010)

#define NAVGRID_CARVE_OPTIONS_ABI_VERSION UINT32_C(1)
#define NAVGRID_CARVE_MAX_CELLS UINT64_C(262144)
#define NAVGRID_CARVE_CANCEL_POLL_INTERVAL_CELLS UINT32_C(64)

/**
 * @brief 한 carve 호출의 cooperative cancellation callback이다.
 *
 * 호출을 실행한 thread에서 동기 호출된다. true는 취소 요청이고 false는 계속 진행이다.
 * Callback과 userdata는 호출 동안만 borrow되며 저장되지 않는다. 같은 navgrid를 변경,
 * 파괴하거나 carve API에 재진입하지 않아야 한다.
 *
 * @return 현재 호출을 취소하려면 true.
 */
typedef bool (*navgrid_carve_cancel_func)(void* userdata);

/**
 * @brief Versioned carve geometry, mutation과 실행 control이다.
 *
 * struct_size는 sizeof(navgrid_carve_options_t), abi_version은
 * NAVGRID_CARVE_OPTIONS_ABI_VERSION으로 설정한다. max_cells는 1 이상
 * NAVGRID_CARVE_MAX_CELLS 이하여야 하며 reserved0은 0이어야 한다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_navgrid_carve_options {
    uint32_t struct_size; /**< 이 value struct의 byte 크기. */
    uint32_t abi_version; /**< NAVGRID_CARVE_OPTIONS_ABI_VERSION. */
    uint32_t radius_cells; /**< Cell 단위의 unsigned dilation radius. */
    uint32_t metric; /**< navgrid_carve_metric_t 값. */
    uint32_t line_coverage; /**< navgrid_line_coverage_t 값. */
    uint32_t match; /**< navgrid_carve_match_t 값. */
    uint32_t flags; /**< NAVGRID_CARVE_* flag의 bitwise OR. */
    uint32_t reserved0; /**< 미래 확장용이며 반드시 0. */
    uint64_t max_cells; /**< 허용할 최대 unique candidate 수. */
    navgrid_carve_cancel_func cancel_func; /**< NULL 가능 call-scoped callback. */
    void* cancel_userdata; /**< Native가 역참조하지 않는 callback userdata. */
} navgrid_carve_options_t;

/**
 * @brief Line footprint의 선택된 blocked cell을 통행 가능하게 만든다.
 *
 * Candidate는 y,x row-major unique 순서다. Callback은 첫 열거 전, 열거/확장 중 최대
 * NAVGRID_CARVE_CANCEL_POLL_INTERVAL_CELLS 간격, mutation 준비 전과 각 commit chunk에서
 * poll된다. ATOMIC이면 모든 준비가 성공한 뒤 한 번에 commit하며 실패 시 grid와 output을
 * 보존한다. ATOMIC이 없으면 64-cell chunk별로 commit하며 실패 후 out_changed_count는 이미
 * 바뀐 실제 cell 수다. 같은 호출을 재시도하면 이미 열린 cell은 세지 않고 같은 순서로
 * 나머지를 처리한다. DRY_RUN은 mutation 없이 같은 changed count를 계산한다.
 *
 * @param[in,out] navgrid 대상 grid.
 * @param[in] start Line 시작 cell center.
 * @param[in] goal Line 끝 cell center.
 * @param[in] options 필수 versioned options.
 * @param[out] out_changed_count 성공 시 실제 effective state 변경 수. Non-atomic partial
 * failure에서는 이미 commit된 수이며, 그 밖의 실패에서는 기존 값이 보존된다.
 * @return NAVSYS_STATUS_OK 또는 구체적인 음수 navsys_status_t.
 * @retval NAVSYS_STATUS_CANCELLED Callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED Callback이 C++ exception을 던졌다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED Raster 또는 candidate 상한을 넘었다.
 * @byul.nullable navgrid false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable options false
 * @byul.nullable out_changed_count false
 * @byul.side_effect mutates:navgrid,out_changed_count-on-success-or-partial
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API navsys_status_t navgrid_carve_line(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    const navgrid_carve_options_t* options,
    size_t* out_changed_count);

/**
 * @brief Center 주변 metric footprint의 선택된 blocked cell을 통행 가능하게 만든다.
 *
 * Line endpoint flag는 허용되지 않고 line_coverage는 NAVGRID_LINE_CENTER_CELLS여야 한다.
 * Status, cancellation, atomic/partial count와 deterministic retry 계약은
 * navgrid_carve_line()과 같다.
 *
 * @param[in,out] navgrid 대상 grid.
 * @param[in] center Area 중심 cell center.
 * @param[in] options 필수 versioned options.
 * @param[out] out_changed_count 성공 또는 non-atomic partial commit의 실제 변경 수.
 * @return NAVSYS_STATUS_OK 또는 구체적인 음수 navsys_status_t.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.nullable options false
 * @byul.nullable out_changed_count false
 * @byul.side_effect mutates:navgrid,out_changed_count-on-success-or-partial
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API navsys_status_t navgrid_carve_area(
    navgrid_t* navgrid,
    const coord_t* center,
    const navgrid_carve_options_t* options,
    size_t* out_changed_count);

/**
 * @brief Legacy footprint로 직선 통로를 만든다.
 * @param[in,out] navgrid 대상 grid.
 * @param[in] start 시작 좌표.
 * @param[in] goal 끝 좌표.
 * @param[in] range Legacy 범위 값.
 * @return 제거됐다고 보고한 legacy obstacle 수이며 실패 정보는 0으로 축약된다.
 * @byul.nullable navgrid false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @deprecated Use navgrid_carve_line; removal requires ABI 2 or later.
 */
BYUL_DEPRECATED("Use navgrid_carve_line; removal requires ABI 2 or later.")
BYUL_API int route_carve_beam(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    int range);

/**
 * @brief Legacy footprint로 center 주변을 연다.
 * @param[in,out] navgrid 대상 grid.
 * @param[in] center 중심 좌표.
 * @param[in] range Legacy 범위 값.
 * @return 제거됐다고 보고한 legacy obstacle 수이며 실패 정보는 0으로 축약된다.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @deprecated Use navgrid_carve_area; removal requires ABI 2 or later.
 */
BYUL_DEPRECATED("Use navgrid_carve_area; removal requires ABI 2 or later.")
BYUL_API int route_carve_bomb(
    navgrid_t* navgrid,
    const coord_t* center,
    int range);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_ROUTE_CARVER_H */
