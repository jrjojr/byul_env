/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file navgrid.h
 * @brief Opaque navigation-grid handle and its public C ABI.
 *
 * Declares grid construction, cell and obstacle mutation, callback binding,
 * deterministic neighbor queries, and ABI compatibility inspection.
 */

#ifndef BYUL_NAVGRID_H
#define BYUL_NAVGRID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "navsys_status.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "navcell.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function pointer to check if a coordinate is blocked.
 *
 * This function determines whether a specific coordinate `(x, y)`
 * is an impassable cell for pathfinding or range operations.
 *
 * @param context External data required for coordinate checking (
 * e.g., navgrid, map)
 * @param x X coordinate to check
 * @param y Y coordinate to check
 * @param userdata Optional user-defined data
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 */
typedef bool (*is_coord_blocked_func)(
    const void* context, int x, int y, void* userdata);

/**
 * @brief Checks if a coordinate is blocked based on navgrid.
 *
 * Determines whether a coordinate is impassable due to walls or obstacles
 * using the internal cell information of the navgrid.
 *
 * @param[in] context Pointer to navgrid object (const navgrid_t*)
 * @param[in] x X coordinate to check
 * @param[in] y Y coordinate to check
 * @param[in] userdata Optional user-defined data (can be unused)
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 * @byul.nullable context false
 * @byul.nullable userdata true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool is_coord_blocked_navgrid(
    const void* context, int x, int y, void* userdata);

typedef enum {
    NAVGRID_DIR_4,
    NAVGRID_DIR_8
} navgrid_dir_mode_t;

/** @brief 내부 표현을 노출하지 않는 navigation grid resource handle이다. */
typedef struct s_navgrid navgrid_t;

#define BYUL_NAVGRID_ABI_VERSION UINT32_C(2)
#define BYUL_NAVGRID_ABI_FINGERPRINT UINT64_C(0x4e4752494402002f)

/** @brief Header와 runtime Navgrid ABI가 일치하지 않는 원인을 구분한다. */
typedef enum e_navgrid_abi_mismatch {
    NAVGRID_ABI_MATCH = 0,
    NAVGRID_ABI_VERSION_MISMATCH = 1,
    NAVGRID_ABI_FINGERPRINT_MISMATCH = 2
} navgrid_abi_mismatch_t;

/** @brief Grid 생존 기간에만 유효한 blocked overlay 식별자다. */
typedef uint64_t navgrid_overlay_id_t;

/**
 * @brief Materialized Navgrid 좌표의 base cell과 built-in blocked 상태 snapshot이다.
 *
 * present가 false이면 cell은 implicit NORMAL/0이고 좌표는 overlay 때문에 materialize됐다.
 * blocked는 custom callback이 아니라 base FORBIDDEN과 overlay를 합친 built-in 상태다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_navgrid_cell_entry {
    coord_t coord;
    navcell_t cell;
    bool present;
    bool blocked;
} navgrid_cell_entry_t;

/**
 * @brief 현재 runtime이 제공하는 canonical Navgrid ABI version을 반환한다.
 *
 * @return =BYUL_NAVGRID_ABI_VERSION=과 대응하는 runtime version.
 *
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint32_t navgrid_get_abi_version(void);

/**
 * @brief 현재 runtime의 canonical Navgrid ABI fingerprint를 반환한다.
 *
 * @return =BYUL_NAVGRID_ABI_FINGERPRINT=와 대응하는 runtime fingerprint.
 *
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint64_t navgrid_get_abi_fingerprint(void);

/**
 * @brief Header/package가 요구하는 Navgrid ABI를 runtime과 비교한다.
 *
 * ABI 2 opaque handle과 별도 compatibility package의 ABI 1 layout fingerprint를
 * 인식한다. 알려진 version의 fingerprint가 다르면 fingerprint mismatch로, 지원하지
 * 않는 version이면 version mismatch로 진단한다.
 *
 * @param[in] expected_version caller header가 요구하는 ABI version.
 * @param[in] expected_fingerprint caller header가 요구하는 ABI fingerprint.
 * @param[out] out_mismatch 일치 여부 또는 mismatch 원인.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK version과 fingerprint가 지원 계약과 일치한다.
 * @retval NAVSYS_STATUS_UNSUPPORTED version 또는 fingerprint가 일치하지 않는다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_mismatch가 NULL이다.
 *
 * @byul.nullable out_mismatch false
 * @byul.side_effect writes:out_mismatch
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t navgrid_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    navgrid_abi_mismatch_t* out_mismatch);

// Constructors and Destructors

/**
 * @brief Creates a navigation grid with default parameters.
 *
 * This function initializes a `navgrid_t` object with 
 * the following default settings:
 * - Grid size: `0 × 0` (interpreted as **infinite** width and height)
 * - Direction mode: `NAVGRID_DIR_8` (8-way movement)
 * - No obstacle-checking function (all coordinates considered traversable)
 *
 * A size of `0 × 0` indicates an **unbounded grid**, 
 * which may be useful for procedural or open-world environments.
 * However, infinite grids can lead to unbounded node expansion 
 * in some pathfinding algorithms.
 *
 * @warning Algorithms without heuristic guidance 
 * (e.g., BFS, DFS, Fringe Search) may enter infinite exploration
 *          on an unbounded grid unless a retry limit is enforced. 
 *          The system uses `MAX_RETRY` as a safeguard (default: 1000).
 *
 * @return Pointer to a newly allocated `navgrid_t` instance, 
 *          or `NULL` on failure.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return navgrid_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navgrid_t* navgrid_create();

/**
 * @brief Creates a new navigation grid with custom dimensions and settings.
 *
 * This function allocates and initializes a `navgrid_t` object 
 * using the specified
 * width, height, and direction mode. 
 * It also allows the caller to optionally provide
 * a custom function to determine whether a given coordinate is blocked.
 *
 * @param[in] width               Grid width (number of columns)
 * @param[in] height              Grid height (number of rows)
 * @param[in] mode                Directional mode (NAVGRID_DIR_4 or NAVGRID_DIR_8)
 * @param[in] is_coord_blocked_fn Optional user-defined function to determine
 *      if a coordinate is blocked.
 *      If NULL, all coordinates are assumed walkable by default.
 *
 * @return A pointer to the newly created navgrid_t object, 
 * or NULL on allocation failure.
 * @byul.nullable is_coord_blocked_fn true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return navgrid_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navgrid_t* navgrid_create_full(int width, int height, 
    navgrid_dir_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

/**
 * @brief Navigation grid와 그 내부 cell map을 해제한다.
 *
 * 같은 grid의 차단 callback 실행 중 호출하면 안전을 위해 아무 작업도 하지 않는다.
 *
 * @param[in,out] navgrid 해제할 navigation grid. NULL이면 아무 작업도 하지 않는다.
 *
 * @byul.nullable navgrid true
 * @byul.side_effect frees:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API void navgrid_destroy(navgrid_t* navgrid);

// Copy and Comparison
/**
 * @brief Allocates an independent copy of a navigation grid.
 * @param[in] navgrid Grid to copy.
 * @return Caller-owned copy, or NULL for invalid input or allocation failure.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return navgrid_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navgrid_t* navgrid_copy(const navgrid_t* navgrid);

/**
 * @brief Computes a stable hash of the current grid value.
 * @param[in] navgrid Grid to inspect.
 * @return Grid hash, or 0 when navgrid is NULL.
 * @byul.nullable navgrid true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API uint32_t navgrid_hash(const navgrid_t* navgrid);

/**
 * @brief Compares two navigation grids by value.
 * @param[in] a First grid.
 * @param[in] b Second grid.
 * @return true when both grids have equal values; otherwise false.
 * @byul.nullable a true
 * @byul.nullable b true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool navgrid_equal(const navgrid_t* a, const navgrid_t* b);

// Property Access
/**
 * @brief Returns the configured grid width.
 * @param[in] navgrid Grid to inspect.
 * @return Configured width, or 0 for NULL.
 * @byul.nullable navgrid true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API int navgrid_get_width(const navgrid_t* navgrid);

/**
 * @brief Replaces the configured grid width.
 * @param[in,out] navgrid Grid to update.
 * @param[in] width New width; 0 denotes an unbounded dimension.
 * @byul.nullable navgrid true
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void navgrid_set_width(navgrid_t* navgrid, int width);

/**
 * @brief Returns the configured grid height.
 * @param[in] navgrid Grid to inspect.
 * @return Configured height, or 0 for NULL.
 * @byul.nullable navgrid true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API int navgrid_get_height(const navgrid_t* navgrid);

/**
 * @brief Replaces the configured grid height.
 * @param[in,out] navgrid Grid to update.
 * @param[in] height New height; 0 denotes an unbounded dimension.
 * @byul.nullable navgrid true
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void navgrid_set_height(navgrid_t* navgrid, int height);

/**
 * @brief Replaces the obstacle callback while preserving legacy userdata behavior.
 * @param[in,out] navgrid Grid to update.
 * @param[in] fn Callback to bind; NULL clears the callback.
 * @byul.nullable navgrid true
 * @byul.nullable fn true
 * @byul.lifetime fn until-rebind
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void navgrid_set_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn);

/**
 * @brief Returns the currently bound obstacle callback.
 * @param[in] navgrid Grid to inspect.
 * @return Borrowed callback, or NULL when unbound or navgrid is NULL.
 * @byul.nullable navgrid true
 * @byul.nullable return true
 * @byul.lifetime return until-rebind
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API is_coord_blocked_func navgrid_get_is_coord_blocked_fn(
    const navgrid_t* navgrid);

/**
 * @brief callback과 userdata binding을 하나의 snapshot으로 조회한다.
 *
 * @param[in] navgrid 조회할 navigation grid.
 * @param[out] out_fn 현재 callback. unbound 상태에서는 NULL이다.
 * @param[out] out_userdata 현재 caller-owned userdata. unbound 상태에서는 NULL이다.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 두 output에 동일 시점의 binding을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 입력 또는 output pointer가 NULL이다.
 *
 * @byul.nullable navgrid false
 * @byul.nullable out_fn false
 * @byul.nullable out_userdata false
 * @byul.lifetime out_fn until-rebind
 * @byul.lifetime out_userdata until-rebind
 * @byul.side_effect writes:out_fn,out_userdata
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t navgrid_fetch_is_coord_blocked_binding(
    const navgrid_t* navgrid,
    is_coord_blocked_func* out_fn,
    void** out_userdata);

/**
 * @brief 좌표 차단 callback과 userdata를 하나의 binding으로 교체한다.
 *
 * @param[in,out] navgrid 변경할 navigation grid.
 * @param[in] fn bind할 callback.
 * @param[in] userdata callback에 전달할 caller 소유 data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK binding이 교체됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT navgrid 또는 fn이 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 grid의 callback 실행 중이다.
 *
 * @byul.nullable navgrid false
 * @byul.nullable fn false
 * @byul.nullable userdata true
 * @byul.lifetime fn until-unbind
 * @byul.lifetime userdata until-unbind
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t navgrid_bind_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn, void* userdata);

/**
 * @brief 좌표 차단 callback binding을 제거해 모든 좌표를 통과 가능하게 한다.
 *
 * @param[in,out] navgrid 변경할 navigation grid.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK callback과 userdata가 NULL로 변경됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT navgrid가 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 grid의 callback 실행 중이다.
 *
 * @byul.nullable navgrid false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t navgrid_unbind_is_coord_blocked_func(
    navgrid_t* navgrid);

/**
 * @brief Returns the configured neighbor-direction mode.
 * @param[in] navgrid Grid to inspect.
 * @return Direction mode; NAVGRID_DIR_8 is returned for NULL.
 * @byul.nullable navgrid true
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* navgrid);

/**
 * @brief Replaces the neighbor-direction mode.
 * @param[in,out] navgrid Grid to update.
 * @param[in] mode New direction mode.
 * @byul.nullable navgrid true
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void navgrid_set_mode(
    navgrid_t* navgrid, navgrid_dir_mode_t mode);

// Obstacle Management
/**
 * @brief Adds a coordinate to the built-in blocked set.
 * @param[in,out] navgrid Grid to update.
 * @param[in] x X coordinate.
 * @param[in] y Y coordinate.
 * @return true when the operation succeeds; otherwise false.
 * @byul.nullable navgrid false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool navgrid_block_coord(navgrid_t* navgrid, int x, int y);

/**
 * @brief Removes a coordinate from the built-in blocked set.
 * @param[in,out] navgrid Grid to update.
 * @param[in] x X coordinate.
 * @param[in] y Y coordinate.
 * @return true when the operation succeeds; otherwise false.
 * @byul.nullable navgrid false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool navgrid_unblock_coord(navgrid_t* navgrid, int x, int y);

/**
 * @brief Checks whether a coordinate lies within the configured extent.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x X coordinate.
 * @param[in] y Y coordinate.
 * @return true when the coordinate is inside; otherwise false.
 * @byul.nullable navgrid false
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool navgrid_is_inside(const navgrid_t* navgrid, int x, int y);
// BYUL_API bool navgrid_is_blocked(const navgrid_t* navgrid, int x, int y);

/**
 * @brief Clears all stored cells and built-in blocked coordinates.
 * @param[in,out] navgrid Grid to clear; NULL has no effect.
 * @byul.nullable navgrid true
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void navgrid_clear(navgrid_t* navgrid);

/**
 * @brief Base cell을 덮지 않는 기본 blocked overlay를 설정한다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] x 대상 X 좌표.
 * @param[in] y 대상 Y 좌표.
 * @param[out] out_changed effective blocked 상태가 바뀌었는지 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 grid와 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_changed-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_block_coord_ex(
    navgrid_t* navgrid, int x, int y, bool* out_changed);

/**
 * @brief 기본 blocked overlay만 제거하고 base cell과 다른 overlay를 보존한다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] x 대상 X 좌표.
 * @param[in] y 대상 Y 좌표.
 * @param[out] out_changed effective blocked 상태가 바뀌었는지 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 grid와 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_changed-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_unblock_coord_ex(
    navgrid_t* navgrid, int x, int y, bool* out_changed);

/**
 * @brief 모든 base cell과 blocked overlay를 원자적으로 비운다.
 *
 * Callback binding, extent와 direction mode는 보존한다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[out] out_changed 저장 내용이 있었는지 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable navgrid false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_changed-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_clear_ex(
    navgrid_t* navgrid, bool* out_changed);

// Cell Map Access
/**
 * @brief Stores a validated base cell at a coordinate.
 * @param[in,out] navgrid Grid to update.
 * @param[in] x X coordinate.
 * @param[in] y Y coordinate.
 * @param[in] cell Cell value to copy.
 * @return true on success; otherwise false.
 * @byul.nullable navgrid false
 * @byul.nullable cell false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API bool navgrid_set_cell(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell);

/**
 * @brief Copies a base cell into caller storage.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x X coordinate.
 * @param[in] y Y coordinate.
 * @param[out] out Cell destination.
 * @return Nonzero when a stored cell was copied; otherwise 0.
 * @byul.nullable navgrid false
 * @byul.nullable out false
 * @byul.side_effect mutates:out-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API int navgrid_fetch_cell(
    const navgrid_t* navgrid, int x, int y, navcell_t* out);

/**
 * @brief 검증한 base cell을 failure-atomic하게 저장한다.
 *
 * Absent prior는 NORMAL/0과 out_had_prior=false로 표현한다. Overlay는 보존한다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] x 대상 X 좌표.
 * @param[in] y 대상 Y 좌표.
 * @param[in] cell 복사할 cell 값.
 * @param[out] out_prior 이전 base cell 또는 implicit default를 받을 storage.
 * @param[out] out_had_prior 이전 sparse entry 존재 여부를 받을 storage.
 * @param[out] out_changed base cell이 바뀌었는지 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 grid와 모든 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable cell false
 * @byul.nullable out_prior false
 * @byul.nullable out_had_prior false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_prior,out_had_prior,out_changed-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_set_cell_ex(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell,
    navcell_t* out_prior, bool* out_had_prior, bool* out_changed);

/**
 * @brief Base cell을 조회하고 absent를 implicit NORMAL/0으로 구분해 반환한다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] x 대상 X 좌표.
 * @param[in] y 대상 Y 좌표.
 * @param[out] out_cell stored cell 또는 implicit default를 받을 storage.
 * @param[out] out_present sparse entry 존재 여부를 받을 storage.
 * @return 공통 Navsys 상태 값. Outside/invalid stored cell 실패는 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_cell false
 * @byul.nullable out_present false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_cell,out_present-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_fetch_cell_ex(
    const navgrid_t* navgrid, int x, int y,
    navcell_t* out_cell, bool* out_present);

/**
 * @brief 좌표 집합을 독립 blocked overlay로 원자적으로 적용한다.
 *
 * Base terrain/height와 다른 overlay를 보존한다. 성공한 overlay는 remove 함수로 해제한다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] coords 적용할 좌표 배열. count가 0이면 NULL을 허용한다.
 * @param[in] count coords element 수.
 * @param[out] out_overlay 새 overlay 식별자를 받을 storage.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 grid와 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable coords true
 * @byul.nullable out_overlay false
 * @byul.nullable out_changed_count false
 * @byul.capacity coords count
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_overlay,out_changed_count-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_apply_blocked_overlay(
    navgrid_t* navgrid, const coord_t* coords, size_t count,
    navgrid_overlay_id_t* out_overlay, size_t* out_changed_count);

/**
 * @brief 지정 overlay만 제거하고 base cell과 나머지 overlay를 드러낸다.
 *
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] overlay 같은 grid에서 받은 overlay 식별자.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 storage.
 * @return 공통 Navsys 상태 값. 알 수 없는 token은 NOT_FOUND다.
 * @byul.nullable navgrid false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_changed_count-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_remove_blocked_overlay(
    navgrid_t* navgrid, navgrid_overlay_id_t overlay,
    size_t* out_changed_count);

/**
 * @brief Returns the legacy borrowed sparse cell-map view.
 * @param[in] navgrid Grid to inspect.
 * @return Borrowed map, or NULL when navgrid is NULL.
 * @byul.nullable navgrid true
 * @byul.nullable return true
 * @byul.lifetime return until-mutation
 * @byul.side_effect none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API const coord_hash_t* navgrid_get_cell_map(const navgrid_t* navgrid);

// Neighbor Search

/**
 * @brief 한 칸 이웃을 canonical E/S/W/N 순서로 caller buffer에 내보낸다.
 *
 * 8방향이면 E/SE/S/SW/W/NW/N/NE 순서다. traversable_only가 true이면 bound
 * callback으로 blocked인 좌표를 제외한다. NULL/0 query는 필요한 수만 반환하고 짧은
 * buffer는 보존하면서 INCOMPLETE를 반환한다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] traversable_only blocked 좌표 제외 여부.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element capacity.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값. Callback 실패를 포함한 오류는 output을 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_export_neighbors(
    const navgrid_t* navgrid, int x, int y, bool traversable_only,
    coord_t* out_coords, size_t capacity, size_t* out_count);

/**
 * @brief Legacy all-range와 같은 topology 영역을 결정적 좌표 순서로 내보낸다.
 *
 * 결과는 X, Y 오름차순이며 callback을 호출하지 않는다. NULL/0 query와 짧은 buffer
 * 계약은 navgrid_export_neighbors()와 같다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] range 0 이상의 legacy range 값.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element capacity.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable navgrid false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_export_neighbors_range(
    const navgrid_t* navgrid, int x, int y, int range,
    coord_t* out_coords, size_t capacity, size_t* out_count);

/**
 * @brief 주어진 각도에 가장 가까운 canonical topology 이웃을 복사한다.
 *
 * 0도는 +X이고 양의 각도는 +Y 방향으로 회전한다. Tie는 normalized candidate angle이
 * 작은 좌표를 선택하며 blocked callback은 적용하지 않는다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] degree 유한한 degree 각도.
 * @param[out] out_coord 선택된 좌표를 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 out_coord를 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_fetch_neighbor_at_degree(
    const navgrid_t* navgrid, int x, int y,
    double degree, coord_t* out_coord);

/**
 * @brief center에서 goal 방향에 가장 가까운 canonical topology 이웃을 복사한다.
 *
 * center와 goal이 같으면 INVALID_ARGUMENT다. Blocked callback은 적용하지 않는다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] center 중심 좌표.
 * @param[in] goal 방향을 정할 목표 좌표.
 * @param[out] out_coord 선택된 좌표를 받을 storage.
 * @return 공통 Navsys 상태 값. 실패하면 out_coord를 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_fetch_neighbor_at_goal(
    const navgrid_t* navgrid, const coord_t* center,
    const coord_t* goal, coord_t* out_coord);

/**
 * @brief Goal 상대 각도 구간과 square range 안의 좌표를 결정적으로 내보낸다.
 *
 * 결과는 X, Y 오름차순이며 center를 제외한다. NULL/0 query와 짧은 buffer 계약은
 * navgrid_export_neighbors()와 같다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[in] center 중심 좌표.
 * @param[in] goal 기준 방향을 정할 목표 좌표.
 * @param[in] start_deg 포함되는 시작 상대 각도.
 * @param[in] end_deg 포함되는 끝 상대 각도.
 * @param[in] range 0 이상의 square radius.
 * @param[out] out_coords 좌표 buffer 또는 count query의 NULL.
 * @param[in] capacity out_coords의 coord_t element capacity.
 * @param[out] out_count 필요한 전체 좌표 수.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_coords-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_export_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg, int range,
    coord_t* out_coords, size_t capacity, size_t* out_count);

/**
 * @brief Base cell과 overlay로 materialize된 좌표 snapshot을 내보낸다.
 *
 * Base map과 overlay 좌표의 합집합을 X, Y 오름차순으로 반환한다. Custom blocked
 * callback이 임의로 만드는 좌표는 열거하지 않는다. NULL/0 query와 짧은 buffer 계약은
 * navgrid_export_neighbors()와 같다.
 *
 * @param[in] navgrid 조회할 grid.
 * @param[out] out_entries entry buffer 또는 count query의 NULL.
 * @param[in] capacity out_entries의 navgrid_cell_entry_t element capacity.
 * @param[out] out_count 필요한 전체 entry 수.
 * @return 공통 Navsys 상태 값. Corrupt base cell이면 buffer와 count를 보존한다.
 * @byul.nullable navgrid false
 * @byul.nullable out_entries true
 * @byul.nullable out_count false
 * @byul.capacity out_entries capacity
 * @byul.count out_count out_entries
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-query-incomplete-success,out_entries-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t navgrid_export_cells(
    const navgrid_t* navgrid, navgrid_cell_entry_t* out_entries,
    size_t capacity, size_t* out_count);

/**
 * @brief Allocates the traversable immediate neighbors in canonical order.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x Center X coordinate.
 * @param[in] y Center Y coordinate.
 * @return Caller-owned coordinate list, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_list_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_list_t* navgrid_copy_neighbors(
    const navgrid_t* navgrid, int x, int y);

/**
 * @brief Allocates all immediate neighbors without blocked filtering.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x Center X coordinate.
 * @param[in] y Center Y coordinate.
 * @return Caller-owned coordinate list, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_list_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_list_t* navgrid_copy_neighbors_all(
    const navgrid_t* navgrid, int x, int y);

/**
 * @brief Allocates all coordinates in the requested legacy square range.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x Center X coordinate.
 * @param[in] y Center Y coordinate.
 * @param[in] range Square range; 0 selects immediate neighbors.
 * @return Caller-owned coordinate list, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_list_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_list_t* navgrid_copy_neighbors_all_range(
    navgrid_t* navgrid, int x, int y, int range);

/**
 * @brief Allocates the canonical neighbor nearest to an angle.
 * @param[in] navgrid Grid to inspect.
 * @param[in] x Center X coordinate.
 * @param[in] y Center Y coordinate.
 * @param[in] degree Direction in degrees, with 0 along positive X.
 * @return Caller-owned coordinate, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_t* navgrid_copy_neighbor_at_degree(const navgrid_t* navgrid, 
    int x, int y, double degree);

/**
 * @brief Allocates the canonical neighbor nearest to the goal direction.
 * @param[in] navgrid Grid to inspect.
 * @param[in] center Center coordinate.
 * @param[in] goal Goal coordinate used to derive the direction.
 * @return Caller-owned coordinate, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_t* navgrid_copy_neighbor_at_goal(const navgrid_t* navgrid, 
    const coord_t* center, const coord_t* goal);

/**
 * @brief Allocates coordinates within an angular sector and square range.
 * @param[in] navgrid Grid to inspect.
 * @param[in] center Center coordinate.
 * @param[in] goal Goal coordinate defining the reference direction.
 * @param[in] start_deg Inclusive relative start angle.
 * @param[in] end_deg Inclusive relative end angle.
 * @param[in] range Square range.
 * @return Caller-owned coordinate list, or NULL on failure.
 * @byul.nullable navgrid false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return coord_list_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API coord_list_t* navgrid_copy_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

#ifdef __cplusplus
}
#endif

#endif // BYUL_NAVGRID_H
