/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_core.h
 * @brief Declares the opaque Maze resource and its checked public C ABI.
 */

#ifndef BYUL_MAZE_CORE_H
#define BYUL_MAZE_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "navgrid.h"
#include "coord.h"
#include "coord_hash.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque Maze resource handle.
 *
 * Create instances with byul_maze_create() or a legacy constructor and release
 * them with maze_destroy(). Mutation of one instance requires external
 * synchronization.
 *
 * @byul.storage opaque-object
 * @byul.copy_semantics deep-copy
 * @byul.thread_safety externally-synchronized
 */
typedef struct s_maze maze_t;

#define BYUL_MAZE_ABI_VERSION UINT32_C(2)
#define BYUL_MAZE_ABI_FINGERPRINT UINT64_C(0x4d415a4502000000)

/** @brief Identifies the reason for a Maze ABI mismatch. */
typedef enum e_byul_maze_abi_mismatch {
    BYUL_MAZE_ABI_MATCH = 0,
    BYUL_MAZE_ABI_VERSION_MISMATCH = 1,
    BYUL_MAZE_ABI_FINGERPRINT_MISMATCH = 2
} byul_maze_abi_mismatch_t;

/**
 * @brief Return the canonical runtime Maze ABI version.
 * @return BYUL_MAZE_ABI_VERSION.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint32_t byul_maze_get_abi_version(void);

/**
 * @brief Return the canonical runtime Maze ABI fingerprint.
 * @return BYUL_MAZE_ABI_FINGERPRINT.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint64_t byul_maze_get_abi_fingerprint(void);

/**
 * @brief Compare a Maze header/package ABI with the loaded runtime.
 * @param[in] expected_version Caller header ABI version.
 * @param[in] expected_fingerprint Caller header ABI fingerprint.
 * @param[out] out_mismatch Match state or mismatch reason.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The requested ABI is supported.
 * @retval NAVSYS_STATUS_UNSUPPORTED Version or fingerprint does not match.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_mismatch is NULL.
 * @byul.nullable out_mismatch false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_mismatch-always
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t byul_maze_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    byul_maze_abi_mismatch_t* out_mismatch);

/**
 * @brief Return the native opaque Maze object size.
 * @return sizeof of the private Maze object.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API size_t byul_maze_sizeof(void);

/**
 * @brief Return the native opaque Maze object alignment.
 * @return alignof of the private Maze object.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API size_t byul_maze_alignof(void);

/** @brief 정규화된 Maze half-open extent 값이다. */
typedef struct byul_maze_extent {
    int32_t origin_x; /**< 최소 X world coordinate. */
    int32_t origin_y; /**< 최소 Y world coordinate. */
    uint32_t width;   /**< X축 cell 수. */
    uint32_t height;  /**< Y축 cell 수. */
} byul_maze_extent_t;

#define BYUL_MAZE_NAVGRID_APPLY_OPTIONS_ABI_VERSION UINT32_C(1)
#define BYUL_MAZE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION UINT32_C(1)

/** @brief Maze blocked set을 grid에 합치는 ABI 1 policy다. */
typedef enum e_byul_maze_navgrid_merge_policy {
    /** Base cell과 다른 provenance overlay를 보존하는 독립 blocked overlay다. */
    BYUL_MAZE_NAVGRID_MERGE_PRESERVE_BASE = 0
} byul_maze_navgrid_merge_policy_t;

/**
 * @brief Maze overlay 준비 중 cooperative cancellation을 요청한다.
 * apply options가 전달한 borrowed userdata를 받고 취소를 요청하면 true를 반환한다.
 */
typedef bool (*byul_maze_navgrid_cancel_func)(void* userdata);

/** @brief Versioned Maze-to-Navgrid apply options다. */
typedef struct s_byul_maze_navgrid_apply_options {
    uint32_t struct_size;
    uint32_t abi_version;
    /** One of byul_maze_navgrid_merge_policy_t. */
    uint32_t merge_policy;
    byul_maze_navgrid_cancel_func cancel_func;
    void* cancel_userdata;
} byul_maze_navgrid_apply_options_t;

/** @brief 한 Navgrid 생존 기간에 귀속되는 Maze overlay value token이다. */
typedef struct s_byul_maze_navgrid_overlay_token {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t owner_cookie;
    navgrid_overlay_id_t overlay;
} byul_maze_navgrid_overlay_token_t;

// Basic constructors / destructors
/**
 * @brief 정규화된 extent를 가진 Maze를 failure-atomic하게 생성한다.
 * @param[in] extent 생성할 nonnegative half-open extent.
 * @param[out] out_maze 성공한 caller-owned Maze를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Maze를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 representable extent가 잘못됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY object 또는 blocked set allocation이 실패했다.
 * @byul.nullable extent false
 * @byul.nullable out_maze false
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_create(
    const byul_maze_extent_t* extent, maze_t** out_maze);

/**
 * @brief Create an empty legacy Maze at the zero origin.
 * @return Caller-owned Maze, or NULL on allocation failure.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API maze_t* maze_create(void);

/**
 * @brief Create a legacy Maze with signed extent metadata.
 * @param[in] x0 Legacy origin X coordinate.
 * @param[in] y0 Legacy origin Y coordinate.
 * @param[in] width Signed width.
 * @param[in] height Signed height.
 * @return Caller-owned Maze, or NULL on allocation failure.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API maze_t* maze_create_full(int x0, int y0, int width, int height);

/**
 * @brief Destroy a caller-owned Maze.
 * @param[in,out] maze Maze to destroy; NULL is accepted.
 * @byul.nullable maze true
 * @byul.side_effect frees:maze
 * @byul.invalidates always maze,all-internal-pointers
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void maze_destroy(maze_t* maze);

/**
 * @brief Clear every blocked coordinate from a Maze.
 * @param[in,out] maze Maze to clear; NULL is ignored.
 * @byul.nullable maze true
 * @byul.side_effect mutates:maze
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void maze_clear(maze_t* maze);

// Copy and comparison
/**
 * @brief Copy a Maze using the legacy nullable-result contract.
 * @param[in] maze Maze to copy.
 * @return Caller-owned copy, or NULL on invalid input or allocation failure.
 * @byul.nullable maze false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error sentinel:null
 * @byul.side_effect allocates
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API maze_t* maze_copy(const maze_t* maze);
/**
 * @brief Maze extent와 blocked set을 failure-atomic하게 깊은 복사한다.
 * @param[in] source 복사할 Maze.
 * @param[out] out_maze 성공한 caller-owned 복사본을 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 독립 복사본을 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY object 또는 blocked set 복사가 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source blocked set이 없다.
 * @byul.nullable source false
 * @byul.nullable out_maze false
 * @byul.lifetime out_maze caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_maze-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_copy(
    const maze_t* source, maze_t** out_maze);
/**
 * @brief Compare Maze extent metadata and blocked coordinates.
 * @param[in] a First Maze.
 * @param[in] b Second Maze.
 * @return True when both Mazes are semantically equal.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API bool maze_equal(const maze_t* a, const maze_t* b);
/**
 * @brief Compute a stable in-process hash of a Maze.
 * @param[in] maze Maze to hash.
 * @return Hash value, or zero for NULL.
 * @byul.nullable maze true
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API uint32_t maze_hash(const maze_t* maze);

// Origin set / get
/**
 * @brief Maze origin과 blocked world coordinate를 함께 이동한다.
 *
 * Signed width와 height는 origin의 반대 방향도 표현하며 정규화된 half-open
 * extent를 이룬다. 0인 축은 빈 extent다. 좌표 또는 origin overflow, allocation
 * failure, extent 밖 blocked key가 발견되면 maze를 변경하지 않는다.
 *
 * @param[in,out] maze 이동할 maze.
 * @param[in] delta_x X축 이동량(grid cell).
 * @param[in] delta_y Y축 이동량(grid cell).
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Origin과 blocked key를 이동했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT maze가 NULL이거나 이동 결과가 int32 범위를
 *     벗어난다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 임시 key set allocation이 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 blocked set이 없거나 기존 key가 extent
 *     밖에 있다.
 * @byul.nullable maze false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:maze-on-success
 * @byul.invalidates success all-internal-pointers
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_translate(
    maze_t* maze, int32_t delta_x, int32_t delta_y);

/**
 * @brief Maze origin을 지정하고 blocked world coordinate를 같은 delta만큼 이동한다.
 *
 * 실패를 보고하지 못하는 ABI 1.x 호환 wrapper다. 실패하면 maze를 보존한다. 새 코드는
 * =byul_maze_translate=를 사용해야 한다.
 *
 * @param[in,out] maze 이동할 maze.
 * @param[in] x0 새 origin X 좌표.
 * @param[in] y0 새 origin Y 좌표.
 * @byul.nullable maze false
 * @byul.side_effect mutates:maze-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_DEPRECATED(
    "Use byul_maze_translate; removal is planned for ABI 2.")
BYUL_API void maze_set_origin(maze_t* maze, int x0, int y0);
/**
 * @brief Fetch the legacy Maze origin.
 * @param[in] maze Maze to query.
 * @param[out] out_x0 Optional origin X output.
 * @param[out] out_y0 Optional origin Y output.
 * @byul.nullable maze false
 * @byul.nullable out_x0 true
 * @byul.nullable out_y0 true
 * @byul.side_effect mutates:out_x0,out_y0
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API void maze_get_origin(
    const maze_t* maze, int* out_x0, int* out_y0);

/**
 * @brief 정규화된 nonnegative half-open extent를 복사한다.
 * @param[in] maze 조회할 Maze.
 * @param[out] out_extent extent를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK extent를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 상태 또는 정규화 결과가 표현 불가능하다.
 * @byul.nullable maze false
 * @byul.nullable out_extent false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_extent-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_get_extent(
    const maze_t* maze, byul_maze_extent_t* out_extent);

// Size getters
/**
 * @brief Fetch the signed legacy Maze width.
 * @param[in] maze Maze to query.
 * @return Signed width, or zero for NULL.
 * @byul.nullable maze true
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API int maze_get_width(const maze_t* maze);
/**
 * @brief Fetch the signed legacy Maze height.
 * @param[in] maze Maze to query.
 * @return Signed height, or zero for NULL.
 * @byul.nullable maze true
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API int maze_get_height(const maze_t* maze);

/**
 * @brief Borrow the legacy read-only blocked coordinate set.
 * @param[in] maze Maze to query.
 * @return Borrowed blocked set, or NULL for NULL input.
 * @byul.nullable maze true
 * @byul.nullable return true
 * @byul.lifetime return borrowed:maze
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API const coord_hash_t* maze_get_blocked_coords(
    const maze_t* maze);

/**
 * @brief Blocked world coordinate 수를 조회한다.
 * @param[in] maze 조회할 Maze.
 * @param[out] out_count key 수를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK count를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 blocked set이 없다.
 * @byul.nullable maze false
 * @byul.nullable out_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_get_blocked_count(
    const maze_t* maze, size_t* out_count);

/**
 * @brief Blocked world coordinate를 caller buffer로 복사한다.
 *
 * buffer가 NULL이고 capacity가 0이면 count query다. Buffer가 작으면 내용을 보존하고
 * out_count에 required count를 기록한 뒤 NAVSYS_STATUS_INCOMPLETE를 반환한다. 순서는
 * 보장하지 않는다.
 *
 * @param[in] maze 조회할 Maze.
 * @param[out] buffer caller-provided coord_t buffer 또는 count query의 NULL.
 * @param[in] capacity buffer의 coord_t element 용량.
 * @param[out] out_count required 또는 exported element 수를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK count query 또는 전체 export를 완료했다.
 * @retval NAVSYS_STATUS_INCOMPLETE buffer가 작아 내용을 보존했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer/capacity 조합이 잘못됐다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 blocked set이 없다.
 * @byul.nullable maze false
 * @byul.nullable buffer true
 * @byul.nullable out_count false
 * @byul.buffer buffer
 * @byul.capacity buffer capacity
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:buffer,out_count-on-success-or-incomplete-count
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_fetch_blocked(
    const maze_t* maze,
    coord_t* buffer,
    size_t capacity,
    size_t* out_count);

/**
 * @brief Query whether a world coordinate is blocked by a Maze.
 * @param[in] maze Maze to query.
 * @param[in] x World X coordinate inside the normalized Maze extent.
 * @param[in] y World Y coordinate inside the normalized Maze extent.
 * @param[out] out_blocked Receives whether the coordinate is blocked.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The query completed.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL.
 * @retval NAVSYS_STATUS_NOT_FOUND The coordinate is outside the Maze extent.
 * @retval NAVSYS_STATUS_CORRUPT_STATE The Maze blocked set is missing.
 * @byul.nullable maze false
 * @byul.nullable out_blocked false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_blocked-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_is_blocked(
    const maze_t* maze, int32_t x, int32_t y, bool* out_blocked);

/**
 * @brief Set or clear a blocked world coordinate in a Maze.
 *
 * On failure the Maze and =out_changed= are preserved.
 *
 * @param[in,out] maze Maze to mutate.
 * @param[in] x World X coordinate inside the normalized Maze extent.
 * @param[in] y World Y coordinate inside the normalized Maze extent.
 * @param[in] blocked True to block the coordinate, false to clear it.
 * @param[out] out_changed Receives whether the blocked set changed.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The mutation completed.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Inserting a coordinate failed.
 * @retval NAVSYS_STATUS_NOT_FOUND The coordinate is outside the Maze extent.
 * @retval NAVSYS_STATUS_CORRUPT_STATE The Maze blocked set is missing.
 * @byul.nullable maze false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:maze,out_changed-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t byul_maze_set_blocked(
    maze_t* maze,
    int32_t x,
    int32_t y,
    bool blocked,
    bool* out_changed);

/**
 * @brief Maze blocked set을 provenance overlay로 원자적으로 적용한다.
 *
 * Grid extent를 변경하지 않으며 모든 coordinate가 현재 grid 안에 있어야 한다.
 * options가 NULL이면 preserve-base와 cancellation 없음이 기본값이다. 모든 실패에서
 * grid, out_overlay와 out_changed_count를 보존한다.
 *
 * @param[in] maze 적용할 Maze.
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] options call-scoped options 또는 기본값을 위한 NULL.
 * @param[out] out_overlay 성공한 overlay token을 받을 위치.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Overlay를 적용했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer 또는 option size가 잘못됐다.
 * @retval NAVSYS_STATUS_UNSUPPORTED option ABI version 또는 policy가 지원되지 않는다.
 * @retval NAVSYS_STATUS_NOT_FOUND coordinate 하나 이상이 grid extent 밖이다.
 * @retval NAVSYS_STATUS_CANCELLED callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED callback이 exception을 던졌다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 준비 allocation이 실패했다.
 * @byul.nullable maze false
 * @byul.nullable navgrid false
 * @byul.nullable options true
 * @byul.nullable out_overlay false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_overlay,out_changed_count-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t byul_maze_apply(
    const maze_t* maze,
    navgrid_t* navgrid,
    const byul_maze_navgrid_apply_options_t* options,
    byul_maze_navgrid_overlay_token_t* out_overlay,
    size_t* out_changed_count);

/**
 * @brief Token이 지정한 Maze overlay만 원자적으로 제거한다.
 *
 * 성공하면 token을 invalidate하고 base cell과 다른 overlay를 보존한다. 다른 grid의
 * token, 이미 제거한 token 또는 소유자가 바뀐 token은 INVALIDATED다. 모든 실패에서
 * grid, token과 output을 보존한다.
 *
 * @param[in,out] navgrid token이 생성된 grid.
 * @param[in,out] overlay 성공한 apply가 생성한 token.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Overlay를 제거하고 token을 invalidate했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer 또는 token size가 잘못됐다.
 * @retval NAVSYS_STATUS_UNSUPPORTED token ABI version이 지원되지 않는다.
 * @retval NAVSYS_STATUS_INVALIDATED token이 해당 grid의 활성 overlay를 가리키지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY rollback 가능한 준비 allocation이 실패했다.
 * @byul.nullable navgrid false
 * @byul.nullable overlay false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,overlay,out_changed_count-on-success
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t byul_maze_remove_overlay(
    navgrid_t* navgrid,
    byul_maze_navgrid_overlay_token_t* overlay,
    size_t* out_changed_count);

/**
 * @brief Apply a Maze using the legacy implicit-resize contract.
 * @param[in] maze Maze to apply.
 * @param[in,out] navgrid Grid to mutate.
 * @byul.nullable maze false
 * @byul.nullable navgrid false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void maze_apply_to_navgrid(const maze_t* maze, navgrid_t* navgrid);

/**
 * @brief Remove a Maze using the legacy coordinate-only contract.
 * @param[in] maze Maze to remove.
 * @param[in,out] navgrid Grid to mutate.
 * @byul.nullable maze false
 * @byul.nullable navgrid false
 * @byul.side_effect mutates:navgrid
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API void maze_remove_from_navgrid(const maze_t* maze, navgrid_t* navgrid);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_MAZE_CORE_H */
