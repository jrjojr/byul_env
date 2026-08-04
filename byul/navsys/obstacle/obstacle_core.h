/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file obstacle_core.h
 * @brief Opaque obstacle resource와 공통 public C ABI를 선언한다.
 *
 * Obstacle의 생명주기, extent, blocked 좌표, Navgrid overlay와 ABI 호환성 검사를
 * 제공한다. 내부 owning layout은 기본 SDK에 노출하지 않는다.
 */

#ifndef BYUL_OBSTACLE_CORE_H
#define BYUL_OBSTACLE_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "coord_hash.h"
#include "navgrid.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 내부 표현을 노출하지 않는 obstacle resource handle이다.
 *
 * Instance는 obstacle_create_checked() 또는 obstacle_create() family로 생성하고
 * obstacle_destroy()로 해제한다. 같은 instance의 mutation은 외부 동기화가 필요하다.
 *
 * @byul.storage opaque-object
 * @byul.copy_semantics deep-copy
 * @byul.thread_safety externally-synchronized
 */
typedef struct s_obstacle obstacle_t;

#define BYUL_OBSTACLE_ABI_VERSION UINT32_C(2)
#define BYUL_OBSTACLE_ABI_FINGERPRINT UINT64_C(0x4f42535402000000)

/** @brief Header와 runtime Obstacle ABI mismatch 원인을 구분한다. */
typedef enum e_obstacle_abi_mismatch {
    OBSTACLE_ABI_MATCH = 0,
    OBSTACLE_ABI_VERSION_MISMATCH = 1,
    OBSTACLE_ABI_FINGERPRINT_MISMATCH = 2
} obstacle_abi_mismatch_t;

/**
 * @brief Runtime이 제공하는 canonical Obstacle ABI version을 반환한다.
 * @return BYUL_OBSTACLE_ABI_VERSION과 대응하는 version.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint32_t obstacle_get_abi_version(void);

/**
 * @brief Runtime의 canonical Obstacle ABI fingerprint를 반환한다.
 * @return BYUL_OBSTACLE_ABI_FINGERPRINT와 대응하는 fingerprint.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API uint64_t obstacle_get_abi_fingerprint(void);

/**
 * @brief Header/package가 요구하는 Obstacle ABI를 runtime과 비교한다.
 *
 * ABI 2 opaque handle과 opt-in compatibility package의 ABI 1 layout을 인식한다.
 *
 * @param[in] expected_version Caller header의 ABI version.
 * @param[in] expected_fingerprint Caller header의 ABI fingerprint.
 * @param[out] out_mismatch 일치 여부 또는 mismatch 원인.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 지원 계약과 일치한다.
 * @retval NAVSYS_STATUS_UNSUPPORTED version 또는 fingerprint가 일치하지 않는다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_mismatch가 NULL이다.
 * @byul.nullable out_mismatch false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_mismatch-always
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t obstacle_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    obstacle_abi_mismatch_t* out_mismatch);

/**
 * @brief Native opaque obstacle object 본체의 byte 크기를 반환한다.
 * @return sizeof(obstacle_t). Caller allocation 허용을 뜻하지 않는다.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API size_t obstacle_sizeof(void);

/**
 * @brief Native opaque obstacle object 본체의 alignment를 반환한다.
 * @return alignof(obstacle_t).
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API size_t obstacle_alignof(void);

#define OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION UINT32_C(1)
#define OBSTACLE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION UINT32_C(1)

/** @brief Obstacle blocked set을 grid에 합치는 지원 policy다. */
typedef enum e_obstacle_navgrid_merge_policy {
    /** Base cell과 다른 overlay를 보존하는 독립 blocked overlay다. */
    OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE = 0
} obstacle_navgrid_merge_policy_t;

/**
 * @brief Obstacle overlay 준비 중 cooperative cancellation을 요청한다.
 *
 * Callback은 apply를 실행한 thread에서 동기 호출되며 true이면 취소한다. Callback과
 * userdata는 호출 동안만 borrow되고 보관되지 않는다. Callback은 대상 obstacle/grid를
 * 변경하거나 같은 grid API에 재진입하지 않아야 한다.
 */
typedef bool (*obstacle_navgrid_cancel_func)(void* userdata);

/**
 * @brief 한 번의 obstacle-to-navgrid 적용 option이다.
 *
 * struct_size는 sizeof(obstacle_navgrid_apply_options_t), abi_version은
 * OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION으로 설정한다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_navgrid_apply_options {
    uint32_t struct_size;
    uint32_t abi_version;
    /** One of obstacle_navgrid_merge_policy_t, stored as an integer so future
     *  policy values can be rejected without invoking C++ invalid-enum UB. */
    uint32_t merge_policy;
    obstacle_navgrid_cancel_func cancel_func;
    void* cancel_userdata;
} obstacle_navgrid_apply_options_t;

/**
 * @brief 한 grid 생존 세대에 귀속된 obstacle overlay value token이다.
 *
 * 성공한 obstacle_apply_to_navgrid_checked()만 token을 생성한다. Caller는 field를
 * 변경하지 않으며 성공한 remove는 token을 zero/invalidate한다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_obstacle_navgrid_overlay_token {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t owner_cookie;
    navgrid_overlay_id_t overlay;
} obstacle_navgrid_overlay_token_t;

// Basic constructors / destructors

/**
 * @brief 정규화된 extent를 가진 obstacle을 생성한다.
 *
 * 음수 width와 height는 origin 반대 방향의 half-open extent를 나타내며 0인 축은
 * 빈 영역이다. 성공한 결과는 caller가 obstacle_destroy()로 해제한다. 실패하면
 * out_obstacle을 보존한다.
 *
 * @param[in] x0 Extent origin의 X 좌표(grid cell).
 * @param[in] y0 Extent origin의 Y 좌표(grid cell).
 * @param[in] width 부호 있는 X축 extent(grid cell).
 * @param[in] height 부호 있는 Y축 extent(grid cell).
 * @param[out] out_obstacle 새 caller-owned obstacle을 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Obstacle을 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_obstacle이 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Object 또는 blocked set allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 예상하지 못한 native 오류가 발생했다.
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_create_checked(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    obstacle_t** out_obstacle);

/**
 * @brief 빈 extent를 가진 obstacle을 생성한다.
 * @return caller가 obstacle_destroy()로 해제할 객체 또는 실패 시 NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API obstacle_t* obstacle_create();
/**
 * @brief 지정한 origin과 extent를 가진 obstacle을 생성한다.
 * @param[in] x0 origin X 좌표.
 * @param[in] y0 origin Y 좌표.
 * @param[in] width X축 extent.
 * @param[in] height Y축 extent.
 * @return caller가 obstacle_destroy()로 해제할 객체 또는 실패 시 NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API obstacle_t* obstacle_create_full(
    int x0, int y0, int width, int height);
/**
 * @brief obstacle과 내부 blocked set을 해제한다.
 * @param[in] obstacle 해제할 객체 또는 NULL.
 * @byul.nullable obstacle true
 */
BYUL_API void obstacle_destroy(obstacle_t* obstacle);

// Copy and comparison

/**
 * @brief Obstacle extent와 blocked key set을 독립적으로 깊은 복사한다.
 *
 * 복사본은 source와 resource를 공유하지 않으며 callback이나 외부 binding을 보유하지
 * 않는다. 실패하면 source와 out_obstacle을 모두 보존한다. 성공한 결과는 caller가
 * obstacle_destroy()로 해제한다.
 *
 * @param[in] source 복사할 obstacle.
 * @param[out] out_obstacle 새 caller-owned 복사본을 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 독립 복사본을 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Object 또는 blocked set 복사에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE Source의 내부 resource가 유효하지 않거나 예상하지
 *     못한 native 오류가 발생했다.
 * @byul.nullable source false
 * @byul.nullable out_obstacle false
 * @byul.lifetime out_obstacle caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_obstacle-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_copy_checked(
    const obstacle_t* source, obstacle_t** out_obstacle);

/**
 * @brief obstacle을 깊은 복사한다.
 * @param[in] obstacle 복사할 객체.
 * @return caller가 obstacle_destroy()로 해제할 복사본 또는 실패 시 NULL.
 * @byul.nullable obstacle false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API obstacle_t* obstacle_copy(const obstacle_t* obstacle);
/**
 * @brief 두 obstacle의 extent와 blocked 좌표가 같은지 비교한다.
 * @param[in] a 첫 번째 객체.
 * @param[in] b 두 번째 객체.
 * @return 같으면 true, 아니면 false.
 * @byul.nullable a false
 * @byul.nullable b false
 */
BYUL_API bool obstacle_equal(const obstacle_t* a, const obstacle_t* b);
/**
 * @brief obstacle의 값 기반 hash를 계산한다.
 * @param[in] obstacle 조회할 객체.
 * @return obstacle의 hash 값.
 * @byul.nullable obstacle false
 */
BYUL_API uint32_t obstacle_hash(const obstacle_t* obstacle);

// Origin set / fetch
/**
 * @brief obstacle의 origin을 설정한다.
 * @param[in,out] obstacle 변경할 객체.
 * @param[in] x0 새 origin X 좌표.
 * @param[in] y0 새 origin Y 좌표.
 * @byul.nullable obstacle false
 */
BYUL_API void obstacle_set_origin(obstacle_t* obstacle, int x0, int y0);
/**
 * @brief obstacle의 origin을 복사한다.
 * @param[in] obstacle 조회할 객체.
 * @param[out] out_x0 X 좌표를 받을 위치.
 * @param[out] out_y0 Y 좌표를 받을 위치.
 * @byul.nullable obstacle false
 * @byul.nullable out_x0 false
 * @byul.nullable out_y0 false
 */
BYUL_API void obstacle_fetch_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0);

// Property access
/**
 * @brief obstacle의 width를 반환한다.
 * @param[in] m 조회할 객체.
 * @return 현재 width.
 * @byul.nullable m false
 */
BYUL_API int obstacle_get_width(const obstacle_t* m);
/**
 * @brief obstacle의 width를 설정한다.
 * @param[in,out] m 변경할 객체.
 * @param[in] width 새 width.
 * @byul.nullable m false
 */
BYUL_API void obstacle_set_width(obstacle_t* m, int width);

/**
 * @brief obstacle의 height를 반환한다.
 * @param[in] m 조회할 객체.
 * @return 현재 height.
 * @byul.nullable m false
 */
BYUL_API int obstacle_get_height(const obstacle_t* m);
/**
 * @brief obstacle의 height를 설정한다.
 * @param[in,out] m 변경할 객체.
 * @param[in] height 새 height.
 * @byul.nullable m false
 */
BYUL_API void obstacle_set_height(obstacle_t* m, int height);

// Direct access to blocked coordinates (read-only)
/**
 * @brief 내부 blocked set의 읽기 전용 borrowed view를 반환한다.
 * @param[in] obstacle 조회할 객체.
 * @return obstacle이 소유한 blocked set.
 * @byul.nullable obstacle false
 * @byul.nullable return false
 * @byul.lifetime return parent:obstacle
 */
BYUL_API const coord_hash_t* obstacle_get_blocked_coords(
    const obstacle_t* obstacle);

/**
 * @brief 한 좌표의 blocked 상태를 failure-atomic하게 설정한다.
 *
 * 좌표는 obstacle의 normalized half-open extent 안에 있어야 한다. 이미 요청한 상태이면
 * 성공하면서 out_changed에 false를 기록한다. 음수 status에서는 obstacle과
 * out_changed를 보존한다.
 *
 * @param[in,out] obstacle 변경할 obstacle.
 * @param[in] x 대상 X 좌표(grid cell).
 * @param[in] y 대상 Y 좌표(grid cell).
 * @param[in] blocked 설정할 blocked 상태.
 * @param[out] out_changed 실제 key set 변경 여부를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 상태를 적용했거나 이미 같은 상태다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND 좌표가 extent 밖에 있다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Blocked key 삽입에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 blocked set이 유효하지 않거나 mutation이
 *     일관되게 완료되지 않았다.
 * @byul.nullable obstacle false
 * @byul.nullable out_changed false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:obstacle,out_changed-on-success
 * @byul.invalidates set_blocked all-internal-pointers
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_set_blocked(
    obstacle_t* obstacle, int32_t x, int32_t y,
    bool blocked, bool* out_changed);

/**
 * @brief Blocked 좌표를 caller buffer에 복사한다.
 *
 * out_coords가 NULL이고 capacity가 0이면 전체 필요 element 수를 out_count에 기록한다.
 * Buffer가 부족하면 buffer를 보존하고 전체 필요 수와 NAVSYS_STATUS_INCOMPLETE를
 * 반환한다. Export 순서는 보장하지 않는다.
 *
 * @param[in] obstacle 내보낼 obstacle.
 * @param[out] out_coords caller-provided coord_t buffer 또는 count query 시 NULL.
 * @param[in] capacity out_coords의 coord_t element 용량.
 * @param[out] out_count 필요한 전체 element 수를 받을 위치.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Count query 또는 전체 export를 완료했다.
 * @retval NAVSYS_STATUS_INCOMPLETE Buffer 용량이 부족하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer 또는 buffer/capacity 조합이
 *     잘못됐다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 blocked set이 유효하지 않다.
 * @byul.nullable obstacle false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.buffer out_coords
 * @byul.capacity out_coords capacity
 * @byul.count out_count out_coords
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-always,out_coords-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t obstacle_export_blocked(
    const obstacle_t* obstacle, coord_t* out_coords,
    size_t capacity, size_t* out_count);

/**
 * @brief 좌표를 blocked 상태로 만든다.
 * @param[in,out] m 변경할 객체.
 * @param[in] x X 좌표.
 * @param[in] y Y 좌표.
 * @return 성공하면 true.
 * @byul.nullable m false
 */
BYUL_API bool obstacle_block_coord(obstacle_t* m, int x, int y);
/**
 * @brief 좌표의 blocked 상태를 제거한다.
 * @param[in,out] m 변경할 객체.
 * @param[in] x X 좌표.
 * @param[in] y Y 좌표.
 * @return 성공하면 true.
 * @byul.nullable m false
 */
BYUL_API bool obstacle_unblock_coord(obstacle_t* m, int x, int y);
/**
 * @brief 좌표가 obstacle extent 안에 있는지 검사한다.
 * @param[in] m 조회할 객체.
 * @param[in] x X 좌표.
 * @param[in] y Y 좌표.
 * @return extent 안이면 true.
 * @byul.nullable m false
 */
BYUL_API bool obstacle_is_inside(const obstacle_t* m, int x, int y);
/**
 * @brief 모든 blocked 좌표를 제거한다.
 * @param[in,out] m 변경할 객체.
 * @byul.nullable m false
 */
BYUL_API void obstacle_clear(obstacle_t* m);

// Neighbor search
/**
 * @brief blocked 좌표를 제외한 즉시 이웃 목록을 생성한다.
 * @deprecated obstacle_create_neighbors()를 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @return caller-owned 목록 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbors; removal is planned for ABI 2.")
BYUL_API coord_list_t* obstacle_clone_neighbors(
    const obstacle_t* m, int x, int y);

/**
 * @brief blocked 여부와 무관한 즉시 이웃 목록을 생성한다.
 * @deprecated obstacle_create_neighbors_all()을 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @return caller-owned 목록 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbors_all; removal is planned for ABI 2.")
BYUL_API coord_list_t* obstacle_clone_neighbors_all(
    const obstacle_t* m, int x, int y);

/**
 * @brief legacy range 이웃 목록을 생성한다.
 * @deprecated obstacle_create_neighbors_all_range()를 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] range legacy range 값.
 * @return caller-owned 목록 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbors_all_range; removal is planned for ABI 2.")
BYUL_API coord_list_t* obstacle_clone_neighbors_all_range(
    obstacle_t* m, int x, int y, int range);

/**
 * @brief 각도에 가장 가까운 이웃 좌표를 생성한다.
 * @deprecated obstacle_create_neighbor_at_degree()를 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] degree 선택 각도.
 * @return caller-owned 좌표 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbor_at_degree; removal is planned for ABI 2.")
BYUL_API coord_t* obstacle_clone_neighbor_at_degree(const obstacle_t* m,
    int x, int y, double degree);

/**
 * @brief goal 방향에 가장 가까운 이웃 좌표를 생성한다.
 * @deprecated obstacle_create_neighbor_at_goal()을 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 목표 좌표.
 * @return caller-owned 좌표 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbor_at_goal; removal is planned for ABI 2.")
BYUL_API coord_t* obstacle_clone_neighbor_at_goal(const obstacle_t* m,
    const coord_t* center, const coord_t* goal);

/**
 * @brief goal 상대 각도 구간의 이웃 목록을 생성한다.
 * @deprecated obstacle_create_neighbors_at_degree_range()를 사용한다.
 * @param[in] m 조회할 obstacle.
 * @param[in] center 중심 좌표.
 * @param[in] goal 목표 좌표.
 * @param[in] start_deg 시작 상대 각도.
 * @param[in] end_deg 끝 상대 각도.
 * @param[in] range Chebyshev 범위.
 * @return caller-owned 목록 또는 실패 시 NULL.
 * @byul.nullable m false
 * @byul.nullable center false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use obstacle_create_neighbors_at_degree_range; removal is planned for ABI 2.")
BYUL_API coord_list_t* obstacle_clone_neighbors_at_degree_range(
    const obstacle_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

/**
 * @brief 좌표의 effective blocked 상태를 조회한다.
 * @param[in] obstacle 조회할 객체.
 * @param[in] x X 좌표.
 * @param[in] y Y 좌표.
 * @return blocked이면 true.
 * @byul.nullable obstacle false
 */
BYUL_API bool obstacle_is_coord_blocked(
    const obstacle_t* obstacle, int x, int y);

/**
 * @brief Obstacle blocked set을 provenance overlay로 원자적으로 적용한다.
 *
 * 모든 blocked 좌표가 현재 grid extent 안에 있어야 하며 grid를 resize하지 않는다.
 * Base terrain/height와 다른 overlay를 보존한다. options가 NULL이면 preserve-base와
 * cancellation 없음이 기본값이다. OOM/cancel/callback 실패를 포함한 모든 실패에서
 * grid와 output을 보존한다.
 *
 * @param[in] obstacle 적용할 obstacle.
 * @param[in,out] navgrid 변경할 grid.
 * @param[in] options Call-scoped option 또는 기본값을 위한 NULL.
 * @param[out] out_overlay 성공한 overlay token을 받을 storage.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Overlay가 적용됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer, option size 또는 값이 잘못됐다.
 * @retval NAVSYS_STATUS_UNSUPPORTED Option ABI version 또는 merge policy를 지원하지 않는다.
 * @retval NAVSYS_STATUS_NOT_FOUND Blocked 좌표 하나 이상이 grid extent 밖이다.
 * @retval NAVSYS_STATUS_CANCELLED Callback이 취소를 요청했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED Callback이 C++ exception을 던졌다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 준비 allocation에 실패했다.
 * @byul.nullable obstacle false
 * @byul.nullable navgrid false
 * @byul.nullable options true
 * @byul.nullable out_overlay false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,out_overlay,out_changed_count-on-success
 * @byul.invalidates apply none
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t obstacle_apply_to_navgrid_checked(
    const obstacle_t* obstacle,
    navgrid_t* navgrid,
    const obstacle_navgrid_apply_options_t* options,
    obstacle_navgrid_overlay_token_t* out_overlay,
    size_t* out_changed_count);

/**
 * @brief Token이 지정한 obstacle overlay만 원자적으로 제거한다.
 *
 * 다른 grid, 이미 제거된 overlay 또는 다른 grid 세대의 token은 INVALIDATED다.
 * 지원하지 않는 token ABI version은 UNSUPPORTED다. 성공하면 token을 zero/invalidate하고
 * base cell과 다른 overlay는 그대로 드러낸다. 실패하면 grid, token과 output을 보존한다.
 *
 * @param[in,out] navgrid token을 생성한 생존 grid.
 * @param[in,out] overlay 적용 성공으로 받은 token.
 * @param[out] out_changed_count effective blocked 상태가 바뀐 좌표 수를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @byul.nullable navgrid false
 * @byul.nullable overlay false
 * @byul.nullable out_changed_count false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:navgrid,overlay,out_changed_count-on-success
 * @byul.invalidates remove overlay
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t obstacle_remove_from_navgrid_checked(
    navgrid_t* navgrid,
    obstacle_navgrid_overlay_token_t* overlay,
    size_t* out_changed_count);

/**
 * @brief obstacle의 blocked 좌표를 navgrid에 적용한다.
 * @deprecated obstacle_apply_to_navgrid_checked()를 사용한다.
 * @param[in] obstacle 적용할 객체.
 * @param[in,out] navgrid 변경할 grid.
 * @byul.nullable obstacle false
 * @byul.nullable navgrid false
 */
BYUL_DEPRECATED("Use obstacle_apply_to_navgrid_checked; removal is planned for ABI 2.")
BYUL_API void obstacle_apply_to_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid);

/**
 * @brief obstacle의 blocked 좌표를 navgrid에서 제거한다.
 * @deprecated obstacle_remove_from_navgrid_checked()를 사용한다.
 * @param[in] obstacle 제거 기준 객체.
 * @param[in,out] navgrid 변경할 grid.
 * @byul.nullable obstacle false
 * @byul.nullable navgrid false
 */
BYUL_DEPRECATED("Use obstacle_remove_from_navgrid_checked; removal is planned for ABI 2.")
BYUL_API void obstacle_remove_from_navgrid(
    const obstacle_t* obstacle, navgrid_t* navgrid);

/**
 * @brief 중심 주위의 legacy square range를 block한다.
 * @deprecated obstacle_block_square()를 사용한다.
 * @param[in,out] obs 변경할 obstacle.
 * @param[in] x 중심 X 좌표.
 * @param[in] y 중심 Y 좌표.
 * @param[in] range legacy range 값.
 * @byul.nullable obs false
 */
BYUL_DEPRECATED("Use obstacle_block_square; removal is planned for ABI 2.")
BYUL_API void obstacle_block_range(obstacle_t* obs, int x, int y, int range);

/**
 * @brief 두 좌표 사이의 legacy 두께 선을 block한다.
 * @deprecated obstacle_block_line()을 사용한다.
 * @param[in,out] obs 변경할 obstacle.
 * @param[in] x0 시작 X 좌표.
 * @param[in] y0 시작 Y 좌표.
 * @param[in] x1 끝 X 좌표.
 * @param[in] y1 끝 Y 좌표.
 * @param[in] range legacy 두께 범위.
 * @byul.nullable obs false
 */
BYUL_DEPRECATED("Use obstacle_block_line; removal is planned for ABI 2.")
BYUL_API void obstacle_block_straight(obstacle_t* obs, 
    int x0, int y0, int x1, int y1, int range);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_OBSTACLE_CORE_H */
