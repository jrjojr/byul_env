/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file cost_coord_pq.h
 * @brief float cost와 coordinate value를 저장하는 priority queue C ABI다.
 */

#ifndef BYUL_COST_COORD_PQ_H
#define BYUL_COST_COORD_PQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque cost-coordinate priority queue resource다.
 * @byul.storage opaque-object
 * @byul.copy_semantics non-copyable
 * @byul.thread_safety external-synchronization
 */
typedef struct s_cost_coord_pq cost_coord_pq_t;

#define BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION UINT32_C(1)

/**
 * @brief Versioned queue creation information이다.
 *
 * struct_size는 sizeof(cost_coord_pq_create_info_t), abi_version은
 * BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION, flags는 0으로 설정한다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_cost_coord_pq_create_info {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t flags;
} cost_coord_pq_create_info_t;

/**
 * @brief finite non-negative cost queue를 생성한다.
 *
 * 실패하면 out_queue를 보존한다.
 *
 * @param[in] info Versioned creation information이다.
 * @param[out] out_queue 성공 시 caller-owned queue를 받는다.
 * @return Common Navsys status value다.
 * @retval NAVSYS_STATUS_OK 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer, size, version 또는 flags가 유효하지 않다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable info false
 * @byul.nullable out_queue false
 * @byul.lifetime out_queue caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_queue-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t cost_coord_pq_create_ex(
    const cost_coord_pq_create_info_t* info,
    cost_coord_pq_t** out_queue);

/**
 * @brief coordinate value를 validated cost로 복사해 삽입한다.
 *
 * Cost는 finite이고 0 이상이어야 하며 -0은 +0으로 정규화된다. 같은 cost는
 * canonical 조회에서 insertion FIFO 순서를 따른다. 실패하면 queue를 보존한다.
 *
 * @param[in,out] queue 변경할 queue다.
 * @param[in] cost finite non-negative priority다.
 * @param[in] coord 복사할 coordinate다.
 * @return Common Navsys status value다.
 * @retval NAVSYS_STATUS_OK 삽입했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 cost가 유효하지 않다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 늘리지 못했다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED insertion sequence를 표현할 수 없다.
 * @byul.nullable queue false
 * @byul.nullable coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:queue-on-success
 * @byul.thread_safety external-synchronization
 * @byul.blocking false
 */
BYUL_API navsys_status_t cost_coord_pq_push_ex(
    cost_coord_pq_t* queue, float cost, const coord_t* coord);

/**
 * @brief 최소 cost와 FIFO coordinate를 caller storage에 복사한다.
 *
 * Queue를 변경하지 않으며 실패하면 두 출력을 모두 보존한다.
 *
 * @param[in] queue 조회할 queue다.
 * @param[out] out_cost 최소 cost를 받는다.
 * @param[out] out_coord coordinate value를 받는다.
 * @return Common Navsys status value다.
 * @retval NAVSYS_STATUS_OK 최소 항목을 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT required pointer가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND queue가 비었다.
 * @byul.nullable queue false
 * @byul.nullable out_cost false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect none
 * @byul.thread_safety external-synchronization
 * @byul.blocking false
 */
BYUL_API navsys_status_t cost_coord_pq_peek_min(
    const cost_coord_pq_t* queue,
    float* out_cost,
    coord_t* out_coord);

/**
 * @brief 최소 cost와 FIFO coordinate를 제거해 caller storage에 복사한다.
 *
 * 실패하면 queue와 두 출력을 모두 보존한다.
 *
 * @param[in,out] queue 변경할 queue다.
 * @param[out] out_cost 제거한 cost를 받는다.
 * @param[out] out_coord 제거한 coordinate value를 받는다.
 * @return Common Navsys status value다.
 * @retval NAVSYS_STATUS_OK 최소 항목을 제거했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT required pointer가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND queue가 비었다.
 * @byul.nullable queue false
 * @byul.nullable out_cost false
 * @byul.nullable out_coord false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:queue-on-success
 * @byul.thread_safety external-synchronization
 * @byul.blocking false
 */
BYUL_API navsys_status_t cost_coord_pq_pop_min(
    cost_coord_pq_t* queue,
    float* out_cost,
    coord_t* out_coord);

/** @brief Legacy 기본 queue 생성 adapter다.
 * @return 새 caller-owned queue 또는 allocation 실패 시 NULL이다.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API cost_coord_pq_t* cost_coord_pq_create(void);

/** @brief Queue와 남은 항목을 파괴한다.
 * @param[in,out] pq 파괴할 queue 또는 NULL이다.
 * @byul.nullable pq true
 * @byul.lifetime pq consumed
 */
BYUL_API void cost_coord_pq_destroy(cost_coord_pq_t* pq);

/** @brief Legacy unrestricted-cost push adapter다.
 * @param[in,out] pq 변경할 queue다.
 * @param[in] cost legacy float priority다.
 * @param[in] c 복사할 coordinate다.
 * @byul.nullable pq false
 * @byul.nullable c false
 */
BYUL_API void cost_coord_pq_push(
    cost_coord_pq_t* pq, float cost, const coord_t* c);

/** @brief Legacy LIFO 최소 항목을 borrowed pointer로 조회한다.
 * @param[in,out] pq 조회할 queue다.
 * @return mutation/destroy 전까지 borrowed인 coordinate 또는 NULL이다.
 * @byul.nullable pq false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:pq
 */
BYUL_API coord_t* cost_coord_pq_peek(cost_coord_pq_t* pq);

/** @brief Legacy LIFO 최소 항목을 제거해 owned pointer로 반환한다.
 * @param[in,out] pq 변경할 queue다.
 * @return coord_destroy가 필요한 caller-owned coordinate 또는 NULL이다.
 * @byul.nullable pq false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API coord_t* cost_coord_pq_pop(cost_coord_pq_t* pq);

/** @brief Legacy 최소 cost를 조회한다.
 * @param[in] pq 조회할 queue다.
 * @return 최소 cost이며 NULL/empty이면 0이다.
 * @byul.nullable pq true
 */
BYUL_API float cost_coord_pq_peek_cost(cost_coord_pq_t* pq);

/** @brief Queue가 NULL이거나 비었는지 반환한다.
 * @param[in] pq 조회할 queue다.
 * @return 비었으면 true다.
 * @byul.nullable pq true
 */
BYUL_API bool cost_coord_pq_is_empty(cost_coord_pq_t* pq);

/** @brief Cost와 무관하게 coordinate 존재 여부를 조회한다.
 * @param[in] pq 조회할 queue다.
 * @param[in] c 찾을 coordinate다.
 * @return 찾으면 true다.
 * @byul.nullable pq false
 * @byul.nullable c false
 */
BYUL_API bool cost_coord_pq_contains(
    cost_coord_pq_t* pq, const coord_t* c);

/** @brief Exact-cost bucket에서 같은 coordinate를 모두 제거한다.
 * @param[in,out] pq 변경할 queue다.
 * @param[in] cost legacy exact cost다.
 * @param[in] c 제거할 coordinate다.
 * @return 하나 이상 제거했으면 true다.
 * @byul.nullable pq false
 * @byul.nullable c false
 */
BYUL_API bool cost_coord_pq_remove(
    cost_coord_pq_t* pq, float cost, const coord_t* c);

/** @brief Legacy int element 수를 반환한다.
 * @param[in] pq 조회할 queue다.
 * @return element 수이며 int 범위를 넘으면 INT_MAX다.
 * @byul.nullable pq true
 */
BYUL_API int cost_coord_pq_length(cost_coord_pq_t* pq);

/** @brief 큰 cost부터 최대 n개를 제거한다.
 * @param[in,out] pq 변경할 queue다.
 * @param[in] n 제거할 최대 개수이며 0 이하는 no-op이다.
 * @byul.nullable pq true
 */
BYUL_API void cost_coord_pq_trim_worst(cost_coord_pq_t* pq, int n);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_COST_COORD_PQ_H */
