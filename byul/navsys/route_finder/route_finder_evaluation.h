/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file route_finder_evaluation.h
 * @brief 경로 탐색의 이동 비용과 휴리스틱 평가 public C ABI를 선언한다.
 *
 * Route Finder 알고리즘이 공유하는 status형 callback과 기본 평가 함수를 제공하는
 * public component다. Callback 결과는 유한한 단정도 실수여야 한다.
 */

#ifndef BYUL_ROUTE_FINDER_EVALUATION_H
#define BYUL_ROUTE_FINDER_EVALUATION_H

#include "byul_config.h"
#include "coord.h"
#include "navgrid.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief 한 grid edge의 이동 비용을 계산한다.
 *
 * Callback은 탐색을 실행한 thread에서 동기 호출된다. 성공 결과는 유한하고 0 이상이어야
 * 하며, 입력 pointer는 호출 중에만 유효하다.
 *
 * @param[in] navgrid 탐색 중인 navigation grid.
 * @param[in] from Edge 시작 좌표.
 * @param[in] to Edge 끝 좌표.
 * @param[out] out_cost 계산한 이동 비용.
 * @param[in] userdata Bind할 때 전달한 caller 소유 data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK out_cost에 유효한 비용을 기록했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED 비용을 계산하지 못했다.
 *
 * @byul.nullable navgrid false
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_cost false
 * @byul.nullable userdata true
 * @byul.lifetime navgrid call-only
 * @byul.lifetime from call-only
 * @byul.lifetime to call-only
 * @byul.lifetime userdata until-unbind
 * @byul.pointer_role userdata callback-userdata
 * @byul.callback_thread caller
 * @byul.reentrant false
 * @byul.callback_failure stop-operation
 */
typedef navsys_status_t (*route_finder_cost_func_ex)(
    const navgrid_t* navgrid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost,
    void* userdata);

/*
 * @brief 좌표에서 goal까지의 추정 비용을 계산한다.
 *
 * Callback은 탐색을 실행한 thread에서 동기 호출된다. 성공 결과는 유한해야 하며,
 * 최적성 보장을 요청하는 알고리즘에서는 0 이상이어야 한다.
 *
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate 계산한 추정 비용.
 * @param[in] userdata Bind할 때 전달한 caller 소유 data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK out_estimate에 유효한 값을 기록했다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED 값을 계산하지 못했다.
 *
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.lifetime from call-only
 * @byul.lifetime goal call-only
 * @byul.lifetime userdata until-unbind
 * @byul.pointer_role userdata callback-userdata
 * @byul.callback_thread caller
 * @byul.reentrant false
 * @byul.callback_failure stop-operation
 */
typedef navsys_status_t (*route_finder_heuristic_func_ex)(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void* userdata);

/**
 * @brief 모든 유효 edge에 단위 비용 1을 반환한다.
 * @param[in] navgrid 평가할 grid.
 * @param[in] from Edge 시작 좌표.
 * @param[in] to Edge 끝 좌표.
 * @param[out] out_cost 단위 비용을 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비용을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable navgrid false
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_cost false
 * @byul.nullable userdata true
 * @byul.side_effect mutates:out_cost
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_cost_unit(
    const navgrid_t* navgrid, const coord_t* from, const coord_t* to,
    float* out_cost, void* userdata);

/**
 * @brief 모든 유효 edge에 비용 0을 반환한다.
 * @param[in] navgrid 평가할 grid.
 * @param[in] from Edge 시작 좌표.
 * @param[in] to Edge 끝 좌표.
 * @param[out] out_cost 0을 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비용을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable navgrid false
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_cost false
 * @byul.nullable userdata true
 * @byul.side_effect mutates:out_cost
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_cost_zero(
    const navgrid_t* navgrid, const coord_t* from, const coord_t* to,
    float* out_cost, void* userdata);

/**
 * @brief 직교 edge에는 1, 대각 edge에는 sqrt(2) 비용을 반환한다.
 * @param[in] navgrid 평가할 grid.
 * @param[in] from Edge 시작 좌표.
 * @param[in] to Edge 끝 좌표.
 * @param[out] out_cost Grid cell 단위 이동 비용을 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비용을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이거나 인접 edge가 아니다.
 * @byul.nullable navgrid false
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_cost false
 * @byul.nullable userdata true
 * @byul.unit out_cost grid-cells
 * @byul.side_effect mutates:out_cost
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_cost_diagonal(
    const navgrid_t* navgrid, const coord_t* from, const coord_t* to,
    float* out_cost, void* userdata);

/**
 * @brief Euclidean 추정 거리를 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate Grid cell 단위 추정 거리를 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 추정 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이거나 결과가 유한하지 않다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate grid-cells
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_euclidean(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

/**
 * @brief Manhattan 추정 거리를 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate Grid cell 단위 추정 거리를 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 추정 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이거나 결과가 유한하지 않다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate grid-cells
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_manhattan(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

/**
 * @brief Chebyshev 추정 거리를 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate Grid cell 단위 추정 거리를 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 추정 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate grid-cells
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_chebyshev(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

/**
 * @brief Octile 추정 거리를 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate Grid cell 단위 추정 거리를 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 추정 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이거나 결과가 유한하지 않다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate grid-cells
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_octile(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

/**
 * @brief 항상 0인 추정 비용을 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate 0을 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 0을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate unitless
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_zero(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

/**
 * @brief 비용 모델과 무관하게 admissible한 기본값 0을 반환한다.
 * @param[in] from 평가할 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_estimate 0을 받는다.
 * @param[in] userdata 사용하지 않는 caller data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 0을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable from false
 * @byul.nullable goal false
 * @byul.nullable out_estimate false
 * @byul.nullable userdata true
 * @byul.unit out_estimate unitless
 * @byul.side_effect mutates:out_estimate
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_finder_heuristic_default(
    const coord_t* from, const coord_t* goal, float* out_estimate,
    void* userdata);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_ROUTE_FINDER_EVALUATION_H */
