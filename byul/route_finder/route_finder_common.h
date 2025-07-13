#ifndef ROUTE_FINDER_COMMON_H
#define ROUTE_FINDER_COMMON_H

#include "byul_config.h"
#include "internal/map.h"
#include "internal/coord.h"
#include "internal/route.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DIAGONAL_COST 1.4142135f  // √2 근사값

/**
 * @brief 비용 함수 타입
 * 
 * @param m 지도 객체
 * @param start 시작 좌표
 * @param goal 목표 좌표
 * @param userdata 사용자 정의 데이터
 * @return float 비용 값
 */
typedef float (*cost_func)(
    const map_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief 휴리스틱 함수 타입
 * 
 * @param start 시작 좌표
 * @param goal 목표 좌표
 * @param userdata 사용자 정의 데이터
 * @return float 추정 거리
 */
typedef float (*heuristic_func)(const coord_t*, const coord_t*, void*);

/**
 * @brief 기본 비용 함수 (이동 가능 여부만 판단) 1.0 반환
 */
BYUL_API float default_cost(
    const map_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief 0을 반환하는 비용 함수 (모든 경로 동일 비용)
 */
BYUL_API float zero_cost(const map_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief 대각선 이동 비용 함수 (√2 근사값 사용)
 */
BYUL_API float diagonal_cost(
    const map_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief 유클리드 거리 휴리스틱
 */
BYUL_API float euclidean_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief 맨해튼 거리 휴리스틱
 */
BYUL_API float manhattan_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief 체비셰프 거리 휴리스틱
 */
BYUL_API float chebyshev_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief 옥타일 거리 휴리스틱 (8방향 이동)
 */
BYUL_API float octile_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief 항상 0을 반환하는 휴리스틱 (탐색 최소화용)
 */
BYUL_API float zero_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief 기본 휴리스틱 ( 유클리드)
 */
BYUL_API float default_heuristic(const coord_t*, const coord_t*, void*);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_FINDER_COMMON_H
