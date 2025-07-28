#ifndef ENTITY_AVOIDANCE_H
#define ENTITY_AVOIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"  // 기존 엔티티 구조체 참조

// ---------------------------------------------------------
// 회피용 경계 구조체
// ---------------------------------------------------------
/**
 * @struct avoidance_boundary_t
 * @brief 엔티티가 회피할 수 있는 영역
 *
 * - center: 경계 중심 좌표
 * - radius: 경계 반경
 */
typedef struct s_avoidance_boundary {
    vec3_t center;
    float radius;
} avoidance_boundary_t;

// ---------------------------------------------------------
// 충돌 예측 및 회피 벡터 계산
// ---------------------------------------------------------
/**
 * @brief 회피 필요 여부 판단
 */
bool entity_avoidance_need(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist);

/**
 * @brief 회피 방향 벡터 계산
 */
vec3_t entity_avoidance_direction(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist,
    float k);

/**
 * @brief self가 other와 충돌할 가능성을 예측하고 회피 벡터를 계산.
 *
 * @param self      대상 엔티티
 * @param other     상대 엔티티
 * @param safe_dist 안전 거리 (이 거리 이내로 접근하면 회피 필요)
 * @param k         회피 강도 계수
 * @return          회피 벡터 (필요 없으면 (0,0,0))
 */
vec3_t entity_avoidance_calc_single(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist,
    float k);

/**
 * @brief self가 여러 엔티티로부터 받을 총합 회피 벡터를 계산.
 *
 * @param self      대상 엔티티
 * @param others    상대 엔티티 배열
 * @param count     others 배열 크기
 * @param safe_dist 안전 거리
 * @param k         회피 강도 계수
 * @return          총합 회피 벡터
 */
vec3_t entity_avoidance_calc_multi(
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float safe_dist,
    float k);

// ---------------------------------------------------------
// 경로 기록 구조체
// ---------------------------------------------------------
#define AVOIDANCE_MAX_STEP 256

/**
 * @struct entity_avoidance_traj_t
 * @brief 회피 이동 경로 기록
 *
 * - path: 이동 위치
 * - count: 기록된 스텝 수
 */
typedef struct s_entity_avoidance_traj {
    vec3_t path[AVOIDANCE_MAX_STEP];
    int count;
} entity_avoidance_traj_t;

// ---------------------------------------------------------
// 회피 시뮬레이션
// ---------------------------------------------------------
/**
 * @brief 엔티티가 목표점을 향해 이동하면서 다른 엔티티를 회피하는 경로를 생성.
 *
 * @param e          대상 엔티티
 * @param others     상대 엔티티 배열
 * @param count      others 배열 크기
 * @param traj       회피 경로 기록 (NULL 가능)
 * @param boundary   회피 가능한 영역 (NULL이면 무제한)
 * @param dt         시간 간격 (초)
 * @param safe_dist  안전 거리
 * @param k          회피 강도 계수
 * @param steps      시뮬레이션 스텝 수
 */
void entity_avoidance_auto(
    entity_dynamic_t* e,
    const entity_dynamic_t* others,
    int count,
    entity_avoidance_traj_t* traj,
    const avoidance_boundary_t* boundary,
    float dt, float safe_dist, float k, int steps);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_AVOIDANCE_H
