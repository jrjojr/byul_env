#ifndef ENTITY_ENCIRCLEMENT_H
#define ENTITY_ENCIRCLEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "internal/entity_dynamic.h"

// ---------------------------------------------------------
// 기초 유틸리티
// ---------------------------------------------------------

/**
 * @brief 적 중심으로 포위 좌표 후보를 계산합니다.
 *
 * - 360°를 ally_count 개로 나누어 원형 포인트 생성
 * - Z축은 무시(2D 평면 기준)
 *
 * @param target        적의 위치
 * @param ally_count    아군 수
 * @param L0            포위 반경
 * @param out_positions 결과 좌표 배열 (ally_count 크기)
 */
void entity_encirclement_generate_ring_positions(
    const vec3_t* target,
    int ally_count,
    float L0,
    vec3_t* out_positions);

/**
 * @brief 특정 좌표에 이미 동료가 자리잡았는지 확인
 *
 * @param allies        아군 엔티티 배열
 * @param ally_count    아군 수
 * @param position      확인할 포위 좌표
 * @param threshold     자리잡음으로 인정할 거리
 * @return true         가까운 동료가 해당 좌표 점유 중
 */
bool entity_encirclement_is_spot_occupied(
    const entity_dynamic_t* allies,
    int ally_count,
    const vec3_t* position,
    float threshold);

/**
 * @brief 가장 가까운 빈 목표 좌표를 아군에게 할당
 *
 * - out_index에 선택된 목표 좌표의 인덱스를 반환
 *
 * @param ally_pos      아군의 현재 위치
 * @param targets       포위 목표 좌표 배열
 * @param target_count  목표 좌표 개수
 * @param used_flags    사용 여부 배열 (0=미사용, 1=사용)
 * @return              선택된 목표 인덱스 (없으면 -1)
 */
int entity_encirclement_find_closest_available_target(
    const vec3_t* ally_pos,
    const vec3_t* targets,
    int target_count,
    int* used_flags);

// ---------------------------------------------------------
// 포위 목표 좌표 관리
// ---------------------------------------------------------

/**
 * @brief 아군들이 적을 기준으로 포위 목표 좌표를 최종 계산
 *
 * - 링 좌표 생성
 * - 자리잡은 아군 좌표 제외
 * - 남은 목표 좌표를 거리 기반으로 아군에게 배정
 *
 * @param allies        아군 엔티티 배열
 * @param ally_count    아군 수
 * @param target        적 위치
 * @param L0            포위 반경
 * @param out_positions 각 아군의 목표 좌표
 */
void entity_encirclement_find_targets(
    const entity_dynamic_t* allies,
    int ally_count,
    const vec3_t* target,
    float L0,
    vec3_t* out_positions);

// ---------------------------------------------------------
// 제어점 계산
// ---------------------------------------------------------
/**
 * @brief 포위 이동을 위한 곡선 제어점(P1, P2)을 계산합니다.
 *
 * @param current_pos   현재 위치 (P0)
 * @param target_pos    목표 위치 (P3)
 * @param velocity      현재 속도 벡터
 * @param out_p1        계산된 제어점 P1
 * @param out_p2        계산된 제어점 P2
 */
void entity_encirclement_calc_control_points(
    const vec3_t* current_pos,
    const vec3_t* target_pos,
    const vec3_t* velocity,
    vec3_t* out_p1,
    vec3_t* out_p2);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_ENCIRCLEMENT_H
