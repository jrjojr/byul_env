#ifndef ENTITY_SPRING_H
#define ENTITY_SPRING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "internal/entity_dynamic.h"

// ---------------------------------------------------------
// 스프링 힘 계산 (Hooke’s Law)
// ---------------------------------------------------------
/**
 * @brief 두 엔티티 간 스프링 기반 힘(반발 + 인력)을 계산합니다.
 *
 * - 목표 거리 L0를 기준으로:
 *   - d < L0 → 반발력 (밀어냄)
 *   - d > L0 → 인력 (당김)
 *   - d = L0 → 힘 없음
 *
 * @param self    기준 엔티티
 * @param other   비교 엔티티
 * @param k       스프링 강성 계수 (k > 0)
 * @param c       감쇠 계수 (0 ~ 1, 속도 감쇠)
 * @param L0      목표 거리 (m)
 * @return        self가 other로부터 받는 스프링형 힘 벡터
 */
vec3_t entity_dynamic_calc_spring_force(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float k,
    float c,
    float L0);

/**
 * @brief self 엔티티가 다른 모든 엔티티로부터 받는 총합 스프링 힘을 계산합니다.
 *
 * @param self    기준 엔티티
 * @param others  비교 엔티티 배열
 * @param count   others 배열 크기
 * @param k       스프링 강성 계수
 * @param c       감쇠 계수
 * @param L0      목표 거리
 * @return        총합 스프링 힘 벡터
 */
vec3_t entity_dynamic_calc_spring_total(
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float k,
    float c,
    float L0);

// ---------------------------------------------------------
// 스프링 기반 시뮬레이션
// ---------------------------------------------------------
#define SPRING_MAX_STEP 256

/**
 * @struct entity_spring_traj_t
 * @brief 스프링 시뮬레이션 경로 기록
 *
 * - path: 위치 기록
 * - force: 각 스텝의 스프링 힘
 * - count: 기록된 스텝 수
 */
typedef struct s_entity_spring_traj {
    vec3_t path[SPRING_MAX_STEP];
    vec3_t force[SPRING_MAX_STEP];
    int count;
} entity_spring_traj_t;

/**
 * @brief 스프링 기반 거리 유지 시뮬레이션
 *
 * - 각 엔티티는 목표 거리(L0)를 기준으로 서로 스프링처럼 당기고 밀어냅니다.
 * - 감쇠 계수 c로 흔들림이 점점 줄어드는 자연스러운 효과를 구현합니다.
 *
 * @param e        엔티티 배열
 * @param count    엔티티 개수
 * @param traj     각 엔티티의 경로 기록 배열 (NULL 가능)
 * @param dt       시간 간격 (초)
 * @param k        스프링 강성 계수
 * @param c        감쇠 계수 (0 ~ 1)
 * @param L0       목표 거리 (m)
 * @param steps    시뮬레이션 스텝 수
 */
void entity_dynamic_auto_spring(
    entity_dynamic_t* e, int count,
    entity_spring_traj_t* traj,
    float dt, float k, float c, float L0,
    int steps);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_SPRING_H
