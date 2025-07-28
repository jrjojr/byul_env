#ifndef ENTITY_SPRING_H
#define ENTITY_SPRING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"
#include "trajectory.h"

// ---------------------------------------------------------
// Spring Force (Hooke's Law)
// ---------------------------------------------------------
/**
 * @brief 두 엔티티 사이의 스프링 힘을 계산합니다.
 *
 * 스프링 모델은 후크의 법칙(Hooke's Law)과 감쇠력을 기반으로 합니다:
 * 
 * F = -k (d - L0) * d_hat - c (v_rel · d_hat) * d_hat
 *
 * - d : 두 엔티티의 거리
 * - L0 : 스프링의 자연 길이 (목표 거리)
 * - k : 스프링 강성 계수 (강할수록 힘이 크게 작용)
 * - c : 감쇠 계수 (속도에 비례하는 저항)
 * - d_hat : 두 엔티티 사이의 단위 벡터
 *
 * @param[out] out   계산된 힘 벡터 (NULL이면 무시)
 * @param[in]  a     기준 엔티티 (힘이 작용하는 쪽)
 * @param[in]  b     상대 엔티티
 * @param[in]  k     스프링 강성 계수 (k > 0)
 * @param[in]  c     감쇠 계수 (>= 0)
 * @param[in]  L0    목표 거리 (m)
 *
 * @note
 * - d < L0 → 반발력 (밀어냄)
 * - d > L0 → 인력 (당김)
 * - d = L0 → 힘 없음
 */
BYUL_API void spring_force(vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    float k,
    float c,
    float L0);

/**
 * @brief 한 엔티티가 여러 엔티티로부터 받는 총합 스프링 힘을 계산합니다.
 *
 * spring_force()를 여러 번 호출하여 자기 자신을 제외한 모든 엔티티와의 힘을 합산합니다.
 *
 * @param[out] out   총합 힘 벡터 (NULL이면 무시)
 * @param[in]  self  기준 엔티티
 * @param[in]  others 비교 엔티티 배열
 * @param[in]  count others 배열 크기
 * @param[in]  k     스프링 강성 계수
 * @param[in]  c     감쇠 계수
 * @param[in]  L0    목표 거리
 */
BYUL_API void spring_force_total(vec3_t* out,
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float k,
    float c,
    float L0);

/**
 * @brief 전체 스프링 기반 거리 유지 시뮬레이션을 수행합니다.
 *
 * 각 엔티티는 다른 모든 엔티티와 스프링으로 연결되어 있다고 가정하며,
 * 후크의 법칙과 감쇠를 통해 자연 길이 L0를 유지하려는 힘이 작용합니다.
 *
 * @param[out] traj   엔티티 궤적 기록 포인터 (NULL 가능)
 * @param[in,out] e   엔티티 배열
 * @param[in]  count  엔티티 개수
 * @param[in]  dt     시뮬레이션 시간 간격 (초)
 * @param[in]  k      스프링 강성 계수
 * @param[in]  c      감쇠 계수
 * @param[in]  L0     목표 거리 (m)
 * @param[in]  steps  시뮬레이션 반복 횟수
 *
 * @note
 * Semi-Implicit Euler Integrator를 사용합니다:
 * 
 * v(t+1) = v(t) + (F/m) * dt
 * x(t+1) = x(t) + v(t+1) * dt
 */
BYUL_API void spring_simulate(trajectory_t* traj,
    entity_dynamic_t* e, int count,
    float dt, float k, float c, float L0,
    int steps);

// ---------------------------------------------------------
// Pairwise Spring Simulation
// ---------------------------------------------------------
/**
 * @brief 엔티티 쌍(pairwise) 간의 스프링 시뮬레이션을 수행합니다.
 *
 * 모든 엔티티 쌍 (i, j)에 대해 스프링 힘을 계산하고, 속도 및 위치를 업데이트합니다.
 *
 * @param[out] traj   엔티티 궤적 기록 포인터 (NULL 가능)
 * @param[in,out] e   엔티티 배열
 * @param[in]  count  엔티티 수
 * @param[in]  dt     시뮬레이션 시간 스텝 (초)
 * @param[in]  k      스프링 강성 계수
 * @param[in]  c      감쇠 계수
 * @param[in]  L0     자연 길이
 * @param[in]  steps  반복 스텝 수
 */
BYUL_API void spring_simulate_pairwise(trajectory_t* traj,
                              entity_dynamic_t* e, int count,
                              float dt, float k, float c, float L0,
                              int steps);

// ---------------------------------------------------------
// Spring Network (Graph-based)
// ---------------------------------------------------------
/**
 * @struct spring_link_t
 * @brief 두 엔티티 간 스프링 연결 정보를 나타내는 구조체.
 *
 * - i, j : 연결된 엔티티 인덱스
 * - k : 스프링 강성 계수
 * - c : 감쇠 계수
 * - L0: 자연 길이
 */
typedef struct s_spring_link {
    int i, j;
    float k;
    float c;
    float L0;
} spring_link_t;

/**
 * @brief 네트워크(그래프) 기반 스프링 시뮬레이션을 수행합니다.
 *
 * 엔티티 배열과 스프링 연결 정보(links)를 이용해,
 * 네트워크 전체에서 스프링 힘을 계산하고 위치 및 속도를 업데이트합니다.
 *
 * @param[out] traj       궤적 기록 포인터 (NULL이면 기록 안 함)
 * @param[in,out] e       엔티티 배열
 * @param[in]  count      엔티티 수
 * @param[in]  links      스프링 연결 배열
 * @param[in]  link_count 링크 개수
 * @param[in]  dt         시뮬레이션 시간 간격 (초)
 * @param[in]  steps      시뮬레이션 반복 스텝 수
 *
 * @note
 * - spring_simulate_pairwise()는 모든 쌍을 계산하지만,
 *   spring_simulate_network()는 지정된 links[]에 대해서만 계산합니다.
 */
BYUL_API void spring_simulate_network(
    trajectory_t* traj,
    entity_dynamic_t* e, int count,
    const spring_link_t* links, int link_count,
    float dt, int steps);

/**
 * @brief Velocity Verlet 적분기를 이용한 반발력(Repulsion) 네트워크 시뮬레이션.
 *
 * 이 함수는 엔티티들 사이의 가상 스프링을 기반으로 한 반발력 네트워크를 시뮬레이션합니다.
 * 각 링크(spring_link_t)는 두 엔티티 사이의 최소 안전거리(L0)를 정의하며,
 * 거리가 이 값보다 좁아질 경우 스프링 반발력과 감쇠력을 계산하여 엔티티를 밀어냅니다.
 * 시뮬레이션은 Velocity Verlet 적분법을 사용하여 안정적으로 위치와 속도를 업데이트합니다.
 *
 * 주요 특징
 * - 각 엔티티는 질량(props.mass), 속도(velocity), 위치(xf.pos)를 가지는 입자(Particle)으로 모델링됩니다.
 * - links 배열은 엔티티 쌍 (i, j)마다 스프링 상수 k, 감쇠 계수 c, 목표 거리 L0를 정의합니다.
 * - 매 시뮬레이션 스텝마다 모든 링크의 힘을 누적 계산한 후, Velocity Verlet 적분기로 위치와 속도를 갱신합니다.
 * - trajectory_t가 주어지면 각 시뮬레이션 스텝의 엔티티 상태(위치/속도)가 기록됩니다.
 *
 * @param[out] traj         시뮬레이션 중 엔티티의 상태를 기록할 trajectory_t 포인터 (NULL 가능)
 * @param[in,out] e         시뮬레이션 대상 엔티티 배열 (entity_dynamic_t[count])
 * @param[in] count         엔티티 개수 (2개 이상이어야 함)
 * @param[in] links         엔티티 간 반발 링크 배열 (spring_link_t[link_count])
 * @param[in] link_count    링크 개수
 * @param[in] dt            시간 스텝 (초 단위)
 * @param[in] steps         시뮬레이션 스텝 수
 *
 * @note
 * - traj가 NULL이면 로그를 기록하지 않고 엔티티의 최종 상태만 업데이트합니다.
 * - Velocity Verlet은 두 번의 절반 속도 업데이트로 오일러 방식보다 안정적입니다.
 * - 모든 엔티티는 props.mass > 0 값을 가져야 힘-가속도 계산이 정상 동작합니다.
 *
 * @warning
 * - count <= 1 또는 link_count <= 0이면 시뮬레이션이 수행되지 않습니다.
 * - dt가 너무 크면 수치적 불안정성이 발생할 수 있으니 0.01~0.05초 정도가 적당합니다.
 *
 * @see spring_link_t
 * @see entity_dynamic_t
 * @see trajectory_t
 */
BYUL_API void repulsion_simulate_network(trajectory_t* traj,
                                entity_dynamic_t* e, int count,
                                const spring_link_t* links, int link_count,
                                float dt, int steps);

/**
 * @brief 외부 힘을 사용해 엔티티를 부드럽게 밀어내는 시뮬레이션
 *
 * - 각 엔티티는 질량(props.mass), 위치(xf.pos), 속도(velocity)를 가진다.
 * - push_forces 배열에서 각 엔티티가 받는 외부 힘을 정의한다.
 * - 스프링 대신 일정한 힘을 계속 가해 사람이 사람을 밀듯이 부드럽게 움직인다.
 * - trajectory_t를 제공하면 각 step마다 위치와 속도를 기록할 수 있다.
 *
 * @param[out] traj        각 step의 궤적을 저장할 수 있는 trajectory_t (NULL 가능)
 * @param[in,out] e        엔티티 배열 (크기 count)
 * @param[in] count        엔티티 개수
 * @param[in] push_forces  각 엔티티에 적용할 외부 힘 배열 (크기 count)
 * @param[in] dt           시뮬레이션 시간 간격 (초)
 * @param[in] steps        시뮬레이션 step 수
 */
BYUL_API void push_simulate_network(trajectory_t* traj,
                                    entity_dynamic_t* e, int count,
                                    const vec3_t* push_forces,
                                    float dt, int steps);                                

#ifdef __cplusplus
}
#endif

#endif // ENTITY_SPRING_H
