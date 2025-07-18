/**
 * @file numeq_mpc.h
 * @brief Model Predictive Control (MPC) 제어 모듈 헤더 (motion_state_t 기반)
 *
 * 이 헤더는 물리 기반 시뮬레이션에서 **위치 + 회전 예측**, 목표 추적, 유도 제어를 위해
 * Model Predictive Control(MPC) 알고리즘을 제공합니다.
 *
 * ## ✅ MPC 개요
 * Model Predictive Control은 다음과 같이 동작합니다:
 * 1. **현재 motion_state_t에서 여러 가속도/각가속도 후보를 적용하여 미래 상태를 예측**
 * 2. **예측된 결과와 목표 지점/자세 간의 비용(cost)을 계산**
 * 3. **비용이 가장 낮은 제어 입력을 선택하여 적용**
 * 4. **다음 프레임에서 다시 반복**
 *
 * MPC는 다음과 같은 상황에 적합합니다:
 * - 포탄/미사일 궤적 + 회전 제어
 * - 환경 변화(바람, 중력) 대응
 * - 목표 위치 + 방향/자세 추종
 * - 제약 조건(최대 가속도/각가속도, 속도 등)
 *
 * 본 모듈은 다음을 지원합니다:
 * - 🔹 단일 목표점 기반 MPC (`numeq_mpc_solve`)
 * - 🔹 다중 waypoint 기반 경로 추종 (`numeq_mpc_solve_route`)
 * - 🔹 방향 유지형 목표 제어 (`numeq_mpc_solve_directional`)
 * - 🔹 사용자 정의 비용 함수 (`mpc_cost_func`)
 * - 🔹 trajectory 예측 결과 저장 및 디버깅
 */

#ifndef NUMEQ_MPC_H
#define NUMEQ_MPC_H

#include "internal/trajectory.h"
#include "internal/numeq_model.h"
#include "internal/numal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 🎯 핵심 구조체 정의
// ---------------------------------------------------------

/**
 * @brief MPC 제어 결과 출력 구조체
 * 
 * MPC 연산 후 즉시 적용 가능한 선형/회전 가속도와
 * 예측된 미래 타겟, 비용을 포함합니다.
 */
typedef struct s_mpc_output {
    vec3_t desired_accel;       /**< 최종 선택된 선형 가속도 */
    vec3_t desired_ang_accel;   /**< 최종 선택된 각가속도 */
    motion_state_t future_state;/**< 예측된 미래 상태 (위치+회전) */
    float cost;                 /**< 총 비용 함수 결과 (낮을수록 우수) */
} mpc_output_t;

/**
 * @brief MPC 시뮬레이션 구성 파라미터
 */
typedef struct s_mpc_config {
    float horizon_sec;          /**< 예측 시간 범위 (초 단위) */
    float step_dt;              /**< 시뮬레이션 시간 간격 (예: 0.05초) */
    float max_accel;            /**< 최대 선형 가속도 크기 제한 */
    float max_ang_accel;        /**< 최대 각가속도 크기 제한 */
    float max_speed;            /**< 최대 선형 속도 제한 */
    float max_ang_speed;        /**< 최대 각속도 제한 */
    float weight_distance;      /**< 거리 오차 비용 가중치 */
    float weight_orientation;   /**< 회전 오차 비용 가중치 */
    float weight_accel;         /**< 가속도 비용 가중치 */
    float weight_ang_accel;     /**< 각가속도 비용 가중치 */
    int max_iter;               /**< 내부 반복 횟수 제한 */
    bool output_trajectory;     /**< 예측 경로 출력 여부 */
    float candidate_step;       /**< 가속도 후보 간격 */
    float ang_candidate_step;   /**< 각가속도 후보 간격 */
} mpc_config_t;

/**
 * @brief 다중 지점 기반 목표 경로
 */
typedef struct s_mpc_target_route {
    const vec3_t* points;       /**< 목표 지점 배열 */
    int count;
    bool loop;
} mpc_target_route_t;

/**
 * @brief 방향 유지 기반 제어 목표
 */
typedef struct s_mpc_direction_target {
    vec3_t direction;           /**< 단위 벡터 (목표 진행 방향) */
    quat_t orientation;         /**< 목표 회전 (옵션) */
    float weight_dir;           /**< 방향 유지 비용 가중치 */
    float weight_rot;           /**< 회전 유지 비용 가중치 */
    float duration;             /**< 유지 시간 */
} mpc_direction_target_t;

// ---------------------------------------------------------
// 📐 사용자 정의 비용 함수 타입
// ---------------------------------------------------------

/**
 * @brief 사용자 정의 비용 함수 타입
 */
typedef float (*mpc_cost_func)(
    const motion_state_t* sim_state, /**< 시뮬레이션 상태 */
    const motion_state_t* target,    /**< 목표 상태 */
    const vec3_t* accel,             /**< 현재 평가 중인 선형 가속도 */
    const vec3_t* ang_accel,         /**< 현재 평가 중인 각가속도 */
    void* userdata);                 /**< 외부 데이터 */

/**
 * @brief 기본 비용 함수 (거리 + 회전)
 */
BYUL_API float numeq_mpc_cost_default(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    void* userdata);

/**
 * @brief 속력 중심 비용 함수
 */
BYUL_API float numeq_mpc_cost_speed(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    void* userdata);

// ---------------------------------------------------------
// 🧠 메인 MPC 함수들
// ---------------------------------------------------------

/**
 * @brief 단일 목표 MPC
 */
BYUL_API bool numeq_mpc_solve(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 경유점 기반 MPC
 */
BYUL_API bool numeq_mpc_solve_route(
    const motion_state_t* current_state,
    const mpc_target_route_t* route,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 방향 유지형 MPC
 */
BYUL_API bool numeq_mpc_solve_directional(
    const motion_state_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MPC_H
