/**
 * @file numeq_mpc.h
 * @brief Model Predictive Control (MPC) 제어 모듈 헤더
 *
 * 이 헤더는 물리 기반 시뮬레이션에서 위치 예측, 목표 추적, 유도 제어를 위해 사용되는
 * Model Predictive Control(MPC) 알고리즘을 제공합니다. 
 *
 * ## ✅ MPC 개요
 * Model Predictive Control은 다음과 같은 절차로 동작합니다:
 *
 * 1. **현재 상태에서 여러 가속도 후보를 적용하여 미래를 예측**
 * 2. **예측된 결과와 목표 지점 사이의 비용(cost)을 계산**
 * 3. **비용이 가장 낮은 가속도를 선택하여 적용**
 * 4. **다음 프레임에서 다시 반복**
 *
 * MPC는 아래와 같은 상황에 적합합니다:
 * - 포탄 및 미사일의 궤적 예측과 유도
 * - 환경 변화(바람, 중력) 대응
 * - 목표 위치/방향/경로 추종
 * - 제약 조건(최대 가속도, 속도 등) 포함된 제어
 *
 * 본 모듈은 다음을 지원합니다:
 * - 🔹 단일 목표점 기반 MPC (`numeq_mpc_solve`)
 * - 🔹 다중 waypoint 기반 경로 추종 (`numeq_mpc_solve_route`)
 * - 🔹 방향 유지형 목표 제어 (`numeq_mpc_solve_directional`)
 * - 🔹 사용자 정의 비용 함수 (`mpc_cost_func`)
 * - 🔹 trajectory 예측 결과 저장 및 디버깅
 *
 * 💡 내부적으로는 상태벡터(state_vector_t)를 기반으로 시간 간격(step_dt)마다 예측을 반복하며,
 *     사용자가 제공한 비용함수를 최소화하는 방향으로 선택합니다.
 */

#ifndef NUMEQ_MPC_H
#define NUMEQ_MPC_H

#include "internal/numeq_common.h"
#include "internal/numeq_model.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 🎯 핵심 구조체 정의
// ---------------------------------------------------------

/**
 * @brief MPC 제어 결과 출력 구조체
 * 
 * MPC 연산 후 즉시 적용 가능한 가속도와 미래 예측 목표점,
 * 그리고 해당 결과의 비용 값을 포함합니다.
 */
typedef struct s_mpc_output {
    vec3_t desired_accel;      /**< 최종 선택된 가속도 (즉시 적용 대상) */
    vec3_t future_target;      /**< 예측된 미래 타겟 (디버깅 또는 보정용) */
    float cost;                /**< 총 비용 함수 결과 (낮을수록 우수) */
} mpc_output_t;

/**
 * @brief MPC 시뮬레이션 구성 파라미터
 * 
 * 예측 시간, 시간 간격, 제약 조건, 비용 가중치 등 포함.
 * MPC 내부 동작을 정밀하게 조절할 수 있는 설정 값입니다.
 */
typedef struct s_mpc_config {
    float horizon_sec;         /**< 예측 시간 범위 (초 단위) */
    float step_dt;             /**< 시뮬레이션 시간 간격 (예: 0.05초) */
    float max_accel;           /**< 최대 가속도 크기 제한 */
    float max_speed;           /**< 최대 속도 제한 (0.0이면 무제한) */
    float weight_distance;     /**< 거리 오차에 대한 비용 가중치 */
    float weight_accel;        /**< 가속도 크기에 대한 비용 가중치 */
    int max_iter;              /**< 내부 반복 횟수 제한 */
    bool output_trajectory;    /**< 예측 경로 출력 여부 (True면 trajectory 저장) */
} mpc_config_t;

/**
 * @brief 예측된 trajectory 저장 구조체
 * 
 * MPC 실행 결과를 시간 순서대로 저장하여 시각화/분석에 활용.
 */
typedef struct s_mpc_trajectory {
    trajectory_sample_t* samples;  /**< 예측된 경로 샘플 배열 */
    int count;                     /**< 유효한 샘플 수 */
    int capacity;                  /**< 할당된 샘플 수 */
} mpc_trajectory_t;

/**
 * @brief 다중 지점 기반의 목표 경로 설정
 */
typedef struct s_mpc_target_route {
    const vec3_t* points;      /**< 목표 지점 배열 (world 기준) */
    int count;                 /**< 총 목표 개수 */
    bool loop;                 /**< true이면 루프 반복 */
} mpc_target_route_t;

/**
 * @brief 방향 유지 기반 제어 목표 설정
 */
typedef struct s_mpc_direction_target {
    vec3_t direction;          /**< 단위 벡터로 표현된 목표 진행 방향 */
    float weight;              /**< 방향 유지에 대한 비용 가중치 */
    float duration;            /**< 해당 방향 유지 예상 시간 (초) */
} mpc_direction_target_t;

// ---------------------------------------------------------
// 📐 사용자 정의 비용 함수 타입
// ---------------------------------------------------------

/**
 * @brief 사용자 정의 비용 함수 타입
 * 
 * 가속도 및 예측 상태에 기반하여 비용(낮을수록 우수)을 계산합니다.
 * MPC는 이 값을 최소화하는 방향으로 동작합니다.
 */
typedef float (*mpc_cost_func)(
    const state_vector_t* sim_state,  /**< 시뮬레이션 상태 */
    const vec3_t* target,             /**< 목표 위치 */
    const vec3_t* accel,              /**< 현재 평가 중인 가속도 */
    void* userdata);                  /**< 외부 전달 데이터 */

/**
 * @brief 기본 비용 함수 구현 (거리 + 가속도 가중치)
 * 
 * 거리 오차^2 + (가속도 크기)^2 × weight_accel 형식으로 계산
 */
BYUL_API float numeq_mpc_cost_default(
    const state_vector_t* sim_state,
    const vec3_t* target,
    const vec3_t* accel,
    void* userdata);

// ---------------------------------------------------------
// 🧠 메인 MPC 함수들
// ---------------------------------------------------------

/**
 * @brief 기본 목표 위치를 향한 MPC 계산 함수
 * 
 * 단일 목표 위치에 대해 예측 기반의 가속도를 계산합니다.
 * 비용 최소화 및 가속도 제약 조건을 반영하며 trajectory 출력도 지원됩니다.
 */
BYUL_API bool numeq_mpc_solve(
    const state_vector_t* current_state,
    const vec3_t* target,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    mpc_trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 경로 경유점 기반 MPC 계산 함수
 * 
 * 다중 waypoints를 순차적으로 통과하도록 제어합니다.
 * loop 설정 시 도달 후 반복 가능.
 */
BYUL_API bool numeq_mpc_solve_route(
    const state_vector_t* current_state,
    const mpc_target_route_t* route,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    mpc_trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 목표 방향을 일정 시간 유지하는 제어용 MPC
 */
BYUL_API bool numeq_mpc_solve_directional(
    const state_vector_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    mpc_trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

// ---------------------------------------------------------
// 🛠 trajectory 메모리 유틸리티
// ---------------------------------------------------------

/**
 * @brief trajectory 구조체를 초기화하고 메모리 할당
 * @param traj 대상 구조체
 * @param capacity 최대 샘플 수
 * @return true 성공 여부
 */
BYUL_API bool mpc_trajectory_init(mpc_trajectory_t* traj, int capacity);

/**
 * @brief trajectory 구조체 내부 메모리 해제
 * @param traj 대상 trajectory
 */
BYUL_API void mpc_trajectory_free(mpc_trajectory_t* traj);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MPC_H
