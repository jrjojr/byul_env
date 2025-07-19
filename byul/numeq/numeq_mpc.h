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
 * @struct mpc_config_t
 * @brief MPC(Model Predictive Control) 시뮬레이션 구성 파라미터
 *
 * 이 구조체는 MPC 기반 경로 예측 및 제어 알고리즘에서 사용되는 다양한 파라미터를 정의합니다.
 * 주로 시간 범위, 속도/가속도 제한, 비용 함수 가중치 등이 포함됩니다.
 *
 * **변수 설명 및 기본값:**
 * - horizon_sec = 1.0f  
 *   예측 시간 범위(초 단위). MPC가 미래를 예측하는 총 시간 구간입니다.  
 *   예: horizon_sec = 1.0f → 1초 후까지의 경로 예측.
 *
 * - step_dt = 0.05f  
 *   시뮬레이션 시간 간격. horizon_sec을 이 값으로 나누어 예측 스텝 수를 계산합니다.  
 *   예: horizon_sec = 1.0f, step_dt = 0.05f → 20스텝 예측.
 *
 * - max_accel = 10.0f  
 *   최대 선형 가속도 크기 제한 (m/s²). MPC 후보 액션에서 선형 가속도를 이 값 이하로 제한합니다.
 *
 * - max_ang_accel = 5.0f  
 *   최대 각가속도 크기 제한 (rad/s²).
 *
 * - max_speed = 50.0f  
 *   최대 선형 속도 제한 (m/s).  
 *   MPC가 경로 최적화 시 속도를 이 범위 안에서 유지하도록 강제합니다.
 *
 * - max_ang_speed = 10.0f  
 *   최대 각속도 제한 (rad/s).
 *
 * - weight_distance = 1.0f  
 *   목표 거리 오차에 대한 비용 가중치.  
 *   값이 클수록 목표 위치에 빠르게 접근하도록 제어됩니다.
 *
 * - weight_orientation = 0.5f  
 *   목표 회전(자세) 오차에 대한 비용 가중치.  
 *   값이 클수록 목표 자세(쿼터니언)에 맞추려는 힘이 강해집니다.
 *
 * - weight_velocity = 0.1f  
 *   속도 안정성을 위한 비용 가중치.  
 *   속도 변화를 최소화하도록 유도합니다.
 *
 * - weight_accel = 0.1f  
 *   가속도 변화에 대한 비용 가중치.  
 *   급격한 가속/감속을 줄이려면 값을 높입니다.
 *
 * - weight_ang_accel = 0.1f  
 *   각가속도 변화에 대한 비용 가중치.
 *
 * - max_iter = 10  
 *   내부 최적화 반복 횟수.  
 *   MPC가 후보 가속도를 반복 탐색할 횟수를 제한합니다.
 *
 * - output_trajectory = false  
 *   true로 설정하면 예측 경로(trajectory)를 외부에 출력/저장합니다.
 *
 * - candidate_step = 0.5f  
 *   선형 가속도 후보 간격.  
 *   예: -max_accel ~ max_accel 범위를 0.5 단위로 샘플링.
 *
 * - ang_candidate_step = 0.1f  
 *   각가속도 후보 간격.
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
    float weight_velocity;
    float weight_accel;         /**< 가속도 비용 가중치 */
    float weight_ang_accel;     /**< 각가속도 비용 가중치 */
    int max_iter;               /**< 내부 반복 횟수 제한 */
    bool output_trajectory;     /**< 예측 경로 출력 여부 */
    float candidate_step;       /**< 가속도 후보 간격 */
    float ang_candidate_step;   /**< 각가속도 후보 간격 */
} mpc_config_t;

/**
 * @brief mpc_config_t 기본값 초기화
 *
 * 기본값:
 * - horizon_sec = 1.0f
 * - step_dt = 0.05f
 * - max_accel = 10.0f
 * - max_ang_accel = 5.0f
 * - max_speed = 50.0f
 * - max_ang_speed = 10.0f
 * - weight_distance = 1.0f
 * - weight_orientation = 0.5f
 * - weight_velocity = 0.1f
 * - weight_accel = 0.1f
 * - weight_ang_accel = 0.1f
 * - max_iter = 10
 * - output_trajectory = false
 * - candidate_step = 0.5f
 * - ang_candidate_step = 0.1f
 *
 * @param cfg 초기화할 mpc_config_t 구조체
 */
BYUL_API void mpc_config_init(mpc_config_t* cfg);

/**
 * @brief mpc_config_t를 지정한 값으로 초기화
 *
 * 이 함수는 전달된 파라미터로 mpc_config_t 구조체를 초기화합니다.
 * 지정하지 않은 항목은 다음 기본값을 참고하여 설정할 수 있습니다.
 *
 * **기본값:**
 * - horizon_sec = 1.0f (미래 1초 동안의 예측)
 * - step_dt = 0.05f (50ms 간격 스텝)
 * - max_accel = 10.0f (최대 선형 가속도 [m/s²])
 * - max_ang_accel = 5.0f (최대 각가속도 [rad/s²])
 * - max_speed = 50.0f (최대 선형 속도 [m/s])
 * - max_ang_speed = 10.0f (최대 각속도 [rad/s])
 * - weight_distance = 1.0f (목표 위치 오차 비용 가중치)
 * - weight_orientation = 0.5f (목표 회전 오차 비용 가중치)
 * - weight_velocity = 0.1f (속도 안정화 비용 가중치)
 * - weight_accel = 0.1f (가속도 비용 가중치)
 * - weight_ang_accel = 0.1f (각가속도 비용 가중치)
 * - max_iter = 10 (MPC 내부 최적화 반복 횟수)
 * - output_trajectory = false (예측 경로 출력 여부)
 * - candidate_step = 0.5f (선형 가속도 후보 간격)
 * - ang_candidate_step = 0.1f (각가속도 후보 간격)
 *
 * @param cfg 초기화할 구조체
 * @param horizon_sec 예측 시간 범위 (초)
 * @param step_dt 시뮬레이션 스텝 간격 (초)
 * @param max_accel 최대 선형 가속도 [m/s²]
 * @param max_ang_accel 최대 각가속도 [rad/s²]
 * @param max_speed 최대 선형 속도 [m/s]
 * @param max_ang_speed 최대 각속도 [rad/s]
 * @param weight_distance 거리 오차 비용 가중치
 * @param weight_orientation 회전 오차 비용 가중치
 * @param weight_velocity 속도 비용 가중치
 * @param weight_accel 가속도 비용 가중치
 * @param weight_ang_accel 각가속도 비용 가중치
 * @param max_iter 내부 반복 횟수
 * @param output_trajectory 예측 경로 출력 여부
 * @param candidate_step 가속도 후보 간격
 * @param ang_candidate_step 각가속도 후보 간격
 */
BYUL_API void mpc_config_init_full(mpc_config_t* cfg,
                          float horizon_sec,
                          float step_dt,
                          float max_accel,
                          float max_ang_accel,
                          float max_speed,
                          float max_ang_speed,
                          float weight_distance,
                          float weight_orientation,
                          float weight_velocity,
                          float weight_accel,
                          float weight_ang_accel,
                          int max_iter,
                          bool output_trajectory,
                          float candidate_step,
                          float ang_candidate_step);

/**
 * @brief mpc_config_t 복사
 */
BYUL_API void mpc_config_copy(mpc_config_t* out, const mpc_config_t* src);

/**
 * @brief 다중 지점 기반 목표 경로
 */
typedef struct s_mpc_target_route {
    const vec3_t* points;       /**< 목표 지점 배열 */
    int count;
    bool loop;
} mpc_target_route_t;

// ---------------------------------------------------------
// mpc_target_route_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief mpc_target_route_t 기본값 초기화
 */
BYUL_API void mpc_target_route_init(mpc_target_route_t* route);

/**
 * @brief mpc_target_route_t 지정 값 초기화
 */
BYUL_API void mpc_target_route_init_full(mpc_target_route_t* route,
                                const vec3_t* points,
                                int count,
                                bool loop);

/**
 * @brief mpc_target_route_t 복사
 */
BYUL_API void mpc_target_route_copy(mpc_target_route_t* out,
                           const mpc_target_route_t* src);


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
// mpc_direction_target_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief mpc_direction_target_t 기본값 초기화
 */
BYUL_API void mpc_direction_target_init(mpc_direction_target_t* target);

/**
 * @brief mpc_direction_target_t 지정 값 초기화
 */
void mpc_direction_target_init_full(mpc_direction_target_t* target,
                                    const vec3_t* direction,
                                    const quat_t* orientation,
                                    float weight_dir,
                                    float weight_rot,
                                    float duration);
/**
 * @brief mpc_direction_target_t 복사
 */
void mpc_direction_target_copy(mpc_direction_target_t* out,
                               const mpc_direction_target_t* src);


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

// ---------------------------------------------------------
// 📐 사용자 정의 비용 함수 타입
// ---------------------------------------------------------

/**
 * @brief 사용자 정의 비용 함수 타입
 *
 * 이 함수 포인터는 MPC에서 특정 가속도 및 각가속도 후보에 대한 비용을 계산하기 위해 사용됩니다.
 * 사용자는 @ref numeq_mpc_cost_default 와 같은 기본 구현을 사용하거나,
 * 새로운 비용 함수를 정의하여 MPC의 최적화 전략을 변경할 수 있습니다.
 *
 * @param sim_state   시뮬레이션 상태 (현재 위치, 속도, 가속도, 자세 포함)
 * @param target      목표 상태 (목표 위치, 목표 속도, 목표 자세 포함)
 * @param userdata    외부 데이터 포인터 (가중치 등 사용자 정의 파라미터)
 * @return float      계산된 비용 값 (작을수록 더 우수한 후보)
 */
typedef float (*mpc_cost_func)(
    const motion_state_t* sim_state, /**< 시뮬레이션 상태 */
    const motion_state_t* target,    /**< 목표 상태 */
    void* userdata);                 /**< 외부 데이터 */

/**
 * @brief 기본 비용 함수 (거리 + 회전)
 *
 * 시뮬레이션 상태와 목표 상태 간의 위치 오차 및 회전 오차를 계산하여 비용을 산출합니다.
 * 가속도에 대한 제약도 포함됩니다.
 *
 * 비용 공식:
 * @f[
 *   cost = w_{dist} \cdot ||p - p_{target}||^2
 *        + w_{rot} \cdot (\Delta \theta)^2
 *        + w_{acc} \cdot ||a||^2
 *        + w_{ang} \cdot ||\alpha||^2
 * @f]
 *
 * @param sim_state   시뮬레이션 상태 (현재 위치, 속도, 가속도, 자세 포함)
 * @param target      목표 상태 (목표 위치 및 자세)
 * @param userdata    mpc_config_t* 또는 사용자 정의 데이터
 * @return float      계산된 비용 값
 */
BYUL_API float numeq_mpc_cost_default(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

/**
 * @brief 속력 중심 비용 함수
 *
 * 현재 속도 크기와 목표 속도 크기의 차이를 기반으로 비용을 계산합니다.
 * 위치 오차는 고려하지 않고, 속도를 특정 범위에 맞추는 데 집중합니다.
 *
 * 비용 공식:
 * @f[
 *   cost = w_{speed} \cdot (||v|| - v_{target})^2
 *        + w_{acc} \cdot ||a||^2
 * @f]
 *
 * @param sim_state   시뮬레이션 상태 (현재 속도 포함)
 * @param target      목표 상태 (target.linear.velocity.x = 목표 속도)
 * @param userdata    mpc_config_t* 또는 사용자 정의 데이터
 * @return float      계산된 비용 값
 */
BYUL_API float numeq_mpc_cost_speed(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

/**
 * @brief 하이브리드 비용 함수 (거리 + 속도 + 회전)
 *
 * 위치 오차, 속도 오차, 회전 오차를 종합적으로 고려하여 비용을 산출합니다.
 * @ref numeq_mpc_cost_default 와 @ref numeq_mpc_cost_speed 의 조합 형태입니다.
 *
 * 비용 공식:
 * @f[
 *   cost = w_{dist} \cdot ||p - p_{target}||^2
 *        + w_{vel} \cdot ||v - v_{target}||^2
 *        + w_{rot} \cdot (\Delta \theta)^2
 *        + w_{acc} \cdot ||a||^2
 *        + w_{ang} \cdot ||\alpha||^2
 * @f]
 *
 * @param sim_state   시뮬레이션 상태 (현재 위치, 속도, 가속도, 자세 포함)
 * @param target      목표 상태 (목표 위치, 목표 속도, 목표 자세)
 * @param userdata    mpc_config_t* 또는 사용자 정의 데이터
 * @return float      계산된 비용 값
 */    
BYUL_API float numeq_mpc_cost_hybrid(
    const motion_state_t* sim_state,
    const motion_state_t* target,
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
