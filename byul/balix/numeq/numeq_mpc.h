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
BYUL_API void mpc_config_assign(mpc_config_t* out, const mpc_config_t* src);

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
BYUL_API void mpc_target_route_assign(mpc_target_route_t* out,
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
void mpc_direction_target_assign(mpc_direction_target_t* out,
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
 * @brief 단일 목표 Model Predictive Control (MPC) 솔버
 *
 * 현재 상태(`current_state`)에서 목표 상태(`target_state`)로 이동하기 위해
 * 가능한 가속도(acceleration) 및 각가속도(angular acceleration) 후보들을
 * 브루트포스 방식으로 평가하고, 비용 함수(`cost_fn`)가 최소화되는
 * 최적의 제어 입력을 계산합니다.
 *
 * ---
 *
 * ## 작동 원리
 * 1. `config->max_accel`, `config->max_ang_accel` 범위에서
 *    각 축별 가속도 후보(-max, 0, +max)를 생성합니다.
 * 2. 모든 조합(총 3³ × 3³ = 729개)에 대해 **시간 지평선(horizon_sec)** 동안
 *    `numeq_integrate_motion_rk4()`로 상태를 적분하며 시뮬레이션을 수행합니다.
 * 3. 각 시뮬레이션에 대해 `cost_fn`을 호출하여 누적 비용(total_cost)을 계산합니다.
 * 4. 최소 비용을 발생시키는 가속도 조합을 **최적값**으로 선택하고
 *    `out_result`에 기록합니다.
 * 5. `out_traj`가 지정되고 `config->output_trajectory == true`이면,
 *    선택된 제어 입력을 적용하여 미래 궤적(trajectory)을 생성합니다.
 *
 * ---
 *
 * ## 매개변수
 * @param[in]  current_state  현재 모션 상태 (위치, 속도, 회전 정보 포함)
 * @param[in]  target_state   목표 모션 상태
 * @param[in]  env            환경 정보 (중력, 풍속, 공기밀도 등)
 *                            현재 버전에서는 사용하지 않지만 향후 확장을 위해 유지됩니다.
 * @param[in]  body           물리 속성 (질량, 항력계수 등)
 *                            현재 버전에서는 사용하지 않지만 향후 확장을 위해 유지됩니다.
 * @param[in]  config         MPC 설정 (horizon_sec, step_dt, max_accel 등)
 * @param[out] out_result     MPC 결과 출력 구조체
 *                            (desired_accel, desired_ang_accel, cost, future_state 포함)
 * @param[out] out_traj       미래 궤적을 저장할 trajectory_t (NULL이면 궤적 저장 안 함)
 * @param[in]  cost_fn        비용 함수 포인터
 *                            - 호출 시점: 각 스텝마다 (sim_state, target_state, cost_userdata)
 *                            - 반환 값: 해당 스텝의 비용 (작을수록 더 나은 결과)
 * @param[in]  cost_userdata  비용 함수에 전달되는 사용자 정의 데이터 (NULL 가능)
 *
 * ---
 *
 * ## 반환값
 * - **true**: MPC 계산 성공
 * - **false**: 입력 포인터(current_state, target_state, config, out_result) 중
 *              하나라도 NULL이면 실패
 *
 * ---
 *
 * ## 사용 예시
 * @code
 * motion_state_t current, target;
 * mpc_config_t cfg;
 * mpc_output_t result;
 * trajectory_t traj;
 *
 * mpc_config_init(&cfg);
 * cfg.max_accel = 5.0f;
 * cfg.max_ang_accel = 2.0f;
 * cfg.horizon_sec = 10;
 * cfg.step_dt = 0.016f; // 60Hz
 *
 * bool ok = numeq_mpc_solve(&current, &target,
 *                           NULL, NULL,
 *                           &cfg,
 *                           &result, &traj,
 *                           my_cost_fn, NULL);
 *
 * if (ok) {
 *     printf("Best Accel: (%f, %f, %f)\n",
 *            result.desired_accel.x,
 *            result.desired_accel.y,
 *            result.desired_accel.z);
 * }
 * @endcode
 *
 * ---
 *
 * ## 주의사항
 * - 브루트포스 탐색(3³ × 3³ = 729회 × horizon_sec)으로 연산량이 큽니다.
 *   실시간 환경에서는 후보 수를 줄이거나 horizon_sec을 줄여야 합니다.
 * - env와 body는 현재 사용되지 않으나 향후 공기저항/중력/질량을 반영할 계획입니다.
 * - cost_fn이 NULL이면 모든 후보의 비용이 0이므로 첫 번째 후보가 선택됩니다.
 */
BYUL_API bool numeq_mpc_solve(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 최적화된 단일 목표 MPC (Fast Version)
 *
 * 기존 `numeq_mpc_solve()`의 **전수 탐색 방식**은 매우 많은 후보를 탐색하여
 * 높은 연산량을 유발합니다. 이를 개선하기 위해 Fast MPC는 **후보군을 대폭 줄이고,**
 * **계산 효율화(Early Exit, Warm Start)를 적용**하여 실시간 성능을 확보합니다.
 *
 * ---
 *
 * ### **알고리즘 개요**
 * 1. **후보군 축소:**  
 *    각 축별 가속도 후보를 `{0, ±max_accel}`로 제한하여 총 3³=27개만 탐색.
 * 2. **Warm Start:**  
 *    이전 스텝의 최적 가속도를 초기 후보로 사용하여,
 *    주변 후보군을 우선 탐색해 불필요한 시뮬레이션을 최소화.
 * 3. **Early Exit:**  
 *    탐색 중 비용이 현재 `best_cost`를 초과하면 해당 후보는 바로 중단.
 *
 * ---
 *
 * ### **장점**
 * - **속도:** 표준 MPC 대비 **최대 10~20배 이상 빠름** (1ms~2ms 수준).
 * - **실시간 제어에 적합:** 60Hz~120Hz 제어 루프에서도 안정적으로 사용 가능.
 * - **Warm Start 활용:** 이전 결과를 활용해 수렴 속도 향상.
 *
 * ### **단점**
 * - **정밀도 부족:**  
 *   후보가 적어 **글로벌 최적해를 놓칠 수 있음.**
 * - **미세 튜닝 불가:**  
 *   `0` 또는 `±max_accel` 수준의 단순 가속도만 고려하므로
 *   **Fine Control이 어렵다.**
 *
 * ---
 *
 * @param[in]  current_state  현재 모션 상태
 * @param[in]  target_state   목표 모션 상태
 * @param[in]  env            환경 정보 (NULL 가능)
 * @param[in]  body           물리 속성 (NULL 가능)
 * @param[in]  config         MPC 설정
 * @param[out] out_result     결과 저장 구조체
 * @param[out] out_traj       예측 궤적 (NULL 가능)
 * @param[in]  cost_fn        비용 함수 포인터
 * @param[in]  cost_userdata  비용 함수 사용자 데이터
 *
 * @return true = 성공, false = 실패
 */
BYUL_API bool numeq_mpc_solve_fast(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

/**
 * @brief 2단계(Coarse-to-Fine) 탐색을 적용한 단일 목표 MPC
 *
 * Fast MPC는 빠르지만 **후보군이 너무 단순**해서 최적해를 찾지 못할 수 있습니다.
 * Coarse2Fine MPC는 이를 보완하기 위해 **2단계 탐색(Coarse Search → Fine Search)**을 적용,
 * **정밀도와 속도의 균형**을 맞춥니다.
 *
 * ---
 *
 * ### **알고리즘 개요**
 * 1. **Coarse Search:**  
 *    각 축별로 `{ -max, 0, +max }` 후보군을 사용해  
 *    대략적인 최적 가속도 방향을 탐색.
 * 2. **Fine Search:**  
 *    Coarse 단계에서 찾은 **best_accel** 주변에서 **±delta 범위**를 작은 단위로 세분화해  
 *    최적해를 한 번 더 탐색.
 *
 * ---
 *
 * ### **장점**
 * - **정밀도:**  
 *   Fast MPC보다 더 **정확한 가속도 벡터를 찾을 가능성 높음.**
 * - **안정성:**  
 *   전역 탐색(coarse) + 국소 최적화(fine)로 **잡음이나 작은 오차에 강함.**
 * - **커스터마이징 용이:**  
 *   coarse와 fine 단계의 해상도를 각각 조절 가능.
 *
 * ### **단점**
 * - **속도 저하:**  
 *   coarse(27 후보) × fine(27 후보) → 총 729회 시뮬레이션 발생 가능.  
 *   현재 벤치마크에서 Fast MPC(1ms) 대비 약 **20~30배 느림** (약 28ms).
 * - **튜닝 복잡성:**  
 *   fine search delta와 step 수 조절이 필요.
 *
 * ---
 *
 * @param[in]  current_state  현재 모션 상태
 * @param[in]  target_state   목표 모션 상태
 * @param[in]  env            환경 정보 (NULL 가능)
 * @param[in]  body           물리 속성 (NULL 가능)
 * @param[in]  config         MPC 설정
 * @param[out] out_result     결과 저장 구조체
 * @param[out] out_traj       예측 궤적 (NULL 가능)
 * @param[in]  cost_fn        비용 함수 포인터
 * @param[in]  cost_userdata  비용 함수 사용자 데이터
 *
 * @return true = 성공, false = 실패
 */
BYUL_API bool numeq_mpc_solve_coarse2fine(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
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
    const environ_t* env,
    const bodyprops_t* body,
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
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MPC_H
