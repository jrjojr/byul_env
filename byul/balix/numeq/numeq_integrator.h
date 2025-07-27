/**
 * @file numeq_integrator.h
 * @brief 운동 방정식(선형 + 회전)을 수치적으로 적분하는 통합 모듈
 *
 * 이 모듈은 motion_state_t를 기반으로 한 선형 및 회전 운동 상태를
 * 다양한 적분 방식으로 예측합니다.
 *
 * 제공 기능:
 * - 선형 운동 적분 (Euler, Semi-Implicit, Verlet, RK4)
 * - 회전 운동 적분 (쿼터니언 기반 Euler, Semi-Implicit, Verlet, RK4)
 * - 선형 + 회전 통합 적분기 (Motion 계열)
 *
 * 추천 사용 시나리오:
 * - 실시간 물리 시뮬레이션 (60Hz, Semi-Implicit Euler)
 * - 고정밀 궤적 계산 (RK4)
 * - 과거 상태가 필요한 특수 효과 (Verlet)
 *
 * @note INTEGRATOR_EULER는 가장 단순하지만 안정성이 떨어질 수 있어
 * 일반적인 게임 물리에서는 INTEGRATOR_SEMI_IMPLICIT 또는 RK4를 권장합니다.
 */
#ifndef NUMEQ_INTEGRATOR_H
#define NUMEQ_INTEGRATOR_H

#include "internal/motion_state.h"

#ifdef __cplusplus
extern "C" {
#endif

// 전방 선언 
typedef struct s_bodyprops bodyprops_t;
typedef struct s_environ environ_t;

// ---------------------------------------------------------
// 📌 적분기 종류 (시뮬레이션 방식 선택)
// ---------------------------------------------------------

/**
 * @brief 적분기 방식 종류
 */
typedef enum e_integrator_type {
    INTEGRATOR_EULER,               ///< 단순 오일러 방식
    INTEGRATOR_SEMI_IMPLICIT,       ///< 반묵시적 오일러 (속도 우선)
    INTEGRATOR_VERLET,              ///< Verlet 방식 (과거 위치 필요)
    INTEGRATOR_RK4,                 ///< 4차 Runge-Kutta 방식 (고정확도)
    INTEGRATOR_RK4_ENV,           ///< 선형 + 환경 4차 Runge-Kutta
    INTEGRATOR_MOTION_EULER,        ///< 선형 + 회전 오일러
    INTEGRATOR_MOTION_SEMI_IMPLICIT,///< 선형 + 회전 반묵시적 오일러
    INTEGRATOR_MOTION_VERLET,       ///< 선형 + 회전 Verlet 방식
    INTEGRATOR_MOTION_RK4,            ///< 선형 + 회전 4차 Runge-Kutta
    INTEGRATOR_MOTION_RK4_ENV           ///< 선형 + 회전 + 환경 4차 Runge-Kutta
} integrator_type_t;

/**
 * @brief 적분 설정 구조체
 *
 * 이 구조체는 다양한 적분기(INTEGRATOR_EULER, RK4 등)의
 * 시간 스텝, 환경, 물리 속성 등을 포함한 실행 설정을 관리합니다.
 *
 * ### 평균 범위(Mean Range)
 * - `time_step`은 보통 0.016f(60Hz) 수준으로 설정하지만,
 *   **평균 시간 간격**으로서 정확히 고정될 필요는 없습니다.
 * - 물리 시뮬레이션 상황에 따라 약간의 변동(±10~20%)이 있어도
 *   안정성에 큰 문제가 없습니다.
 * - `time_step` 값이 지나치게 커지면 정확도가 떨어지고,
 *   너무 작으면 계산량이 폭증할 수 있으므로 평균 범위 내에서 조정하는 것이 이상적입니다.
 */
typedef struct s_integrator_config {
    integrator_type_t type;           ///< 사용할 적분 방식
    float time_step;                  ///< 시간 간격 (dt), 평균적으로 0.016f(60Hz) 추천
    motion_state_t* prev_state;       ///< Verlet 방식에서 참조할 과거 상태
    const environ_t* env;             ///< 환경 정보 (중력, 바람 등)
    const bodyprops_t* body;          ///< 물체의 물리 속성 (질량, 항력 등)
    void* userdata;                   ///< 기타 사용자 데이터 포인터 (옵션)
} integrator_config_t;


/**
 * @brief 적분 설정 구조체를 기본값으로 초기화합니다.
 *
 * @details
 * - `type`은 INTEGRATOR_MOTION_RK4로 초기화됩니다.
 * - `time_step`은 0.016f (60Hz)로 설정됩니다.
 * - `prev_state`, `env`, `body`, `userdata`는 모두 NULL로 초기화됩니다.
 *
 * @param[out] cfg 초기화할 적분 설정 구조체 포인터
 */
BYUL_API void integrator_config_init(integrator_config_t* cfg);

/**
 * @brief 적분 설정 구조체를 지정된 값으로 초기화합니다.
 *
 * @param[out] cfg         초기화할 설정 구조체
 * @param[in]  type        사용할 적분 방식
 * @param[in]  time_step   시간 간격 (dt, 평균 0.016f(60Hz) 권장)
 * @param[in]  prev_state  Verlet 방식에서 사용할 과거 상태 (없으면 NULL)
 * @param[in]  env         환경 정보 (중력, 바람 등)
 * @param[in]  body        물체의 물리 속성 (질량, 항력 등)
 * @param[in]  userdata    사용자 정의 데이터 포인터
 *
 * @note time_step은 평균 범위를 벗어나더라도 동작하지만,
 *       너무 큰 값은 정확도 저하, 너무 작은 값은 성능 저하를 유발합니다.
 */
BYUL_API void integrator_config_init_full(integrator_config_t* cfg,
                                          integrator_type_t type,
                                          float time_step,
                                          motion_state_t* prev_state,
                                          const environ_t* env,
                                          const bodyprops_t* body,
                                          void* userdata);

BYUL_API void integrator_config_assign(
    integrator_config_t* out, const integrator_config_t* src);

// ---------------------------------------------------------
// 🧩 공통 인터페이스
// ---------------------------------------------------------

/**
 * @brief 상태 벡터를 시간에 따라 적분합니다. 방식은 config에 따라 자동 선택됩니다.
 *
 * @param state      [in/out] 상태벡터 (position, velocity, acceleration)
 * @param config     [in] 적분 방식 및 dt 설정
 *
 * @note 내부적으로 선택된 방식에 따라 분기 호출됩니다.
 *
 */
BYUL_API void numeq_integrate(
    motion_state_t* state, const integrator_config_t* config);

// ---------------------------------------------------------
// 🎯 각 방식별 수치 적분 함수
// ---------------------------------------------------------

/**
 * @brief 오일러 방식 적분
 *
 * @details
 * vₜ₊₁ = vₜ + a·dt  
 * pₜ₊₁ = pₜ + vₜ·dt
 *
 * 가장 단순하지만 정확도가 낮고 불안정할 수 있습니다.
 *
 */
BYUL_API void numeq_integrate_euler(motion_state_t* state, float dt);

/**
 * @brief 세미-묵시적 오일러 방식 적분
 *
 * @details
 * vₜ₊₁ = vₜ + a·dt  
 * pₜ₊₁ = pₜ + vₜ₊₁·dt
 *
 * 안정성이 높아 대부분의 실시간 시뮬레이션에서 추천됩니다.
 *
 */
BYUL_API void numeq_integrate_semi_implicit(
    motion_state_t* state, float dt);

/**
 * @brief Verlet 적분 방식 (이차 정확도)
 *
 * @details
 * pₜ₊₁ = 2pₜ - pₜ₋₁ + a·dt²
 *
 * 과거 위치 벡터가 별도로 필요합니다.
 * 감쇠 진동이나 트레일 효과 등에 유용합니다.
 *
 */
BYUL_API void numeq_integrate_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);
/**
 * @brief 4차 Runge-Kutta 적분 방식
 *
 * @details
 * 고정확도의 물리 예측에 적합하며,
 * MPC, 유도 미사일, 복잡한 역학 시뮬레이션에서 사용됩니다.
 *
 */
BYUL_API void numeq_integrate_rk4(motion_state_t* state, float dt);

BYUL_API void numeq_integrate_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

BYUL_API void numeq_integrate_attitude_euler(
    motion_state_t* state, float dt);

// ---------------------------------------------------------
// 회전 적분 (Semi-Implicit Euler)
// ---------------------------------------------------------
BYUL_API void numeq_integrate_attitude_semi_implicit(
    motion_state_t* state, float dt);

// ---------------------------------------------------------
// 회전 적분 (RK4)
// ---------------------------------------------------------
BYUL_API void numeq_integrate_attitude_rk4(
    motion_state_t* state, float dt);

BYUL_API void numeq_integrate_attitude_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);    

// 회전(자세) Verlet 적분
BYUL_API void numeq_integrate_attitude_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// 선형 + 회전 통합 Verlet 적분기
BYUL_API void numeq_integrate_motion_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// 선형 + 회전 통합 Euler 적분기
BYUL_API void numeq_integrate_motion_euler(
    motion_state_t* state, float dt);

// 선형 + 회전 통합 Semi-Implicit Euler 적분기
BYUL_API void numeq_integrate_motion_semi_implicit(
    motion_state_t* state, float dt);

/**
 * @brief 선형 + 회전 통합 RK4 적분기 (환경 무시)
 *
 * @details
 * 4차 Runge-Kutta(RK4) 방법을 사용하여 motion_state_t의 선형 및 회전 상태를
 * dt 시간만큼 적분합니다. 
 * 이 버전은 환경(env)과 물체의 물리 속성(body)을 고려하지 않고,
 * state에 포함된 가속도(acceleration)와 각가속도(angular_acceleration)를
 * 그대로 사용합니다.
 *
 * @param state [in/out] 현재 운동 상태 (position, velocity, orientation 등).
 *                       계산 후 dt초 후의 상태로 갱신됩니다.
 * @param dt    [in] 적분 시간 간격 (초 단위).
 *
 * @note 외부 환경 효과(중력, 항력 등)가 필요하다면 
 *       @ref numeq_integrate_motion_rk4_env 를 사용하세요.
 */    
BYUL_API void numeq_integrate_motion_rk4(
    motion_state_t* state, float dt);

/**
 * @brief 선형 + 회전 통합 RK4 적분기 (환경 반영)
 *
 * @details
 * 4차 Runge-Kutta(RK4) 적분법을 사용하여 motion_state_t의 선형 및 회전 운동 상태를
 * dt 시간만큼 시뮬레이션합니다. 
 * 이 버전은 환경 정보(@p env)와 물체의 물리 속성(@p body)를 사용하여 
 * 각 단계별 가속도(acceleration)와 각가속도(angular_acceleration)를 
 * 재계산합니다.
 *
 * @param state [in/out] 현재 운동 상태 (motion_state_t).
 *                       함수 실행 후 dt초 후 상태로 업데이트됩니다.
 * @param dt    [in] 적분 시간 간격 (초 단위).
 * @param env   [in] 환경 정보 (중력, 바람, 공기밀도 등).
 * @param body  [in] 물체의 물리 속성 (질량, 항력계수, 마찰 등).
 *
 * @note 항력이나 외부 토크 등 비선형 항이 포함된 경우에도 안정적인 예측을 제공합니다.
 */
BYUL_API void numeq_integrate_motion_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_INTEGRATOR_H
