#ifndef NUMEQ_INTEGRATOR_H
#define NUMEQ_INTEGRATOR_H

#include "internal/trajectory.h" //motion_state_t

#ifdef __cplusplus
extern "C" {
#endif

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
    INTEGRATOR_MOTION_EULER,        ///< 선형 + 회전 오일러
    INTEGRATOR_MOTION_SEMI_IMPLICIT,///< 선형 + 회전 반묵시적 오일러
    INTEGRATOR_MOTION_VERLET,       ///< 선형 + 회전 Verlet 방식
    INTEGRATOR_MOTION_RK4           ///< 선형 + 회전 4차 Runge-Kutta
} integrator_type_t;

/**
 * @brief 적분 설정 구조체
 */
typedef struct s_integrator_config {
    integrator_type_t type;       ///< 사용할 적분 방식
    float time_step;              ///< 시간 간격 (dt)
    motion_state_t* prev_state;   ///< Verlet 방식에서 참조할 과거 상태
    void* userdata;               ///< 사용자 데이터 포인터 (옵션)
} integrator_config_t;

    //INTEGRATOR_EULER로 하지 않는 이유는 
    // 재수 없으면 계산량이 너무 많아서 무한루프에 걸린다
    // cfg->type = INTEGRATOR_MOTION_RK4; /
    // cfg->time_step = 0.016f; // 기본 60Hz
    // cfg->prev_state = nullptr;
    // cfg->userdata = nullptr;
BYUL_API void integrator_config_init(integrator_config_t* cfg);

BYUL_API void integrator_config_init_full(integrator_config_t* cfg,
                                 integrator_type_t type,
                                 float time_step,
                                 motion_state_t* prev_state,
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

// 선형 + 회전 통합 RK4 적분기
BYUL_API void numeq_integrate_motion_rk4(
    motion_state_t* state, float dt);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_INTEGRATOR_H
