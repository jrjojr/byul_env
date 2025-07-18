#ifndef NUMEQ_INTEGRATOR_H
#define NUMEQ_INTEGRATOR_H

#include "internal/numeq_common.h"

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
    INTEGRATOR_EULER,           ///< 단순 오일러 방식
    INTEGRATOR_SEMI_IMPLICIT,   ///< 반묵시적 오일러 (속도 우선)
    INTEGRATOR_VERLET,          ///< Verlet 방식 (과거 위치 필요)
    INTEGRATOR_RK4              ///< 4차 Runge-Kutta 방식 (고정확도)
} integrator_type_t;

/**
 * @brief 적분 설정 구조체
 */
typedef struct s_integrator_config {
    integrator_type_t type;     ///< 사용할 적분 방식
    float time_step;            ///< 시간 간격 (dt)
} integrator_config_t;

// ---------------------------------------------------------
// 🧩 공통 인터페이스
// ---------------------------------------------------------

/**
 * @brief 상태 벡터를 시간에 따라 적분합니다. 방식은 config에 따라 자동 선택됩니다.
 *
 * @param state      [in/out] 상태벡터 (position, velocity, acceleration)
 * @param accel      [in] 외부 가속도 입력 (예: 중력, 유도력 등)
 * @param config     [in] 적분 방식 및 dt 설정
 *
 * @note 내부적으로 선택된 방식에 따라 분기 호출됩니다.
 *
 * @code
 * integrator_config_t cfg = {
 *     .type = INTEGRATOR_SEMI_IMPLICIT,
 *     .time_step = 0.016f
 * };
 * vec3_t gravity = {0, -9.8f, 0};
 * numeq_integrate(&state, &gravity, &cfg);
 * @endcode
 */
BYUL_API void numeq_integrate(state_vector_t* state,
                     const vec3_t* accel,
                     const integrator_config_t* config);

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
 * @code
 * numeq_integrate_euler(&state, &accel, 0.01f);
 * @endcode
 */
BYUL_API void numeq_integrate_euler(state_vector_t* state,
                           const vec3_t* accel,
                           float dt);

/**
 * @brief 세미-묵시적 오일러 방식 적분
 *
 * @details
 * vₜ₊₁ = vₜ + a·dt  
 * pₜ₊₁ = pₜ + vₜ₊₁·dt
 *
 * 안정성이 높아 대부분의 실시간 시뮬레이션에서 추천됩니다.
 *
 * @code
 * numeq_integrate_semi_implicit(&state, &accel, 0.016f);
 * @endcode
 */
BYUL_API void numeq_integrate_semi_implicit(state_vector_t* state,
                                   const vec3_t* accel,
                                   float dt);

/**
 * @brief Verlet 적분 방식 (이차 정확도)
 *
 * @details
 * pₜ₊₁ = 2pₜ - pₜ₋₁ + a·dt²
 *
 * 과거 위치 벡터가 별도로 필요합니다.
 * 감쇠 진동이나 트레일 효과 등에 유용합니다.
 *
 * @code
 * vec3_t prev = state.position;
 * numeq_integrate_verlet(&state.position, &prev, &accel, 0.016f);
 * @endcode
 */
BYUL_API void numeq_integrate_verlet(vec3_t* position,
                            vec3_t* prev_position,
                            const vec3_t* accel,
                            float dt);

/**
 * @brief 4차 Runge-Kutta 적분 방식
 *
 * @details
 * 고정확도의 물리 예측에 적합하며,
 * MPC, 유도 미사일, 복잡한 역학 시뮬레이션에서 사용됩니다.
 *
 * @code
 * numeq_integrate_rk4(&state, &accel, 0.016f);
 * @endcode
 */
BYUL_API void numeq_integrate_rk4(state_vector_t* state,
                         const vec3_t* accel,
                         float dt);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_INTEGRATOR_H
