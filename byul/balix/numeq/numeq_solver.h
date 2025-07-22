#ifndef NUMEQ_SOLVER_H
#define NUMEQ_SOLVER_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 1. 수학적 기본 해 찾기 함수군
// ---------------------------------------------------------

/**
 * @brief 이차방정식 ax² + bx + c = 0의 해를 구합니다. (실근만 반환)
 *
 * @param a 이차항 계수 (a != 0)
 * @param b 일차항 계수
 * @param c 상수항
 * @param[out] out_x1 첫 번째 해 (작은 값)
 * @param[out] out_x2 두 번째 해 (큰 값)
 *
 * @return 해가 존재하면 true, 없으면 false (판별식 < 0)
 *
 * @note a가 0이면 1차 방정식 bx + c = 0으로 처리됩니다.
 * @note 실근만 반환하며, 허근은 false를 반환합니다.
 *
 * **예시**
 * @code
 * float x1, x2;
 * if (numeq_solve_quadratic(1.0f, -3.0f, 2.0f, &x1, &x2)) {
 *     // x1 = 1.0, x2 = 2.0
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_quadratic(
    float a, float b, float c, float* out_x1, float* out_x2);

/**
 * @brief 3차방정식 ax³ + bx² + cx + d = 0의 해를 구합니다. (실근만 반환)
 *
 * @param a   3차항 계수 (a != 0)
 * @param b   2차항 계수
 * @param c   1차항 계수
 * @param d   상수항
 * @param[out] out_roots 해 배열 (최대 3개의 실근 저장, 오름차순)
 * @param[out] out_count 실근 개수 (1~3)
 *
 * @return 해가 존재하면 true, 없으면 false
 *
 * **예시**
 * @code
 * float roots[3];
 * int count = 0;
 * if (numeq_solve_cubic(1.0f, -6.0f, 11.0f, -6.0f, roots, &count)) {
 *     // roots = {1.0, 2.0, 3.0}, count = 3
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_cubic(float a, float b, float c, float d, 
    float* out_roots, int* out_count);

typedef float (*numeq_func_f32)(float x, void* userdata);

/**
 * @brief 이분법으로 f(x) = 0의 근을 찾습니다.
 *
 * @param func       함수 포인터 (f(x))
 * @param userdata   사용자 데이터 (옵션)
 * @param a          구간 시작
 * @param b          구간 끝
 * @param tol        허용 오차
 * @param[out] out_root 근사 해
 *
 * @return 해를 찾으면 true, 실패 시 false
 */
BYUL_API bool numeq_solve_bisection(numeq_func_f32 func,
                           void* userdata,
                           float a, float b,
                           float tol,
                           float* out_root);

// ---------------------------------------------------------
// 2. 탄도 방정식 기반 물리 해 찾기 함수군
// ---------------------------------------------------------

// 주어진 y 위치에 도달하는 시간 t 계산 (수직 위치 조건)
// solve: y(t) = target_y
BYUL_API bool numeq_solve_time_for_y(const linear_state_t* state,
                            float target_y,
                            float* out_time);

// 주어진 위치에 도달하는 시간 t 계산 (xz 기준 + 근사)
// solve: |pos(t).xz - target.xz| < ε
BYUL_API bool numeq_solve_time_for_position(const linear_state_t* state,
                                   const vec3_t* target_pos,
                                   float tolerance,
                                   float max_time,
                                   float* out_time);

// 특정 거리까지 도달하는 데 필요한 속도 계산 (수평 사거리)
// solve: x(v) = target_range
BYUL_API bool numeq_solve_velocity_for_range(float distance,
                                    float gravity,
                                    float* out_velocity);

// 최고점 도달 시간 및 위치 계산
// solve: vy(t) == 0 → apex
BYUL_API bool numeq_solve_apex(const linear_state_t* state,
                      vec3_t* out_apex_pos,
                      float* out_apex_time);

// 발사체가 멈추는 조건(속도 = 0) 도달 시간 계산 (수평 감쇠 포함)
BYUL_API bool numeq_solve_stop_time(const linear_state_t* state,
                           float tolerance,
                           float* out_time);

// ---------------------------------------------------------
// 3. 향후 확장: 벡터 기반 수치 해 찾기
// ---------------------------------------------------------

// f(t) = vec(t) 와 target_vec 사이 거리 최소화
// 반환: 최소 거리 시점 t
typedef void (*numeq_vec3_func)(float t, vec3_t* out, void* userdata);
BYUL_API bool numeq_solve_time_for_vec3(numeq_vec3_func func,
                               void* userdata,
                               const vec3_t* target,
                               float t_min, float t_max,
                               float tol,
                               float* out_t);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_SOLVER_H
