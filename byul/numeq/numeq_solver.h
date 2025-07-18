#ifndef NUMEQ_SOLVER_H
#define NUMEQ_SOLVER_H

#include "internal/numeq_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 1. 수학적 기본 해 찾기 함수군
// ---------------------------------------------------------

// 이차방정식 ax^2 + bx + c = 0 해석적 해 (실근만 반환)
// 반환값: 해 존재 여부 (실근 존재시 true)
BYUL_API bool numeq_solve_quadratic(float a, float b, float c, float* out_x1, float* out_x2);

// f(x) = 0 형태 함수에 대해 이분법으로 근을 찾음
// 사용자 정의 함수포인터 + 범위 지정
typedef float (*numeq_func_f32)(float x, void* userdata);

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
BYUL_API bool numeq_solve_time_for_y(const state_vector_t* state,
                            float target_y,
                            float* out_time);

// 주어진 위치에 도달하는 시간 t 계산 (xz 기준 + 근사)
// solve: |pos(t).xz - target.xz| < ε
BYUL_API bool numeq_solve_time_for_position(const state_vector_t* state,
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
BYUL_API bool numeq_solve_apex(const state_vector_t* state,
                      vec3_t* out_apex_pos,
                      float* out_apex_time);

// 발사체가 멈추는 조건(속도 = 0) 도달 시간 계산 (수평 감쇠 포함)
BYUL_API bool numeq_solve_stop_time(const state_vector_t* state,
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
