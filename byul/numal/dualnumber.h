/**
 * @file dualnumber.h
 * @brief DualNumber는 자동 미분(Forward-mode automatic differentiation)을 위한 핵심 구조입니다.
 *
 * DualNumber는 a + bε 형식을 가지며, ε는 무한소(infinitesimal)로 정의됩니다.
 * 가장 중요한 성질은 ε² = 0 입니다. 따라서 ε항은 남지만, 그 제곱 이상은 모두 사라지게 됩니다.
 *
 * 📌 핵심 개념:
 *   - 실수 부분(re): 함수의 원래 값
 *   - 듀얼 부분(du): 미분 값 (함수의 기울기)
 *
 * 예: f(x) = x³, x = 2.0 + 1ε 를 넣으면
 *     f(x + ε) = 8 + 12ε  → 여기서 12는 f'(2) = 3·2²
 *
 * 사용 예시:
 *   dualnumber_t* x = dualnumber_new_full(2.0f, 1.0f); // ε항 계수는 항상 1
 *   dualnumber_t* y = dualnumber_powf(x, 3.0f);        // f(x) = x³
 *   printf("f(x)=%.1f, f'(x)=%.1f\\n", y->re, y->du);  // 결과: 8.0, 12.0
 *
 * 이 구조는 최적화, 물리 시뮬레이션, 미분방정식 해석 등 다양한 분야에 쓰입니다.
 * GPU 가속 이전의 수치해석 구조로도 활용 가능합니다.
 */
#ifndef DUALNUMBER_H
#define DUALNUMBER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_config.h"
#include <math.h>

// DualNumber: a + b * eps, where eps^2 = 0
typedef struct {
    float re;  // real part
    float du;  // dual part
} dualnumber_t;

// 메모리 관리
BYUL_API dualnumber_t* dualnumber_new();
BYUL_API dualnumber_t* dualnumber_new_full(float re, float du);
BYUL_API void          dualnumber_free(dualnumber_t* d);

// 기본 연산
BYUL_API dualnumber_t* dualnumber_neg(const dualnumber_t* a);
BYUL_API dualnumber_t* dualnumber_add(const dualnumber_t* a, const dualnumber_t* b);
BYUL_API dualnumber_t* dualnumber_sub(const dualnumber_t* a, const dualnumber_t* b);
BYUL_API dualnumber_t* dualnumber_mul(const dualnumber_t* a, const dualnumber_t* b);
BYUL_API dualnumber_t* dualnumber_div(const dualnumber_t* a, const dualnumber_t* b);

// 스칼라 연산
BYUL_API dualnumber_t* dualnumber_scale(const dualnumber_t* a, float s);
BYUL_API dualnumber_t* dualnumber_invscale(const dualnumber_t* a, float s);

// 거듭제곱
BYUL_API dualnumber_t* dualnumber_powf(const dualnumber_t* a, float n);

// 비교 및 복사
BYUL_API int            dualnumber_equal(const dualnumber_t* a, const dualnumber_t* b);
BYUL_API unsigned int   dualnumber_hash(const dualnumber_t* a);
BYUL_API dualnumber_t*  dualnumber_copy(const dualnumber_t* src);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // DUALNUMBER_H
