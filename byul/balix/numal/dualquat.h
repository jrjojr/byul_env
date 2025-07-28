#ifndef DUALQUAT_H
#define DUALQUAT_H

#include "quat.h"
#include "vec3.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 듀얼 쿼터니언 (회전 + 위치)
 * real: 회전, dual: 이동 성분
 */
typedef struct dualquat_t {
    quat_t real;
    quat_t dual;
} dualquat_t;

// ---------------------------------------------------------
// 🎯 생성 / 초기화 / 복사
// ---------------------------------------------------------

/** @brief 단위 듀얼 쿼터니언 (회전=단위, 위치=0) */
BYUL_API void dualquat_init(dualquat_t* out);

/**
 * @brief 회전(quat) + 위치(vec3)로 초기화
 * @param rot 회전 쿼터니언 (NULL이면 단위 회전)
 * @param vec 위치 벡터 (NULL이면 원점)
 */
BYUL_API void dualquat_init_quat_vec(
    dualquat_t* out, const quat_t* rot, const vec3_t* vec);

/** @brief 3x3 회전 행렬에서 초기화 (이동=0) */
BYUL_API void dualquat_init_from_mat3(dualquat_t* out, const float* mat3x3);

/** @brief 4x4 변환 행렬에서 초기화 */
BYUL_API void dualquat_init_from_mat4(dualquat_t* out, const float* mat4x4);

/** @brief 복사 */
BYUL_API void dualquat_assign(dualquat_t* out, const dualquat_t* src);

// ---------------------------------------------------------
// 🧪 비교 / 해시
// ---------------------------------------------------------

BYUL_API int dualquat_equal(const dualquat_t* a, const dualquat_t* b);
BYUL_API uint32_t dualquat_hash(const dualquat_t* dq);

// ---------------------------------------------------------
// 🔄 변환 / 추출
// ---------------------------------------------------------

/**
 * @brief 듀얼 쿼터니언을 회전/위치로 분해
 * @param rot (NULL 가능) 회전 결과
 * @param vec (NULL 가능) 위치 결과
 */
BYUL_API void dualquat_to_quat_vec(
    const dualquat_t* dq, quat_t* rot, vec3_t* vec);

/** @brief 3x3 회전 행렬로 변환 */
BYUL_API void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3);

/** @brief 4x4 변환 행렬로 변환 */
BYUL_API void dualquat_to_mat4(const dualquat_t* dq, float* out_mat4);

// ---------------------------------------------------------
// ➕ 연산
// ---------------------------------------------------------

BYUL_API void dualquat_add(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

BYUL_API void dualquat_sub(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

BYUL_API void dualquat_mul(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

BYUL_API void dualquat_scale(dualquat_t* out, 
    const dualquat_t* a, float scalar);

BYUL_API float dualquat_dot(const dualquat_t* a, const dualquat_t* b);
BYUL_API float dualquat_length(const dualquat_t* dq);

/** @brief 켤레 */
BYUL_API void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq);

/** @brief 역변환 (inverse) */
BYUL_API void dualquat_inverse(dualquat_t* out, const dualquat_t* dq);

/** @brief 정규화 */
BYUL_API void dualquat_normalize(dualquat_t* io);
BYUL_API void dualquat_unit(dualquat_t* out, const dualquat_t* dq);

/** @brief 부호 정렬 (real.w < 0이면 -1 곱함) */
BYUL_API void dualquat_align(dualquat_t* out, const dualquat_t* dq);

// ---------------------------------------------------------
// 📐 보간
// ---------------------------------------------------------

/** @brief LERP (선형 보간 후 정규화 필요) */
BYUL_API void dualquat_lerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief NLERP (정규화 선형 보간) */
BYUL_API void dualquat_nlerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief SLERP (구면 선형 보간) */
BYUL_API void dualquat_slerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/**
 * @brief 가중 평균 (w1*a + w2*b 후 정규화)
 */
BYUL_API void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2);

// ---------------------------------------------------------
// 🚀 포인트 변환
// ---------------------------------------------------------

/**
 * @brief 포인트 변환 (out = dq * in)
 */
BYUL_API void dualquat_apply_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out);

/**
 * @brief 포인트 역변환 (out = dq⁻¹ * in)
 */
BYUL_API void dualquat_apply_inverse_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out);

/**
 * @brief 포인트 제자리 변환
 */
BYUL_API void dualquat_apply_to_point_inplace(
    const dualquat_t* dq, vec3_t* io_point);

// ---------------------------------------------------------
// 🏷️ 단위 생성자
// ---------------------------------------------------------
static inline void dualquat_identity(dualquat_t* out) {
    dualquat_init(out);
}

#ifdef __cplusplus
}
#endif

#endif // DUALQUAT_H
