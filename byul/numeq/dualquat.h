#ifndef DUALQUAT_H
#define DUALQUAT_H

#include "internal/quat.h"
#include "internal/vec3.h"
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 듀얼 회전 구조체 전방 선언 (회전 + 위치 통합 표현)
 */
typedef struct dualquat_t {
    quat_t real;
    quat_t dual;
} dualquat_t;

// -----------------------------
// 생성 / 해제 / 복사
// -----------------------------

/** @brief 단위 듀얼 회전 (회전=단위, 위치=0) 생성 */
BYUL_API dualquat_t* dualquat_new(void);

/** @brief 듀얼 회전 해제 */
BYUL_API void dualquat_free(dualquat_t* dq);

/** @brief 복사 */
BYUL_API void dualquat_copy(dualquat_t* dst, const dualquat_t* src);

// -----------------------------
// 비교 / 해시
// -----------------------------

/** @brief 두 듀얼 회전이 같은지 비교 */
BYUL_API int dualquat_equal(const dualquat_t* a, const dualquat_t* b);

/** @brief 해시값 계산 */
BYUL_API uint32_t dualquat_hash(const dualquat_t* dq);

// -----------------------------
// 생성 및 추출 (회전 + 위치)
// -----------------------------

/**
 * @brief 회전(quat)과 위치(vec3)로부터 듀얼 회전 생성
 */
BYUL_API void dualquat_from_quat_vec(
    dualquat_t* out,
    const quat_t* rot,
    const vec3_t* vec
);

/**
 * @brief 듀얼 회전에서 회전(quat)과 위치(vec3)를 추출
 */
BYUL_API void dualquat_to_quat_vec(
    const dualquat_t* dq,
    quat_t* rot,
    vec3_t* vec
);

// -----------------------------
// 연산
// -----------------------------

/** @brief 정규화 */
BYUL_API void dualquat_normalize(dualquat_t* out, const dualquat_t* dq);

/** @brief 켤레 */
BYUL_API void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq);

// -----------------------------
// 변환 / 적용
// -----------------------------

/** @brief 듀얼 회전을 벡터에 적용 (회전 + 이동) */
BYUL_API void dualquat_transform_point(const dualquat_t* dq, vec3_t* io_point);

/** @brief 4x4 행렬로 변환 (column-major) */
BYUL_API void dualquat_to_mat4(const dualquat_t* dq, float* out_mat4);

/** @brief 4x4 행렬로부터 생성 */
BYUL_API void dualquat_from_mat4(dualquat_t* out, const float* mat4x4);

/** @brief 3x3 회전 행렬 변환 (이동 정보는 제외) */
BYUL_API void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3);

/** @brief 3x3 행렬로부터 생성 (이동은 0으로 설정) */
BYUL_API void dualquat_from_mat3(dualquat_t* out, const float* mat3);

// -----------------------------
// 보간
// -----------------------------

/** @brief 선형 보간 (LERP) */
BYUL_API void dualquat_lerp(dualquat_t* out, const dualquat_t* a, const dualquat_t* b, float t);

/** @brief 구면 선형 보간 (SLERP) */
BYUL_API void dualquat_slerp(dualquat_t* out, const dualquat_t* a, const dualquat_t* b, float t);

/** @brief 듀얼 쿼터니언 덧셈: out = a + b */
BYUL_API void dualquat_add(dualquat_t* out, const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언 뺄셈: out = a - b */
BYUL_API void dualquat_sub(dualquat_t* out, const dualquat_t* a, const dualquat_t* b);

/** @brief 곱셈 (합성 변환) */
BYUL_API void dualquat_mul(dualquat_t* out, const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언 스케일: out = a * scalar */
BYUL_API void dualquat_scale(dualquat_t* out, const dualquat_t* a, float scalar);

/** @brief 듀얼 쿼터니언의 내적 (real + dual dot) */
BYUL_API float dualquat_dot(const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언의 크기(norm): sqrt(real.norm^2 + dual.norm^2) */
BYUL_API float dualquat_length(const dualquat_t* a);

/**
 * @brief 듀얼 쿼터니언의 역을 계산합니다.
 * 
 * conj(real) + ε·(-conj(dual)) 방식으로 역변환을 구성합니다.
 * 이 연산은 rigid transform의 역행렬에 해당하며,
 * 회전과 이동 모두를 반대로 적용하는 데 사용됩니다.
 * 
 * @param[out] out 결과를 저장할 듀얼 쿼터니언
 * @param[in] dq 원본 듀얼 쿼터니언
 */
BYUL_API void dualquat_inverse(dualquat_t* out, const dualquat_t* dq);

/**
 * @brief 듀얼 쿼터니언의 부호를 정렬합니다.
 * 
 * real.w < 0일 경우 dualquat 전체에 -1을 곱해 정규화된 방향성을 유지합니다.
 * SLERP 등의 보간 과정에서 방향 뒤틀림을 방지하기 위해 사용됩니다.
 * 
 * @param[out] out 부호 정렬된 결과
 * @param[in] dq 원본 듀얼 쿼터니언
 */
BYUL_API void dualquat_align(dualquat_t* out, const dualquat_t* dq);

/**
 * @brief 두 듀얼 쿼터니언의 가중 평균을 계산합니다.
 * 
 * 단순 선형 보간 w1·a + w2·b 후 정규화 과정을 거쳐 결과를 생성합니다.
 * 이는 물리적으로 정확한 rigid transform 보간은 아니지만,
 * 다중 샘플 블렌딩 등에서 부드러운 결과를 생성하는 데 유용합니다.
 * 
 * @param[out] out 결과 듀얼 쿼터니언
 * @param[in] a 첫 번째 듀얼 쿼터니언
 * @param[in] w1 첫 번째 가중치
 * @param[in] b 두 번째 듀얼 쿼터니언
 * @param[in] w2 두 번째 가중치
 */
BYUL_API void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2);

// -----------------------------
// dualquat_identity: 단위 회전 + 위치 0
// -----------------------------
void dualquat_identity(dualquat_t* out);

#ifdef __cplusplus
}
#endif

#endif // DUALQUAT_H
