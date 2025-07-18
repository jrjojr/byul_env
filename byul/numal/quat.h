#ifndef QUAT_H
#define QUAT_H

#include <stdint.h>
#include "byul_config.h"
#include "internal/vec3.h" // vec3_t 필요

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------
// 🎛️ 회전 순서 정의 (Euler order)
// ----------------------------------------

typedef enum euler_order_t {
    EULER_ORDER_ZYX, /// (Roll → Yaw → Pitch), 일반적으로 가장 많이 사용됨
    EULER_ORDER_XYZ,
    EULER_ORDER_XZY,
    EULER_ORDER_YXZ,
    EULER_ORDER_YZX,
    EULER_ORDER_ZXY
} euler_order_t;

// ----------------------------------------
// 🌀 회전 구조체 전방 선언
// ----------------------------------------

typedef struct s_quat{
    float w;
    float x;
    float y;
    float z;
} quat_t;

// ----------------------------------------
// 🎯 생성자 / 소멸자 / 복사
// ----------------------------------------

/** @brief 단위 회전(w=1, x=y=z=0)을 생성합니다. */
BYUL_API quat_t* quat_new(void);

BYUL_API quat_t* quat_new_full(float w, float x, float y, float z);

/** @brief 축+라디안 회전 생성 (동적 할당됨) */
BYUL_API quat_t* quat_new_axis_angle(const vec3_t* axis, float radians);

/** @brief 축+도(degree) 회전 생성 (동적 할당됨) */
BYUL_API quat_t* quat_new_axis_deg(const vec3_t* axis, float degrees);

/** @brief quat 해제 (quat_new_*()로 생성된 것만) */
BYUL_API void quat_free(quat_t* r);

/** @brief 복사본 생성 */
BYUL_API quat_t* quat_copy(const quat_t* src);

BYUL_API void quat_get(const quat_t* src, 
    float* out_w, float* out_x, float* out_y, float* out_z);

BYUL_API void quat_set(quat_t* src, 
    float w, float x, float y, float z);    

// ----------------------------------------
// 🧪 비교 / 해시 / 유효성
// ----------------------------------------

/** @brief 두 회전이 같은지 비교합니다. */
BYUL_API int quat_equal(const quat_t* a, const quat_t* b);

/** @brief 회전 해시값 계산 (32비트) */
BYUL_API uint32_t quat_hash(const quat_t* r);

/** @brief 회전 값이 유효한지 확인 (norm ≈ 1.0인지, 0 벡터인지 등) */
BYUL_API int quat_is_valid(const quat_t* r);


// ----------------------------------------
// 🏗️ 회전 구성 및 변환
// ----------------------------------------

/**
 * @brief 오일러 각도로 회전 설정 (out에 저장)
 * 
 * @param out 결과 회전
 * @param radians_x X축 회전 (pitch)
 * @param radians_y Y축 회전 (yaw)
 * @param radians_z Z축 회전 (roll)
 * @param order 회전 순서 (예: EULER_ORDER_ZYX)
 */
BYUL_API void quat_from_euler(quat_t* out,
    float radians_x, float radians_y, float radians_z,
    euler_order_t order);

BYUL_API void quat_from_axis_angle(quat_t* out, 
    const vec3_t* axis, float radians);

// -----------------------------
// quat_from_axis_deg
// -----------------------------
BYUL_API void quat_from_axis_deg(quat_t* out, 
    const vec3_t* axis, float degrees);

/** @brief 회전을 단위 회전으로 재설정 */
BYUL_API void quat_reset(quat_t* out);


// ----------------------------------------
// 🔁 회전 연산
// ----------------------------------------

/** @brief 두 회전을 곱합니다. (r = a ∘ b) */
BYUL_API void quat_mul(quat_t* out, const quat_t* a, const quat_t* b);

/** @brief 회전의 켤레 (벡터 반전용) */
BYUL_API void quat_conjugate(quat_t* out, const quat_t* in);

/** @brief 회전의 역 (역회전) */
BYUL_API void quat_inverse(quat_t* out, const quat_t* in);

/** @brief 회전을 벡터에 적용합니다. (회전 후 좌표 반환) */
BYUL_API void quat_rotate_vector(vec3_t* out, 
    const quat_t* r, const vec3_t* v);

BYUL_API void quat_apply_to_vec3(
    const quat_t* q, const vec3_t* v, vec3_t* out);

// ----------------------------------------
// 🔄 회전 ↔ 행렬 변환
// ----------------------------------------

/** @brief 회전을 3x3 회전 행렬(float[9], column-major)로 변환 */
BYUL_API void quat_to_mat3(const quat_t* r, float* out_mat3x3);

/** @brief 회전을 4x4 회전 행렬(float[16], column-major)로 변환 */
BYUL_API void quat_to_mat4(const quat_t* r, float* out_mat4x4);

/** @brief 3x3 행렬 → 회전 */
BYUL_API void quat_from_mat3(quat_t* out, const float* mat3x3);

/** @brief 4x4 행렬 → 회전 */
BYUL_API void quat_from_mat4(quat_t* out, const float* mat4x4);


// ----------------------------------------
// 📐 회전 보간
// ----------------------------------------

/** @brief 두 회전 선형 보간 (빠르지만 부정확) */
BYUL_API void quat_lerp(quat_t* out, 
    const quat_t* a, const quat_t* b, float t);

/** @brief 두 회전 구면 선형 보간 (정확하고 부드러움) */
BYUL_API void quat_slerp(quat_t* out, 
    const quat_t* a, const quat_t* b, float t);

/**
 * @brief 회전을 회전축 + 회전각(라디안)으로 변환합니다.
 * 
 * @param r 회전 정보
 * @param out_axis 회전 축 (정규화된 벡터)
 * @param out_radians 회전각 (라디안)
 */
BYUL_API void quat_to_axis_angle(
    const quat_t* r, vec3_t* out_axis, float* out_radians);

/**
 * @brief 회전 기준 전방 벡터를 계산합니다.
 * 
 * @param r 회전 정보
 * @param out_dir 전방 벡터 (보통 z=-1 기준이 회전된 값)
 */
BYUL_API void quat_get_forward(const quat_t* r, vec3_t* out_dir);

/**
 * @brief 회전 기준의 '위쪽' 방향 벡터를 계산합니다.
 * 
 * @param r 회전 정보
 * @param out_dir 회전된 up 벡터
 */
BYUL_API void quat_get_up(const quat_t* r, vec3_t* out_dir);

/**
 * @brief 회전 기준의 '오른쪽' 방향 벡터를 계산합니다.
 * 
 * @param r 회전 정보
 * @param out_dir 회전된 right 벡터
 */
BYUL_API void quat_get_right(const quat_t* r, vec3_t* out_dir);

/**
 * @brief 회전을 오일러 각도로 변환합니다.
 * 
 * @param r 입력 회전
 * @param out_x X축 회전각 (라디안, pitch)
 * @param out_y Y축 회전각 (라디안, yaw)
 * @param out_z Z축 회전각 (라디안, roll)
 * @param order 출력 회전 순서 (입력과 같게 유지해야 정확)
 */
BYUL_API void quat_to_euler(const quat_t* r,
    float* out_x, float* out_y, float* out_z,
    euler_order_t order);

/** @brief 두 쿼터니언을 더합니다. out = a + b */
BYUL_API void quat_add(quat_t* out, const quat_t* a, const quat_t* b);

/** @brief 두 쿼터니언을 뺍니다. out = a - b */
BYUL_API void quat_sub(quat_t* out, const quat_t* a, const quat_t* b);

/** @brief 쿼터니언을 스칼라로 스케일링합니다. */
BYUL_API void quat_scale(quat_t* out, const quat_t* a, float s);

/** @brief 쿼터니언을 정규화합니다 (norm = 1) */
BYUL_API void quat_normalize(quat_t* out, const quat_t* a);

/** @brief 두 쿼터니언의 내적을 계산합니다. */
BYUL_API float quat_dot(const quat_t* a, const quat_t* b);

/** @brief 쿼터니언의 크기(norm)를 반환합니다. */
BYUL_API float quat_length(const quat_t* a);

// 단위 회전
BYUL_API void quat_identity(quat_t* out);

#ifdef __cplusplus
}
#endif

#endif // QUAT_H
