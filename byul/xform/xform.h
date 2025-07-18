#ifndef XFORM_H
#define XFORM_H

#include "internal/vec3.h"
#include "internal/quat.h"
#include "internal/dualquat.h"
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_xform {
    dualquat_t dq;
}xform_t;

// ---------------------------------------------------------
// 📌 생성 / 복사 / 비교
// ---------------------------------------------------------

/**
 * @brief 단위 변환을 생성합니다. (위치: 0, 회전: 없음)
 * @return 새로 생성된 단위 xform 포인터 (heap 할당됨)
 */
BYUL_API xform_t* xform_new_identity(void);

// /**
//  * @brief 위치 + 축-각 기반 회전으로 초기화된 xform 생성
//  * @param pos 초기 위치 (NULL이면 원점)
//  * @param axis 회전 축 단위 벡터
//  * @param radians 회전 각도 (라디안)
//  * @return 생성된 xform 포인터
//  */
BYUL_API xform_t* xform_new_from_axis_angle(
    const vec3_t* pos, const vec3_t* axis, float radians);

/**
 * @brief 오일러 각도로 회전 설정 (out에 저장)
 * 
 * @param out 결과 회전
 * @param radians_x X축 회전 (pitch)
 * @param radians_y Y축 회전 (yaw)
 * @param radians_z Z축 회전 (roll)
 * @param order 회전 순서 (예: EULER_ORDER_ZYX)
 */
BYUL_API xform_t* xform_new_from_euler(
    const vec3_t* pos,
    float yaw, float pitch, float roll,
    euler_order_t order);

/**
 * @brief xform 복사본 생성
 * @param src 원본 xform
 * @return 복사된 xform 포인터
 */
BYUL_API xform_t* xform_clone(const xform_t* src);

/** 해제 */
BYUL_API void xform_free(xform_t* xf);

/** 비교 */
BYUL_API bool xform_equal(const xform_t* a, const xform_t* b);

// ---------------------------------------------------------
// 📌 위치 / 회전 Get/Set
// ---------------------------------------------------------

BYUL_API void xform_get_position(const xform_t* xf, vec3_t* out);

BYUL_API void xform_set_position(xform_t* xf, const vec3_t* pos);

BYUL_API void xform_get_axis_angle(
    const xform_t* xf, vec3_t* out_axis, float* out_radians);

BYUL_API void xform_set_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);

// BYUL_API void xform_get_euler(
//     const xform_t* xf, float* out_yaw, float* out_pitch, float* out_roll);

// BYUL_API void xform_set_euler(
//     xform_t* xf, float yaw, float pitch, float roll);

/**
 * @brief 오일러 각도로 회전 설정
 * 
 * @param xf 대상 xform
 * @param yaw Y축 회전 (Yaw)
 * @param pitch X축 회전 (Pitch)
 * @param roll Z축 회전 (Roll)
 * @param order 회전 순서 (예: EULER_ORDER_ZYX)
 */
BYUL_API void xform_set_euler(
    xform_t* xf,
    float yaw, float pitch, float roll,
    euler_order_t order);

/**
 * @brief 오일러 각도로 회전 설정 (out에 저장)
 * 
 * @param out 결과 회전
 * @param radians_x X축 회전 (pitch)
 * @param radians_y Y축 회전 (yaw)
 * @param radians_z Z축 회전 (roll)
 * @param order 회전 순서 (예: EULER_ORDER_ZYX)
 */
BYUL_API void xform_get_euler(
    const xform_t* xf,
    float* out_yaw, float* out_pitch, float* out_roll,
    euler_order_t order);

// ---------------------------------------------------------
// 📌 이동 / 회전 조작
// ---------------------------------------------------------

/** @brief 월드 기준 위치 이동 */
BYUL_API void xform_translate(xform_t* xf, const vec3_t* delta_world);

/** @brief 로컬 기준 위치 이동 */
BYUL_API void xform_translate_local(xform_t* xf, const vec3_t* delta_local);

/** @brief 월드 기준 회전 (축-각) */
BYUL_API void xform_rotate_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);

/** @brief 로컬 기준 회전 (축-각) */
BYUL_API void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);

// ---------------------------------------------------------
// 📌 적용 (벡터 변환)
// ---------------------------------------------------------

/**
 * @brief 로컬 공간의 점을 월드 공간으로 변환합니다.
 * @param xf 변환 정보
 * @param local 로컬 좌표
 * @param out_world 변환된 결과
 */
BYUL_API void xform_apply_to_point(
    const xform_t* xf, const vec3_t* local, vec3_t* out_world);

    /**
 * @brief 로컬 방향 벡터를 월드 방향으로 변환합니다. (정규화됨)
 * @param xf 변환 정보
 * @param local_dir 방향 벡터 (크기 무관)
 * @param out_dir 정규화된 월드 방향 벡터
 */
BYUL_API void xform_apply_to_direction(
    const xform_t* xf, const vec3_t* local_dir, vec3_t* out_dir);

// ---------------------------------------------------------
// 📌 행렬 변환 (GPU / 디버깅용)
// ---------------------------------------------------------

/**
 * @brief 4x4 행렬로 변환 (열 우선, OpenGL 스타일)
 * @param xf 변환 정보
 * @param out_mat4_16 16개 float 배열
 */
BYUL_API void xform_to_mat4(const xform_t* xf, float* out_mat4_16);

/**
 * @brief 두 xform 사이를 선형 보간합니다. (위치+회전 모두 선형)
 * 
 * @param out 결과 저장할 xform
 * @param a 시작 xform
 * @param b 끝 xform
 * @param t 보간 계수 (0.0 ~ 1.0)
 */
BYUL_API void xform_lerp(xform_t* out, 
    const xform_t* a, const xform_t* b, float t);

/**
 * @brief 두 xform 사이를 구면 선형 보간합니다. (회전은 slerp, 위치는 선형)
 * 
 * @param out 결과 저장할 xform
 * @param a 시작 xform
 * @param b 끝 xform
 * @param t 보간 계수 (0.0 ~ 1.0)
 */
BYUL_API void xform_slerp(xform_t* out, 
    const xform_t* a, const xform_t* b, float t);

/**
 * @brief 로컬 공간의 변환을 부모 공간에 적용하여 최종 결과를 계산합니다.
 * 
 * 이 함수는 주로 객체를 부모 위치에 상대적으로 배치할 때 사용됩니다.
 * out = parent ∘ local 변환
 */
BYUL_API void xform_apply(xform_t* out, 
    const xform_t* parent, const xform_t* local);

/**
 * @brief 부모 기준에서 상대적인 로컬 변환을 계산합니다.
 * 
 * out = inverse(parent) ∘ world
 * 
 * 주로 부모 공간에서의 상대 위치, 회전 등을 추정할 때 사용됩니다.
 * 
 * @param out 결과 저장할 로컬 xform
 * @param parent 부모 xform (적용을 제거할 기준)
 * @param world 적용된 월드 xform
 */
BYUL_API void xform_apply_inverse(xform_t* out, 
    const xform_t* parent, const xform_t* world);

#ifdef __cplusplus
}
#endif

#endif // XFORM_H
