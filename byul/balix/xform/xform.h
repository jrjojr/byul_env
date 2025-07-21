#ifndef XFORM_H
#define XFORM_H

#include "internal/vec3.h"
#include "internal/quat.h"       // euler_order_t 정의
#include "internal/dualquat.h"
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// xform 위치 좌표 한계값 정의 (무한맵용 범위)
// ---------------------------------------------------------
#ifndef XFORM_POS_MAX
#define XFORM_POS_MAX   (99999.0f)   ///< xform 위치 좌표 최대값
#endif

#ifndef XFORM_POS_MIN
#define XFORM_POS_MIN   (-99999.0f)  ///< xform 위치 좌표 최소값
#endif

/**
 * @brief 위치(Position), 회전(Rotation), 스케일(Scale)을 통합 표현하는 변환 구조체.
 *
 * - 위치: vec3_t (dualquat 내부 translation)
 * - 회전: dualquat_t 기반
 * - 스케일: 기본값 (1,1,1)
 *
 * @note 위치 좌표는 항상 [XFORM_POS_MIN, XFORM_POS_MAX] 범위를 유지해야 합니다.
 *       xform_set_position(), xform_translate() 등에서 자동으로 클램프(clamp) 처리가 필요합니다.
 */
typedef struct s_xform {
    dualquat_t dq;
    vec3_t scale;   ///< 스케일 (기본값: 1,1,1)
} xform_t;

// ---------------------------------------------------------
// 📌 생성 / 복사 / 비교
// ---------------------------------------------------------

BYUL_API void xform_init(xform_t* out);

BYUL_API void xform_init_axis_angle(
    xform_t* out, const vec3_t* pos,
    const vec3_t* axis, float radians);

BYUL_API void xform_init_axis_angle_deg(
    xform_t* out, const vec3_t* pos,
    const vec3_t* axis, float degrees);

BYUL_API void xform_init_euler(
    xform_t* out,
    const vec3_t* pos,
    float yaw, float pitch, float roll,
    euler_order_t order);

BYUL_API void xform_init_euler_deg(
    xform_t* out,
    const vec3_t* pos,
    float yaw_deg, float pitch_deg, float roll_deg,
    euler_order_t order);

BYUL_API void xform_assign(xform_t* out, const xform_t* src);

BYUL_API bool xform_equal(const xform_t* a, const xform_t* b);

// ---------------------------------------------------------
// 📌 위치 / 회전 / 스케일 Get/Set
// ---------------------------------------------------------

/**
 * @brief xform의 현재 위치를 반환합니다.
 */
BYUL_API void xform_get_position(const xform_t* xf, vec3_t* out);

/**
 * @brief xform의 위치를 설정합니다.
 * @note pos가 [XFORM_POS_MIN, XFORM_POS_MAX] 범위를 벗어나면 자동으로 클램프 처리됩니다.
 */
BYUL_API void xform_set_position(xform_t* xf, const vec3_t* pos);

BYUL_API void xform_get_axis_angle(
    const xform_t* xf, vec3_t* out_axis, float* out_radians);
BYUL_API void xform_get_axis_angle_deg(
    const xform_t* xf, vec3_t* out_axis, float* out_degrees);

BYUL_API void xform_set_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);
BYUL_API void xform_set_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees);

BYUL_API void xform_set_euler(
    xform_t* xf,
    float yaw, float pitch, float roll,
    euler_order_t order);
BYUL_API void xform_set_euler_deg(
    xform_t* xf,
    float yaw_deg, float pitch_deg, float roll_deg,
    euler_order_t order);

BYUL_API void xform_get_euler(
    const xform_t* xf,
    float* out_yaw, float* out_pitch, float* out_roll,
    euler_order_t order);
BYUL_API void xform_get_euler_deg(
    const xform_t* xf,
    float* out_yaw_deg, float* out_pitch_deg, float* out_roll_deg,
    euler_order_t order);

BYUL_API void xform_set_scale(xform_t* xf, float sx, float sy, float sz);
BYUL_API void xform_get_scale(const xform_t* xf, vec3_t* out_scale);

// ---------------------------------------------------------
// 📌 이동 / 회전 조작
// ---------------------------------------------------------

/**
 * @brief 월드 좌표계 기준으로 xform을 이동시킵니다.
 * @note 이동 후 위치는 [XFORM_POS_MIN, XFORM_POS_MAX]로 클램프됩니다.
 */
BYUL_API void xform_translate(xform_t* xf, const vec3_t* delta_world);

/**
 * @brief 로컬 좌표계 기준으로 xform을 이동시킵니다.
 * @note 이동 후 위치는 [XFORM_POS_MIN, XFORM_POS_MAX]로 클램프됩니다.
 */
BYUL_API void xform_translate_local(xform_t* xf, const vec3_t* delta_local);

BYUL_API void xform_rotate_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);
BYUL_API void xform_rotate_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees);

BYUL_API void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);
BYUL_API void xform_rotate_local_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees);

// ---------------------------------------------------------
// 📌 고급 변환 유틸 (Inverse / Mul / LookAt)
// ---------------------------------------------------------

BYUL_API void xform_inverse(xform_t* out, const xform_t* src);
BYUL_API void xform_mul(xform_t* out, const xform_t* a, const xform_t* b);
BYUL_API void xform_look_at(
    xform_t* out,
    const vec3_t* eye,
    const vec3_t* target,
    const vec3_t* up);
BYUL_API void xform_from_forward(
    xform_t* out,
    const vec3_t* forward,
    const vec3_t* up);
BYUL_API void xform_align_vectors(
    xform_t* out,
    const vec3_t* from,
    const vec3_t* to);

// ---------------------------------------------------------
// 📌 적용 (좌표 변환)
// ---------------------------------------------------------

BYUL_API void xform_apply_to_point(
    const xform_t* xf, const vec3_t* local, vec3_t* out_world);
BYUL_API void xform_apply_to_point_inverse(
    const xform_t* xf, const vec3_t* world, vec3_t* out_local);

BYUL_API void xform_apply_to_direction(
    const xform_t* xf, const vec3_t* local_dir, vec3_t* out_dir);
BYUL_API void xform_apply_to_direction_inverse(
    const xform_t* xf, const vec3_t* world_dir, vec3_t* out_local_dir);

// ---------------------------------------------------------
// 📌 행렬 변환 (GPU / 디버깅용)
// ---------------------------------------------------------

BYUL_API void xform_to_mat4(const xform_t* xf, float* out_mat4_16);

BYUL_API void xform_lerp(xform_t* out,
    const xform_t* a, const xform_t* b, float t);
BYUL_API void xform_slerp(xform_t* out,
    const xform_t* a, const xform_t* b, float t);
BYUL_API void xform_nlerp(xform_t* out,
    const xform_t* a, const xform_t* b, float t);

BYUL_API void xform_apply(xform_t* out,
    const xform_t* parent, const xform_t* local);
BYUL_API void xform_apply_inverse(xform_t* out,
    const xform_t* parent, const xform_t* world);

// ---------------------------------------------------------
// 📌 디버깅
// ---------------------------------------------------------

/**
 * @brief xform 정보를 콘솔에 출력 (위치, 회전, 스케일)
 */
BYUL_API void xform_print(const xform_t* xf);

#ifdef __cplusplus
}
#endif

#endif // XFORM_H
