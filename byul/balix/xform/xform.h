#ifndef XFORM_H
#define XFORM_H

#include "vec3.h"
#include "quat.h"       // euler_order_t definition
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// xform position coordinate limits (for infinite maps)
// ---------------------------------------------------------
#ifndef XFORM_POS_MAX
#define XFORM_POS_MAX   (99999.0f)   ///< Maximum xform position value
#endif

#ifndef XFORM_POS_MIN
#define XFORM_POS_MIN   (-99999.0f)  ///< Minimum xform position value
#endif

/**
 * @brief Transform structure representing Position, Rotation, and Scale.
 *
 * - Position: vec3_t pos
 * - Rotation: quat_t rot
 * - Scale: default (1,1,1)
 *
 * @note Position coordinates must always remain within [XFORM_POS_MIN, XFORM_POS_MAX].
 *       Functions like xform_set_position() and xform_translate() should apply clamp automatically.
 */
typedef struct s_xform {
    vec3_t pos;     ///< Position
    quat_t rot;     ///< Rotation
    vec3_t scale;   ///< Scale (default: 1,1,1)
} xform_t;

// ---------------------------------------------------------
// Create / Copy / Compare
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
// Position / Rotation / Scale Get/Set
// ---------------------------------------------------------

BYUL_API void xform_get_position(const xform_t* xf, vec3_t* out);
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
// Translation / Rotation Operations
// ---------------------------------------------------------

BYUL_API void xform_translate(xform_t* xf, const vec3_t* delta_world);
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
// Advanced Transform Utilities (Inverse / Mul / LookAt)
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
// Application (Coordinate Transform)
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
// Matrix Conversion (for GPU / Debug)
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
// Debug
// ---------------------------------------------------------

BYUL_API void xform_print(const xform_t* xf);

#ifdef __cplusplus
}
#endif

#endif // XFORM_H
