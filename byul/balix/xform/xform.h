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
#ifndef XFORM_MAX_POS
#define XFORM_MAX_POS   (99999.0f)   ///< Maximum xform position value
#endif

#ifndef XFORM_MIN_POS
#define XFORM_MIN_POS   (-99999.0f)  ///< Minimum xform position value
#endif

/**
 * @brief Transform structure representing Position, Rotation, and Scale.
 *
 * - Position: vec3_t pos
 * - Rotation: quat_t rot
 * - Scale: default (1,1,1)
 *
 * @note Position coordinates must always remain within [XFORM_MIN_POS, XFORM_MAX_POS].
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

BYUL_API void xform_init_axis_deg(
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
BYUL_API void xform_get_axis_deg(
    const xform_t* xf, vec3_t* out_axis, float* out_degrees);

BYUL_API void xform_set_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);
BYUL_API void xform_set_axis_deg(
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
BYUL_API void xform_rotate_axis_deg(
    xform_t* xf, const vec3_t* axis, float degrees);

BYUL_API void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians);
BYUL_API void xform_rotate_local_axis_deg(
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

/**
 * @brief Constructs a transform from a forward and up direction.
 *
 * @param out Output transform.
 * @param forward Forward direction vector.
 * @param up Up direction vector.
 */
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

/**
 * @brief Rotates a transform (xform) around a given pivot point.
 *
 * @param xf    The transform to be rotated (in/out)
 * @param q     The rotation quaternion
 * @param pivot The pivot point in world space
 */
BYUL_API void xform_rotate_around_pivot(
    xform_t* xf,
    const quat_t* q,
    const vec3_t* pivot);

BYUL_API void xform_slerp_pivot(
    xform_t* out,
    const xform_t* a,
    const xform_t* b,
    float t,
    const vec3_t* pivot);

/**
 * @brief Generates a transform that looks at a target from a pivot point.
 *
 * @param xf Output transform.
 * @param target Target position to look at.
 * @param up Up direction vector.
 * @param pivot Position to place the transform and rotate from.
 */    
BYUL_API void xform_look_at_pivot(
    xform_t* xf,
    const vec3_t* target,
    const vec3_t* up,
    const vec3_t* pivot);

/**
 * @brief Rotates a transform around a pivot point using a local-space rotation.
 *
 * @param xf Transform to rotate (in/out).
 * @param q  Rotation quaternion to apply.
 * @param pivot Pivot point in world coordinates.
 */    
BYUL_API void xform_rotate_local_around_pivot(
    xform_t* xf,
    const quat_t* q,
    const vec3_t* pivot);

/**
 * @brief Orients the transform to face a given direction.
 *
 * @param xf Transform to modify.
 * @param direction Forward direction to face.
 * @param up World up direction.
 */    
BYUL_API void xform_look_to(
    xform_t* xf,
    const vec3_t* direction,
    const vec3_t* up);

/**
 * @brief Aligns one of the transform's local axes to a target direction.
 *
 * @param xf Transform to modify.
 * @param axis_index Axis to align (0:x, 1:y, 2:z).
 * @param target_dir Target direction to align the axis with.
 */    
BYUL_API void xform_align_axis(
    xform_t* xf,
    int axis_index,              // 0:x, 1:y, 2:z
    const vec3_t* target_dir);

/**
 * @brief Reflects a transform across a plane defined by a point and normal.
 *
 * @param out Output reflected transform.
 * @param src Source transform to reflect.
 * @param plane_point A point on the reflection plane.
 * @param plane_normal Normal vector of the reflection plane.
 */    
BYUL_API void xform_reflect(
    xform_t* out,
    const xform_t* src,
    const vec3_t* plane_point,
    const vec3_t* plane_normal);

// ---------------------------------------------------------
// Debug
// ---------------------------------------------------------

BYUL_API void xform_print(const xform_t* xf);

#ifdef __cplusplus
}
#endif

#endif // XFORM_H
