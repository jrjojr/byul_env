#ifndef QUAT_H
#define QUAT_H

#include <stdint.h>
#include "byul_config.h"
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Euler rotation order
// ---------------------------------------------------------

typedef enum euler_order_t {
    EULER_ORDER_ZYX,  ///< (Roll -> Yaw -> Pitch), commonly used
    EULER_ORDER_XYZ,
    EULER_ORDER_XZY,
    EULER_ORDER_YXZ,
    EULER_ORDER_YZX,
    EULER_ORDER_ZXY
} euler_order_t;

// ---------------------------------------------------------
// Quaternion structure
// ---------------------------------------------------------

typedef struct s_quat {
    float w;
    float x;
    float y;
    float z;
} quat_t;

// ---------------------------------------------------------
// Constructors / Initialization
// ---------------------------------------------------------

/** @brief Initialize to identity rotation (w=1, x=y=z=0) */
BYUL_API void quat_init(quat_t* out);

/** @brief Initialize with specified values */
BYUL_API void quat_init_full(quat_t* out, 
    float w, float x, float y, float z);

/** @brief Initialize with axis + radians angle */
BYUL_API void quat_init_axis_angle(quat_t* out, 
    const vec3_t* axis, float radians);

/** @brief Initialize with axis + degrees angle */
BYUL_API void quat_init_axis_deg(quat_t* out, 
    const vec3_t* axis, float degrees);

/** @brief Initialize with Euler angles (radians) */
BYUL_API void quat_init_euler(quat_t* out,
    float radians_x, float radians_y, float radians_z,
    euler_order_t order);

/** @brief Initialize with Euler angles (degrees) */
BYUL_API void quat_init_euler_deg(quat_t* out,
    float deg_x, float deg_y, float deg_z,
    euler_order_t order);

/** @brief Initialize with angular velocity (omega) * dt */
BYUL_API void quat_init_angular_velocity(quat_t* out, 
    const vec3_t* omega, float dt);

/** @brief Initialize with the minimal rotation aligning two vectors */
BYUL_API void quat_init_two_vector(quat_t* out, 
    const vec3_t* from, const vec3_t* to);

/** @brief Initialize with 3 axis vectors (right, up, forward) */
BYUL_API void quat_init_axes(quat_t* out, 
    const vec3_t* xaxis, const vec3_t* yaxis, const vec3_t* zaxis);

// ---------------------------------------------------------
// Copy / Set
// ---------------------------------------------------------

BYUL_API void quat_assign(quat_t* out, const quat_t* src);
BYUL_API void quat_get(const quat_t* src, 
    float* out_w, float* out_x, float* out_y, float* out_z);

BYUL_API void quat_set(quat_t* out, float w, float x, float y, float z);

// ---------------------------------------------------------
// Comparison / Validation
// ---------------------------------------------------------

BYUL_API int quat_equal(const quat_t* a, const quat_t* b);
BYUL_API uint32_t quat_hash(const quat_t* q);
BYUL_API int quat_is_valid(const quat_t* q);
BYUL_API void quat_reset(quat_t* out); ///< Reset to identity rotation

// ---------------------------------------------------------
// Rotation operations
// ---------------------------------------------------------

BYUL_API void quat_mul(quat_t* out, const quat_t* a, const quat_t* b);
BYUL_API void quat_conjugate(quat_t* out, const quat_t* in);
BYUL_API void quat_inverse(quat_t* out, const quat_t* in);

BYUL_API void quat_rotate_vector(const quat_t* q, const vec3_t* v, 
    vec3_t* out);

BYUL_API void quat_apply_to_vec3(
    const quat_t* q, const vec3_t* v, vec3_t* out);

// ---------------------------------------------------------
// Matrix conversions
// ---------------------------------------------------------

BYUL_API void quat_to_mat3(const quat_t* q, float* out_mat3x3);
BYUL_API void quat_to_mat4(const quat_t* q, float* out_mat4x4);
BYUL_API void quat_init_from_mat3(quat_t* out, const float* mat3x3);
BYUL_API void quat_init_from_mat4(quat_t* out, const float* mat4x4);

// ---------------------------------------------------------
// Rotation interpolation
// ---------------------------------------------------------

BYUL_API void quat_lerp(quat_t* out, 
    const quat_t* a, const quat_t* b, float t);

BYUL_API void quat_slerp(quat_t* out, 
    const quat_t* a, const quat_t* b, float t);

// ---------------------------------------------------------
// Conversion utilities
// ---------------------------------------------------------

BYUL_API void quat_to_axis_angle(const quat_t* q, 
    vec3_t* out_axis, float* out_radians);

BYUL_API void quat_get_forward(const quat_t* q, vec3_t* out_dir);
BYUL_API void quat_get_up(const quat_t* q, vec3_t* out_dir);
BYUL_API void quat_get_right(const quat_t* q, vec3_t* out_dir);

BYUL_API void quat_to_euler(const quat_t* q, 
    float* out_x, float* out_y, float* out_z, euler_order_t order);

// ---------------------------------------------------------
// Arithmetic operations
// ---------------------------------------------------------

BYUL_API void quat_add(quat_t* out, const quat_t* a, const quat_t* b);
BYUL_API void quat_sub(quat_t* out, const quat_t* a, const quat_t* b);
BYUL_API void quat_scale(quat_t* out, const quat_t* a, float s);
BYUL_API void quat_normalize(quat_t* io);
BYUL_API void quat_unit(quat_t* out, const quat_t* a);
BYUL_API float quat_dot(const quat_t* a, const quat_t* b);
BYUL_API float quat_length(const quat_t* a);

// ---------------------------------------------------------
// Identity rotation
// ---------------------------------------------------------

BYUL_API void quat_identity(quat_t* out);

/**
 * @brief Rotates a point around a given pivot using a quaternion.
 *
 * @param q       The rotation quaternion
 * @param point   The input position to be rotated
 * @param pivot   The center point of rotation
 * @param out     The resulting rotated position
 */
BYUL_API void quat_rotate_around_pivot(
    const quat_t* q,
    const vec3_t* point,
    const vec3_t* pivot,
    vec3_t* out);


#ifdef __cplusplus
}
#endif

#endif // QUAT_H
