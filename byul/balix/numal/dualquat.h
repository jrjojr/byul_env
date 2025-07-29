#ifndef DUALQUAT_H
#define DUALQUAT_H

#include "quat.h"
#include "vec3.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dual Quaternion (Rotation + Translation)
 * real: rotation, dual: translation
 */
typedef struct dualquat_t {
    quat_t real;
    quat_t dual;
} dualquat_t;

// ---------------------------------------------------------
// Constructors / Initialization / Copy
// ---------------------------------------------------------

/** @brief Identity dual quaternion (rotation = identity, translation = 0) */
BYUL_API void dualquat_init(dualquat_t* out);

/**
 * @brief Initialize with rotation (quat) + position (vec3)
 * @param rot Rotation quaternion (NULL means identity rotation)
 * @param vec Position vector (NULL means origin)
 */
BYUL_API void dualquat_init_quat_vec(
    dualquat_t* out, const quat_t* rot, const vec3_t* vec);

/** @brief Initialize from 3x3 rotation matrix (translation = 0) */
BYUL_API void dualquat_init_from_mat3(dualquat_t* out, const float* mat3x3);

/** @brief Initialize from 4x4 transformation matrix */
BYUL_API void dualquat_init_from_mat4(dualquat_t* out, const float* mat4x4);

/** @brief Copy */
BYUL_API void dualquat_assign(dualquat_t* out, const dualquat_t* src);

// ---------------------------------------------------------
// Comparison / Hash
// ---------------------------------------------------------

BYUL_API int dualquat_equal(const dualquat_t* a, const dualquat_t* b);
BYUL_API uint32_t dualquat_hash(const dualquat_t* dq);

// ---------------------------------------------------------
// Conversion / Extraction
// ---------------------------------------------------------

/**
 * @brief Decompose a dual quaternion into rotation and translation
 * @param rot (NULL allowed) Output rotation
 * @param vec (NULL allowed) Output translation
 */
BYUL_API void dualquat_to_quat_vec(
    const dualquat_t* dq, quat_t* rot, vec3_t* vec);

/** @brief Convert to 3x3 rotation matrix */
BYUL_API void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3);

/** @brief Convert to 4x4 transformation matrix */
BYUL_API void dualquat_to_mat4(const dualquat_t* dq, float* out_mat4);

// ---------------------------------------------------------
// Operations
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

/** @brief Conjugate */
BYUL_API void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq);

/** @brief Inverse transformation */
BYUL_API void dualquat_inverse(dualquat_t* out, const dualquat_t* dq);

/** @brief Normalize */
BYUL_API void dualquat_normalize(dualquat_t* io);
BYUL_API void dualquat_unit(dualquat_t* out, const dualquat_t* dq);

/** @brief Align sign (if real.w < 0, multiply by -1) */
BYUL_API void dualquat_align(dualquat_t* out, const dualquat_t* dq);

// ---------------------------------------------------------
// Interpolation
// ---------------------------------------------------------

/** @brief LERP (linear interpolation, requires normalization) */
BYUL_API void dualquat_lerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief NLERP (normalized linear interpolation) */
BYUL_API void dualquat_nlerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief SLERP (spherical linear interpolation) */
BYUL_API void dualquat_slerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/**
 * @brief Weighted blend (w1*a + w2*b, followed by normalization)
 */
BYUL_API void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2);

// ---------------------------------------------------------
// Point Transformation
// ---------------------------------------------------------

/**
 * @brief Transform a point (out = dq * in)
 */
BYUL_API void dualquat_apply_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out);

/**
 * @brief Inverse point transformation (out = dq^-1 * in)
 */
BYUL_API void dualquat_apply_inverse_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out);

/**
 * @brief In-place point transformation
 */
BYUL_API void dualquat_apply_to_point_inplace(
    const dualquat_t* dq, vec3_t* io_point);

// ---------------------------------------------------------
// Identity Constructor
// ---------------------------------------------------------
static inline void dualquat_identity(dualquat_t* out) {
    dualquat_init(out);
}

#ifdef __cplusplus
}
#endif

#endif // DUALQUAT_H
