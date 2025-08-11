#ifndef VEC3_H
#define VEC3_H

#include <stdint.h>
#include <stddef.h>

#include "byul_common.h"

// World-scale absolute tolerances (tune in one place)
#ifndef VEC3_ABS_EPS_LEN
#define VEC3_ABS_EPS_LEN 1e-6f      // actual length threshold
#endif
#ifndef VEC3_ABS_EPS_LEN2
#define VEC3_ABS_EPS_LEN2 (VEC3_ABS_EPS_LEN * VEC3_ABS_EPS_LEN)  // 1e-12f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3D vector structure
 * 
 * Represents 3D coordinates or directions.
 * Primarily used for position, delta (movement), or direction.
 */
typedef struct s_vec3 {
    float x; ///< X coordinate
    float y; ///< Y coordinate
    float z; ///< Z coordinate
} vec3_t;

BYUL_API void vec3_init_full(vec3_t* out, float x, float y, float z);

/**
 * @brief Initialize vec3 with default values (0,0,0).
 */
BYUL_API void vec3_init(vec3_t* out);

/**
 * @brief Copy vec3.
 */
BYUL_API void vec3_assign(vec3_t* out, const vec3_t* src);

/**
 * @brief Compare equality of two vectors.
 * 
 * @param a Vector A
 * @param b Vector B
 */
BYUL_API bool vec3_equal(const vec3_t* a, const vec3_t* b);

/**
 * @brief Compare each component of two vec3 with the same tolerance.
 *
 * @param a   First vector
 * @param b   Second vector
 * @param tol Allowed tolerance (applied to all x, y, z)
 * @return true if all component differences are within tolerance
 */
BYUL_API bool vec3_equal_tol(const vec3_t* a, const vec3_t* b, float tol);

/**
 * @brief Compare each component of two vec3 with separate positive/negative tolerances.
 *
 * @param a        First vector
 * @param b        Second vector
 * @param tol_pos  Positive direction tolerance (applied when b >= a)
 * @param tol_neg  Negative direction tolerance (applied when b < a)
 * @return true if all component differences are within tolerances
 */
BYUL_API bool vec3_equal_tol_all(
    const vec3_t* a, const vec3_t* b,
    float tol_pos, float tol_neg);

/**
 * @brief Compute a hash value for vec3.
 * 
 * Converts float values to integers for simple hashing.
 * 
 * @param v Vector to hash
 * @return unsigned int Hash value
 */
BYUL_API unsigned int vec3_hash(const vec3_t* v);

BYUL_API void vec3_zero(vec3_t* out);

BYUL_API void vec3_negate(vec3_t* out, const vec3_t* a);

BYUL_API void vec3_add(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_sub(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_mul(vec3_t* out, const vec3_t* a, const vec3_t* b);

/**
 * @brief Divide a vector by another vector (element-wise division).
 *
 * Each component performs a / b.  
 * If any component of b is 0.0f or less than FLOAT_EPSILON, the result for that component is set to INFINITY.
 *
 * @param[out] out Result vector
 * @param[in]  a   Input vector
 * @param[in]  b   Divisor vector
 */
BYUL_API void vec3_div(vec3_t* out, const vec3_t* a, const vec3_t* b);

/**
 * @brief Divide a vector by a scalar.
 *
 * @param[out] out   Result vector
 * @param[in]  a     Input vector
 * @param[in]  scalar Divisor (if 0.0f or <= FLOAT_EPSILON, result is INFINITY)
 */
BYUL_API void vec3_div_scalar(vec3_t* out, const vec3_t* a, float scalar);

BYUL_API void vec3_scale(vec3_t* out, const vec3_t* a, float scalar);

BYUL_API float vec3_dot(const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_cross(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API float vec3_length(const vec3_t* a);

/**
 * @brief Returns the squared length (without sqrt for performance).
 */
BYUL_API float vec3_length_sq(const vec3_t* a);

BYUL_API void vec3_normalize(vec3_t* a);

BYUL_API void vec3_unit(vec3_t* out, const vec3_t* src);

BYUL_API float vec3_distance(const vec3_t* a, const vec3_t* b);

BYUL_API float vec3_distance_sq(const vec3_t* a, const vec3_t* b);

/**
 * @brief Calculate linear interpolation between start and goal.
 * 
 * @param out Result position (output)
 * @param start Start position
 * @param goal Target position
 * @param t Interpolation factor (0.0 ~ 1.0)
 */
BYUL_API void vec3_lerp(vec3_t* out, 
    const vec3_t* start, const vec3_t* goal, float t);

/**
 * @brief Convert a vec3 into a 4x4 translation matrix (position only).
 * 
 * Creates a transformation matrix with only translation, no rotation.
 * 
 * @param v Position vector
 * @param out_mat4 16-element float array (column-major order)
 */
BYUL_API void vec3_to_mat4(const vec3_t* v, float* out_mat4);

/**
 * @brief Check if vec3 is approximately (0,0,0).
 *
 * @param v Vector to check
 * @return true if all components are close to 0
 * @return false otherwise
 */
BYUL_API bool vec3_is_zero(const vec3_t* v);

/**
 * @brief Convert vec3 to string.
 * @param v Vector to convert
 * @param buffer_size Buffer size
 * @param buffer Output buffer (at least 64 bytes recommended)
 * @return buffer (for convenience)
 */
BYUL_API char* vec3_to_string(
    const vec3_t* v, size_t buffer_size, char* buffer);

/**
 * @brief Print vec3 to console.
 * @param v Vector to print
 */
BYUL_API void vec3_print(const vec3_t* v);

/**
 * @brief Add another vector to the current vector (in-place addition).
 *
 * @param io Target vector (result is stored here)
 * @param other Vector to add
 */
BYUL_API void vec3_iadd(vec3_t* io, const vec3_t* other);

/**
 * @brief Subtract another vector from the current vector (in-place subtraction).
 */
BYUL_API void vec3_isub(vec3_t* io, const vec3_t* other);

/**
 * @brief Scale the current vector by a scalar (in-place scaling).
 */
BYUL_API void vec3_iscale(vec3_t* io, float scalar);

/**
 * @brief out = a + b * scalar
 *
 * @param out Result vector
 * @param a   Base vector
 * @param b   Vector to scale and add
 * @param scalar Scalar value
 */
BYUL_API void vec3_madd(vec3_t* out, const vec3_t* a, const vec3_t* b, float scalar);

/**
 * @brief Predict position using constant acceleration (p + v*t + 0.5*a*t^2).
 *
 * @param out Result vector (output)
 * @param p   Initial position
 * @param v   Velocity vector
 * @param a   Acceleration vector
 * @param t   Time (seconds)
 */
BYUL_API void vec3_project(vec3_t* out, const vec3_t* p,
                           const vec3_t* v, const vec3_t* a, float t);

/**
 * @brief Reflects a vector across a plane defined by a normal.
 *
 * @param out Output reflected vector.
 * @param v   Input vector to reflect.
 * @param n   Normal vector of the plane (must be normalized).
 */
BYUL_API void vec3_reflect(vec3_t* out, const vec3_t* v, const vec3_t* n);                           
#ifdef __cplusplus
}
#endif

#endif // VEC3_H