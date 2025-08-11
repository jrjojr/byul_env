#ifndef FLOAT_COMMON_H
#define FLOAT_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Epsilon value for float comparison
// This is relative error, not absolute error, which is why it is 1e-5f instead of 1e-6f.
// For example, 1.000001 and 1.000002 are considered the same.
// CHECK(float_equal(1.00001f, 1.000019f));
// CHECK(float_equal(1.00001f, 1.000001f));
// CHECK_FALSE(float_equal(1.00001f, 1.000020f));
// CHECK_FALSE(float_equal(1.00001f, 1.000000f));
// Relative epsilon for float comparisons near magnitude ~1.
// Do not confuse with absolute zero thresholds.
#define FLOAT_EPSILON 1e-5f

/**
 * @def FLOAT_EPSILON_TINY
 * @brief Absolute lower bound used for checking very small values
 */
#define FLOAT_EPSILON_TINY      1e-8f

#define SQRT2_INV 0.70710678118f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923  // PI/2
#endif

/**
 * @def M_TWO_PI
 * @brief 2 * PI
 */
#define M_TWO_PI    (2.0f * M_PI)

/**
 * @def M_HALF_PI
 * @brief PI / 2
 */
#define M_HALF_PI   (0.5f * M_PI)

/** @brief Convert degrees to radians */
#define DEG2RAD(x) ((x) * 0.017453292519943295f)

/**
 * @brief Convert radians to degrees
 * RAD2DEG(rad)  ((rad) * (180.0f / M_PI))
 */
#define RAD2DEG(rad)  ((rad) * 57.29577951308232f)

BYUL_API int  float_compare(float a, float b, void* userdata);

BYUL_API int  int_compare(int a, int b, void* userdata);

// ---------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------

/**
 * @brief Compare floats using relative error
 * @param a First value
 * @param b Second value
 * @return true if values are sufficiently close
 */
BYUL_API bool float_equal(float a, float b);

/**
 * @brief Compare floats using a tolerance
 *
 * @param a   First value
 * @param b   Second value
 * @param tol Allowed tolerance (e.g., 1e-5f)
 * @return true if the difference is within tol
 */
BYUL_API bool float_equal_tol(float a, float b, float tol);

/**
 * @brief Check if b is within positive/negative tolerance around a
 *
 * @param a       Reference value
 * @param b       Value to compare
 * @param tol_pos Positive direction tolerance (when b >= a)
 * @param tol_neg Negative direction tolerance (when b < a)
 * @return true if within tolerance range, otherwise false
 *
 * Example:
 *   a = 1.0, tol_pos = 0.002, tol_neg = 0.001
 *   b = 1.002  -> true  (1.002 - 1.0 = 0.002 <= tol_pos)
 *   b = 0.999  -> true  (1.0 - 0.999 = 0.001 <= tol_neg)
 *   b = 0.998  -> false (1.0 - 0.998 = 0.002 > tol_neg)
 *
 * @note If tol_pos or tol_neg is negative, fabsf() is automatically applied.
 */
BYUL_API bool float_equal_tol_all(
    float a, float b, float tol_pos, float tol_neg);

/**
 * @brief Check if a value is close to zero
 * @param x Value to check
 * @return true if |x| < EPSILON
 */
BYUL_API bool float_zero(float x);

/**
 * @brief Safe division
 * @param a Numerator
 * @param b Denominator
 * @param fallback Value to return if b is zero
 * @return a / b or fallback
 */
BYUL_API float float_safe_div(float a, float b, float fallback);

/**
 * @brief Calculate square (x²)
 */
BYUL_API float float_square(float x);

/**
 * @brief Clamp a value to a specified range
 */
BYUL_API float float_clamp(float x, float min_val, float max_val);

/**
 * @brief Get the sign of a value
 * @return 1.0f if positive, -1.0f if negative, 0.0f if zero
 */
BYUL_API float float_sign(float x);

/**
 * @brief Convert degrees to radians
 */
BYUL_API float float_deg2rad(float deg);

/**
 * @brief Convert radians to degrees
 */
BYUL_API float float_rad2deg(float rad);

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor in [0,1]
 */
BYUL_API float float_lerp(float a, float b, float t);

/**
 * @brief Inverse linear interpolation
 * @param a Start value
 * @param b End value
 * @param value Value to evaluate
 * @return Position of value within [a,b] as a percentage (0~1)
 */
BYUL_API float float_inv_lerp(float a, float b, float value);

/**
 * @brief Remap a value from one range to another
 * @param in_min Input minimum
 * @param in_max Input maximum
 * @param out_min Output minimum
 * @param out_max Output maximum
 * @param value Input value
 * @return Remapped output value
 */
BYUL_API float float_remap(float in_min, float in_max, 
                     float out_min, float out_max, float value);

/**
 * @brief Clamp a value between 0.0 and 1.0
 *
 * If the input is greater than 1, returns 1.
 * If the input is less than 0, returns 0.
 * Useful for interpolation factors, opacity, or normalized values.
 *
 * @param x Input value
 * @return Value clamped between 0.0 and 1.0
 *
 * @note Equivalent to clamp(x, 0.0f, 1.0f).
 */
BYUL_API float float_clamp01(float x);

/**
 * @brief Smoothstep interpolation
 * @param edge0 Start boundary
 * @param edge1 End boundary
 * @param x Current value
 * @return Smoothly interpolated result (0~1)
 */
BYUL_API float float_smoothstep(float edge0, float edge1, float x);

#ifdef __cplusplus
}
#endif

#endif // FLOAT_COMMON_H
