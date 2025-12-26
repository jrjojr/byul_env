#ifndef NUMEQ_SOLVER_H
#define NUMEQ_SOLVER_H

#include "byul_config.h"
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 1. Basic mathematical equation solvers
// ---------------------------------------------------------

/**
 * @brief Solve a linear equation ax + b = 0.
 *
 * @param a  Coefficient of x
 * @param b  Constant term
 * @param[out] out_x Solution (only if a != 0)
 *
 * @return true if solution exists, false if a == 0
 *
 * Example:
 * @code
 * float x;
 * if (numeq_solve_linear(2.0f, -4.0f, &x)) {
 *     // x = 2.0
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_linear(float a, float b, float* out_x);

/**
 * @brief Solve a quadratic equation a*x^2 + b*x + c = 0 (real roots only).
 *
 * Finds the real roots of the quadratic (or linear when |a| is negligible)
 * and returns them in ascending order. Degenerate and edge cases are handled:
 * - If |a| < eps, the equation is treated as linear b*x + c = 0.
 * - If the discriminant is negative, no real roots exist.
 * - If the discriminant is zero, both outputs receive the same root.
 *
 * @param a        Coefficient of x^2
 * @param b        Coefficient of x
 * @param c        Constant term
 * @param[out] out_x1  Smallest root (for linear case, out_x1 = out_x2)
 * @param[out] out_x2  Largest root
 *
 * @return true if at least one real root exists; false otherwise.
 *
 * @note
 * - Roots are ordered so that *out_x1 <= *out_x2.
 * - Linear fallback occurs when |a| < eps (implementation-defined epsilon).
 * - This non-stable variant may suffer from cancellation when |b| ~ sqrt(D).
 *   Prefer numeq_solve_quadratic_stable for high-accuracy timing (e.g., TOI).
 *
 * @warning
 * - Passing NULL pointers for outputs yields false.
 * - Behavior is undefined for NaN inputs.
 *
 * @see numeq_solve_quadratic_stable
 *
 * @code
 * float x1, x2;
 * if (numeq_solve_quadratic(1.0f, -3.0f, 2.0f, &x1, &x2)) {
 *     // x1 == 1.0f, x2 == 2.0f
 * }
 * // Linear example: 0*x^2 + 2*x - 8 = 0  => x = 4
 * if (numeq_solve_quadratic(0.0f, 2.0f, -8.0f, &x1, &x2)) {
 *     // x1 == x2 == 4.0f
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_quadratic(
    float a, float b, float c, float* out_x1, float* out_x2);

/**
 * @brief Solve A*t^2 + B*t + C = 0 using a numerically stable formulation.
 *
 * Uses the stable form:
 *   q  = -0.5 * (B + sign(B) * sqrt(B^2 - 4AC))
 *   t0 = q / A
 *   t1 = C / q
 * which greatly reduces cancellation when |B| ~ sqrt(discriminant).
 * The roots are returned in ascending order.
 *
 * @param A    Quadratic coefficient (must satisfy |A| >= eps)
 * @param B    Linear coefficient
 * @param C    Constant term
 * @param[out] t0  Smallest root
 * @param[out] t1  Largest root
 *
 * @return true if real roots exist and |A| >= eps; false otherwise.
 *
 * @note
 * - This function does not handle the linear case; callers should branch to a
 *   linear solver when |A| < eps (implementation-defined epsilon).
 * - Roots are ordered so that *t0 <= *t1.
 * - Recommended for collision timing and other precision-critical code paths.
 *
 * @warning
 * - Passing NULL pointers for outputs yields false.
 * - If the discriminant is slightly negative due to floating-point noise,
 *   consider clamping with a small epsilon before calling this function.
 *
 * @see numeq_solve_quadratic
 *
 * @code
 * float r0, r1;
 * if (numeq_solve_quadratic_stable(1.0f, -3.0f, 2.0f, &r0, &r1)) {
 *     // r0 == 1.0f, r1 == 2.0f (ordered)
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_quadratic_stable(
    float A, float B, float C, float* t0, float* t1);

/**
 * @brief Solve a cubic equation ax^3 + bx^2 + cx + d = 0 (real roots only).
 *
 * @param a   Coefficient of x^3 (a != 0)
 * @param b   Coefficient of x^2
 * @param c   Coefficient of x
 * @param d   Constant term
 * @param[out] out_roots  Array to store roots (up to 3 real roots in ascending order)
 * @param[out] out_count  Number of real roots found (1~3)
 *
 * @return true if real roots exist, false otherwise
 *
 * Example:
 * @code
 * float roots[3];
 * int count = 0;
 * if (numeq_solve_cubic(1.0f, -6.0f, 11.0f, -6.0f, roots, &count)) {
 *     // roots = {1.0, 2.0, 3.0}, count = 3
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_cubic(float a, float b, float c, float d, 
    float* out_roots, int* out_count);

typedef float (*numeq_func_f32)(float x, void* userdata);

/**
 * @brief Find a root of f(x) = 0 using the bisection method.
 *
 * @param func       Function pointer f(x)
 * @param userdata   Optional user data
 * @param a          Interval start
 * @param b          Interval end
 * @param tol        Tolerance
 * @param[out] out_root  Approximate root
 *
 * @return true if root found, false otherwise
 */
BYUL_API bool numeq_solve_bisection(numeq_func_f32 func,
                           void* userdata,
                           float a, float b,
                           float tol,
                           float* out_root);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_SOLVER_H
