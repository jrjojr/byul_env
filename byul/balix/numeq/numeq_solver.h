#ifndef NUMEQ_SOLVER_H
#define NUMEQ_SOLVER_H

#include "trajectory.h"

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
 * @brief Solve a quadratic equation ax^2 + bx + c = 0 (real roots only).
 *
 * @param a  Coefficient of x^2 (a != 0)
 * @param b  Coefficient of x
 * @param c  Constant term
 * @param[out] out_x1  First root (smallest)
 * @param[out] out_x2  Second root (largest)
 *
 * @return true if real roots exist, false if discriminant < 0
 *
 * @note If a == 0, it is treated as a linear equation bx + c = 0.
 *
 * Example:
 * @code
 * float x1, x2;
 * if (numeq_solve_quadratic(1.0f, -3.0f, 2.0f, &x1, &x2)) {
 *     // x1 = 1.0, x2 = 2.0
 * }
 * @endcode
 */
BYUL_API bool numeq_solve_quadratic(
    float a, float b, float c, float* out_x1, float* out_x2);

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

// ---------------------------------------------------------
// 2. Physics-based ballistic equation solvers
// ---------------------------------------------------------

/**
 * @brief Solve time t when y(t) = target_y.
 */
BYUL_API bool numeq_solve_time_for_y(const linear_state_t* state,
                            float target_y,
                            float* out_time);

/**
 * @brief Solve time t when |pos(t).xz - target.xz| < epsilon.
 */
BYUL_API bool numeq_solve_time_for_position(const linear_state_t* state,
                                   const vec3_t* target_pos,
                                   float tolerance,
                                   float max_time,
                                   float* out_time);

/**
 * @brief Solve velocity needed to reach a given horizontal distance.
 */
BYUL_API bool numeq_solve_velocity_for_range(float distance,
                                    float gravity,
                                    float* out_velocity);

/**
 * @brief Solve apex time and position (when vy(t) == 0).
 */
BYUL_API bool numeq_solve_apex(const linear_state_t* state,
                      vec3_t* out_apex_pos,
                      float* out_apex_time);

/**
 * @brief Solve time when projectile stops (velocity = 0).
 */
BYUL_API bool numeq_solve_stop_time(const linear_state_t* state,
                           float tolerance,
                           float* out_time);

// ---------------------------------------------------------
// 3. Vector-based root finding (future extension)
// ---------------------------------------------------------

/**
 * @brief Find t that minimizes |f(t) - target_vec|.
 */
typedef void (*numeq_vec3_func)(float t, vec3_t* out, void* userdata);
BYUL_API bool numeq_solve_time_for_vec3(numeq_vec3_func func,
                               void* userdata,
                               const vec3_t* target,
                               float t_min, float t_max,
                               float tol,
                               float* out_t);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_SOLVER_H
