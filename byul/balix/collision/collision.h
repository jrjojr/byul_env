/* precise policy: 1D exact-time first, 
* else segment TOI + single Newton correction (O(1), no loops)
*/
#ifndef COLLISION_H
#define COLLISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "vec3.h"

#ifndef BYUL_TOI_COLINEAR_COS
#define BYUL_TOI_COLINEAR_COS 0.999f   // cos(theta) threshold for near colinearity
#endif

#ifndef BYUL_TOI_CURVATURE_THRESH
#define BYUL_TOI_CURVATURE_THRESH 0.25f // higher => fewer Newton corrections
#endif

/**
 * @brief Detect precise intersection between a moving point 
 * and a plane within a simulation tick.
 *
 * This function determines if a projectile or moving entity, 
 * traveling from
 * @p pos_prev to @p pos_curr over a time step @p dt, 
 * intersects with a plane defined
 * by @p plane_point and @p plane_normal. It uses a two-stage approach:
 *
 * 1. **Ray-plane intersection** 
 * on the segment direction as the primary method.
 * 2. **Fallback quadratic/linear solver** 
 * when the motion is affected by acceleration.
 *
 * The function returns the exact impact position and time, 
 * interpolated within the tick,
 * and snaps the final impact point onto the plane to remove numerical drift.
 *
 * @param[in]  pos_prev      Position at the start of the tick (t = t_prev).
 * @param[in]  pos_curr      Position at the end of the tick (t = t_prev + dt).
 * @param[in]  vel_prev      Velocity vector at the start of the tick.
 * @param[in]  accel         Constant acceleration vector during the tick.
 * @param[in]  plane_point   Any point lying on the plane.
 * @param[in]  plane_normal  Normal vector of the plane 
 * (does not need to be normalized; will be normalized internally).
 * @param[in]  t_prev        Absolute simulation time at the start of the tick.
 * @param[in]  dt            Simulation time step duration.
 * @param[out] impact_pos    World-space position of the intersection point 
 * (snapped to the plane).
 * @param[out] impact_time   Absolute simulation time 
 * when the intersection occurs.
 *
 * @return true if the segment between @p pos_prev and @p pos_curr intersects 
 * the plane within the tick, false otherwise.
 *
 * @note
 * - The primary ray-plane test is used if the intersection occurs 
 * along the segment direction.
 * - If the ray-plane intersection lies outside the segment, 
 * a fallback quadratic or linear
 *   equation solver is used to find the exact time `t` 
 * when the moving point crosses the plane.
 * - The fallback handles both linear motion (`a == 0`) 
 * and accelerated motion.
 * - A small epsilon is used to reject degenerate cases 
 * (e.g., zero-length motion or zero-length normal).
 *
 * @warning
 * - The computed @p impact_time will always lie 
 * in the range `[t_prev, t_prev + dt]` when returning true.
 * - If @p accel is non-zero, 
 * ensure it reflects constant acceleration during the tick; 
 * variable acceleration
 *   may require sub-stepping for accurate collision timing.
 */
BYUL_API bool detect_plane_collision(
    const vec3_t* pos_prev,
    const vec3_t* pos_curr,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    const vec3_t* plane_point,
    const vec3_t* plane_normal,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time);

/**
 * @brief Time-of-impact (TOI) with a static sphere using a segment-based solver.
 *
 * Solves a closed-form quadratic on the segment between
 *   P0 = p0 at t_prev
 *   P1 = p0 + v0*dt + 0.5*a*dt^2
 * to find s in [0,1] such that |(P0 - C) + s*(P1 - P0)|^2 = R^2.
 * The hit time is t_hit = s * dt (no iteration). The impact position is then
 * evaluated on the kinematic model r(t) = p0 + v0*t + 0.5*a*t^2 at t = t_hit,
 * and snapped onto the sphere to eliminate tiny penetration.
 *
 * @param[in]  pos_prev      Position at the start of the tick (t_prev).
 * @param[in]  vel_prev      Velocity at the start of the tick.
 * @param[in]  accel         Constant acceleration over [t_prev, t_prev + dt].
 * @param[in]  target_pos    Sphere center in world space.
 * @param[in]  radius        Sphere radius (>= 0).
 * @param[in]  t_prev        Absolute time at the start of the tick.
 * @param[in]  dt            Tick duration (> 0).
 * @param[out] impact_pos    World-space impact point (snapped to the sphere).
 * @param[out] impact_time   Absolute time of impact (t_prev + s*dt).
 *
 * @return true if an intersection occurs within the tick; false otherwise.
 *
 * @pre  All pointers are non-null, dt > 0, radius >= 0.
 * @post On success, *impact_pos lies on the sphere surface and *impact_time is set.
 *
 * @details
 * - Loop-free, O(1) time; stable quadratic solver recommended.
 * - Time t_hit derives from segment parameter s (not the exact kinematic root
 *   when acceleration is large), which is typically sufficient for CCD and VFX.
 * - Acceleration is assumed constant during the tick.
 *
 * @warning
 * - Degenerate segments (very small |P1 - P0|) return false.
 * - For very large curvature (|a|*dt) the time may be biased relative to
 *   the exact kinematic root. Use detect_sphere_collision_precise for higher fidelity.
 *
 * @see numeq_solve_quadratic_stable, vec3_project
 *
 * @code
 * vec3_t hit; float thit;
 * if (detect_sphere_collision(&p0,&v0,&a,&center,R,t0,dt,&hit,&thit)) {
 *     // use hit and thit
 * }
 * @endcode
 */
BYUL_API bool detect_sphere_collision(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    const vec3_t* target_pos,
    float radius,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time);

/**
 * @brief High-fidelity TOI with a static sphere: 1D exact time + single Newton policy.
 *
 * Policy (loop-free, O(1)):
 * 1) 1D exact time: If the relative state {u0 = p0 - C, v0, a} is nearly colinear
 *    (cosine test), project onto the principal axis and solve
 *      x(t) = x0 + v*t + 0.5*a*t^2 = +/- R
 *    exactly (quadratic) to get the earliest t_hit in [0, dt].
 * 2) Otherwise, compute a baseline segment TOI (as in detect_sphere_collision)
 *    yielding t0 = s*dt, and if curvature metric
 *      k = |a_perp|*dt / |v_mid|
 *    exceeds a threshold, apply a single Newton correction:
 *      t1 = t0 - f(t0)/f'(t0), clamped to [0, dt],
 *    where f(t) = |u0 + v0 t + 0.5 a t^2|^2 - R^2.
 * 3) Evaluate r(t_hit) and snap to the sphere surface.
 *
 * @param[in]  pos_prev      Position at the start of the tick (t_prev).
 * @param[in]  vel_prev      Velocity at the start of the tick.
 * @param[in]  accel         Constant acceleration over [t_prev, t_prev + dt].
 * @param[in]  target_pos    Sphere center in world space.
 * @param[in]  target_radius Sphere radius (>= 0).
 * @param[in]  t_prev        Absolute time at the start of the tick.
 * @param[in]  dt            Tick duration (> 0).
 * @param[out] impact_pos    World-space impact point (snapped to the sphere).
 * @param[out] impact_time   Absolute time of impact.
 *
 * @return true if an intersection occurs within the tick; false otherwise.
 *
 * @pre  All pointers are non-null; dt > 0; target_radius >= 0.
 * @post On success, *impact_pos lies on the sphere surface and *impact_time is set.
 *
 * @details
 * - Still O(1), loop-free: the Newton step is a single fixed update (max 1).
 * - 1D branch returns the true kinematic root t in [0, dt].
 * - Non-1D branch reduces time bias for high curvature via one Newton step.
 * - Deterministic for identical inputs and thresholds.
 *
 * @warning
 * - Colinearity and curvature thresholds control branch selection; choose
 *   conservative defaults and tune via profiling.
 * - Degenerate motion (|P1 - P0| ~ 0) returns false.
 *
 * @note
 * - For moving spheres, use the corresponding ..._moving routine that applies
 *   the same policy to relative motion {u0, vrel, arel}.
 *
 * @see detect_sphere_collision, numeq_solve_quadratic_stable, vec3_project
 *
 * @code
 * vec3_t hit; float thit;
 * if (detect_sphere_collision_precise(&p0,&v0,&a,&center,R,t0,dt,&hit,&thit)) {
 *     // higher-fidelity TOI
 * }
 * @endcode
 */
BYUL_API bool detect_sphere_collision_precise(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    const vec3_t* target_pos,
    float target_radius,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time);

/**
 * @brief Closed-form TOI between an accelerating projectile 
 * and a moving (optionally accelerating) sphere center.
 *
 * Loop-free segment formulation:
 *   Projectile: P0 = pos_prev
 *               P1 = pos_prev + vel_prev*dt + 0.5*accel*dt^2
 *   Target:     C0 = target_pos
 *               C1 = target_pos + target_vel*dt + 0.5*target_accel*dt^2
 * Let u0 = P0 - C0 and d = (P1 - P0) - (C1 - C0). Solve for s in [0,1]:
 *   | u0 + s*d |^2 = R^2
 * Then t_hit = s * dt. The impact point is evaluated on each kinematic model
 * at t = t_hit and snapped to the sphere surface to eliminate tiny penetration.
 *
 * Assumptions:
 * - Both accelerations are constant over [t_prev, t_prev + dt].
 * - No iteration; O(1) time with a stable quadratic solver.
 *
 * @param[in]  pos_prev       Projectile position at the start of the tick (t_prev).
 * @param[in]  vel_prev       Projectile velocity at the start of the tick.
 * @param[in]  accel          Projectile acceleration (constant during dt).
 * @param[in]  target_pos     Sphere center at the start of the tick.
 * @param[in]  target_vel     Sphere center velocity.
 * @param[in]  target_accel   Sphere center acceleration (may be NULL => zero).
 * @param[in]  target_radius  Sphere radius (>= 0).
 * @param[in]  t_prev         Absolute time at the start of the tick.
 * @param[in]  dt             Tick duration (> 0).
 * @param[out] impact_pos     Impact point in world space (snapped to the sphere).
 * @param[out] impact_time    Absolute impact time (t_prev + s*dt).
 *
 * @return true if an intersection occurs within the tick; false otherwise.
 *
 * @pre  All pointer parameters (except target_accel) 
 * are non-null; dt > 0; target_radius >= 0.
 * @post On success, *impact_pos lies on the sphere surface 
 * and *impact_time is set.
 *
 * @details
 * - Time t_hit derives from the segment parameter s and may differ from the exact
 *   kinematic root when curvature is high. For higher fidelity, use
 *   detect_sphere_collision_moving_precise().
 * - Deterministic for identical inputs.
 *
 * @warning
 * - Degenerate relative segment (very small |d|) returns false.
 * - Numerical robustness depends on the quadratic solver for small discriminants.
 *
 * @see numeq_solve_quadratic_stable, vec3_project,
 *      detect_sphere_collision (static target),
 *      detect_sphere_collision_moving_precise (higher fidelity)
 *
 * @code
 * vec3_t hit; float thit;
 * if (detect_sphere_collision_moving(&p0,&v0,&a,&c0,&vc,&ac,R,t0,dt,&hit,&thit)) {
 *     // use hit and thit
 * }
 * @endcode
 */
BYUL_API bool detect_sphere_collision_moving(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    const vec3_t* target_pos,
    const vec3_t* target_vel,
    const vec3_t* target_accel,
    float target_radius,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time);

/**
 * @brief High-fidelity TOI for a moving sphere: 
 * 1D exact time + single Newton policy (loop-free).
 *
 * Policy on relative motion r(t) = u0 + v_rel t + 0.5 a_rel t^2:
 * 1) 1D exact-time branch:
 *    If {u0 = p0 - c0, v_rel = v_p - v_c, a_rel = a_p - a_c} are nearly colinear
 *    (cosine test), project onto the principal axis and solve
 *      x(t) = x0 + v*t + 0.5*a*t^2 = +/- R
 *    exactly (quadratic) to get the earliest t_hit in [0, dt].
 * 2) Otherwise:
 *    Compute baseline segment TOI on the relative segment to get t0 = s*dt.
 *    If curvature metric
 *      k = |a_rel_perp|*dt / |v_rel_mid|
 *    exceeds a threshold, apply a single Newton correction:
 *      t1 = t0 - f(t0)/f'(t0), clamped to [0, dt],
 *    where f(t) = |u0 + v_rel t + 0.5 a_rel t^2|^2 - R^2.
 * 3) Evaluate projectile and target at t_hit and snap to the sphere surface.
 *
 * Complexity and determinism:
 * - O(1), no loops (Newton is a single fixed update).
 * - Deterministic for identical inputs and thresholds.
 *
 * Thresholds (tunable compile-time constants):
 * - BYUL_TOI_COLINEAR_COS (default 0.999f): colinearity cosine threshold.
 * - BYUL_TOI_CURVATURE_THRESH (default 0.25f): curvature threshold for Newton.
 *
 * @param[in]  pos_prev       Projectile position at the start of the tick (t_prev).
 * @param[in]  vel_prev       Projectile velocity at the start of the tick.
 * @param[in]  accel          Projectile acceleration (constant during dt).
 * @param[in]  target_pos     Sphere center at the start of the tick.
 * @param[in]  target_vel     Sphere center velocity.
 * @param[in]  target_accel   Sphere center acceleration (may be NULL => zero).
 * @param[in]  target_radius  Sphere radius (>= 0).
 * @param[in]  t_prev         Absolute time at the start of the tick.
 * @param[in]  dt             Tick duration (> 0).
 * @param[out] impact_pos     Impact point in world space (snapped to the sphere).
 * @param[out] impact_time    Absolute impact time.
 *
 * @return true if an intersection occurs within the tick; false otherwise.
 *
 * @pre  All pointer parameters (except target_accel) 
 * are non-null; dt > 0; target_radius >= 0.
 * @post On success, *impact_pos lies on the sphere surface 
 * and *impact_time is set.
 *
 * @details
 * - 1D branch returns the true kinematic root in [0, dt].
 * - Non-1D branch reduces time bias via one Newton step only 
 * when curvature is large;
 *   otherwise it matches the baseline segment solver for maximum performance.
 *
 * @warning
 * - Degenerate relative segment (very small |d|) returns false.
 * - Choose conservative thresholds first; tune later via profiling.
 *
 * @see detect_sphere_collision_moving, detect_sphere_collision_precise,
 *      numeq_solve_quadratic_stable, vec3_project
 *
 * @code
 * vec3_t hit; float thit;
 * if (detect_sphere_collision_moving_precise(
 *  &p0,&v0,&a,&c0,&vc,&ac,R,t0,dt,&hit,&thit)) {
 *     // higher-fidelity TOI under relative motion
 * }
 * @endcode
 */
BYUL_API bool detect_sphere_collision_moving_precise(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    const vec3_t* target_pos,
    const vec3_t* target_vel,
    const vec3_t* target_accel,
    float target_radius,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time);

/**
 * Projectile vs moving triangle (both translating, constant acceleration), 
 * loop-free TOI.
 * Plane normal is fixed; triangle not rotate
 */
BYUL_API bool detect_triangle_collision_moving(
    // projectile state at t_prev
    const vec3_t* P0, const vec3_t* Vp, const vec3_t* Ap,
    // triangle vertices at t_prev (A0,B0,C0)
    const vec3_t* A0, const vec3_t* B0, const vec3_t* C0,
    // triangle translation kinematics over the tick
    const vec3_t* Vt, const vec3_t* At,   // At can be NULL => zero
    float t_prev, float dt,
    vec3_t* impact_pos, float* impact_time);  
    
/**
 * Ultra-TOI against a rotating triangle 
 * (translation + constant angular velocity), loop-free.
 *
 * Time of impact (TOI) is solved on a fixed plane normal 
 * from t_prev (small-rotation assumption),
 * then at that single time t_hit we evaluate:
 *   - projectile position via r_p(t) = P0 + Vp t + 0.5 Ap t^2
 *   - triangle vertices rotated about 'tri_center' 
 * by angle = |omega| * t (Rodrigues)
 *     and translated by T(t) = Vt t + 0.5 At t^2
 * and run a barycentric inside test. 
 * If the earliest root fails inside test, the second root
 * (if any) is tried once. No loops, O(1) work.
 *
 * This targets sub-tick accuracy 
 * for high angular velocity without substepping.
 */
BYUL_API bool detect_triangle_collision_rotating(
    // projectile state at t_prev
    const vec3_t* P0, const vec3_t* Vp, const vec3_t* Ap,
    // triangle vertices at t_prev (world)
    const vec3_t* A0, const vec3_t* B0, const vec3_t* C0,
    // triangle kinematics (translation) over the tick
    const vec3_t* Vt,           // triangle linear velocity (may be NULL => zero)
    const vec3_t* At,           // triangle linear acceleration (may be NULL => zero)
    // rotation about 'tri_center' with constant angular velocity 'omega' (world)
    const vec3_t* tri_center,   // rotation center in world
    const vec3_t* omega,        // angular velocity vector (rad/s), constant over dt
    // time window
    float t_prev, float dt,
    // outputs
    vec3_t* impact_pos, float* impact_time);

/**
 * Precise sub-tick TOI against a rotating triangle with angular acceleration (loop-free).
 *
 * Assumptions:
 * - Triangle undergoes translation (Vt, At) and rotation about a fixed world axis k.
 * - Angular velocity is omega(t) = omega0 + alpha * t (vector form), but axis is fixed.
 * - Theta(t) = dot(omega0, k) * t + 0.5 * dot(alpha, k) * t^2.
 * - Time of impact is solved on the plane normal at t_prev via a quadratic; at each candidate
 *   time the projectile uses r_p(t) and the triangle is evaluated with Rodrigues + translation,
 *   then barycentric inside test is run. Two roots max, no loops.
 */
BYUL_API bool detect_triangle_collision_rotating_alpha(
    // projectile at t_prev
    const vec3_t* P0, const vec3_t* Vp, const vec3_t* Ap,
    // triangle vertices at t_prev (world)
    const vec3_t* A0, const vec3_t* B0, const vec3_t* C0,
    // triangle translation kinematics
    const vec3_t* Vt,     // may be NULL => zero
    const vec3_t* At,     // may be NULL => zero
    // rotation about fixed axis 'k_axis' (unit), with omega0, alpha (world vectors)
    const vec3_t* tri_center,
    const vec3_t* k_axis_unit, // must be unit-length
    const vec3_t* omega0,      // angular velocity at t_prev (rad/s)
    const vec3_t* alpha,       // angular acceleration (rad/s^2)
    // time window
    float t_prev, float dt,
    // outputs
    vec3_t* impact_pos, float* impact_time);

#ifdef __cplusplus
}
#endif

#endif // COLLISION_H
