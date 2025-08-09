#ifndef BODYPROPS_H
#define BODYPROPS_H

#include "byul_common.h"
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct bodyprops_t
 * @brief Physical properties of an object.
 *
 * Contains values for mass, friction, restitution, air resistance, and cross-sectional area.
 */
typedef struct s_bodyprops {
    float mass;             /**< Mass (kg), default 1.0 */
    float drag_coef;        /**< Air drag coefficient (Cd), requires object shape. */
    float cross_section;    /**< Cross-sectional area (m^2), used for drag calculation. */
    float restitution;      /**< Restitution coefficient (0: absorb, 1: full bounce). */
    float friction;         /**< Friction coefficient (0~1, 0: none, 1: maximum). */

    /**
     * @brief Coefficient for Magnus effect (spin-induced lift).
     *
     * This coefficient controls the strength of the Magnus force generated 
     * by the cross product of angular velocity and linear velocity.
     *
     * Applied formula (part of total acceleration):
     *     a_magnus = k_magnus * (omega x velocity)
     *
     * Where:
     * - omega: Angular velocity vector (rad/s)
     * - velocity: Linear velocity vector (m/s)
     *
     * Value Ranges:
     * - Reality-based:     0.05 ~ 0.3      (e.g., spinning baseball, artillery shell)
     * - Game-designed:     0.3 ~ 1.5       (emphasized curving for magical or exaggerated arcs)
     * - Maximum allowed:   <= 5.0           (above this may cause unstable RK4 behavior)
     *
     * Notes:
     * - Higher values result in stronger curvature of projectile trajectory.
     * - Affects accuracy and realism of spin-based motion prediction.
     * - Should be used carefully in RK4-based simulations to avoid divergence.
     */
    float k_magnus;

    /**
     * @brief Coefficient for gyroscopic drift (rotation-induced deviation).
     *
     * This coefficient controls the strength of drift caused by the cross product 
     * of angular acceleration and linear velocity over a short time window.
     *
     * Applied formula (part of total acceleration):
     *     a_gyro = k_gyro * time * (alpha x velocity)
     *
     * Where:
     * - alpha: Angular acceleration vector (rad/s^2)
     * - velocity: Linear velocity vector (m/s)
     * - time: Predictive window (typically 0.01 ~ 0.1 seconds)
     *
     * Value Ranges:
     * - Reality-based:     0.01 ~ 0.2      (e.g., gyroscopic drift on slow spin-down)
     * - Game-designed:     0.2 ~ 1.0       (for spin-guided effects)
     * - Maximum allowed:   <= 4.0           (above this may destabilize integration)
     *
     * Notes:
     * - Represents rotational inertia effects (especially on spin acceleration or deceleration).
     * - Enhances realism for drones, missiles, or magic projectiles that curve mid-flight.
     * - Used in combination with Magnus for total spin-induced effects.
     */
    float k_gyro;

} bodyprops_t;

// ---------------------------------------------------------
// bodyprops_t utilities
// ---------------------------------------------------------

/**
 * @brief Initialize a bodyprops_t structure with default physical values.
 *
 * This function sets the physical properties of an object to predefined
 * default values, suitable for generic spherical objects in simulation.
 *
 * Default values:
 * - mass          = 1.0 kg
 * - drag_coef     = 0.47 (sphere in air)
 * - cross_section = 0.01 m^2 (~= 10 cm x 10 cm)
 * - restitution   = 0.5 (semi-elastic collision)
 * - friction      = 0.1 (low sliding resistance)
 * - k_magnus      = 0.2 (moderate spin lift)
 * - k_gyro        = 0.05 (weak spin drift)
 *
 * @param[out] body Pointer to the structure to initialize.
 *                  If NULL, the function has no effect.
 */
BYUL_API void bodyprops_init(bodyprops_t* body);

/**
 * @brief Fully initialize a bodyprops_t structure with all physical properties.
 *
 * This function sets all available physical parameters of an object, including
 * linear properties (mass, drag, friction) and spin-related parameters
 * (Magnus and gyroscopic coefficients).
 *
 * This is the complete initializer for bodyprops_t.
 * Use this when full control over projectile or object behavior is needed.
 *
 * Parameters:
 * - mass          : Object mass [kg]
 * - drag_coef     : Air drag coefficient (Cd), dimensionless
 * - cross_section : Cross-sectional area [m^2]
 * - restitution   : Collision bounce factor [0.0 ~ 1.0]
 * - friction      : Sliding friction coefficient [0.0 ~ 1.0]
 * - k_magnus      : Coefficient for Magnus force from spin [0.0 ~ 5.0]
 * - k_gyro        : Coefficient for gyroscopic drift [0.0 ~ 4.0]
 *
 * @param[out] body         Pointer to structure to initialize.
 *                          If NULL, no action is taken.
 * @param[in]  mass         Mass of the object in kilograms.
 * @param[in]  drag_coef    Air resistance coefficient (typical: 0.1 ~ 1.0).
 * @param[in]  cross_section Area exposed to air drag in square meters.
 * @param[in]  restitution  Elasticity of collisions (0 = no bounce, 1 = perfect bounce).
 * @param[in]  friction     Surface friction factor (0 = frictionless, 1 = max resistance).
 * @param[in]  k_magnus     Spin lift coefficient (higher = more curve on spin).
 * @param[in]  k_gyro       Spin-induced drift coefficient (higher = more drift).
 */
BYUL_API void bodyprops_init_full(bodyprops_t* body,
                                  float mass,
                                  float drag_coef,
                                  float cross_section,
                                  float restitution,
                                  float friction,
                                  float k_magnus,
                                  float k_gyro);

/**
 * @brief Copy bodyprops_t values.
 *
 * @param out Destination structure.
 * @param src Source structure.
 */
BYUL_API void bodyprops_assign(
    bodyprops_t* out, const bodyprops_t* src);

/**
 * @brief Apply velocity reduction due to friction.
 *
 * - friction (0~1): damping factor (0 = no damping, 1 = strong friction).
 * - The reduction is linear with friction, and greater time increases the reduction.
 *
 * @param[in,out] velocity Velocity vector (m/s).
 * @param[in] body         Object properties (uses friction value).
 * @param[in] time           Time interval (seconds).
 */
BYUL_API void bodyprops_apply_friction(vec3_t* velocity,
                                  const bodyprops_t* body,
                                  float time);

/**
 * @brief Apply velocity reduction and calculate heat from friction.
 *
 * - friction (0~1): damping factor.
 * - The lost kinetic energy is calculated in Joules and returned as heat.
 *
 * @param[in,out] velocity Velocity vector (m/s).
 * @param[in] body         Object properties (uses mass and friction).
 * @param[in] time           Time interval (seconds).
 * @return Generated heat energy (Joules).
 */
BYUL_API float bodyprops_apply_friction_heat(vec3_t* velocity,
                                        const bodyprops_t* body,
                                        float time);

/**
 * @brief Estimate the time required for the object to stop due to friction.
 *
 * This function assumes linear damping:
 *     v(t) = v0 * (1 - u * t)
 *     stop when v(t) = 0
 *     -> t_stop = 1 / u
 *
 * If friction is 0 or initial velocity is near zero, returns 0.
 *
 * @param velocity  Current velocity vector (m/s).
 * @param body      Object's physical properties (uses friction).
 * @return Estimated time to stop [seconds], or 0.0 if not applicable.
 */
BYUL_API float bodyprops_estimate_stop_time(const vec3_t* velocity,
                                   const bodyprops_t* body);

#ifdef __cplusplus
}
#endif

#endif // BODYPROPS_H
