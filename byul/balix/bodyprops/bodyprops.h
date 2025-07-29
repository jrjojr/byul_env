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
} bodyprops_t;

// ---------------------------------------------------------
// bodyprops_t utilities
// ---------------------------------------------------------

/**
 * @brief Initialize bodyprops_t with default values.
 *
 * Default values:
 * - mass = 1.0f
 * - drag_coef = 0.47f (sphere reference)
 * - cross_section = 0.01f (10 cm^2)
 * - restitution = 0.5f
 * - friction = 0.1f
 *
 * @param body Pointer to bodyprops_t to initialize (ignored if NULL).
 */
BYUL_API void bodyprops_init(bodyprops_t* body);

/**
 * @brief Initialize bodyprops_t with specified values.
 *
 * @param body          Pointer to the structure to initialize (ignored if NULL).
 * @param mass          Mass [kg].
 * @param drag_coef     Air drag coefficient.
 * @param cross_section Cross-sectional area [m^2].
 * @param restitution   Restitution coefficient.
 * @param friction      Friction coefficient.
 */
BYUL_API void bodyprops_init_full(bodyprops_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction);

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
 * - The reduction is linear with friction, and greater dt increases the reduction.
 *
 * @param[in,out] velocity Velocity vector (m/s).
 * @param[in] body         Object properties (uses friction value).
 * @param[in] dt           Time interval (seconds).
 */
BYUL_API void bodyprops_apply_friction(vec3_t* velocity,
                                  const bodyprops_t* body,
                                  float dt);

/**
 * @brief Apply velocity reduction and calculate heat from friction.
 *
 * - friction (0~1): damping factor.
 * - The lost kinetic energy is calculated in Joules and returned as heat.
 *
 * @param[in,out] velocity Velocity vector (m/s).
 * @param[in] body         Object properties (uses mass and friction).
 * @param[in] dt           Time interval (seconds).
 * @return Generated heat energy (Joules).
 */
BYUL_API float bodyprops_apply_friction_heat(vec3_t* velocity,
                                        const bodyprops_t* body,
                                        float dt);

#ifdef __cplusplus
}
#endif

#endif // BODYPROPS_H
