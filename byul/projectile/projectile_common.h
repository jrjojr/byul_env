#ifndef PROJECTILE_COMMON_H
#define PROJECTILE_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_common.h"
#include "entity_dynamic.h"

// ---------------------------------------------------------
// Struct and Callback Type Declarations
// ---------------------------------------------------------
typedef struct s_projectile projectile_t;

/**
 * @brief Collision callback function type
 * @param proj      The projectile that collided
 * @param userdata  User-defined data
 */
typedef void (*projectile_hit_cb)(const projectile_t* proj, void* userdata);
// typedef void (*projectile_hit_cb)(const void* projectile, void* userdata);


/**
 * @enum projectile_attr_t
 * @brief Enumeration of physical attributes of a projectile (bit flags).
 *
 * This enumeration defines multiple attributes that a projectile can have
 * using a bit flag representation.  
 * A single projectile can possess multiple attributes simultaneously,
 * enabling diverse tactical use.
 *
 * ---
 * Attribute Descriptions
 * 
 * IMPACT  
 * Man-made projectile designed for strong impact.
 * Due to its heavy mass and solid material, it releases concentrated kinetic energy
 * upon impact to deal significant damage.
 *
 * Example projectiles: hammers, maces, heavy metal bullets.
 * 
 * PIERCE  
 * A fast and sharp projectile that penetrates targets without stopping.
 * Armed with speed and sharpness, it can easily pierce through armor or thick barriers.
 *
 * Example projectiles: bullets, armor-piercing rounds (AP), high-speed crossbow bolts.
 * 
 * ANCHOR  
 * A projectile that sticks into the target upon impact and holds in place.
 * After hitting, it restricts movement and provides additional tactical effects.
 *
 * Example projectiles: shuriken, javelin, throwing knives, poisoned needles.
 *
 * NONE (No attribute) (Pure natural force)  
 * A projectile used in its natural form without artificial design or processing.
 * Unlike man-made projectiles, its natural texture, form, and unpredictable motion
 * can confuse the enemy.
 *
 * Key characteristics and strengths:
 * - Quick and flexible availability: easily obtainable from the environment.
 * - Unpredictable effects due to irregular shape and mass distribution.
 * - Ignores artificial attribute resistances.
 * - Maximizes interaction with the battlefield environment (breaking obstacles, etc.).
 *
 * Example projectiles: stones, rock fragments, branches, natural debris, clumps of earth.
 * 
 * ---
 * ## Composite Attribute Explanation
 *
 * Projectiles can combine **IMPACT**, **PIERCE**, **ANCHOR** attributes.
 *
 * 1. IMPACT + PIERCE  
 *    - Characteristics: Both penetration and strong impact.
 *    - Example: slug rounds, heavy crossbow bolts, spiked maces.
 *
 * 2. IMPACT + ANCHOR  
 *    - Characteristics: Strong impact with anchoring.
 *    - Example: throwing axes, large shuriken, metal spikes.
 *
 * 3. PIERCE + ANCHOR  
 *    - Characteristics: Penetrates and sticks into the target.
 *    - Example: arrows (at medium speed), throwing knives, steel spears.
 *
 * 4. IMPACT + PIERCE + ANCHOR  
 *    - Characteristics: Has all three attributes.
 *    - Example: enhanced javelins, spiked hammers, drill-like projectiles.
 *
 * ---
 * ## Attribute Interactions
 *
 * - IMPACT -> ANCHOR  
 *   Heavy impact projectiles can knock off or destroy anchored ones.
 *
 * - ANCHOR -> PIERCE  
 *   Anchored projectiles can interfere with or stop piercing ones.
 *
 * - PIERCE -> IMPACT  
 *   Piercing projectiles can easily penetrate impact projectiles.
 *
 * ---
 * @note Arrows may have both PIERCE and ANCHOR depending on speed or conditions.
 */
typedef enum {
    PROJECTILE_ATTR_NONE   = 0,       ///< No attribute: natural projectile (e.g., stone)
    PROJECTILE_ATTR_IMPACT = 1 << 0,  ///< Impact: delivers strong force upon hitting
    PROJECTILE_ATTR_PIERCE = 1 << 1,  ///< Pierce: sharp projectile that penetrates targets
    PROJECTILE_ATTR_ANCHOR = 1 << 2   ///< Anchor: sticks into the target on hit
} projectile_attr_t;

/**
 * @struct s_projectile
 * @brief Structure defining **common properties of all projectiles** (e.g., shells, missiles).
 *
 * Projectiles are classified into **Impact**, **Pierce**, and **Anchor** types.
 *
 * @note The radius is used as the **hit detection range**, not the explosion radius.
 */
struct s_projectile {
    /**
     * @brief Base dynamic entity.
     * @details Includes position, velocity, rotation, and other physical data.
     */
    entity_dynamic_t base;

    /**
     * @brief Base damage. Default is 1.0f.
     * @details Represents the basic damage value upon hitting a target.
     */
    float damage;

    /**
     * @brief Projectile attribute flags.
     * @details Combines PROJECTILE_ATTR_* macros (IMPACT / PIERCE / ANCHOR).
     */
    projectile_attr_t attrs;

    /**
     * @brief Collision callback function.
     * @details Invoked when the projectile hits a target or obstacle.
     * If NULL, no action is performed on collision.
     */
    projectile_hit_cb on_hit;

    /**
     * @brief User data for collision callback.
     * @details Passed as an argument when on_hit is called.
     */
    void* hit_userdata;
};

// ---------------------------------------------------------
// Initialization Functions
// ---------------------------------------------------------

/**
 * @brief Initializes a projectile_t structure with **default values**.
 *
 * This function safely initializes a `projectile_t` instance.  
 * Internally, it calls `entity_dynamic_init()` for the `base` field,  
 * and sets the **attrs** field to `PROJECTILE_ATTR_NONE`.
 *
 * **Default behavior:**
 * - `proj->base` : initialized via entity_dynamic_init()
 * - `proj->damage` : 1.0f
 * - `proj->attrs` : PROJECTILE_ATTR_NONE
 * - `proj->on_hit` : NULL
 * - `proj->hit_userdata` : NULL
 *
 * @param[out] proj Pointer to projectile_t to initialize (does nothing if NULL).
 *
 * **Example:**
 * @code
 * projectile_t arrow;
 * projectile_init(&arrow);
 * arrow.attrs = PROJECTILE_ATTR_PIERCE | PROJECTILE_ATTR_ANCHOR; // arrow attributes
 * arrow.damage = 25.0f;
 * @endcode
 */
BYUL_API void projectile_init(projectile_t* proj);

/**
 * @brief Fully initializes a projectile_t with **user-defined values**.
 *
 * Sets the projectile's **attrs**, `base`, `damage`,  
 * `on_hit`, and `hit_userdata` based on the provided parameters.  
 * If `base` is NULL, `entity_dynamic_init()` is called to set defaults.
 *
 * @param[out] proj         Pointer to the projectile to initialize (ignored if NULL)
 * @param[in]  base         Dynamic entity data (if NULL, default is used)
 * @param[in]  attrs        Projectile attributes (IMPACT | PIERCE | ANCHOR combination)
 * @param[in]  damage       Base damage of the projectile
 * @param[in]  on_hit       Callback function to call on collision (nullable)
 * @param[in]  hit_userdata User data to pass to the callback (nullable)
 *
 * **Example:**
 * @code
 * entity_dynamic_t dyn;
 * entity_dynamic_init(&dyn);
 *
 * projectile_t spear;
 * projectile_init_full(&spear, &dyn,
 *     PROJECTILE_ATTR_IMPACT | PROJECTILE_ATTR_ANCHOR, // attributes
 *     50.0f,                                           // damage
 *     on_spear_hit, user_data);
 * @endcode
 */
BYUL_API void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_attr_t attrs,
    float damage,
    projectile_hit_cb on_hit,
    void* hit_userdata
);

/**
 * @brief Copies a projectile_t from another.
 * @param[out] out Destination
 * @param[in]  src Source
 */
BYUL_API void projectile_assign(projectile_t* out, const projectile_t* src);

// ---------------------------------------------------------
// Update and Behavior Functions
// ---------------------------------------------------------
/**
 * @brief Updates the state of a projectile
 *
 * - position = position + velocity * dt
 * - rotation = angular_velocity * dt applied
 * - Checks lifetime and calls on_hit callback if expired
 *
 * @param proj Projectile
 * @param dt   Time interval (seconds)
 */
BYUL_API void projectile_update(projectile_t* proj, float dt);

// ---------------------------------------------------------
// Default Collision Callback
// ---------------------------------------------------------
/**
 * @brief Default collision callback
 * Prints the damage on collision.
 */
BYUL_API void projectile_default_hit_cb(
    const projectile_t* projectile, void* userdata);

/**
 * @brief Default expiration callback
 *
 * Called when a projectile reaches the end of its lifetime without a collision.
 * Typical behavior: trigger detonation, spawn VFX/SFX, apply radial damage,
 * and mark the projectile as finished.
 *
 * Notes:
 * - Should be invoked exactly once per projectile lifetime.
 * - Must not fire if a hit already consumed the projectile (guard with state).
 *
 * @param projectile  Read-only pointer to the projectile data.
 * @param userdata    Opaque user data passed at registration time.
 */
BYUL_API void projectile_default_expire_cb(
    const projectile_t* projectile, void* userdata);

/**
 * @struct launch_param_t
 * @brief Defines initial parameters required to launch a projectile toward a target.
 *
 * - **direction**: Launch direction toward the target (unit vector).
 * - **force**: Initial force applied at launch (Newton, N).
 *      - 1 N = 1 kg * 1 m/s^2.
 *      - Example recommended values:
 *        * 1 kg projectile -> 10 ~ 100 N (10 ~ 30 m/s initial velocity).
 *        * 10 kg projectile -> 500 ~ 5000 N (20 ~ 100 m/s initial velocity).
 * - **time_to_hit**: Estimated time to reach the target (seconds).
 *
 * @note
 * - direction is always normalized.
 * - force is converted into the actual initial velocity vector
 *   based on mass and projectile properties.
 */
typedef struct s_launch_param {
    vec3_t direction;    ///< Launch direction (unit vector)
    float  force;        ///< Initial launch force (Newton, N)
    float  time_to_hit;  ///< Estimated time to hit target (seconds)
} launch_param_t;


/**
 * @brief Calculates launch parameters for a projectile to reach the given target position.
 *
 * This function ignores **environmental factors (gravity, wind, etc.)**  
 * and only considers the projectile's **own properties (mass, friction, etc.)**.
 * 
 * - After launch, velocity decreases gradually due to friction but does not drop
 *   because of gravity.
 * - Once velocity reaches 0, the projectile stops at its location.
 * - Calculates initial force (`force`), direction (`direction`),
 *   and expected time to hit (`time_to_hit`).
 *
 * @param[out] out           Calculation result (direction, initial force, time_to_hit).
 * @param[in]  proj          Projectile info (start position, mass, friction, etc.).
 * @param[in]  target        Target position (world coordinates).
 * @param[in]  initial_force_scalar Initial force at launch (Newton, N).
 * @return `true` if calculation succeeded, `false` if target cannot be reached with given force.
 */
BYUL_API bool projectile_calc_launch_param(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force_scalar
);

/**
 * @brief Calculates launch parameters to reach a target while considering environment factors.
 *
 * Accounts for gravity, wind, air drag, and other environmental data (`environ_t`)
 * to calculate initial launch direction (`direction`), force (`force`),
 * and time-to-hit (`time_to_hit`).
 *
 * @param[out] out           Calculation result (direction, force, time_to_hit).
 * @param[in]  proj          Projectile info (start position, mass, etc.).
 * @param[in]  env           Environment info (gravity, wind, drag, etc.).
 * @param[in]  target        Target position (world coordinates).
 * @param[in]  initial_force_scalar Initial force at launch (Newton, N).
 * @return `true` if calculation succeeded, `false` if target cannot be reached under given conditions.
 */
BYUL_API bool projectile_calc_launch_param_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force_scalar
);

/**
 * @brief Calculates required launch parameters (inverse calculation) for a given target and hit time.
 *
 * This function ignores **environmental factors (gravity, wind, etc.)**,  
 * and calculates the initial launch direction (`direction`) and required initial force (`force`)
 * based on the **projectile's start position, target position, and given hit_time**.
 *
 * - Projectile velocity decreases only due to friction, without gravity effects.
 * - force is calculated so the projectile reaches the target exactly at hit_time.
 * - time_to_hit is set to the input hit_time.
 *
 * @param[out] out      Calculation result (direction, force, time_to_hit).
 * @param[in]  proj     Projectile info (start position, mass, friction, etc.).
 * @param[in]  target   Target position (world coordinates).
 * @param[in]  hit_time Desired time to hit the target (seconds, must be > 0).
 * @return `true` if calculation succeeded, `false` if the target cannot be reached within given time.
 */
BYUL_API bool projectile_calc_launch_param_inverse(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float hit_time
);

/**
 * @brief Calculates required launch parameters (inverse calculation) considering environment factors.
 *
 * Accounts for gravity, wind, and air drag (`environ_t`) to calculate
 * the initial launch direction (`direction`) and force (`force`)
 * to reach the target at a given hit_time.
 *
 * @param[out] out      Calculation result (direction, force, time_to_hit).
 * @param[in]  proj     Projectile info (start position, mass, etc.).
 * @param[in]  env      Environment info (gravity, wind, drag, etc.).
 * @param[in]  target   Target position (world coordinates).
 * @param[in]  hit_time Desired time to hit the target (seconds).
 * @return `true` if calculation succeeded, `false` if target cannot be reached under given conditions.
 */
BYUL_API bool projectile_calc_launch_param_inverse_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,    
    const vec3_t* target,
    float hit_time
);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_COMMON_H
