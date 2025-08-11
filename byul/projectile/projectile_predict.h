#ifndef PROJECTILE_PREDICT_H
#define PROJECTILE_PREDICT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "projectile_common.h"
#include "trajectory.h"
#include "propulsion.h"
#include "guidance.h"
#include "environ.h"
#include "ground.h"
#include "entity_dynamic.h"
#include "numeq_filters.h"

/**
 * @struct projectile_result_t
 * @brief Structure for projectile trajectory prediction results
 *
 * This structure contains the projectile's start/target information,
 * trajectory data, predicted impact time, and impact position.
 */
typedef struct s_projectile_result {
    // Input metadata
    vec3_t start_pos;         /**< Projectile start position (world coordinates) */
    vec3_t target_pos;        /**< Target position (world coordinates, vector) */
    vec3_t initial_velocity;  /**< Initial velocity vector (m/s) */

    // Result data
    float impact_time;        /**< Predicted impact time (seconds) */
    vec3_t impact_pos;        /**< Predicted impact position (world coordinates) */
    bool bool_impacted;               /**< Whether the prediction is bool_impacted (true if impact occurs) */

    trajectory_t* trajectory; /**< Predicted trajectory data (dynamically allocated) */
} projectile_result_t;

// ---------------------------------------------------------
// projectile_result_t Management Functions
// ---------------------------------------------------------

/**
 * @brief Creates a default projectile_result_t object.
 *
 * Internally creates a trajectory with a default capacity (100)
 * and initializes impact_time, impact_pos, and bool_impacted values.
 *
 * @return Pointer to the created projectile_result_t (dynamically allocated).
 * @note Must be freed with projectile_result_destroy() after use.
 */
BYUL_API projectile_result_t* projectile_result_create();

/**
 * @brief Creates a projectile_result_t object with a specified trajectory capacity.
 *
 * @param capacity Maximum number of samples allocated to trajectory.
 * @return Pointer to the created projectile_result_t (dynamically allocated).
 * @note Must be freed with projectile_result_destroy() after use.
 */
BYUL_API projectile_result_t* projectile_result_create_full(int capacity);

/**
 * @brief Creates a deep copy of an existing projectile_result_t.
 *
 * @param src Source projectile_result_t (must not be NULL).
 * @return Pointer to the cloned projectile_result_t (dynamically allocated).
 * @note Must be freed with projectile_result_destroy() after use.
 */
BYUL_API projectile_result_t* projectile_result_copy(
    const projectile_result_t* src);

/**
 * @brief Resets a projectile_result_t object to its initial state.
 *
 * - Resets impact_time, impact_pos, and bool_impacted values.
 * - Keeps the trajectory but clears its internal data.
 *
 * @param res Pointer to the projectile_result_t to reset (may be NULL).
 */
BYUL_API void projectile_result_reset(projectile_result_t* res);

/**
 * @brief Frees and reallocates trajectory memory with a new capacity.
 *
 * @param res         Pointer to the projectile_result_t (must not be NULL).
 * @param new_capacity New maximum number of samples to allocate.
 */
BYUL_API void projectile_result_resize(projectile_result_t* res, int new_capacity);

/**
 * @brief Fully frees a projectile_result_t object and resets it to NULL.
 *
 * - Similar to reset but also frees trajectory.
 *
 * @param res Pointer to the projectile_result_t to free (may be NULL).
 */
BYUL_API void projectile_result_free(projectile_result_t* res);

/**
 * @brief Frees a projectile_result_t object and its internal trajectory.
 *
 * @param res Pointer to the projectile_result_t to free (may be NULL).
 * @note Internally calls trajectory_destroy() and frees res itself.
 */
BYUL_API void projectile_result_destroy(projectile_result_t* res);

/**
 * @brief Prints the contents of a projectile_result_t to standard output.
 *
 * @param result Pointer to the projectile_result_t to print.
 */
BYUL_API void projectile_result_print(const projectile_result_t* result);

/// Recommended buffer size for string conversion
#define PROJECTILE_RESULT_STR_BUFSIZE 512
/**
 * @brief Converts a projectile_result_t to a string.
 *
 * @param result       Pointer to the projectile_result_t to convert.
 * @param buffer_size  Buffer size (recommended: PROJECTILE_RESULT_STR_BUFSIZE).
 * @param buffer       Buffer to store the result string.
 * @return Pointer to buffer (for chaining).
 */
BYUL_API char* projectile_result_to_string(
    const projectile_result_t* result,
    size_t buffer_size, char* buffer);

/**
 * @brief Prints detailed information of a projectile_result_t.
 *
 * - Prints basic data (bool_impacted, impact_time, impact_pos).
 * - Calls trajectory_print() if trajectory exists.
 *
 * @param result Pointer to the projectile_result_t to print.
 */
BYUL_API void projectile_result_print_detailed(
    const projectile_result_t* result);

/**
 * @brief Converts detailed information of a projectile_result_t to a string.
 *
 * This function includes all trajectory points in addition to basic data
 * (bool_impacted, impact_time, impact_pos). If trajectory has many points,
 * the resulting string can be large. Use a sufficiently large buffer
 * (recommended: PROJECTILE_RESULT_STR_BUFSIZE * 100 or more).
 *
 * @param result       Pointer to the projectile_result_t to convert (must not be NULL).
 * @param buffer       Buffer to store the result string (must not be NULL).
 * @param buffer_size  Buffer size (recommend at least 25KB for large trajectory data).
 * @return Pointer to buffer (for chaining or printf).
 */
BYUL_API char* projectile_result_to_string_detailed(
    const projectile_result_t* result,
    char* buffer,
    size_t buffer_size);

/**
 * @brief Calculates the initial force using the first two samples of the trajectory.
 *
 * F = m * a, a = (v1 - v0) / delta t  
 * where v0 and v1 are the velocity vectors of trajectory sample 0 and 1.
 *
 * @param result  Projectile_result_t containing trajectory data (must not be NULL).
 * @param mass    Projectile mass (kg).
 * @return Initial force (Newtons, N). Returns 0.0f if inbool_impacted.
 */
BYUL_API float projectile_result_calc_initial_force_scalar(
    const projectile_result_t* result,
    float mass);    

// ---------------------------------------------------------
// Projectile Trajectory Prediction
// ---------------------------------------------------------

/**
 * @brief Simulates a projectile trajectory 
 * and calculates collision and trajectory data.
 *
 * This function predicts the trajectory based 
 * on the initial state of the projectile (`projectile_t`)
 * and the given environment information, 
 * and checks if a target collision occurs.
 * - **Guidance function (`guidance_func`)** 
 * can adjust projectile direction in real time.
 * - **Propulsion (`propulsion_t`)** applies thrust acceleration if provided.
 * - **Environment (`environ_t`)** accounts for gravity, 
 * wind, drag, and other forces.
 *
 * @param[in]  proj         Initial projectile state 
 * @param[in]  time_step    Simulation time step (seconds).
 * (position, velocity, mass, etc.).
 * @param[in]  target       Target dynamic data (NULL if no target).
 * @param[in]  env          Environment data  default : NULL
 * @param[in]  ground          Ground data  default : NULL
 * (NULL if no environmental effects).
 * @param[in]  propulsion   Propulsion data default : NULL
 * (NULL if no propulsion).
 * @param[in]  guidance_fn  Guidance function pointer (NULL if no guidance).
 * @param[out] out          Structure to store trajectory 
 *      and collision results (must not be NULL). 
 *
 * @retval true   Collision occurs (with target or ground).
 * @retval false  No collision (did not hit within max simulation time).
 */
BYUL_API bool projectile_predict(
    const projectile_t* proj,
    float time_step,
    const entity_dynamic_t* target,
    const environ_t* env,
    const ground_t* ground,
    propulsion_t* propulsion,
    guidance_func guidance_fn,
    projectile_result_t* out);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_PREDICT_H
