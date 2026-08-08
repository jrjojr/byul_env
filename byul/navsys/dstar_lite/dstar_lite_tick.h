/**
 * @file dstar_lite_tick.h
 * @brief D* Lite Tick-based Time Control Module
 *
 * This module provides a tick-based interface for dstar_lite_t,
 * enabling the route to move one step at a time at regular time intervals (dt).
 * The movement interval is determined by unit distance (unit_m)
 * and movement speed (speed_sec).
 *
 * Internally, this module is attached to the byul_tick system and
 * is called automatically. The tick loop ends when either the total
 * movement time (max_time) is exceeded or the goal is reached.
 *
 * Estimated formula for max_time:
 *     max_time ~= (distance / speed) * 1.25  // includes 25% margin
 *
 * Example:
 *   Distance = 10m, Speed = 1m/s -> max_time ~= 12.5 seconds
 */
#ifndef DSTAR_LITE_TICK_H
#define DSTAR_LITE_TICK_H

#include "byul_config.h"
#include "byul_tick.h"
#include "dstar_lite.h"
#include "coord.h"
#include "navsys_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Maximum steps per single tick cycle
#define MAX_STEP 64
#define BYUL_DSTAR_LITE_TICK_DEFAULT_MAX_STEPS 64u

/** Explicit canonical controller state. */
typedef enum e_dstar_lite_tick_state {
    DSTAR_LITE_TICK_STATE_DETACHED = 0,
    DSTAR_LITE_TICK_STATE_ATTACHED = 1,
    DSTAR_LITE_TICK_STATE_RUNNING = 2,
    DSTAR_LITE_TICK_STATE_COMPLETED = 3,
    DSTAR_LITE_TICK_STATE_CANCELLED = 4,
    DSTAR_LITE_TICK_STATE_FAILED = 5
} dstar_lite_tick_state_t;

/** Versioned canonical controller configuration. */
typedef struct s_dstar_lite_tick_create_info {
    uint32_t struct_size;
    uint32_t abi_version;
    dstar_lite_t* planner;
    float tile_size_m;
    float speed_m_per_sec;
    float max_duration_sec;
    uint32_t max_steps_per_update;
} dstar_lite_tick_create_info_t;

#define DSTAR_LITE_TICK_CREATE_INFO_VERSION 1u

/**
 * @struct s_dstar_lite_tick
 * @brief D* Lite tick-based controller state structure
 */
typedef struct s_dstar_lite_tick {
    dstar_lite_t* base;          ///< Target D* Lite object

    float max_time;              ///< Total allowed tick time (in seconds)

    float unit_m;                ///< Distance per tile (in meters)
    float speed_sec;             ///< Movement speed (meters per second)

    float cur_time;              ///< Accumulated total tick time
    float cur_elapsed_time;      ///< Elapsed time for current step

    coord_t s_last;              ///< Last processed coordinate
    bool ticked;                 ///< Whether attached to tick system

    float max_elapsed_time;      ///< [Unused] Reserved for future expansion
} dstar_lite_tick_t;

/**
 * @brief Initializes canonical tick configuration defaults.
 * @param[out] out_info Destination configuration.
 * @param[in] planner Borrowed planner controlled by the tick controller.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable out_info false
 * @byul.nullable planner false
 */
BYUL_API navsys_status_t dstar_lite_tick_create_info_init(
    dstar_lite_tick_create_info_t* out_info, dstar_lite_t* planner);

/**
 * @brief Creates a detached controller that borrows its planner.
 * @param[in] info Valid versioned configuration.
 * @param[out] out_controller Receives the caller-owned controller on success.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable info false
 * @byul.nullable out_controller false
 */
BYUL_API navsys_status_t dstar_lite_tick_create_ex(
    const dstar_lite_tick_create_info_t* info,
    dstar_lite_tick_t** out_controller);

/**
 * @brief Attaches a detached controller to a tick source.
 * @param[in,out] controller Controller to attach.
 * @param[in,out] tick Tick source that borrows the controller.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable controller false
 * @byul.nullable tick false
 */
BYUL_API navsys_status_t dstar_lite_tick_start(
    dstar_lite_tick_t* controller, tick_t* tick);

/**
 * @brief Synchronously detaches an attached controller.
 * @param[in,out] controller Controller to stop.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable controller false
 */
BYUL_API navsys_status_t dstar_lite_tick_stop(
    dstar_lite_tick_t* controller);

/**
 * @brief Advances deterministic controller time and movement.
 * @param[in,out] controller Controller to advance.
 * @param[in] delta_seconds Non-negative elapsed time in seconds.
 * @param[out] out_steps Optional destination for the number of moved tiles.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable controller false
 * @byul.nullable out_steps true
 */
BYUL_API navsys_status_t dstar_lite_tick_advance(
    dstar_lite_tick_t* controller, float delta_seconds,
    uint32_t* out_steps);

/**
 * @brief Returns the explicit controller lifecycle state.
 * @param[in] controller Controller to inspect.
 * @return Current state, or DSTAR_LITE_TICK_STATE_FAILED for invalid input.
 * @byul.nullable controller false
 */
BYUL_API dstar_lite_tick_state_t dstar_lite_tick_get_state(
    const dstar_lite_tick_t* controller);

/**
 * @brief Copies the current planner position.
 * @param[in] controller Controller to inspect.
 * @param[out] out_position Destination coordinate.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable controller false
 * @byul.nullable out_position false
 */
BYUL_API navsys_status_t dstar_lite_tick_fetch_position(
    const dstar_lite_tick_t* controller, coord_t* out_position);

/**
 * @brief Copies elapsed monotonic controller time in seconds.
 * @param[in] controller Controller to inspect.
 * @param[out] out_elapsed_seconds Destination elapsed time.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable controller false
 * @byul.nullable out_elapsed_seconds false
 */
BYUL_API navsys_status_t dstar_lite_tick_get_elapsed_seconds(
    const dstar_lite_tick_t* controller, float* out_elapsed_seconds);

/**
 * @brief Creates a D* Lite tick controller with default values.
 *
 * The following defaults are used:
 * - unit_m = 1.0 (1 tile = 1 meter)
 * - speed_sec = 1.0 (1 m/s)
 * - max_time = 10.0 seconds
 *
 * Must be attached to the tick system using
 * dstar_lite_tick_prepare() or dstar_lite_tick_prepare_full().
 *
 * @param[in] dsl Borrowed target D* Lite object.
 * @return A caller-owned tick controller, or NULL on failure.
 * @byul.nullable dsl false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_tick_t* dstar_lite_tick_create(dstar_lite_t* dsl);

/**
 * @brief Creates a D* Lite tick controller with custom max_time.
 *
 * Uses default values for other settings:
 * - unit_m = 1.0
 * - speed_sec = 1.0
 *
 * Recommended to set at least 10.0 seconds for simple maps.
 *
 * Estimated max_time formula:
 *     max_time ~= (distance / speed) * 1.25
 *
 * Must be attached using dstar_lite_tick_prepare() or
 * dstar_lite_tick_prepare_full() before use.
 *
 * @param[in] dsl Borrowed target D* Lite object.
 * @param[in] max_time Total allowed tick duration in seconds.
 * @return A caller-owned tick controller, or NULL on failure.
 * @byul.nullable dsl false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_tick_t* dstar_lite_tick_create_full(
    dstar_lite_t* dsl, float max_time);

/**
 * @brief Frees a tick controller after synchronously detaching it.
 * @param[in,out] dst Controller to free; NULL is accepted.
 * @byul.nullable dst true
 */
BYUL_API void dstar_lite_tick_destroy(dstar_lite_tick_t* dst);

/**
 * @brief Copies controller configuration into a detached controller.
 * @param[in] src Controller to copy.
 * @return A caller-owned detached copy, or NULL on failure.
 * @byul.nullable src false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_tick_t* dstar_lite_tick_copy(const dstar_lite_tick_t* src);

/**
 * @brief Resets the tick controller to initial state.
 *
 * Use this to restart or reuse the controller after stopping.
 *
 * The following fields are reset:
 * - cur_time = 0.0f
 * - cur_elapsed_time = 0.0f
 * - s_last = {0, 0}
 * - ticked = false
 *
 * Other fields (base, speed, unit_m, max_time) remain unchanged.
 *
 * @param[in,out] dst Tick controller to reset; NULL is accepted.
 * @byul.nullable dst true
 */
BYUL_API void dstar_lite_tick_reset(dstar_lite_tick_t* dst);

/**
 * @brief Attaches a tick controller to the tick system.
 *
 * Once attached, dstar_lite_tick_update() is automatically
 * called at each tick interval.
 *
 * Uses default values for unit_m, speed, and max_time.
 *
 * @param[in,out] dst Pre-created controller.
 * @param[in,out] tk Tick system handle.
 * @byul.nullable dst false
 * @byul.nullable tk false
 */
BYUL_API void dstar_lite_tick_prepare(
    dstar_lite_tick_t* dst, tick_t* tk);

/**
 * @brief Configures and attaches tick controller with detailed parameters.
 *
 * Parameters:
 * - unit_m: distance per tile (meters)
 * - speed_sec: movement speed (meters/second)
 * - max_time: total allowed tick time (seconds)
 *
 * This function attaches the controller to the tick system immediately.
 *
 * @param[in,out] dst Pre-created controller.
 * @param[in] unit_m Unit distance per tile.
 * @param[in] speed_sec Speed in meters per second.
 * @param[in] max_time Maximum allowed tick duration.
 * @param[in,out] tk Tick system object.
 * @byul.nullable dst false
 * @byul.nullable tk false
 */
BYUL_API void dstar_lite_tick_prepare_full(
    dstar_lite_tick_t* dst,
    float unit_m,
    float speed_sec,
    float max_time,
    tick_t* tk);

/**
 * @brief Called automatically every tick to update movement state.
 *
 * This function accumulates dt and attempts to move to the next
 * coordinate if the elapsed time exceeds the threshold.
 *
 * It should not be called manually by the user.
 *
 * Once the goal is reached or max_time is exceeded, the tick
 * controller is detached automatically.
 *
 * @param[in,out] dst Tick controller.
 * @param[in] dt Delta time since last tick in seconds.
 * @byul.nullable dst false
 */
BYUL_API void dstar_lite_tick_update(dstar_lite_tick_t* dst, float dt);

/**
 * @brief Detaches the tick controller from the tick system.
 *
 * This disables further automatic updates. It is usually called
 * after reaching the goal or on user-triggered events.
 *
 * @param[in,out] dst Tick controller to detach.
 * @param[in,out] tk Tick system object.
 * @byul.nullable dst false
 * @byul.nullable tk false
 */
BYUL_API void dstar_lite_tick_complete(
    dstar_lite_tick_t* dst, tick_t* tk);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_TICK_H
