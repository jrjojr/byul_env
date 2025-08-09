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

#include "byul_common.h"
#include "byul_tick.h"
#include "dstar_lite.h"
#include "coord.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Maximum steps per single tick cycle
#define MAX_STEP 64

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
 * @param dsl Target D* Lite object
 * @return Initialized tick controller, or NULL on failure
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
 * @param dsl Target D* Lite object
 * @param max_time Total allowed tick duration (in seconds)
 * @return Initialized tick controller, or NULL on failure
 */
BYUL_API dstar_lite_tick_t* dstar_lite_tick_create_full(
    dstar_lite_t* dsl, float max_time);

/**
 * @brief Frees memory allocated to tick controller
 * @param dst Tick controller to free
 */
BYUL_API void dstar_lite_tick_destroy(dstar_lite_tick_t* dst);

/**
 * @brief Copies a tick controller object
 * @param src Tick controller to copy
 * @return Duplicated controller object
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
 * @param dst Tick controller to reset (NULL is ignored)
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
 * @param dst Tick controller (must be pre-created)
 * @param tk Tick system handle
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
 * @param dst Tick controller (must be pre-created)
 * @param unit_m Unit distance per tile
 * @param speed_sec Speed in meters per second
 * @param max_time Maximum allowed tick duration
 * @param tk Tick system object
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
 * @param dst Tick controller
 * @param dt Delta time since last tick (seconds)
 */
BYUL_API void dstar_lite_tick_update(dstar_lite_tick_t* dst, float dt);

/**
 * @brief Detaches the tick controller from the tick system.
 *
 * This disables further automatic updates. It is usually called
 * after reaching the goal or on user-triggered events.
 *
 * @param dst Tick controller to detach
 * @param tk Tick system object
 */
BYUL_API void dstar_lite_tick_complete(
    dstar_lite_tick_t* dst, tick_t* tk);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_TICK_H
