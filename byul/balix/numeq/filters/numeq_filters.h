/**
 * @file numeq_filters.h
 * @brief Common interface for various state estimation filters (Kalman, EKF, UKF, etc.)
 *
 * This module provides a unified interface (filter_interface_t) to control
 * different state estimation filters such as Kalman Filter, EKF, and UKF
 * through adapters and function pointer-based structures.
 *
 * ---
 *
 * ## Key Features
 * - Control different filters (Kalman, EKF, UKF) using the **same API**.
 * - Provides three core callbacks: `time_update`, `measurement_update`, `get_state`.
 * - Binds each filter's internal structure to `filter_state`.
 *
 * ---
 *
 * ## Example Usage
 *
 * ### 1) Initialize Kalman Filter and Create Interface
 * @code
 * kalman_filter_vec3_t kf;
 * vec3_t init_pos = {0,0,0}, init_vel = {0,0,0};
 * kalman_vec3_init_full(&kf, &init_pos, &init_vel, 0.01f, 1.0f, 0.1f);
 *
 * // Convert Kalman filter to common interface
 * filter_interface_t kalman_iface = make_kalman_vec3_interface(&kf);
 * @endcode
 *
 * ### 2) Update Loop
 * @code
 * // 1. Time Update
 * kalman_iface.time_update(kalman_iface.filter_state);
 *
 * // 2. Measurement Update
 * kalman_iface.measurement_update(kalman_iface.filter_state, &measured_pos, NULL);
 *
 * // 3. Read Current State
 * vec3_t pos, vel;
 * kalman_iface.get_state(kalman_iface.filter_state, &pos, &vel);
 * printf("Filter result: pos=(%.2f,%.2f,%.2f)\n", pos.x, pos.y, pos.z);
 * @endcode
 *
 * ---
 *
 * @note EKF and UKF filters can also be connected in the same way by creating adapters.
 */
#ifndef NUMEQ_FILTERS_H
#define NUMEQ_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "numeq_kalman.h"  // Includes Kalman basic implementation

// =========================================================
// Common Filter Interface Definition
// =========================================================

/**
 * @typedef filter_time_update_func
 * @brief Function pointer for time update.
 */
typedef void (*filter_time_update_func)(void* filter);

/**
 * @typedef filter_measurement_update_func
 * @brief Function pointer for measurement update.
 */
typedef void (*filter_measurement_update_func)(void* filter,
                                               const vec3_t* measured_pos,
                                               const vec3_t* measured_vel);

/**
 * @typedef filter_get_state_func
 * @brief Function pointer for retrieving current filter state (position, velocity).
 */
typedef void (*filter_get_state_func)(void* filter,
                                      vec3_t* out_pos,
                                      vec3_t* out_vel);

/**
 * @struct filter_interface_t
 * @brief Unified interface for controlling all filters.
 *
 * `filter_state` points to the internal structure of the filter (Kalman, EKF, etc.)
 * and calls time and measurement updates via function pointers.
 */
typedef struct s_filter_interface {
    void* filter_state;                       /**< Pointer to filter state structure */
    filter_time_update_func time_update;      /**< Time update function */
    filter_measurement_update_func measurement_update; /**< Measurement update function */
    filter_get_state_func get_state;          /**< Function to get current state */
} filter_interface_t;

// =========================================================
// Kalman Filter Adapter
// =========================================================

/**
 * @brief Create an adapter from kalman_filter_vec3_t to filter_interface_t.
 * @param kf Pointer to Kalman filter.
 * @return Filter interface structure.
 */
BYUL_API filter_interface_t make_kalman_vec3_interface(
    kalman_filter_vec3_t* kf);

#ifdef __cplusplus
}
#endif
#endif // NUMEQ_FILTERS_H
