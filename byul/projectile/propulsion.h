#ifndef PROPULSION_H
#define PROPULSION_H

#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct propulsion_t
 * @brief Propulsion system state structure for projectiles
 *
 * Manages thrust power, including the thrust controller and fuel status.
 */
typedef struct s_propulsion {
    // --- Basic Performance ---
    float max_thrust;            ///< Maximum thrust (N). Must be >= 0.0f
    float current_thrust;        ///< Current thrust (calculated by controller)
    float fuel_capacity;         ///< Total fuel capacity (kg)
    float fuel_remaining;        ///< Remaining fuel (kg)
    float burn_rate;             ///< Fuel consumption rate (kg/s). Base rate per 1N thrust

    // --- Efficiency ---
    float efficiency;            ///< Propulsion efficiency (0.0 ~ 1.0). Example: 0.7 = 70%
    float thermal_loss;          ///< Thermal loss factor (0.0 ~ 0.2). Portion of thrust lost
    float energy_density;        ///< Fuel energy density (MJ/kg), used for thrust calculations

    // --- Dynamic Response ---
    float response_time;         ///< Time required to reach target thrust (s)
    float max_thrust_rate;       ///< Maximum thrust change rate (N/s)
    float delay_time;            ///< Delay before control input affects output (s)

    // --- Heat and Wear ---
    float heat;                  ///< Accumulated heat (arbitrary units)
    float heat_dissipation_rate; ///< Heat dissipation rate (per unit time)
    float wear_level;            ///< Wear level (0.0 ~ 1.0). 1.0 indicates performance degradation

    // --- Controller ---
    controller_t* controller;    ///< Controller for thrust (PID, MPC, etc.)
    bool active;                 ///< Propulsion system active state
} propulsion_t;


// ---------------------------------------------------------------------------
// Initialization and State Management
// ---------------------------------------------------------------------------

/**
 * @brief Initialize propulsion_t with default values
 *
 * This function sets all fields in propulsion_t to realistic default values.
 * After initialization, the engine is inactive (active=false), and fuel and performance
 * parameters are set to the initial defaults.
 *
 * ### Default Settings:
 * - **max_thrust = 120.0f**  
 *   - Max thrust in N (small rocket engine or drone motor level)
 *
 * - **current_thrust = 0.0f**  
 * - **fuel_capacity = 50.0f** (kg)
 * - **fuel_remaining = 50.0f** (kg)
 * - **burn_rate = 0.05f** (kg/s, about 2 minutes at 100 N thrust)
 * - **efficiency = 0.7f** (70% energy conversion efficiency)
 * - **thermal_loss = 0.05f** (5% thermal loss)
 * - **energy_density = 42.0f** (MJ/kg, kerosene reference)
 * - **response_time = 0.8f** (s)
 * - **max_thrust_rate = 30.0f** (N/s)
 * - **delay_time = 0.2f** (s)
 * - **heat = 0.0f** (initial heat)
 * - **heat_dissipation_rate = 0.3f** (heat loss per unit time)
 * - **wear_level = 0.0f** (0.0 ~ 1.0)
 * - **controller = NULL**
 * - **active = false**
 *
 * @param p Pointer to propulsion_t to initialize (does nothing if NULL)
 */
BYUL_API void propulsion_init(propulsion_t* p);

/**
 * @brief Fully initialize propulsion_t with user-defined parameters
 *
 * Recommended to set max_thrust, fuel_capacity, and burn_rate within **standard ranges**.
 *
 * ### Recommended Ranges:
 * - **max_thrust:** 10.0f ~ 1000.0f (N)
 * - **fuel_capacity:** 1.0f ~ 500.0f (kg)
 * - **burn_rate:** 0.01f ~ 5.0f (kg/s)
 *
 * @param p Pointer to propulsion_t (does nothing if NULL)
 * @param max_thrust Maximum thrust (N)
 * @param fuel_capacity Total fuel (kg)
 * @param burn_rate Fuel consumption rate (kg/s)
 * @param ctrl Controller pointer (optional)
 * @param active Initial activation state
 */
BYUL_API void propulsion_init_full(propulsion_t* p,
                                   float max_thrust,
                                   float fuel_capacity,
                                   float burn_rate,
                                   controller_t* ctrl,
                                   bool active);

/**
 * @brief Copy propulsion_t state
 */
BYUL_API void propulsion_assign(propulsion_t* dst, const propulsion_t* src);

/**
 * @brief Reset propulsion_t to initial state (reset fuel/thrust)
 */
BYUL_API void propulsion_reset(propulsion_t* p);

// ---------------------------------------------------------------------------
// Updates and Status
// ---------------------------------------------------------------------------

/**
 * @brief Update propulsion system state
 *
 * Updates the propulsion system for a time interval `dt` toward `target_thrust`.  
 * The controller, if connected, is used to compute actual thrust, while fuel consumption,
 * efficiency, wear, and heat are taken into account.
 *
 * ### Key Steps:
 * 1. Clamp `target_thrust` between 0 and max_thrust.
 * 2. Apply controller computation if available.
 * 3. Apply efficiency and loss factors.
 * 4. Limit thrust change rate.
 * 5. Calculate fuel consumption.
 * 6. Update heat and wear.
 *
 * @param p Pointer to propulsion_t (does nothing if NULL)
 * @param target_thrust Target thrust (N), within 0 ~ max_thrust
 * @param dt Time interval (s), must be > 0
 */
BYUL_API void propulsion_update(propulsion_t* p, float target_thrust, float dt);

/**
 * @brief Returns current thrust (N)
 */
BYUL_API float propulsion_get_thrust(const propulsion_t* p);

/**
 * @brief Checks if fuel is empty
 */
BYUL_API bool propulsion_is_empty(const propulsion_t* p);

/**
 * @brief Returns fuel ratio (0.0 ~ 1.0)
 */
BYUL_API float propulsion_get_fuel_ratio(const propulsion_t* p);

/**
 * @brief Returns maximum runtime (s) at current thrust
 */
BYUL_API float propulsion_get_max_runtime(const propulsion_t* p);

// ---------------------------------------------------------------------------
// Fuel Management
// ---------------------------------------------------------------------------

/**
 * @brief Refuel propulsion system
 */
BYUL_API void propulsion_refuel(propulsion_t* p, float amount);

/**
 * @brief Force fuel consumption
 */
BYUL_API void propulsion_consume(propulsion_t* p, float amount);

/**
 * @brief Returns total possible impulse (N·s) with remaining fuel
 */
BYUL_API float propulsion_get_remaining_impulse(const propulsion_t* p);

// ---------------------------------------------------------------------------
// Prediction
// ---------------------------------------------------------------------------

/**
 * @brief Predict runtime for a given desired_thrust (N)
 *
 * runtime = fuel_remaining / (burn_rate * desired_thrust)
 *
 * @param p Pointer to propulsion_t
 * @param desired_thrust Desired thrust (N)
 * @return Time in seconds, or 0.0f if not possible
 */
BYUL_API float propulsion_predict_runtime(
    const propulsion_t* p, float desired_thrust);

/**
 * @brief Predicts time until fuel depletion at current thrust
 */
BYUL_API float propulsion_predict_empty_time(const propulsion_t* p);

/**
 * @brief Predicts possible average thrust (N) over a given duration
 *
 * avg_thrust = min(max_thrust,
 *                  fuel_remaining / (burn_rate * duration))
 *
 * @param p Pointer to propulsion_t
 * @param duration Duration (s)
 * @return Average thrust (N), or 0.0f if not possible
 */
BYUL_API float propulsion_predict_max_thrust(
    const propulsion_t* p, float duration);

// ---------------------------------------------------------------------------
// Controller Management
// ---------------------------------------------------------------------------

/**
 * @brief Set propulsion system active/inactive
 */
BYUL_API void propulsion_set_active(propulsion_t* p, bool active);

/**
 * @brief Attach a new controller to propulsion system
 */
BYUL_API void propulsion_attach_controller(
    propulsion_t* p, controller_t* ctrl);

/**
 * @brief Detach current controller
 */
BYUL_API void propulsion_detach_controller(propulsion_t* p);

// ---------------------------------------------------------------------------
// Debug/Logging
// ---------------------------------------------------------------------------

/**
 * @brief Print current propulsion status to console
 */
BYUL_API void propulsion_print(const propulsion_t* p);

/**
 * @brief Convert current propulsion status to string
 */
BYUL_API const char* propulsion_to_string(
    const propulsion_t* p, char* buffer, size_t buffer_size);

/**
 * @brief Convert current propulsion status to JSON format string
 * Example: {"thrust":80.0, "fuel":45.0, "capacity":100.0, "active":1}
 */
BYUL_API const char* propulsion_to_json(
    const propulsion_t* p, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // PROPULSION_H
