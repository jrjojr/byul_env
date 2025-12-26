#ifndef ENVIRON_H
#define ENVIRON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_config.h"
#include "vec3.h"

// Forward declarations
typedef struct s_bodyprops bodyprops_t;
typedef struct s_projectile projectile_t;

/**
 * @typedef environ_func
 * @brief Function pointer for calculating external acceleration.
 *
 * @param[in] env        Environment data pointer.
 * @param[in] delta_time         Time step (seconds).
 * @param[in] userdata   User-defined data.
 * @param[out] out_accel Calculated acceleration vector.
 * @return Pointer to the calculated acceleration vector.
 */
typedef const vec3_t* (*environ_func)(
    const struct s_environ* env,
    void* userdata,
    vec3_t* out_accel
);

// ---------------------------------------------------------
// Environment structure
// ---------------------------------------------------------
/**
 * @struct environ_t
 * @brief Simulation environment data and external acceleration function pointer.
 *
 * This structure defines external environmental factors considered during trajectory prediction 
 * for projectiles or entities.
 * - Stores **gravity**, **wind_vel**, **air density**, etc.
 * - Uses `environ_func` to calculate dynamic or user-defined environmental acceleration.
 */
typedef struct s_environ {
    vec3_t gravity;        /**< Gravitational acceleration (m/s^2), default {0, -9.81, 0}. */
    vec3_t wind_vel;           /**< Wind velocity (m/s). */
    float air_density;     /**< Air density (kg/m^3), default 1.225. */
    float humidity;        /**< Humidity [%]. */
    float temperature;     /**< Temperature [degC]. */
    float pressure;        /**< Atmospheric pressure [Pa]. */

    environ_func environ_fn;

    void* userdata;        /**< User data passed to environ_func. */
} environ_t;

// ---------------------------------------------------------
// Initialization functions
// ---------------------------------------------------------
/**
 * @brief Initialize environ_t with default values.
 *
 * - Gravity: {0, -9.81, 0}
 * - Wind: {0, 0, 0}
 * - Air density: 1.225 kg/m^3
 * - Humidity: 50 %
 * - Temperature: 20 degC
 * - Pressure: 101,325 Pa
 * - environ_func: environ_calc_gravity
 *
 * @param[out] env Environment structure to initialize.
 */
BYUL_API void environ_init(environ_t* env);

/**
 * @brief Initialize environ_t with specified values.
 *
 * @param[out] env        Environment structure to initialize.
 * @param[in] gravity     Gravity vector (NULL to use default).
 * @param[in] wind_vel        Wind vector (NULL to use default).
 * @param[in] air_density Air density (kg/m^3).
 * @param[in] humidity    Humidity [%].
 * @param[in] temperature Temperature [degC].
 * @param[in] pressure    Atmospheric pressure [Pa].
 * @param[in] environ_fn  External acceleration function pointer (NULL to use default).
 * @param[in] userdata    User data passed to the acceleration function.
 */
BYUL_API void environ_init_full(environ_t* env,
                                    const vec3_t* gravity,
                                    const vec3_t* wind_vel,
                                    float air_density,
                                    float humidity,
                                    float temperature,
                                    float pressure,
                                    environ_func environ_fn,
                                    void* userdata);

/**
 * @brief Copy environment data.
 * @param[out] out Destination.
 * @param[in]  src Source.
 */
BYUL_API void environ_assign(environ_t* out, 
                                 const environ_t* src);

/**< Wind acceleration (m/s^2). */
BYUL_API void environ_apply_wind(
    environ_t* env, const vec3_t* accel, float dt);

/**
 * @brief Distort the acceleration vector by environmental effects 
 * (uniformly, without separating gravity).
 *
 * The function applies the environment's distortion model 
 * (e.g., drag, cross-wind bias, density/humidity
 * attenuation, temperature/pressure scaling) 
 * directly to the input acceleration vector as-is.
 * It does NOT attempt to separate or preserve gravity: 
 * if gravity is mixed into @p accel, it is distorted
 * together with the rest of the vector.
 *
 * Typical intent:
 * // example: magnitude reduced, cross-wind added
 *   accel = (100, 0, 0)  -->  distort -->  (80, -9.8, 10)   
 *
 * @param[in]    env    Environment parameters 
 * (wind, density/humidity, temperature, pressure, gravity, etc.).
 *                      Note: env->gravity is NOT separated in this API; 
 * it may or may not influence the model
 *                      depending on your distortion implementation, 
 * but no gravity split is performed here.
 * @param[inout] accel  Acceleration to distort. 
 * On return, contains the distorted acceleration.
 *
 * @warning Use environ_distort_accel_except_gravity() 
 * if you need gravity to remain physically intact
 *          (i.e., only external forces are distorted).
 */
BYUL_API void environ_distort_accel(const environ_t* env, vec3_t* accel);

/**
 * @brief Distort only the external-force component of acceleration; 
 * never distort gravity.
 *
 * This function optionally splits gravity from the input, 
 * applies environmental distortion ONLY to the
 * external part, and then decides whether to include gravity 
 * in the output based on @p include_gravity.
 *
 * Definitions:
 *   Let g = env->gravity, D(.) = environment distortion operator (
 * drag, wind, humidity, etc.).
 *
 * Workflow:
 *   if (include_gravity == true)
 *     Input is assumed to include gravity:           a_in  = a_ext + g
 *     External forces are distorted only:            a_out = D(a_ext) + g
 *   else
 *     Input is assumed external-only (no gravity):   a_in  = a_ext
 *     Output excludes gravity by design:             a_out = D(a_ext)
 *
 * Examples (g = (0, -9.8, 0)):
 *   include_gravity = true:
 *     accel = (100, -9.8, 0)  ->  split -> a_ext=(100,0,0)  
 * ->  D -> (80,0,10)  -> re-add g -> (80, -9.8, 10)
 *   include_gravity = false:
 *     accel = (100, 0, 0)     ->  treat as a_ext           
 * ->  D -> (80,0,10)   -> no gravity in output
 *
 * @param[in]    env          Environment parameters 
 * (gravity, wind, density/humidity, temperature, pressure).
 * @param[in]    include_gravity  true  = input @p accel includes gravity 
 * and the output must include gravity;
 *                            false = input is external-only 
 * and the output must exclude gravity.
 * @param[inout] accel        On input, the raw acceleration 
 * as defined by @p include_gravity.
 *     On return, the distorted result per the policy above.
 *
 * @note This API guarantees that gravity is never distorted. 
 * Only the external component is passed through D(.).
 * @warning If @p include_gravity does not reflect 
 * the actual composition of @p accel, the result will be inconsistent.
 *          Ensure the flag matches your integrator's convention.
 */
BYUL_API void environ_distort_accel_except_gravity(
    const environ_t* env, bool include_gravity, vec3_t* accel);


// ---------------------------------------------------------
// Default environment functions
// ---------------------------------------------------------
/**
 * @brief Environment function with zero external acceleration.
 */
BYUL_API const vec3_t* environ_calc_none(
    const environ_t* env, void* userdata, vec3_t* out_accel);

/**
 * @brief Environment function applying standard gravity only.
 */
BYUL_API const vec3_t* environ_calc_gravity(
    const environ_t* env, void* userdata, vec3_t* out_accel);

/**
 * @brief Environment function applying gravity + fixed wind.
 */
BYUL_API const vec3_t* environ_calc_gravity_wind(
    const environ_t* env, void* userdata, vec3_t* out_accel);

// ---------------------------------------------------------
// Periodic environment
// ---------------------------------------------------------
/**
 * @struct environ_periodic_t
 * @brief Environment data with periodic wind and external force variations.
 */
typedef struct s_environ_periodic {
    vec3_t base_wind;      ///< Base wind vector.
    vec3_t gust_amplitude; ///< Wind variation amplitude.
    float gust_frequency;  ///< Wind frequency (Hz).
    float elapsed;            ///< Elapsed time (s).
    float delta_time;
    vec3_t gravity;        ///< Gravity vector (default {0, -9.81, 0}).
} environ_periodic_t;

/**
 * @brief Initialize periodic environment data with default values.
 *
 * @param[out] out Periodic environment structure to initialize.
 */
BYUL_API void environ_periodic_init(environ_periodic_t* out);

/**
 * @brief Initialize periodic environment data with specified values.
 *
 * @param[in]  base_wind  Base wind vector.
 * @param[in]  gust_amp   Wind amplitude vector.
 * @param[in]  gust_freq  Wind frequency (Hz).
 * @param[in]  gravity    Gravity vector.
 * @param[out] out        Periodic environment structure to initialize.
 */
BYUL_API void environ_periodic_init_full(
    const vec3_t* base_wind,
    const vec3_t* gust_amp,
    float gust_freq,
    const vec3_t* gravity,
    environ_periodic_t* out);

/**
 * @brief Copy periodic environment data.
 * @param[out] out Destination.
 * @param[in]  src Source.
 */
BYUL_API void environ_periodic_assign(
    const environ_periodic_t* src, environ_periodic_t* out);

/**
 * @brief Environment function applying periodic wind and gravity.
 */
BYUL_API const vec3_t* environ_calc_periodic(
    const environ_t* env, void* userdata, vec3_t* out_accel);

#ifdef __cplusplus
}
#endif

#endif // ENVIRON_H
