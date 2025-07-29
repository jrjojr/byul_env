#ifndef ENVIRON_H
#define ENVIRON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "vec3.h"

// Forward declarations
typedef struct s_bodyprops bodyprops_t;
typedef struct s_projectile projectile_t;

/**
 * @typedef environ_func
 * @brief Function pointer for calculating external acceleration.
 *
 * @param[in] env        Environment data pointer.
 * @param[in] dt         Time step (seconds).
 * @param[in] userdata   User-defined data.
 * @param[out] out_accel Calculated acceleration vector.
 * @return Pointer to the calculated acceleration vector.
 */
typedef const vec3_t* (*environ_func)(
    const struct s_environ* env,
    float dt,
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
 * - Stores **gravity**, **wind**, **air density**, etc.
 * - Uses `environ_func` to calculate dynamic or user-defined environmental acceleration.
 */
typedef struct s_environ {
    vec3_t gravity;        /**< Gravitational acceleration (m/s²), default {0, -9.81, 0}. */
    vec3_t wind;           /**< Wind acceleration (m/s²). */
    float air_density;     /**< Air density (kg/m³), default 1.225. */
    float humidity;        /**< Humidity [%]. */
    float temperature;     /**< Temperature [°C]. */
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
 * - Air density: 1.225 kg/m³
 * - Humidity: 50 %
 * - Temperature: 20 °C
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
 * @param[in] wind        Wind vector (NULL to use default).
 * @param[in] air_density Air density (kg/m³).
 * @param[in] humidity    Humidity [%].
 * @param[in] temperature Temperature [°C].
 * @param[in] pressure    Atmospheric pressure [Pa].
 * @param[in] environ_fn  External acceleration function pointer (NULL to use default).
 * @param[in] userdata    User data passed to the acceleration function.
 */
BYUL_API void environ_init_full(environ_t* env,
                                    const vec3_t* gravity,
                                    const vec3_t* wind,
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

// ---------------------------------------------------------
// External force adjustments
// ---------------------------------------------------------
/**
 * @brief Apply consistent environmental adjustments to the given acceleration vector.
 *
 * The input acceleration (accel) may or may not include gravity. Regardless, 
 * environmental factors like drag, wind, humidity, temperature, and pressure 
 * are applied uniformly.
 *
 * @param[in]    env    Environment data (wind, humidity, temperature, etc.).
 * @param[inout] accel  Acceleration vector to adjust (gravity inclusion is ignored).
 */
BYUL_API void environ_adjust_accel(
    const environ_t* env, vec3_t* accel);

/**
 * @brief Separate gravity from the acceleration vector, apply environmental adjustments to 
 *        external forces only, and then optionally reapply gravity.
 *
 * The function supports both cases where the input acceleration vector includes gravity or not, 
 * based on the `has_gravity` flag.
 *
 * Workflow:
 * - has_gravity = true:
 *    1) Subtract env->gravity from accel to extract external forces.
 *    2) Apply environmental adjustments (factor) to the external forces.
 *    3) Re-add env->gravity after adjustment.
 * - has_gravity = false:
 *    1) Treat accel as external forces.
 *    2) Apply environmental adjustments (factor).
 *    3) Do not add gravity.
 *
 * @param[in]    env            Environment data (gravity, wind, etc.).
 * @param[in]    has_gravity    true if accel includes gravity, false if it only has external forces.
 * @param[inout] accel          Acceleration vector to adjust.
 */
BYUL_API void environ_adjust_accel_gsplit(
    const environ_t* env, bool has_gravity, vec3_t* accel);

// ---------------------------------------------------------
// Default environment functions
// ---------------------------------------------------------
/**
 * @brief Environment function with zero external acceleration.
 */
BYUL_API const vec3_t* environ_calc_none(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

/**
 * @brief Environment function applying standard gravity only.
 */
BYUL_API const vec3_t* environ_calc_gravity(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

/**
 * @brief Environment function applying gravity + fixed wind.
 */
BYUL_API const vec3_t* environ_calc_gravity_wind(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

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
    float time;            ///< Elapsed time (s).
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
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

#ifdef __cplusplus
}
#endif

#endif // ENVIRON_H
