/**
 * @file numeq_kalman.h
 * @brief Kalman Filter module
 *
 * Provides recursive state estimation using the Kalman Filter
 * for 1D scalar and 3D vector states (position, velocity).
 *
 * Based on noisy measurements from sensors, the system state is
 * updated through Time Update (Prediction) and Measurement Update
 * steps to minimize noise and provide more accurate estimates.
 */

#ifndef NUMEQ_KALMAN_H
#define NUMEQ_KALMAN_H

#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// 1. Scalar Kalman Filter (1D)
// =========================================================

/**
 * @struct kalman_filter_t
 * @brief 1D Kalman Filter
 *
 * Default initial values:
 * - x = 0 (initial state)
 * - p = 1 (initial error covariance)
 * - q = 0.01 (process noise)
 * - r = 1.0 (measurement noise)
 * - k = 0 (Kalman gain)
 *
 * Typical ranges:
 * - x: usually -1000 to +1000 (depends on sensor range)
 * - p: 0.001 to 100 (error covariance)
 * - q: 0.0001 to 0.1 (process noise)
 * - r: 0.01 to 10 (measurement noise)
 */
typedef struct s_kalman_filter {
    float x;   /**< State estimate */
    float p;   /**< Error covariance */
    float q;   /**< Process noise */
    float r;   /**< Measurement noise */
    float k;   /**< Kalman gain */
} kalman_filter_t;

/**
 * @brief Initialize 1D Kalman filter with default values.
 *
 * Default values:
 * - x = 0
 * - p = 1
 * - q = 0.01
 * - r = 1
 * - k = 0
 *
 * @param kf Pointer to Kalman filter
 */
BYUL_API void kalman_init(kalman_filter_t* kf);

/**
 * @brief Initialize 1D Kalman filter with custom values.
 *
 * @param kf Pointer to Kalman filter
 * @param init_x Initial state estimate (recommended range: -1000 to +1000)
 * @param init_p Initial error covariance (recommended range: 0.001 to 100)
 * @param process_noise Process noise Q (recommended range: 0.0001 to 0.1)
 * @param measurement_noise Measurement noise R (recommended range: 0.01 to 10)
 */
BYUL_API void kalman_init_full(kalman_filter_t* kf, float init_x, float init_p,
                               float process_noise, float measurement_noise);

/**
 * @brief Copy kalman_filter_t.
 */
BYUL_API void kalman_assign(kalman_filter_t* dst, const kalman_filter_t* src);

/**
 * @brief Time update (Prediction step).
 *
 * Prediction step: state x is unchanged and error covariance p is updated as p + q.
 *
 * @param kf Pointer to Kalman filter
 */
BYUL_API void kalman_time_update(kalman_filter_t* kf);

/**
 * @brief Measurement update.
 *
 * Corrects the state estimate based on a new measurement.
 *
 * @param kf Pointer to Kalman filter
 * @param measured New measurement (z)
 * @return Corrected state estimate
 */
BYUL_API float kalman_measurement_update(kalman_filter_t* kf, float measured);


// =========================================================
// 2. Vector Kalman Filter (3D)
// =========================================================

/**
 * @struct kalman_filter_vec3_t
 * @brief 3D Kalman filter for position and velocity estimation
 *
 * Default initial values:
 * - position = (0,0,0)
 * - velocity = (0,0,0)
 * - error_p = (1,1,1)
 * - q = 0.01
 * - r = 1.0
 * - dt = 0.1
 *
 * Typical ranges:
 * - position: -1000 to +1000 (meters)
 * - velocity: -50 to +50 (m/s)
 * - q: 0.0001 to 0.1 (process noise)
 * - r: 0.01 to 10 (measurement noise)
 * - dt: 0.01 to 0.1 (seconds)
 */
typedef struct s_kalman_filter_vec3 {
    vec3_t position;   /**< Estimated position */
    vec3_t velocity;   /**< Estimated velocity */

    vec3_t error_p;    /**< Error covariance (x, y, z) */
    float q;           /**< Process noise */
    float r;           /**< Measurement noise */
    float dt;          /**< Time interval (s) */
} kalman_filter_vec3_t;

/**
 * @brief Initialize 3D Kalman filter with default values.
 *
 * Default values:
 * - position = (0,0,0)
 * - velocity = (0,0,0)
 * - error_p = (1,1,1)
 * - q = 0.01
 * - r = 1.0
 * - dt = 0.1
 *
 * @param kf Pointer to Kalman filter
 */
BYUL_API void kalman_vec3_init(kalman_filter_vec3_t* kf);

/**
 * @brief Initialize 3D Kalman filter with custom values.
 *
 * @param kf Pointer to Kalman filter
 * @param init_pos Initial position (recommended: -1000 to +1000)
 * @param init_vel Initial velocity (recommended: -50 to +50)
 * @param process_noise Process noise Q (recommended: 0.0001 to 0.1)
 * @param measurement_noise Measurement noise R (recommended: 0.01 to 10)
 * @param dt Prediction step interval (seconds, recommended: 0.01 to 0.1)
 */
BYUL_API void kalman_vec3_init_full(kalman_filter_vec3_t* kf,
                                    const vec3_t* init_pos,
                                    const vec3_t* init_vel,
                                    float process_noise,
                                    float measurement_noise,
                                    float dt);

/**
 * @brief Copy kalman_filter_vec3_t.
 */
BYUL_API void kalman_vec3_assign(kalman_filter_vec3_t* dst,
                           const kalman_filter_vec3_t* src);
                                                               
/**
 * @brief Time update (Prediction step).
 *
 * Extrapolates state as position = position + velocity * dt,
 * and increases error covariance P = P + Q.
 *
 * @param kf Pointer to Kalman filter
 */
BYUL_API void kalman_vec3_time_update(kalman_filter_vec3_t* kf);

/**
 * @brief Measurement update.
 *
 * Corrects the state based on a new measured position.
 *
 * @param kf Pointer to Kalman filter
 * @param measured_pos Measured position
 */
BYUL_API void kalman_vec3_measurement_update(kalman_filter_vec3_t* kf,
                                             const vec3_t* measured_pos);

/**
 * @brief Predict future position (linear extrapolation).
 *
 * @param kf Kalman filter state
 * @param future_dt Prediction time (seconds)
 * @param out_predicted_pos Predicted position vector
 */
BYUL_API void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                                  float future_dt,
                                  vec3_t* out_predicted_pos);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_KALMAN_H
