#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include "numal.h"
#include "motion_state.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Flight Path Samples (Time + State)
// ---------------------------------------------------------

/**
 * @struct trajectory_sample_t
 * @brief Structure representing a physical state sample at a specific time.
 */
typedef struct s_trajectory_sample {
    float t;                 /**< Time (in seconds) */
    motion_state_t state;    /**< Motion state at the given time */
} trajectory_sample_t;

/**
 * @struct trajectory_t
 * @brief Predicted trajectory data in chronological order.
 */
typedef struct s_trajectory {
    trajectory_sample_t* samples; /**< Array of predicted trajectory samples */
    int count;                    /**< Number of valid samples */
    int capacity;                 /**< Allocated sample capacity */
} trajectory_t;

// ---------------------------------------------------------
// Trajectory Memory Management Utilities
// ---------------------------------------------------------

/**
 * @brief Creates a new trajectory and allocates memory.
 *
 * Dynamically allocates a trajectory_t structure and its internal samples array,
 * initializing count to 0.
 *
 * @param capacity Maximum number of samples the trajectory can hold.
 * @return Pointer to the allocated trajectory_t on success, nullptr on failure.
 *
 * @note Must call trajectory_destroy() after use to free both the structure
 *       and its internal memory.
 */
BYUL_API trajectory_t* trajectory_create_full(int capacity);

/**
 * @brief Creates a trajectory with default capacity (100 samples).
 *
 * @return Pointer to a trajectory_t allocated with capacity = 100.
 *
 * @note Must call trajectory_destroy() after use to free memory.
 */
BYUL_API trajectory_t* trajectory_create();

/**
 * @brief Initializes a trajectory_t with default capacity (100).
 *
 * @param traj Pointer to trajectory_t to initialize.
 *
 * @warning
 * - Must declare `trajectory_t traj = {};` before calling this function.
 *   (If samples pointer is uninitialized, calling delete[] may cause SIGSEGV.)
 * - Prefer using trajectory_create() instead of this function if possible.
 *
 * @note Existing samples array will be deleted and reallocated.
 */
BYUL_API void trajectory_init(trajectory_t* traj);

/**
 * @brief Initializes a trajectory_t with a specified capacity.
 *
 * @param traj Pointer to trajectory_t to initialize.
 * @param capacity Maximum number of samples (capacity > 0).
 *
 * @warning
 * - Must declare `trajectory_t traj = {};` before calling this function.
 * - Prefer using trajectory_create_full() instead of this function if possible.
 *
 * @note Existing samples array will be deleted and reallocated.
 */
BYUL_API void trajectory_init_full(trajectory_t* traj, int capacity);

/**
 * @brief Frees the internal memory of a trajectory.
 *
 * Frees the samples array within trajectory_t and resets count and capacity to 0.
 *
 * @param traj Pointer to trajectory_t (NULL allowed).
 *
 * @warning Does not free the trajectory_t itself.
 *          Use trajectory_destroy() if the structure was dynamically allocated
 *          with trajectory_create_full().
 */
BYUL_API void trajectory_free(trajectory_t* traj);

/**
 * @brief Frees both the trajectory structure and its internal memory.
 *
 * Calls trajectory_free() and also deallocates the trajectory_t pointer itself.
 *
 * @param traj Pointer to trajectory_t (NULL allowed).
 */
BYUL_API void trajectory_destroy(trajectory_t* traj);

/**
 * @brief Performs a deep copy of trajectory data.
 *
 * Copies all samples from src to out. Reallocates out->samples if needed.
 *
 * @param out Destination trajectory (must be a valid pointer).
 * @param src Source trajectory.
 *
 * @note If out->capacity < src->count, memory will be reallocated.
 */
BYUL_API void trajectory_assign(trajectory_t* out, const trajectory_t* src);

/**
 * @brief Creates a clone of a trajectory.
 *
 * Allocates and returns a new trajectory_t with a deep copy of src.
 *
 * @param src Source trajectory to copy.
 * @return New trajectory_t* (dynamically allocated), nullptr on failure.
 *
 * @note Must call trajectory_destroy() after use to free memory.
 */
BYUL_API trajectory_t* trajectory_copy(const trajectory_t* src);

/**
 * @brief Clears all data in a trajectory.
 *
 * Resets count to 0, but keeps capacity and samples memory intact for reuse.
 *
 * @param traj Pointer to trajectory (NULL allowed).
 */
BYUL_API void trajectory_clear(trajectory_t* traj);

/**
 * @brief Resizes the trajectory_t capacity.
 *
 * @param traj Pointer to trajectory_t.
 * @param new_cap New capacity (must be > 0).
 *
 * @note Existing sample data is preserved, but if count > new_cap,
 *       count will be truncated to new_cap.
 */
BYUL_API void trajectory_resize(trajectory_t* traj, int new_cap);

/**
 * @brief Adds a sample to the trajectory.
 * @param traj Target trajectory.
 * @param t Time value.
 * @param state Motion state to add.
 * @return True if added successfully.
 */
BYUL_API bool trajectory_add_sample(
    trajectory_t* traj, float t, const motion_state_t* state);

/**
 * @brief Returns the number of samples in the trajectory.
 * @param traj Target trajectory.
 * @return Sample count.
 */
BYUL_API int trajectory_length(const trajectory_t* traj);

/**
 * @brief Returns the maximum capacity of the trajectory.
 * @param traj Target trajectory.
 * @return Capacity.
 */
BYUL_API int trajectory_capacity(const trajectory_t* traj);

/**
 * @brief Interpolates the position on the trajectory at time t.
 *
 * Uses the stored samples in trajectory_t to compute the position
 * at time t using linear interpolation between adjacent samples.
 * - If t <= first sample time, returns the first sample position.
 * - If t >= last sample time, returns the last sample position.
 *
 * @param[in] traj     Trajectory data.
 * @param[in] t        Time to interpolate (seconds).
 * @param[out] out_pos Output vector (returns false if NULL).
 * @return True if interpolation was successful.
 */
BYUL_API bool trajectory_interpolate_position(
    const trajectory_t* traj, float t, vec3_t* out_pos);

/**
 * @brief Estimates velocity at time t.
 *
 * Computes the velocity at time t based on sample positions.
 * - Requires at least 2 samples.
 * - Returns false if insufficient data.
 *
 * @param[in] traj  Trajectory data.
 * @param[in] t     Time (seconds).
 * @param[out] out_vel Output velocity vector (returns false if NULL).
 * @return True if successful.
 */
BYUL_API bool trajectory_estimate_velocity(
    const trajectory_t* traj, float t, vec3_t* out_vel);

/**
 * @brief Estimates acceleration at time t.
 *
 * Approximates acceleration by differentiating velocity between adjacent samples.
 * - Requires at least 3 samples.
 *
 * @param[in] traj  Trajectory data.
 * @param[in] t     Time (seconds).
 * @param[out] out_acc Output acceleration vector (returns false if NULL).
 * @return True if successful.
 */
BYUL_API bool trajectory_estimate_acceleration(
    const trajectory_t* traj, float t, vec3_t* out_acc);

// ---------------------------------------------------------
// Trajectory Output Utilities
// ---------------------------------------------------------

#define TRAJECTORY_STR_BUFSIZE 51200
/**
 * @brief Converts a trajectory_t into a human-readable string.
 *
 * Converts all points (time, position, velocity) of the given trajectory
 * into a formatted string stored in the provided buffer.
 *
 * @param traj    Pointer to trajectory_t (NULL returns "(null)").
 * @param size    Buffer size (recommend TRAJECTORY_STR_BUFSIZE or larger).
 * @param buffer  Buffer to store the string (NULL returns NULL). 
 * @return buffer Pointer to the provided buffer.
 *
 * @note
 * - If traj is NULL, returns "[Trajectory] (null)".
 * - Output may be truncated if the buffer is too small.
 */
BYUL_API char* trajectory_to_string(
    const trajectory_t* traj, size_t size, char* buffer);

/**
 * @brief Prints the trajectory content to console.
 * @param traj Target trajectory.
 */
BYUL_API void trajectory_print(const trajectory_t* traj);

/**
 * @brief Extracts position list from trajectory.
 * @param traj Target trajectory.
 * @param out_list Array to store result.
 * @param max Maximum number of positions to extract.
 * @return Number of extracted positions.
 */
BYUL_API int trajectory_get_positions(
    const trajectory_t* traj, vec3_t* out_list, int max);

/**
 * @brief Extracts speed list from trajectory.
 * @param traj Target trajectory.
 * @param out_list Array to store speed values.
 * @param max Maximum number of speeds to extract.
 * @return Number of extracted speeds.
 */
BYUL_API int trajectory_get_speeds(
    const trajectory_t* traj, float* out_list, int max);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_H
