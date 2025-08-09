#ifndef BYUL_TICK_H
#define BYUL_TICK_H

#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function pointer type for tick callbacks.
 * Each tick function is called with its registered context and delta time.
 */
typedef void (*tick_func)(void* context, float dt);

/**
 * @brief Entry in the tick list.
 */
typedef struct s_tick_entry {
    tick_func func;       ///< The function to call.
    void* context;        ///< The context to pass to the function.
} tick_entry_t;

/**
 * @brief Opaque tick system that manages all registered tick functions.
 */
typedef struct s_tick tick_t;

BYUL_API tick_t* tick_create();

BYUL_API void tick_destroy(tick_t* tick);

/**
 * @brief Calls all registered tick functions with the given delta time.
 * @param tick Tick system handle
 * @param dt Delta time (in seconds)
 */
BYUL_API void tick_update(tick_t* tick, float dt);

/**
 * @brief Registers a tick function with an associated context.
 * @param tick Tick system handle
 * @param func Tick function to call each frame
 * @param context The context passed to the function on each call
 * @return 0 on success, non-zero on duplicate or error
 */
BYUL_API int tick_attach(tick_t* tick, tick_func func, void* context);

/**
 * @brief Unregisters a specific tick function and context pair.
 * @param tick Tick system handle
 * @param func Function to detach
 * @param context Context that was originally registered with the function
 * @return 0 on success, non-zero if not found
 */
BYUL_API int tick_detach(tick_t* tick, tick_func func, void* context);

/**
 * @brief Lists all currently attached tick functions.
 * @param tick Tick system handle
 * @param out Array to store tick_entry_t entries
 * @param max_count Maximum number of entries to write
 * @return Actual number of entries written
 */
BYUL_API int tick_list_attached(tick_t* tick, tick_entry_t* out, int max_count);

#ifdef __cplusplus
}
#endif

#endif // BYUL_TICK_H
