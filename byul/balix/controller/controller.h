#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include "numeq.h"

#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Controller Types
// ---------------------------------------------------------
typedef enum e_controller_type {
    CONTROLLER_NONE = 0,   /**< No control */
    CONTROLLER_BANGBANG,   /**< Bang-Bang control */
    CONTROLLER_PID,        /**< PID control */
    CONTROLLER_MPC         /**< Model Predictive Control */
} controller_type_t;

// ---------------------------------------------------------
// Common Controller Interface
// ---------------------------------------------------------
typedef struct s_controller controller_t;

typedef float (*controller_compute_func)(
    controller_t* ctrl, float target, float measured, float dt);

typedef void (*controller_reset_func)(controller_t* ctrl);

// ---------------------------------------------------------
// Controller Structures
// ---------------------------------------------------------

// PID implementation
typedef struct s_pid_impl {
    pid_controller_t pid;
    float output_limit;
} pid_impl_t;

/**
 * @brief Initialize pid_impl_t with default values.
 *
 * Initializes PID implementation (`pid_impl_t`) with default settings.
 *
 * @param impl Pointer to the pid_impl_t structure to initialize.
 *
 * @note Internally calls `pid_init()` to initialize the PID controller.
 */
BYUL_API void pid_impl_init(pid_impl_t* impl);

/**
 * @brief Initialize pid_impl_t with specified values.
 *
 * Initializes `pid_impl_t` with user-specified PID parameters (Kp, Ki, Kd, dt) 
 * and output limit.
 *
 * @param impl Pointer to the pid_impl_t structure to initialize.
 * @param kp Proportional gain (recommended range: 0.0 ~ 10.0).
 * @param ki Integral gain (recommended range: 0.0 ~ 1.0).
 * @param kd Derivative gain (recommended range: 0.0 ~ 1.0).
 * @param dt Control period (sec) (recommended range: 0.001 ~ 0.1).
 * @param output_limit Output limit (no limit if <= 0).
 *
 * @note Internally calls `pid_init_full()` to configure the PID controller.
 */
BYUL_API void pid_impl_init_full(pid_impl_t* impl,
                                 float kp, float ki, float kd,
                                 float dt, float output_limit);

/**
 * @brief Copy pid_impl_t state.
 *
 * Deep copies PID parameters and internal states from `src` to `dst`.
 *
 * @param dst Destination pid_impl_t.
 * @param src Source pid_impl_t.
 *
 * @note Both `dst` and `src` must be valid pointers.
 */
BYUL_API void pid_impl_assign(pid_impl_t* dst, const pid_impl_t* src);

// Bang-Bang implementation
typedef struct s_bangbang_impl {
    float max_output;
} bangbang_impl_t;

/**
 * @brief Initialize bangbang_impl_t with default values.
 *
 * Initializes Bang-Bang controller with default settings.
 * - max_output = 1.0f
 *
 * @param impl Pointer to the bangbang_impl_t structure to initialize.
 *
 * @note Bang-Bang control outputs +max_output if below target,
 *       or -max_output if above target.
 */
BYUL_API void bangbang_impl_init(bangbang_impl_t* impl);

/**
 * @brief Initialize bangbang_impl_t with specified values.
 *
 * Sets the user-defined maximum output (max_output) for the Bang-Bang controller.
 *
 * @param impl Pointer to the bangbang_impl_t structure to initialize.
 * @param max_output Maximum output value (recommended: >= 0.0).
 *
 * @note max_output is used as the maximum absolute output in both directions.
 */
BYUL_API void bangbang_impl_init_full(bangbang_impl_t* impl, float max_output);

/**
 * @brief Copy bangbang_impl_t state.
 *
 * Copies the `max_output` value from `src` to `dst`.
 *
 * @param dst Destination bangbang_impl_t.
 * @param src Source bangbang_impl_t.
 *
 * @note Both `dst` and `src` must be valid pointers.
 */
BYUL_API void bangbang_impl_assign(bangbang_impl_t* dst,
                                   const bangbang_impl_t* src);

// MPC implementation
typedef struct s_mpc_impl {
    mpc_config_t config;   // MPC configuration
    motion_state_t target; // Target velocity/position (x-axis based)
    environ_t env;         // Environment information (if required)
    bodyprops_t body;      // Physical properties (mass, etc.)
    mpc_cost_func cost_fn;
} mpc_impl_t;

/**
 * @brief Initialize mpc_impl_t with default values.
 *
 * Initializes MPC (Model Predictive Control) with default settings:
 * - `mpc_config_t` is initialized via `mpc_config_init()`.
 * - `motion_state_t` is initialized via `motion_state_init()`.
 * - `environ_t` is initialized via `environment_init()`.
 * - `bodyprops_t` is initialized via `body_properties_init()`.
 * - `cost_fn` is set to `numeq_mpc_cost_default`.
 *
 * @param impl Pointer to the mpc_impl_t structure to initialize.
 *
 * @note This function initializes all internal structures with default values.
 */
BYUL_API void mpc_impl_init(mpc_impl_t* impl);

/**
 * @brief Initialize mpc_impl_t with specified values.
 *
 * Initializes `mpc_impl_t` with user-specified MPC settings (config),
 * target state (target), environment (env), body properties (body),
 * and cost function (cost_fn).
 *
 * @param impl Pointer to the mpc_impl_t structure to initialize.
 * @param cfg   MPC configuration (NULL uses default values).
 * @param target Target motion state (NULL uses default values).
 * @param env Environment information (NULL uses default values).
 * @param body Physical properties (NULL uses default values).
 * @param cost_fn Cost function pointer (NULL sets numeq_mpc_cost_default).
 *
 * @note Any NULL parameter is replaced by its default initialization.
 */
BYUL_API void mpc_impl_init_full(mpc_impl_t* impl,
                                 const mpc_config_t* cfg,
                                 const motion_state_t* target,
                                 const environ_t* env,
                                 const bodyprops_t* body,
                                 mpc_cost_func cost_fn);

/**
 * @brief Copy mpc_impl_t state.
 *
 * Deep copies MPC configuration, target state, environment, body properties,
 * and cost function pointer from `src` to `dst`.
 *
 * @param dst Destination mpc_impl_t.
 * @param src Source mpc_impl_t.
 *
 * @note Both `dst` and `src` must be valid pointers.
 */
BYUL_API void mpc_impl_assign(mpc_impl_t* dst, const mpc_impl_t* src);

struct s_controller {
    controller_type_t type;          /**< Controller type */
    void* impl;                      /**< Implementation data (PID, MPC, etc.) */
    controller_compute_func compute; /**< Control output calculation function */
    controller_reset_func reset;     /**< Reset internal states */
    void* userdata;                  /**< User-defined data (optional) */
};

// ---------------------------------------------------------
// Controller Creation / Initialization
// ---------------------------------------------------------

/**
 * @brief Create a PID controller.
 */
BYUL_API controller_t* controller_create_pid(
    float kp, float ki, float kd, float dt, float output_limit);

/**
 * @brief Create a Bang-Bang controller.
 * @param max_output Maximum output.
 */
BYUL_API controller_t* controller_create_bangbang(float max_output);

/**
 * @brief Create an MPC controller (Model Predictive Control).
 */
BYUL_API controller_t* controller_create_mpc(
    const mpc_config_t* config,
    const environ_t* env,
    const bodyprops_t* body);

/**
 * @brief Destroy a controller.
 */
BYUL_API void controller_destroy(controller_t* ctrl);

// ---------------------------------------------------------
// Common Controller Functions
// ---------------------------------------------------------

/**
 * @brief Compute control output.
 * @param ctrl Controller.
 * @param target Target value.
 * @param measured Current measured value.
 * @param dt Time step.
 * @return Control output.
 */
static inline float controller_compute(
    controller_t* ctrl, float target, float measured, float dt) {
    return (ctrl && ctrl->compute) ? 
        ctrl->compute(ctrl, target, measured, dt) : 0.0f;
}

/**
 * @brief Reset controller state.
 */
static inline void controller_reset(controller_t* ctrl) {
    if (ctrl && ctrl->reset) ctrl->reset(ctrl);
}

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H
