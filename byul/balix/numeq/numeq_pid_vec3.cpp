#include "internal/numeq_pid_vec3.h"

// ---------------------------------------------------------
// 벡터 PID
// ---------------------------------------------------------

void pid_vec3_init(pid_controller_vec3_t* pid) {
    pid_init(&pid->x);
    pid_init(&pid->y);
    pid_init(&pid->z);
}


void pid_vec3_init_full(pid_controller_vec3_t* pid,
                   float kp, float ki, float kd,
                   float dt) {
    pid_init_full(&pid->x, kp, ki, kd, dt);
    pid_init_full(&pid->y, kp, ki, kd, dt);
    pid_init_full(&pid->z, kp, ki, kd, dt);
}

void pid_vec3_reset(pid_controller_vec3_t* pid) {
    pid_reset(&pid->x);
    pid_reset(&pid->y);
    pid_reset(&pid->z);
}

void pid_vec3_set_state(pid_controller_vec3_t* pid,
                        const vec3_t* integral,
                        const vec3_t* prev_error) {
    pid_set_state(&pid->x, integral->x, prev_error->x);
    pid_set_state(&pid->y, integral->y, prev_error->y);
    pid_set_state(&pid->z, integral->z, prev_error->z);
}

void pid_vec3_update(pid_controller_vec3_t* pid,
                     const vec3_t* target,
                     const vec3_t* measured,
                     vec3_t* out_control) {
    out_control->x = pid_update(&pid->x, target->x, measured->x);
    out_control->y = pid_update(&pid->y, target->y, measured->y);
    out_control->z = pid_update(&pid->z, target->z, measured->z);
}

void pid_vec3_preview(const pid_controller_vec3_t* pid,
                      const vec3_t* target,
                      const vec3_t* measured,
                      vec3_t* out_control) {
    out_control->x = pid_preview(&pid->x, target->x, measured->x);
    out_control->y = pid_preview(&pid->y, target->y, measured->y);
    out_control->z = pid_preview(&pid->z, target->z, measured->z);
}

void pid_vec3_assign(pid_controller_vec3_t* dst,
                   const pid_controller_vec3_t* src) {
    pid_assign(&dst->x, &src->x);
    pid_assign(&dst->y, &src->y);
    pid_assign(&dst->z, &src->z);
}
