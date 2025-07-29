#include "motion_state.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>

void linear_state_init(linear_state_t* out) {
    if (!out) return;
    vec3_zero(&out->position);
    vec3_zero(&out->velocity);
    vec3_zero(&out->acceleration);
}

void linear_state_init_full(linear_state_t* out,
                            const vec3_t* position,
                            const vec3_t* velocity,
                            const vec3_t* acceleration) {
    if (!out) return;
    out->position = *position;
    out->velocity = *velocity;
    out->acceleration = *acceleration;
}

void linear_state_assign(linear_state_t* out, const linear_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void attitude_state_init(attitude_state_t* out) {
    if (!out) return;
    quat_identity(&out->orientation);
    vec3_zero(&out->angular_velocity);
    vec3_zero(&out->angular_acceleration);
}

void attitude_state_init_full(attitude_state_t* out,
                              const quat_t* orientation,
                              const vec3_t* angular_velocity,
                              const vec3_t* angular_acceleration) {
    if (!out) return;
    out->orientation = *orientation;
    out->angular_velocity = *angular_velocity;
    out->angular_acceleration = *angular_acceleration;
}

void attitude_state_assign(attitude_state_t* out, const attitude_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void motion_state_init(motion_state_t* out) {
    if (!out) return;
    linear_state_init(&out->linear);
    attitude_state_init(&out->angular);
}

void motion_state_init_full(motion_state_t* out,
                            const vec3_t* position,
                            const vec3_t* velocity,
                            const vec3_t* acceleration,
                            const quat_t* orientation,
                            const vec3_t* angular_velocity,
                            const vec3_t* angular_acceleration) {
    if (!out) return;
    linear_state_init_full(&out->linear, position, velocity, acceleration);
    attitude_state_init_full(&out->angular, 
        orientation, angular_velocity, angular_acceleration);
}

void motion_state_assign(motion_state_t* out, const motion_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}
