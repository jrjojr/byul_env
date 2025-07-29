#include "propulsion.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

void propulsion_init(propulsion_t* p) {
    if (!p) return;

    p->max_thrust = 120.0f;
    p->current_thrust = 0.0f;
    p->fuel_capacity = 50.0f;
    p->fuel_remaining = 50.0f;
    p->burn_rate = 0.05f;

    p->efficiency = 0.7f;
    p->thermal_loss = 0.05f;
    p->energy_density = 42.0f;

    p->response_time = 0.8f;
    p->max_thrust_rate = 30.0f;
    p->delay_time = 0.2f;

    p->heat = 0.0f;
    p->heat_dissipation_rate = 0.3f;
    p->wear_level = 0.0f;

    p->controller = NULL;
    p->active = true;
}

void propulsion_init_full(propulsion_t* p,
                          float max_thrust,
                          float fuel_capacity,
                          float burn_rate,
                          controller_t* ctrl,
                          bool active) {
    if (!p) return;
    propulsion_init(p);
    p->max_thrust = (max_thrust > 0.0f) ? max_thrust : 100.0f;
    p->fuel_capacity = (fuel_capacity > 0.0f) ? fuel_capacity : 100.0f;
    p->fuel_remaining = p->fuel_capacity;
    p->burn_rate = (burn_rate > 0.0f) ? burn_rate : 1.0f;
    p->controller = ctrl;
    p->active = active;
}

void propulsion_assign(propulsion_t* dst, const propulsion_t* src) {
    if (!dst || !src) return;
    memcpy(dst, src, sizeof(propulsion_t));
}

void propulsion_reset(propulsion_t* p) {
    if (!p) return;
    p->current_thrust = 0.0f;
    p->fuel_remaining = p->fuel_capacity;
    p->active = false;
}

void propulsion_update(propulsion_t* p, float target_thrust, float dt) {
    if (!p || dt <= 0.0f) return;

    if (!p->active || p->fuel_remaining <= 0.0f) {
        p->current_thrust = 0.0f;
        p->active = false;
        return;
    }

    if (target_thrust > p->max_thrust) target_thrust = p->max_thrust;
    if (target_thrust < 0.0f) target_thrust = 0.0f;

    float desired_thrust = target_thrust;
    if (p->controller) {
        float control = controller_compute(
            p->controller, target_thrust, p->current_thrust, dt);
        if (control > p->max_thrust) control = p->max_thrust;
        if (control < 0.0f) control = 0.0f;
        desired_thrust = control;
    }

    desired_thrust *= p->efficiency;
    desired_thrust *= (1.0f - p->thermal_loss);
    desired_thrust *= (1.0f - p->wear_level * 0.3f);

    float max_delta = p->max_thrust_rate * dt;
    float delta = desired_thrust - p->current_thrust;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    p->current_thrust += delta;

    float fuel_needed = p->burn_rate * p->current_thrust * dt;
    if (fuel_needed >= p->fuel_remaining) {
        p->current_thrust = p->fuel_remaining / (p->burn_rate * dt);
        p->fuel_remaining = 0.0f;
        p->active = false;
    } else {
        p->fuel_remaining -= fuel_needed;
    }

    p->heat += p->current_thrust * 0.05f;
    p->heat -= p->heat_dissipation_rate * dt;
    if (p->heat < 0.0f) p->heat = 0.0f;

    p->wear_level += 0.0001f * p->current_thrust * dt;
    if (p->wear_level > 1.0f) p->wear_level = 1.0f;
}

float propulsion_get_thrust(const propulsion_t* p) {
    return (p && p->active) ? p->current_thrust : 0.0f;
}

bool propulsion_is_empty(const propulsion_t* p) {
    return (!p || p->fuel_remaining <= 0.0f);
}

float propulsion_get_fuel_ratio(const propulsion_t* p) {
    if (!p || p->fuel_capacity <= 0.0f) return 0.0f;
    return p->fuel_remaining / p->fuel_capacity;
}

float propulsion_get_max_runtime(const propulsion_t* p) {
    if (!p || p->current_thrust <= 0.0f || p->burn_rate <= 0.0f) return 0.0f;
    return p->fuel_remaining / (p->burn_rate * p->current_thrust);
}

float propulsion_get_remaining_impulse(const propulsion_t* p) {
    return (p && p->burn_rate > 0.0f) ? p->fuel_remaining / p->burn_rate : 0.0f;
}

void propulsion_refuel(propulsion_t* p, float amount) {
    if (!p || amount <= 0.0f) return;
    p->fuel_remaining += amount;
    if (p->fuel_remaining > p->fuel_capacity)
        p->fuel_remaining = p->fuel_capacity;
}

void propulsion_consume(propulsion_t* p, float amount) {
    if (!p || amount <= 0.0f) return;
    p->fuel_remaining -= amount;
    if (p->fuel_remaining < 0.0f) {
        p->fuel_remaining = 0.0f;
        p->active = false;
    }
}

float propulsion_predict_runtime(const propulsion_t* p, float desired_thrust) {
    if (!p || desired_thrust <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    return p->fuel_remaining / (p->burn_rate * desired_thrust);
}

float propulsion_predict_empty_time(const propulsion_t* p) {
    if (!p || p->current_thrust <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    return p->fuel_remaining / (p->burn_rate * p->current_thrust);
}

float propulsion_predict_max_thrust(const propulsion_t* p, float duration) {
    if (!p || duration <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    float possible = p->fuel_remaining / (p->burn_rate * duration);
    return (possible > p->max_thrust) ? p->max_thrust : possible;
}

void propulsion_set_active(propulsion_t* p, bool active) {
    if (p) p->active = active;
}

void propulsion_attach_controller(propulsion_t* p, controller_t* ctrl) {
    if (p) p->controller = ctrl;
}

void propulsion_detach_controller(propulsion_t* p) {
    if (p) p->controller = NULL;
}

void propulsion_print(const propulsion_t* p) {
    if (!p) {
        printf("propulsion: (null)\n");
        return;
    }
    printf("Thrust=%.2fN, Fuel=%.2f/%.2fkg, Active=%d\n",
           p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
}

const char* propulsion_to_string(
    const propulsion_t* p, char* buffer, size_t buffer_size) {
    if (!p || !buffer || buffer_size == 0) return NULL;
    snprintf(buffer, buffer_size,
             "Thrust=%.2fN, Fuel=%.2f/%.2fkg, Active=%d",
             p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
    return buffer;
}

const char* propulsion_to_json(
    const propulsion_t* p, char* buffer, size_t buffer_size) {
    if (!p || !buffer || buffer_size == 0) return NULL;
    snprintf(buffer, buffer_size,
             "{\"thrust\":%.2f,\"fuel\":%.2f,\"capacity\":%.2f,\"active\":%d}",
             p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
    return buffer;
}
