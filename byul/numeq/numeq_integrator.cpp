#include "internal/numeq_integrator.h"
#include "internal/vec3.hpp"
#include <cassert>

// ---------------------------------------------------------
// 오일러 방식 적분 (Euler)
// ---------------------------------------------------------
void numeq_integrate_euler(linear_state_t* state,
                           const vec3_t* accel,
                           float dt) {
    Vec3 v(state->velocity);
    Vec3 p(state->position);
    Vec3 a(*accel);

    Vec3 v_next = v + a * dt;
    Vec3 p_next = p + v * dt;

    state->velocity = v_next;
    state->position = p_next;
    state->acceleration = a;
}

// ---------------------------------------------------------
// 세미-묵시적 오일러 (Semi-Implicit Euler)
// ---------------------------------------------------------
void numeq_integrate_semi_implicit(linear_state_t* state,
                                   const vec3_t* accel,
                                   float dt) {
    Vec3 a(*accel);
    Vec3 v = Vec3(state->velocity) + a * dt;
    Vec3 p = Vec3(state->position) + v * dt;

    state->velocity = v;
    state->position = p;
    state->acceleration = a;
}

// ---------------------------------------------------------
// Verlet 적분 (과거 위치 필요)
// ---------------------------------------------------------
void numeq_integrate_verlet(vec3_t* position,
                            vec3_t* prev_position,
                            const vec3_t* accel,
                            float dt) {
    assert(position && prev_position && accel);

    Vec3 p(*position);
    Vec3 p_prev(*prev_position);
    Vec3 a(*accel);

    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    *prev_position = p;
    *position = new_pos;
}

// ---------------------------------------------------------
// 4차 Runge-Kutta 적분 (RK4)
// ---------------------------------------------------------
void numeq_integrate_rk4(linear_state_t* state,
                         const vec3_t* accel,
                         float dt) {
    Vec3 v0(state->velocity);
    Vec3 a0(*accel);

    Vec3 k1_v = a0 * dt;
    Vec3 k1_p = v0 * dt;

    Vec3 k2_v = a0 * dt;
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;

    Vec3 k3_v = a0 * dt;
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;

    Vec3 k4_v = a0 * dt;
    Vec3 k4_p = (v0 + k3_v) * dt;

    Vec3 delta_v = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);
    Vec3 delta_p = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);

    state->velocity = v0 + delta_v;
    state->position = Vec3(state->position) + delta_p;
    state->acceleration = a0;
}

// ---------------------------------------------------------
// 공통 적분기 인터페이스
// ---------------------------------------------------------
void numeq_integrate(linear_state_t* state,
                     const vec3_t* accel,
                     const integrator_config_t* config) {
    switch (config->type) {
        case INTEGRATOR_EULER:
            numeq_integrate_euler(state, accel, config->time_step);
            break;
        case INTEGRATOR_SEMI_IMPLICIT:
            numeq_integrate_semi_implicit(state, accel, config->time_step);
            break;
        case INTEGRATOR_RK4:
            numeq_integrate_rk4(state, accel, config->time_step);
            break;
        default:
            // Verlet는 prev_position이 별도로 관리되어야 하므로 여기서는 제외
            assert(false && "Verlet integration requires explicit prev_position");
            break;
    }
}
