#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <vector>
#include "internal/numeq_solver.h"
#include "internal/vec3.hpp"

// -----------------------------
// 1. 수학 해석 함수
// -----------------------------

bool numeq_solve_quadratic(float a, float b, float c, 
    float* out_x1, float* out_x2) {

    if (!out_x1 || !out_x2 || a == 0.0f) return false;
    float D = b * b - 4 * a * c;
    if (D < 0.0f) return false;
    float sqrtD = std::sqrt(D);
    *out_x1 = (-b - sqrtD) / (2 * a);
    *out_x2 = (-b + sqrtD) / (2 * a);
    return true;
}

bool numeq_solve_bisection(numeq_func_f32 func, void* userdata,
                           float a, float b, float tol, float* out_root) {
    if (!func || !out_root || a >= b) return false;
    float fa = func(a, userdata);
    float fb = func(b, userdata);
    if (fa * fb > 0.0f) return false;

    for (int i = 0; i < 100; ++i) {
        float mid = 0.5f * (a + b);
        float fmid = func(mid, userdata);
        if (std::fabs(fmid) < tol || (b - a) < tol) {
            *out_root = mid;
            return true;
        }
        if (fa * fmid < 0.0f) {
            b = mid;
            fb = fmid;
        } else {
            a = mid;
            fa = fmid;
        }
    }
    *out_root = 0.5f * (a + b);
    return true;
}

// -----------------------------
// 2. 물리 해석 함수
// -----------------------------

bool numeq_solve_time_for_y(const linear_state_t* s, 
    float target_y, float* out_time) {

    if (!s || !out_time) return false;
    float a = 0.5f * s->acceleration.y;
    float b = s->velocity.y;
    float c = s->position.y - target_y;
    float t1, t2;
    if (!numeq_solve_quadratic(a, b, c, &t1, &t2)) return false;
    *out_time = std::min(t1, t2) > 0 ? std::min(t1, t2) : std::max(t1, t2);
    return *out_time >= 0.0f;
}

bool numeq_solve_time_for_position(const linear_state_t* state,
                                   const vec3_t* target_pos,
                                   float tolerance,
                                   float max_time,
                                   float* out_time) {
    if (!state || !target_pos || !out_time) return false;

    const int steps = 100;
    float best_t = 0.0f;
    float best_d = std::numeric_limits<float>::max();

    for (int i = 0; i <= steps; ++i) {
        float t = max_time * i / steps;
        Vec3 p = Vec3(state->position) +
                 Vec3(state->velocity) * t +
                 Vec3(state->acceleration) * (0.5f * t * t);

        float dx = p.v.x - target_pos->x;
        float dz = p.v.z - target_pos->z;
        float dist = std::sqrt(dx * dx + dz * dz);

        if (dist < best_d) {
            best_d = dist;
            best_t = t;
        }

        if (dist < tolerance) break;
    }

    *out_time = best_t;
    return true;
}


bool numeq_solve_velocity_for_range(float d, float g, float* out_v) {
    if (!out_v || d <= 0.0f || g <= 0.0f) return false;
    *out_v = std::sqrt(d * g);
    return true;
}

bool numeq_solve_apex(const linear_state_t* s, 
    vec3_t* out_pos, float* out_time) {

    if (!s || !out_pos || !out_time || s->acceleration.y == 0.0f) 
        return false;

    float t = -s->velocity.y / s->acceleration.y;
    *out_time = t;
    Vec3 apex = Vec3(s->position) + Vec3(
        s->velocity) * t + Vec3(s->acceleration) * (0.5f * t * t);

    *out_pos = apex.v;
    return true;
}

bool numeq_solve_stop_time(const linear_state_t* s, 
    float tol, float* out_time) {

    if (!s || !out_time) return false;
    float v = Vec3(s->velocity).length();
    float a = Vec3(s->acceleration).length();
    if (a <= 0.0f) return false;
    *out_time = v / a;
    return true;
}

bool numeq_solve_time_for_vec3(numeq_vec3_func func,
                               void* userdata,
                               const vec3_t* target,
                               float t_min, float t_max,
                               float tol,
                               float* out_t) {
    if (!func || !target || !out_t || t_min >= t_max) return false;

    float best_t = t_min;
    float best_dist = std::numeric_limits<float>::max();

    const int steps = 100;
    for (int i = 0; i <= steps; ++i) {
        float t = t_min + (t_max - t_min) * i / steps;
        Vec3 p;
        Vec3 vec_t = Vec3(target->x, target->y, target->z);
        func(t, &p.v, userdata);
        float d = Vec3(p - vec_t).length();
        if (d < best_dist) {
            best_dist = d;
            best_t = t;
        }
        if (d < tol) break;
    }

    *out_t = best_t;
    return true;
}
