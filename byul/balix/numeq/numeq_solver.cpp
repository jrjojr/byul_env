#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <vector>
#include "internal/numeq_solver.h"
#include "internal/vec3.hpp"
#include "internal/common.h"

// -----------------------------
// 1. 수학 해석 함수
// -----------------------------
bool numeq_solve_linear(float a, float b, float* out_x)
{
    if (fabsf(a) < 1e-8f) return false;  // a = 0이면 해 없음
    if (out_x) *out_x = -b / a;
    return true;
}

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

// -----------------------------
// 1.x 3차 방정식 해석 함수
// -----------------------------
bool numeq_solve_cubic(float a, float b, float c, float d,
                       float* out_roots, int* out_count) {
    if (!out_roots || !out_count) return false;
    if (fabs(a) < 1e-8f) {
        // 3차항이 0이면 2차방정식으로 처리
        float x1, x2;
        if (!numeq_solve_quadratic(b, c, d, &x1, &x2)) return false;
        *out_count = (x1 == x2) ? 1 : 2;
        out_roots[0] = x1;
        if (*out_count == 2) out_roots[1] = x2;
        return true;
    }

    // 표준화: x = y - b/(3a)
    float A = b / a;
    float B = c / a;
    float C = d / a;

    float sq_A = A * A;
    float p = (1.0f / 3.0f) * (-1.0f / 3.0f * sq_A + B);
    float q = (1.0f / 2.0f) * (2.0f / 27.0f * A * sq_A - A * B / 3.0f + C);

    float D = q * q + p * p * p; // 판별식
    int count = 0;

    if (fabs(D) < 1e-8f) {
        if (fabs(q) < 1e-8f) {
            // 삼중근
            out_roots[0] = -A / 3.0f;
            count = 1;
        } else {
            // 중근
            float u = cbrtf(-q);
            out_roots[0] = 2.0f * u - A / 3.0f;
            out_roots[1] = -u - A / 3.0f;
            count = 2;
        }
    } else if (D < 0.0f) {
        // 3개의 실근
        float phi = acosf(-q / sqrtf(-p * p * p));
        float t = 2.0f * sqrtf(-p);
        out_roots[0] = t * cosf(phi / 3.0f) - A / 3.0f;
        out_roots[1] = t * cosf((phi + 2.0f * M_PI) / 3.0f) - A / 3.0f;
        out_roots[2] = t * cosf((phi + 4.0f * M_PI) / 3.0f) - A / 3.0f;
        count = 3;
        std::sort(out_roots, out_roots + 3);
    } else {
        // 1개의 실근
        float sqrt_D = sqrtf(D);
        float u = cbrtf(-q + sqrt_D);
        float v = cbrtf(-q - sqrt_D);
        out_roots[0] = u + v - A / 3.0f;
        count = 1;
    }

    *out_count = count;
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

