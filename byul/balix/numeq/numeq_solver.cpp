#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <vector>
#include "numeq_solver.h"
#include "vec3.hpp"
#include "float_common.h"

bool numeq_solve_linear(float a, float b, float* out_x)
{
    if (fabsf(a) < 1e-8f) return false;
    if (out_x) *out_x = -b / a;
    return true;
}

bool numeq_solve_quadratic(float a, float b, float c, 
    float* out_x1, float* out_x2) {

    if (!out_x1 || !out_x2 || float_equal(a,0.0f) ) return false;
    float D = b * b - 4 * a * c;
    if (D < 0.0f) return false;
    float sqrtD = std::sqrt(D);
    *out_x1 = (-b - sqrtD) / (2 * a);
    *out_x2 = (-b + sqrtD) / (2 * a);
    return true;
}

bool numeq_solve_quadratic_stable(float A, float B, float C, 
    float* t0, float* t1)
{
    // Solve A t^2 + B t + C = 0 with improved numerical stability.
    if (fabsf(A) < 1e-12f) return false;
    float D = B*B - 4.0f*A*C;
    if (D < 0.0f) return false;
    float sqrtD = sqrtf(D);

    // Stable form: q = -0.5 * (B + sign(B) * sqrtD)
    float q = -0.5f * (B + (B >= 0.0f ? sqrtD : -sqrtD));
    float r0 = q / A;
    float r1 = (fabsf(q) > 1e-20f) ? (C / q) : r0; // fallback if q very small

    if (r0 <= r1) { *t0 = r0; *t1 = r1; }
    else          { *t0 = r1; *t1 = r0; }
    return true;
}

bool numeq_solve_cubic(float a, float b, float c, float d,
                       float* out_roots, int* out_count) {
    if (!out_roots || !out_count) return false;
    if (fabs(a) < 1e-8f) {

        float x1, x2;
        if (!numeq_solve_quadratic(b, c, d, &x1, &x2)) return false;
        *out_count = (x1 == x2) ? 1 : 2;
        out_roots[0] = x1;
        if (*out_count == 2) out_roots[1] = x2;
        return true;
    }

    float A = b / a;
    float B = c / a;
    float C = d / a;

    float sq_A = A * A;
    float p = (1.0f / 3.0f) * (-1.0f / 3.0f * sq_A + B);
    float q = (1.0f / 2.0f) * (2.0f / 27.0f * A * sq_A - A * B / 3.0f + C);

    float D = q * q + p * p * p;
    int count = 0;

    if (fabs(D) < 1e-8f) {
        if (fabs(q) < 1e-8f) {

            out_roots[0] = -A / 3.0f;
            count = 1;
        } else {

            float u = cbrtf(-q);
            out_roots[0] = 2.0f * u - A / 3.0f;
            out_roots[1] = -u - A / 3.0f;
            count = 2;
        }
    } else if (D < 0.0f) {

        float phi = acosf(-q / sqrtf(-p * p * p));
        float t = 2.0f * sqrtf(-p);
        out_roots[0] = t * cosf(phi / 3.0f) - A / 3.0f;
        out_roots[1] = t * cosf((phi + 2.0f * M_PI) / 3.0f) - A / 3.0f;
        out_roots[2] = t * cosf((phi + 4.0f * M_PI) / 3.0f) - A / 3.0f;
        count = 3;
        std::sort(out_roots, out_roots + 3);
    } else {

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
