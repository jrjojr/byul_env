#include "float_common.h"
#include <cmath>

int float_compare(float a, float b, void*) {
    return float_equal(a, b) ? 0 : (a < b ? -1 : 1);
}

int int_compare(int a, int b, void*) {
    return (a > b) - (a < b);
}

bool float_equal(float a, float b) {
    if (a == b) return true;
    float diff = std::fabs(a - b);
    float largest = std::fmax(std::fabs(a), std::fabs(b));
    return diff <= FLOAT_EPSILON * largest;
}

bool float_equal_tol(float a, float b, float tol) {
    if (tol < 0.0f) tol = -tol; // convert negative tolerance to positive
    return std::fabs(a - b) <= tol;
}

bool float_equal_tol_all(
    float a, float b, float tol_pos, float tol_neg) {

    // convert negative tolerance to positive
    if (tol_pos < 0.0f) tol_pos = -tol_pos;
    if (tol_neg < 0.0f) tol_neg = -tol_neg;

    float diff = b - a;
    if (diff >= 0.0f) {
        return diff <= tol_pos;  // positive direction tolerance
    } else {
        return -diff <= tol_neg; // negative direction tolerance
    }
}

bool float_zero(float x) {
    return std::fabs(x) <= FLOAT_EPSILON_TINY;
}

float float_safe_div(float a, float b, float fallback) {
    return float_zero(b) ? fallback : (a / b);
}

// ---------------------------------------------------------
// Basic math functions
// ---------------------------------------------------------

float float_square(float x) {
    return x * x;
}

float float_clamp(float x, float min_val, float max_val) {
    return (x < min_val) ? min_val : ((x > max_val) ? max_val : x);
}

float float_sign(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

float float_deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float float_rad2deg(float rad) {
    return rad * (180.0f / M_PI);
}

// ---------------------------------------------------------
// Linear interpolation / normalization / range conversion
// ---------------------------------------------------------

float float_lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

float float_inv_lerp(float a, float b, float value) {
    return float_zero(b - a) ? 0.0f : (value - a) / (b - a);
}

float float_remap(float in_min, float in_max, 
    float out_min, float out_max, float value) {

    float t = float_inv_lerp(in_min, in_max, value);
    return float_lerp(out_min, out_max, t);
}

float float_clamp01(float x) {
    return float_clamp(x, 0.0f, 1.0f);
}

float float_smoothstep(float edge0, float edge1, float x) {
    float t = float_clamp01(float_inv_lerp(edge0, edge1, x));
    return t * t * (3.0f - 2.0f * t);
}
