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
    if (tol < 0.0f) tol = -tol; // tol이 음수면 양수로 변환
    return fabsf(a - b) <= tol;
}

bool float_equal_tol_all(
    float a, float b, float tol_pos, float tol_neg) {

    // 잘못된 부호가 들어와도 양수로 변환
    if (tol_pos < 0.0f) tol_pos = -tol_pos;
    if (tol_neg < 0.0f) tol_neg = -tol_neg;

    float diff = b - a;
    if (diff >= 0.0f) {
        return diff <= tol_pos;  // 양수 방향 허용 범위
    } else {
        return -diff <= tol_neg; // 음수 방향 허용 범위
    }
}

bool float_zero(float x) {
    return fabsf(x) <= FLOAT_EPSILON;
}


float float_safe_div(float a, float b, float fallback) {
    return float_zero(b) ? fallback : (a / b);
}

// ---------------------------------------------------------
// 기본 수학 함수
// ---------------------------------------------------------

float square(float x) {
    return x * x;
}

float clamp(float x, float min_val, float max_val) {
    return (x < min_val) ? min_val : ((x > max_val) ? max_val : x);
}

float sign(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float rad2deg(float rad) {
    return rad * (180.0f / M_PI);
}

// ---------------------------------------------------------
// 선형 보간 / 정규화 / 범위 변환
// ---------------------------------------------------------

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

float inv_lerp(float a, float b, float value) {
    return float_zero(b - a) ? 0.0f : (value - a) / (b - a);
}

float remap(float in_min, float in_max, 
    float out_min, float out_max, float value) {

    float t = inv_lerp(in_min, in_max, value);
    return lerp(out_min, out_max, t);
}

float clamp01(float x) {
    return clamp(x, 0.0f, 1.0f);
}

float smoothstep(float edge0, float edge1, float x) {
    float t = clamp01(inv_lerp(edge0, edge1, x));
    return t * t * (3.0f - 2.0f * t);
}
