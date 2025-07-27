/* internal/vec3.cpp */
#include "internal/vec3.h"
#include "internal/common.h"
#include <cmath>
#include <new>
#include <stdio.h>
#include <cstring>

// (x, y, z)로 초기화
void vec3_init_full(vec3_t* out, float x, float y, float z) {
    if (!out) return;
    out->x = x;
    out->y = y;
    out->z = z;
}

// (0, 0, 0)으로 초기화
void vec3_init(vec3_t* out) {
    vec3_init_full(out, 0.0f, 0.0f, 0.0f);
}

// src 벡터를 out에 복사
void vec3_assign(vec3_t* out, const vec3_t* src) {
    if (!out || !src) return;
    *out = *src;
}

bool vec3_equal(const vec3_t* a, const vec3_t* b) {
    if (!a || !b) return false;
    return float_equal(a->x, b->x) &&
           float_equal(a->y, b->y) &&
           float_equal(a->z, b->z);
}

/* ---------------------------------------------------------
// vec3_equal_tol
// ---------------------------------------------------------
/**
 * @brief vec3의 각 성분을 동일한 공차(tolerance)로 비교합니다.
 *
 * @param a   첫 번째 벡터
 * @param b   두 번째 벡터
 * @param tol 허용 오차 (모든 x,y,z에 동일하게 적용)
 * @return 모든 성분 차이가 tol 이하이면 true
 */
bool vec3_equal_tol(const vec3_t* a, const vec3_t* b, float tol)
{
    if (!a || !b) return false;
    return float_equal_tol(a->x, b->x, tol) &&
           float_equal_tol(a->y, b->y, tol) &&
           float_equal_tol(a->z, b->z, tol);
}

// ---------------------------------------------------------
// vec3_equal_tol_all
// ---------------------------------------------------------
/**
 * @brief vec3의 각 성분을 양수/음수 방향의 개별 공차로 비교합니다.
 *
 * @param a        첫 번째 벡터
 * @param b        두 번째 벡터
 * @param tol_pos  양수 방향 공차 (b >= a일 때 허용 오차)
 * @param tol_neg  음수 방향 공차 (b < a일 때 허용 오차)
 * @return 모든 성분 차이가 개별 공차 내이면 true
 */
bool vec3_equal_tol_all(
    const vec3_t* a, const vec3_t* b,
    float tol_pos, float tol_neg)
{
    if (!a || !b) return false;
    return float_equal_tol_all(a->x, b->x, tol_pos, tol_neg) &&
           float_equal_tol_all(a->y, b->y, tol_pos, tol_neg) &&
           float_equal_tol_all(a->z, b->z, tol_pos, tol_neg);
}

uint32_t vec3_hash(const vec3_t* v) {
    if (!v) return 0;
    const uint32_t* data = reinterpret_cast<const uint32_t*>(v);
    uint32_t hash = 2166136261u;
    for (int i = 0; i < 3; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

void vec3_zero(vec3_t* out) {
    out->x = out->y = out->z = 0.0f;
}

void vec3_negate(vec3_t* out, const vec3_t* a) {
    out->x = -a->x;
    out->y = -a->y;
    out->z = -a->z;
}

void vec3_add(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}

void vec3_sub(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

void vec3_mul(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    out->x = a->x * b->x;
    out->y = a->y * b->y;
    out->z = a->z * b->z;
}

void vec3_div(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;

    out->x = (fabsf(b->x) > FLOAT_EPSILON) ? (a->x / b->x) : INFINITY;
    out->y = (fabsf(b->y) > FLOAT_EPSILON) ? (a->y / b->y) : INFINITY;
    out->z = (fabsf(b->z) > FLOAT_EPSILON) ? (a->z / b->z) : INFINITY;

#ifdef DEBUG
    if (b->x == 0.0f || b->y == 0.0f || b->z == 0.0f) {
        fprintf(stderr, "[vec3_div] Division by zero detected! Returning INF where applicable.\n");
    }
#endif
}

void vec3_div_scalar(vec3_t* out, const vec3_t* a, float scalar) {
    if (!out || !a) return;

    if (scalar == 0.0f) {
#ifdef DEBUG
        fprintf(stderr, "[vec3_div_scalar] Division by zero! Returning INF vector.\n");
#endif
        *out = (vec3_t){ INFINITY, INFINITY, INFINITY };
        return;
    }

    out->x = a->x / scalar;
    out->y = a->y / scalar;
    out->z = a->z / scalar;
}

void vec3_scale(vec3_t* out, const vec3_t* a, float s) {
    out->x = a->x * s;
    out->y = a->y * s;
    out->z = a->z * s;
}

float vec3_dot(const vec3_t* a, const vec3_t* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3_cross(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

float vec3_length(const vec3_t* a) {
    return std::sqrt(vec3_dot(a, a));
}

float vec3_length_sq(const vec3_t* a) {
    if (!a) return 0.0f;
    return a->x * a->x + a->y * a->y + a->z * a->z;
}

void vec3_normalize(vec3_t* a) {
    float len = vec3_length(a);
    if (len > 0.0f) {
        a->x /= len;
        a->y /= len;
        a->z /= len;
    } else {
        vec3_zero(a);
    }
}

void vec3_unit(vec3_t* out, const vec3_t* src) {
    float len_sq = src->x * src->x + src->y * src->y + src->z * src->z;
    if (len_sq > 1e-8f) { // 길이가 0에 가까운지 체크
        float inv_len = 1.0f / std::sqrt(len_sq);
        out->x = src->x * inv_len;
        out->y = src->y * inv_len;
        out->z = src->z * inv_len;
    } else {
        // 길이가 0이라면 0 벡터 반환
        out->x = 0.0f;
        out->y = 0.0f;
        out->z = 0.0f;
    }
}


float vec3_distance(const vec3_t* a, const vec3_t* b) {
    vec3_t d;
    vec3_sub(&d, a, b);
    return vec3_length(&d);
}

void vec3_lerp(vec3_t* out, const vec3_t* a, const vec3_t* b, float t) {
    out->x = a->x + (b->x - a->x) * t;
    out->y = a->y + (b->y - a->y) * t;
    out->z = a->z + (b->z - a->z) * t;
}

void vec3_to_mat4(const vec3_t* v, float out[16]) {
    std::memset(out, 0, sizeof(float) * 16);
    out[0] = out[5] = out[10] = out[15] = 1.0f;
    out[12] = v->x;
    out[13] = v->y;
    out[14] = v->z;
}

bool vec3_is_zero(const vec3_t* v)
{
    if (!v) return true;
    return (float_zero(v->x) &&
            float_zero(v->y) &&
            float_zero(v->z));
}

#include "internal/vec3.h"
#include <stdio.h>

char* vec3_to_string(const vec3_t* v, char* buffer, size_t buffer_size) {
    if (!v || !buffer || buffer_size < 32) return NULL;
    snprintf(buffer, buffer_size, "(%.3f, %.3f, %.3f)", v->x, v->y, v->z);
    return buffer;
}

void vec3_print(const vec3_t* v) {
    if (!v) {
        printf("(null vec3)\n");
        return;
    }
    printf("(%.6f, %.6f, %.6f)\n", v->x, v->y, v->z);
}

void vec3_iadd(vec3_t* io, const vec3_t* other) {
    io->x += other->x;
    io->y += other->y;
    io->z += other->z;
}

void vec3_isub(vec3_t* io, const vec3_t* other) {
    io->x -= other->x;
    io->y -= other->y;
    io->z -= other->z;
}

void vec3_iscale(vec3_t* io, float scalar) {
    io->x *= scalar;
    io->y *= scalar;
    io->z *= scalar;
}

void vec3_madd(vec3_t* out, 
    const vec3_t* a, const vec3_t* b, float scalar) {
    out->x = a->x + b->x * scalar;
    out->y = a->y + b->y * scalar;
    out->z = a->z + b->z * scalar;
}

void vec3_project(vec3_t* out, 
    const vec3_t* p, const vec3_t* v, const vec3_t* a, float t) {
    // out = p + v*t + 0.5*a*t²
    out->x = p->x + v->x * t + 0.5f * a->x * t * t;
    out->y = p->y + v->y * t + 0.5f * a->y * t * t;
    out->z = p->z + v->z * t + 0.5f * a->z * t * t;
}
