#include "internal/vec3.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <new>
#include "internal/common.h"

using Eigen::Matrix4f;
using Eigen::Vector3f;

static inline Vector3f vec3_to_vec3f(const vec3_t* t) {
    return t ? Vector3f(t->x, t->y, t->z) : Vector3f(0.0f, 0.0f, 0.0f);
}

static inline void vec3f_to_vec3(vec3_t* out, const Vector3f& v) {
    if (!out) return;
    out->x = v.x();
    out->y = v.y();
    out->z = v.z();
}

vec3_t* vec3_new_full(float x, float y, float z) {
    return new (std::nothrow) vec3_t{x, y, z};
}

vec3_t* vec3_new(void) {
    return vec3_new_full(0.0f, 0.0f, 0.0f);
}

void vec3_free(vec3_t* v) {
    delete v;
}

vec3_t* vec3_copy(const vec3_t* src) {
    if (!src) return nullptr;
    return vec3_new_full(src->x, src->y, src->z);
}

int vec3_equal(const vec3_t* a, const vec3_t* b) {
    if (!a || !b) return 0;
    return a->x == b->x && a->y == b->y && a->z == b->z;
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
    out->x = 0.0f;
    out->y = 0.0f;
    out->z = 0.0f;
}

void vec3_add(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;
    vec3f_to_vec3(out, vec3_to_vec3f(a) + vec3_to_vec3f(b));
}

void vec3_sub(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;
    vec3f_to_vec3(out, vec3_to_vec3f(a) - vec3_to_vec3f(b));
}

void vec3_mul(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;
    out->x = a->x * b->x;
    out->y = a->y * b->y;
    out->z = a->z * b->z;
}

void vec3_div(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;

    out->x = (b->x > -FLOAT_EPSILON && b->x < FLOAT_EPSILON) ? 0.0f : a->x / b->x;
    out->y = (b->y > -FLOAT_EPSILON && b->y < FLOAT_EPSILON) ? 0.0f : a->y / b->y;
    out->z = (b->z > -FLOAT_EPSILON && b->z < FLOAT_EPSILON) ? 0.0f : a->z / b->z;
}

void vec3_scale(vec3_t* out, const vec3_t* a, float scalar) {
    if (!out || !a) return;
    vec3f_to_vec3(out, vec3_to_vec3f(a) * scalar);
}

float vec3_dot(const vec3_t* a, const vec3_t* b) {
    if (!a || !b) return 0.0f;
    return vec3_to_vec3f(a).dot(vec3_to_vec3f(b));
}

void vec3_cross(vec3_t* out, const vec3_t* a, const vec3_t* b) {
    if (!out || !a || !b) return;
    vec3f_to_vec3(out, vec3_to_vec3f(a).cross(vec3_to_vec3f(b)));
}

float vec3_length(const vec3_t* a) {
    if (!a) return 0.0f;
    return vec3_to_vec3f(a).norm();
}

void vec3_normalize(vec3_t* out, const vec3_t* a) {
    if (!out || !a) return;
    Vector3f v = vec3_to_vec3f(a);
    float len = v.norm();
    if (len < FLOAT_EPSILON) {
        out->x = out->y = out->z = 0.0f;
    } else {
        vec3f_to_vec3(out, v.normalized());
    }
}

float vec3_distance(const vec3_t* a, const vec3_t* b) {
    if (!a || !b) return 0.0f;
    return (vec3_to_vec3f(a) - vec3_to_vec3f(b)).norm();
}

void vec3_lerp(vec3_t* out, const vec3_t* start, const vec3_t* goal, float t) {
    if (!out || !start || !goal) return;
    out->x = (1.0f - t) * start->x + t * goal->x;
    out->y = (1.0f - t) * start->y + t * goal->y;
    out->z = (1.0f - t) * start->z + t * goal->z;
}

void vec3_to_mat4(const vec3_t* v, float* out_mat4) {
    if (!v || !out_mat4) return;
    
    Matrix4f mat = Matrix4f::Identity();
    mat(0, 3) = v->x;
    mat(1, 3) = v->y;
    mat(2, 3) = v->z;

    // Eigen은 column-major라서 OpenGL/DirectX에 쓸 때는 memcpy로 바로 넘길 수 있음
    std::memcpy(out_mat4, mat.data(), sizeof(float) * 16);
}
