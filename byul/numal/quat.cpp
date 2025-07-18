// Eigen 기반 재작성: quat.cpp
#include "internal/quat.h"
#include <cmath>
#include <cstring>
#include <Eigen/Geometry>
#include "internal/common.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;
using Eigen::Map;

// ---------------------- 변환 유틸 ----------------------

static inline Quaternionf to_eigen(const quat_t* q) {
    return q ? Quaternionf(q->w, q->x, q->y, q->z) : Quaternionf::Identity();
}

static inline void from_eigen(quat_t* out, const Quaternionf& q) {
    if (!out) return;

    out->w = q.w(); 
    out->x = q.x(); 
    out->y = q.y(); 
    out->z = q.z();
}

static inline Vector3f vec3_to_eigen(const vec3_t* v) {
    return v ? Vector3f(v->x, v->y, v->z) : Vector3f(0, 0, 0);
}

static inline void eigen_to_vec3(vec3_t* out, const Vector3f& v) {
    if (!out) return;

    out->x = v.x(); 
    out->y = v.y(); 
    out->z = v.z();
}

// ---------------------- 생성 / 해제 ----------------------

void quat_get(const quat_t* src, float* w, float* x, float* y, float* z) {
    if (!src) return;

    if (w) *w = src->w; 
    if (x) *x = src->x;
    if (y) *y = src->y; 
    if (z) *z = src->z;
}

void quat_set(quat_t* r, float w, float x, float y, float z) {
    if (!r) return;

    r->w = w; 
    r->x = x; 
    r->y = y; 
    r->z = z;
}

quat_t* quat_new(void) {
    quat_t* r = new quat_t();
    r->w = 1.0f; r->x = r->y = r->z = 0.0f;
    return r;
}

quat_t* quat_new_full(float w, float x, float y, float z) {
    quat_t* r = new quat_t();
    r->w = w; r->x = x; r->y = y; r->z = z;
    return r;
}

quat_t* quat_new_axis_angle(const vec3_t* axis, float radians) {
    if (!axis) return nullptr;
    quat_t* r = new quat_t();
    from_eigen(r, Quaternionf(
        AngleAxisf(radians, vec3_to_eigen(axis).normalized())));
    return r;
}

quat_t* quat_new_axis_deg(const vec3_t* axis, float degrees) {
    return quat_new_axis_angle(axis, degrees * M_PI / 180.0f);
}

void quat_free(quat_t* r) { delete r; }

quat_t* quat_copy(const quat_t* src) {
    if (!src) return nullptr;
    return quat_new_full(src->w, src->x, src->y, src->z);
}

// ---------------------- 비교 / 해시 / 유효성 ----------------------

int quat_equal(const quat_t* a, const quat_t* b) {
    return a && b &&
        float_equal(a->w, b->w) &&
        float_equal(a->x, b->x) &&
        float_equal(a->y, b->y) &&
        float_equal(a->z, b->z);
}

uint32_t quat_hash(const quat_t* r) {
    if (!r) return 0;
    vec3_t axis; float angle;
    quat_to_axis_angle(r, &axis, &angle);
    uint32_t h1, h2;
    memcpy(&h1, &angle, sizeof(float));
    memcpy(&h2, &axis.x, sizeof(float));
    return h1 ^ (h2 << 1);
}

int quat_is_valid(const quat_t* r) {
    return r && (to_eigen(r).norm() > 0.0001f);
}

// ---------------------- 오일러 초기화 ----------------------

void quat_from_euler(quat_t* out, 
    float rx, float ry, float rz, euler_order_t order) {

    if (!out) return;
    Quaternionf qx = Quaternionf(AngleAxisf(rx, Vector3f::UnitX()));
    Quaternionf qy = Quaternionf(AngleAxisf(ry, Vector3f::UnitY()));
    Quaternionf qz = Quaternionf(AngleAxisf(rz, Vector3f::UnitZ()));
    Quaternionf q;
    switch (order) {
        case EULER_ORDER_ZYX: q = qz * qy * qx; break;
        case EULER_ORDER_XYZ: q = qx * qy * qz; break;
        case EULER_ORDER_XZY: q = qx * qz * qy; break;
        case EULER_ORDER_YXZ: q = qy * qx * qz; break;
        case EULER_ORDER_YZX: q = qy * qz * qx; break;
        case EULER_ORDER_ZXY: q = qz * qx * qy; break;
        default: q = Quaternionf::Identity(); break;
    }
    from_eigen(out, q);
}

void quat_from_axis_angle(quat_t* out, const vec3_t* axis, float radians) {
    if (!out || !axis) return;
    vec3_t norm = *axis;
    float len = std::sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
    if (len < 1e-6f) {
        quat_identity(out);
        return;
    }
    norm.x /= len; norm.y /= len; norm.z /= len;
    float half = radians * 0.5f;
    float s = std::sin(half);
    quat_set(out, std::cos(half), norm.x * s, norm.y * s, norm.z * s);
}

// -----------------------------
// quat_from_axis_deg
// -----------------------------

void quat_from_axis_deg(quat_t* out, const vec3_t* axis, float degrees) {
    quat_from_axis_angle(out, axis, degrees * 3.14159265359f / 180.0f);
}


void quat_reset(quat_t* out) {
    if (out) { out->w = 1.0f; out->x = out->y = out->z = 0.0f; }
}

// ---------------------- 연산 ----------------------

void quat_conjugate(quat_t* out, const quat_t* in) {
    if (!out || !in) return;
    from_eigen(out, to_eigen(in).conjugate());
}

void quat_inverse(quat_t* out, const quat_t* in) {
    if (!out || !in) return;
    from_eigen(out, to_eigen(in).inverse());
}

void quat_rotate_vector(vec3_t* out, const quat_t* r, const vec3_t* v) {
    if (!out || !r || !v) return;
    eigen_to_vec3(out, to_eigen(r) * vec3_to_eigen(v));
}

void quat_apply_to_vec3(const quat_t* q, const vec3_t* v, vec3_t* out) {
    if (!q || !v || !out) return;

    // 회전 쿼터니언
    Quaternionf rot = to_eigen(q);
    Vector3f vin(v->x, v->y, v->z);

    // 회전 적용
    Vector3f result = rot * vin;

    out->x = result.x();
    out->y = result.y();
    out->z = result.z();
}


// ---------------------- 행렬 변환 ----------------------

void quat_to_mat3(const quat_t* r, float* out_mat3x3) {
    if (!r || !out_mat3x3) return;
    Matrix3f m = to_eigen(r).toRotationMatrix();
    memcpy(out_mat3x3, m.data(), sizeof(float) * 9);
}

void quat_to_mat4(const quat_t* r, float* out_mat4x4) {
    if (!r || !out_mat4x4) return;
    Matrix4f m = Matrix4f::Identity();
    m.block<3,3>(0,0) = to_eigen(r).toRotationMatrix();
    memcpy(out_mat4x4, m.data(), sizeof(float) * 16);
}

void quat_from_mat3(quat_t* out, const float* mat3x3) {
    if (!out || !mat3x3) return;
    Map<const Matrix3f> m(mat3x3);
    from_eigen(out, Quaternionf(m));
}

void quat_from_mat4(quat_t* out, const float* mat4x4) {
    if (!out || !mat4x4) return;
    Map<const Matrix4f> m(mat4x4);
    from_eigen(out, Quaternionf(m.block<3,3>(0,0)));
}

// ---------------------- 보간 ----------------------

void quat_lerp(quat_t* out, const quat_t* a, const quat_t* b, float t) {
    if (!out || !a || !b) return;
    Quaternionf qa = to_eigen(a);
    Quaternionf qb = to_eigen(b);
    from_eigen(out, qa.slerp(t, qb));
}

void quat_slerp(quat_t* out, const quat_t* a, const quat_t* b, float t) {
    if (!out || !a || !b) return;
    Quaternionf qa = to_eigen(a);
    Quaternionf qb = to_eigen(b);
    from_eigen(out, qa.slerp(t, qb));
}

// ---------------------- 해석 ----------------------

void quat_to_axis_angle(const quat_t* r, vec3_t* axis, float* radians) {
    if (!r || !axis || !radians) return;
    AngleAxisf aa(to_eigen(r));
    *radians = aa.angle();
    eigen_to_vec3(axis, aa.axis());
}

void quat_get_forward(const quat_t* r, vec3_t* out_dir) {
    if (!r || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(r) * Vector3f(0, 0, -1));
}

void quat_get_up(const quat_t* r, vec3_t* out_dir) {
    if (!r || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(r) * Vector3f(0, 1, 0));
}

void quat_get_right(const quat_t* r, vec3_t* out_dir) {
    if (!r || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(r) * Vector3f(1, 0, 0));
}

void quat_to_euler(const quat_t* r, 
    float* x, float* y, float* z, euler_order_t order) {
        
    if (!r || !x || !y || !z) return;
    Matrix3f m = to_eigen(r).toRotationMatrix();
    switch (order) {
        case EULER_ORDER_ZYX: 
            *y = asinf(-m(2,0)); 
            *x = atan2f(m(2,1), m(2,2)); 
            *z = atan2f(m(1,0), m(0,0)); 
            break;
        case EULER_ORDER_XYZ: 
            *y = asinf(-m(0,2)); 
            *x = atan2f(m(1,2), m(2,2)); 
            *z = atan2f(m(0,1), m(0,0)); 
            break;
        case EULER_ORDER_XZY: 
            *z = asinf(m(0,1));  
            *x = atan2f(-m(2,1), m(1,1)); 
            *y = atan2f(-m(0,2), m(0,0)); 
            break;
        case EULER_ORDER_YXZ: 
            *x = asinf(m(1,2));  
            *y = atan2f(-m(0,2), m(2,2)); 
            *z = atan2f(-m(1,0), m(1,1)); 
            break;
        case EULER_ORDER_YZX: 
            *z = asinf(-m(1,0)); 
            *y = atan2f(m(2,0), m(0,0));  
            *x = atan2f(m(1,2), m(1,1)); 
            break;
        case EULER_ORDER_ZXY: 
            *x = asinf(m(2,1));  
            *z = atan2f(-m(0,1), m(1,1)); 
            *y = atan2f(-m(2,0), m(2,2)); 
            break;
        default: 
            *x = *y = *z = 0.0f; 
            break;
    }
}

// ---------------------- 기초 연산 ----------------------

void quat_add(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;

    Eigen::Vector4f sum = to_eigen(a).coeffs() + to_eigen(b).coeffs();
    // Eigen 쿼터니언의 coeffs() 순서는 (x, y, z, w)
    out->x = sum[0];
    out->y = sum[1];
    out->z = sum[2];
    out->w = sum[3];
}

void quat_sub(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;

    Eigen::Vector4f diff = to_eigen(a).coeffs() - to_eigen(b).coeffs();
    out->x = diff[0];
    out->y = diff[1];
    out->z = diff[2];
    out->w = diff[3];
}

void quat_mul(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;
    from_eigen(out, to_eigen(a) * to_eigen(b));
}

void quat_scale(quat_t* out, const quat_t* a, float s) {
    if (!out || !a) return;

    Eigen::Vector4f scaled = to_eigen(a).coeffs() * s;
    out->x = scaled[0];
    out->y = scaled[1];
    out->z = scaled[2];
    out->w = scaled[3];
}

void quat_normalize(quat_t* out, const quat_t* a) {
    if (!out || !a) return;
    from_eigen(out, to_eigen(a).normalized());
}

float quat_dot(const quat_t* a, const quat_t* b) {
    if (!a || !b) return 0.0f;
    return to_eigen(a).dot(to_eigen(b));
}

float quat_length(const quat_t* a) {
    if (!a) return 0.0f;
    return to_eigen(a).norm();
}

void quat_identity(quat_t* out) {
    if (!out) return;
    out->w = 1.0f;
    out->x = 0.0f;
    out->y = 0.0f;
    out->z = 0.0f;
}
