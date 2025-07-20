// Eigen 기반 재작성: quat.cpp
#include "internal/quat.h"
#include <cmath>
#include <cstring>
#include <Eigen/Geometry>
#include "internal/common.h"

// ---------------------------------------------------------
// 🔧 변환 유틸
// ---------------------------------------------------------

static inline Eigen::Quaternionf to_eigen(const quat_t* q) {
    return q ? Eigen::Quaternionf(q->w, q->x, q->y, q->z) : Eigen::Quaternionf::Identity();
}

static inline void from_eigen(quat_t* out, const Eigen::Quaternionf& q) {
    if (!out) return;
    out->w = q.w();
    out->x = q.x();
    out->y = q.y();
    out->z = q.z();
}

static inline Eigen::Vector3f vec3_to_eigen(const vec3_t* v) {
    return v ? Eigen::Vector3f(v->x, v->y, v->z) : Eigen::Vector3f(0, 0, 0);
}

static inline void eigen_to_vec3(vec3_t* out, const Eigen::Vector3f& v) {
    if (!out) return;
    out->x = v.x();
    out->y = v.y();
    out->z = v.z();
}

// ---------------------------------------------------------
// 🎯 생성 / 초기화
// ---------------------------------------------------------

void quat_init(quat_t* out) {
    if (!out) return;
    out->w = 1.0f;
    out->x = 0.0f;
    out->y = 0.0f;
    out->z = 0.0f;
}

void quat_init_full(quat_t* out, float w, float x, float y, float z) {
    if (!out) return;
    out->w = w;
    out->x = x;
    out->y = y;
    out->z = z;
}

void quat_init_axis_angle(quat_t* out, const vec3_t* axis, float radians) {
    if (!out || !axis) return;
    Eigen::AngleAxisf aa(radians, vec3_to_eigen(axis).normalized());
    from_eigen(out, Eigen::Quaternionf(aa));
}

void quat_init_axis_deg(quat_t* out, const vec3_t* axis, float degrees) {
    quat_init_axis_angle(out, axis, degrees * static_cast<float>(M_PI) / 180.0f);
}

void quat_init_two_vector(quat_t* out, const vec3_t* from, const vec3_t* to) {
    if (!out || !from || !to) return;
    Eigen::Vector3f f = vec3_to_eigen(from).normalized();
    Eigen::Vector3f t = vec3_to_eigen(to).normalized();
    float dot_val = f.dot(t);

    if (dot_val > 0.9999f) {
        quat_init(out);
        return;
    }
    if (dot_val < -0.9999f) {
        Eigen::Vector3f orth = f.unitOrthogonal();
        Eigen::AngleAxisf aa(static_cast<float>(M_PI), orth);
        from_eigen(out, Eigen::Quaternionf(aa));
        return;
    }

    Eigen::Vector3f axis = f.cross(t);
    float angle = acosf(dot_val);
    Eigen::AngleAxisf aa(angle, axis.normalized());
    from_eigen(out, Eigen::Quaternionf(aa));
}

void quat_init_euler(quat_t* out, float rx, float ry, float rz, euler_order_t order) {
    if (!out) return;
    Eigen::Quaternionf qx(Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf qy(Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf qz(Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf q;

    switch (order) {
        case EULER_ORDER_ZYX: q = qz * qy * qx; break;
        case EULER_ORDER_XYZ: q = qx * qy * qz; break;
        case EULER_ORDER_XZY: q = qx * qz * qy; break;
        case EULER_ORDER_YXZ: q = qy * qx * qz; break;
        case EULER_ORDER_YZX: q = qy * qz * qx; break;
        case EULER_ORDER_ZXY: q = qz * qx * qy; break;
        default: q = Eigen::Quaternionf::Identity(); break;
    }
    from_eigen(out, q);
}

void quat_init_euler_deg(quat_t* out, float deg_x, float deg_y, float deg_z, euler_order_t order) {
    quat_init_euler(out,
        deg_x * static_cast<float>(M_PI) / 180.0f,
        deg_y * static_cast<float>(M_PI) / 180.0f,
        deg_z * static_cast<float>(M_PI) / 180.0f,
        order);
}

void quat_init_angular_velocity(quat_t* out, const vec3_t* omega, float dt) {
    if (!out || !omega) return;
    Eigen::Vector3f omega_vec = vec3_to_eigen(omega);
    float angle = omega_vec.norm() * dt;
    if (angle < 1e-8f) {
        quat_init(out);
        return;
    }
    Eigen::AngleAxisf aa(angle, omega_vec.normalized());
    from_eigen(out, Eigen::Quaternionf(aa));
}

void quat_init_axes(quat_t* out, 
    const vec3_t* xaxis, const vec3_t* yaxis, const vec3_t* zaxis) {
    if (!out || !xaxis || !yaxis || !zaxis) return;

    Eigen::Matrix3f m;
    m << xaxis->x, yaxis->x, zaxis->x,
         xaxis->y, yaxis->y, zaxis->y,
         xaxis->z, yaxis->z, zaxis->z;

    Eigen::Quaternionf q(m);
    from_eigen(out, q.normalized());
}


void quat_assign(quat_t* out, const quat_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void quat_reset(quat_t* out) {
    quat_init(out);
}

void quat_get(const quat_t* src, float* out_w, float* out_x, float* out_y, float* out_z) {
    if (!src) return;
    if (out_w) *out_w = src->w;
    if (out_x) *out_x = src->x;
    if (out_y) *out_y = src->y;
    if (out_z) *out_z = src->z;
}

void quat_set(quat_t* out, float w, float x, float y, float z) {
    if (!out) return;
    out->w = w;
    out->x = x;
    out->y = y;
    out->z = z;
}

// ---------------------------------------------------------
// 🧪 비교 / 유효성
// ---------------------------------------------------------

int quat_equal(const quat_t* a, const quat_t* b) {
    return a && b &&
        float_equal(a->w, b->w) &&
        float_equal(a->x, b->x) &&
        float_equal(a->y, b->y) &&
        float_equal(a->z, b->z);
}

uint32_t quat_hash(const quat_t* q) {
    if (!q) return 0;
    uint32_t h = 0;
    const uint32_t* pw = reinterpret_cast<const uint32_t*>(&q->w);
    const uint32_t* px = reinterpret_cast<const uint32_t*>(&q->x);
    const uint32_t* py = reinterpret_cast<const uint32_t*>(&q->y);
    const uint32_t* pz = reinterpret_cast<const uint32_t*>(&q->z);
    h ^= *pw + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= *px + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= *py + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= *pz + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
}

int quat_is_valid(const quat_t* q) {
    return q && (to_eigen(q).norm() > 0.0001f);
}

// ---------------------------------------------------------
// 🔄 행렬 변환
// ---------------------------------------------------------

void quat_to_mat3(const quat_t* q, float* out_mat3x3) {
    if (!q || !out_mat3x3) return;
    Eigen::Matrix3f m = to_eigen(q).toRotationMatrix();
    memcpy(out_mat3x3, m.data(), sizeof(float) * 9);
}

void quat_to_mat4(const quat_t* q, float* out_mat4x4) {
    if (!q || !out_mat4x4) return;
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3,3>(0,0) = to_eigen(q).toRotationMatrix();
    memcpy(out_mat4x4, m.data(), sizeof(float) * 16);
}

void quat_init_from_mat3(quat_t* out, const float* mat3x3) {
    if (!out || !mat3x3) return;
    Eigen::Map<const Eigen::Matrix3f> m(mat3x3);
    from_eigen(out, Eigen::Quaternionf(m));
}

void quat_init_from_mat4(quat_t* out, const float* mat4x4) {
    if (!out || !mat4x4) return;
    Eigen::Map<const Eigen::Matrix4f> m(mat4x4);
    from_eigen(out, Eigen::Quaternionf(m.block<3,3>(0,0)));
}

// ---------------------------------------------------------
// 🔁 벡터 회전
// ---------------------------------------------------------

void quat_rotate_vector(const quat_t* q, const vec3_t* v, vec3_t* out) {
    if (!out || !q || !v) return;
    eigen_to_vec3(out, to_eigen(q) * vec3_to_eigen(v));
}

// ---------------------------------------------------------
// ➕ 연산
// ---------------------------------------------------------

void quat_add(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;
    Eigen::Vector4f sum = to_eigen(a).coeffs() + to_eigen(b).coeffs();
    out->x = sum[0]; out->y = sum[1]; out->z = sum[2]; out->w = sum[3];
}

void quat_sub(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;
    Eigen::Vector4f diff = to_eigen(a).coeffs() - to_eigen(b).coeffs();
    out->x = diff[0]; out->y = diff[1]; out->z = diff[2]; out->w = diff[3];
}

void quat_mul(quat_t* out, const quat_t* a, const quat_t* b) {
    if (!out || !a || !b) return;
    from_eigen(out, to_eigen(a) * to_eigen(b));
}

void quat_scale(quat_t* out, const quat_t* a, float s) {
    if (!out || !a) return;
    Eigen::Vector4f scaled = to_eigen(a).coeffs() * s;
    out->x = scaled[0]; out->y = scaled[1]; out->z = scaled[2]; out->w = scaled[3];
}

void quat_normalize(quat_t* io) {
    if (!io) return;
    Eigen::Quaternionf q = to_eigen(io);
    q.normalize();
    from_eigen(io, q);
}

void quat_unit(quat_t* out, const quat_t* a) {
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

void quat_conjugate(quat_t* out, const quat_t* in) {
    if (!out || !in) return;
    from_eigen(out, to_eigen(in).conjugate());
}

void quat_inverse(quat_t* out, const quat_t* in) {
    if (!out || !in) return;
    from_eigen(out, to_eigen(in).inverse());
}

void quat_identity(quat_t* out) {
    quat_init(out);
}

// ---------------------------------------------------------
// 📐 보간
// ---------------------------------------------------------

void quat_lerp(quat_t* out, const quat_t* a, const quat_t* b, float t) {
    if (!out || !a || !b) return;
    Eigen::Quaternionf qa = to_eigen(a);
    Eigen::Quaternionf qb = to_eigen(b);
    Eigen::Quaternionf q(
        qa.w() * (1.0f - t) + qb.w() * t,
        qa.x() * (1.0f - t) + qb.x() * t,
        qa.y() * (1.0f - t) + qb.y() * t,
        qa.z() * (1.0f - t) + qb.z() * t
    );
    from_eigen(out, q.normalized());
}

void quat_slerp(quat_t* out, const quat_t* a, const quat_t* b, float t) {
    if (!out || !a || !b) return;
    from_eigen(out, to_eigen(a).slerp(t, to_eigen(b)));
}

// ---------------------------------------------------------
// 🔍 해석
// ---------------------------------------------------------

void quat_to_axis_angle(const quat_t* q, vec3_t* axis, float* radians) {
    if (!q || !axis || !radians) return;
    Eigen::AngleAxisf aa(to_eigen(q));
    *radians = aa.angle();
    eigen_to_vec3(axis, aa.axis());
}

void quat_get_forward(const quat_t* q, vec3_t* out_dir) {
    if (!q || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(q) * Eigen::Vector3f(0, 0, -1));
}

void quat_get_up(const quat_t* q, vec3_t* out_dir) {
    if (!q || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(q) * Eigen::Vector3f(0, 1, 0));
}

void quat_get_right(const quat_t* q, vec3_t* out_dir) {
    if (!q || !out_dir) return;
    eigen_to_vec3(out_dir, to_eigen(q) * Eigen::Vector3f(1, 0, 0));
}

void quat_to_euler(const quat_t* q, float* x, float* y, float* z, euler_order_t order) {
    if (!q || !x || !y || !z) return;
    Eigen::Matrix3f m = to_eigen(q).toRotationMatrix();
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
        default: *x = *y = *z = 0.0f; break;
    }
}
