#include "dualquat.h"
#include "quat.h"
#include "vec3.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstring>

using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::Matrix4f;
using Eigen::Matrix3f;

static inline Quaternionf to_eigen_quat(const quat_t* q) {
    return q ? Quaternionf(q->w, q->x, q->y, q->z) : Quaternionf::Identity();
}

static inline void from_eigen_quat(quat_t* out, const Quaternionf& q) {
    if (!out) return;
    out->w = q.w(); out->x = q.x(); out->y = q.y(); out->z = q.z();
}

void dualquat_init(dualquat_t* out) {
    if (!out) return;
    quat_identity(&out->real);
    quat_set(&out->dual, 0, 0, 0, 0);
}

void dualquat_init_quat_vec(dualquat_t* out, 
    const quat_t* rot, const vec3_t* vec) {
    if (!out) return;

    Quaternionf real = rot ? to_eigen_quat(rot) : Quaternionf::Identity();
    Quaternionf dual(0, 0, 0, 0);

    if (vec) {
        Vector3f t(vec->x, vec->y, vec->z);
        Quaternionf trans(0, t.x(), t.y(), t.z());
        dual = trans * real;
        dual.coeffs() *= 0.5f;
    }

    from_eigen_quat(&out->real, real);
    from_eigen_quat(&out->dual, dual);
}

void dualquat_init_from_mat4(dualquat_t* out, const float* mat4x4) {
    if (!out || !mat4x4) return;
    Eigen::Map<const Matrix4f> m(mat4x4);
    Matrix3f rot = m.block<3,3>(0,0);
    Vector3f t = m.block<3,1>(0,3);

    Quaternionf real(rot);
    Quaternionf dual = Quaternionf(0, t.x(), t.y(), t.z()) * real;
    dual.coeffs() *= 0.5f;

    from_eigen_quat(&out->real, real);
    from_eigen_quat(&out->dual, dual);
}

void dualquat_init_from_mat3(dualquat_t* out, const float* mat3x3) {
    if (!out || !mat3x3) return;
    Eigen::Map<const Matrix3f> m(mat3x3);
    Quaternionf real(m);

    from_eigen_quat(&out->real, real);
    quat_set(&out->dual, 0, 0, 0, 0);
}

void dualquat_assign(dualquat_t* out, const dualquat_t* src) {
    if (!out || !src) return;
    out->real = src->real;
    out->dual = src->dual;
}

int dualquat_equal(const dualquat_t* a, const dualquat_t* b) {
    return quat_equal(&a->real, &b->real) && quat_equal(&a->dual, &b->dual);
}

uint32_t dualquat_hash(const dualquat_t* dq) {
    return quat_hash(&dq->real) ^ (quat_hash(&dq->dual) << 1);
}

void dualquat_to_quat_vec(const dualquat_t* dq, quat_t* rot, vec3_t* vec) {
    if (!dq) return;

    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);

    if (rot) {
        quat_set(rot, real.w(), real.x(), real.y(), real.z());
    }

    if (vec) {
        Quaternionf t_q = dual * real.conjugate();
        Vector3f t = 2.0f * Vector3f(t_q.x(), t_q.y(), t_q.z());
        vec->x = t.x(); vec->y = t.y(); vec->z = t.z();
    }
}

void dualquat_to_mat4(const dualquat_t* dq, float* out_mat4) {
    if (!dq || !out_mat4) return;
    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);

    Matrix4f mat = Matrix4f::Identity();
    mat.block<3,3>(0,0) = real.toRotationMatrix();

    Quaternionf t_q = dual * real.conjugate();
    Vector3f t = 2.0f * Vector3f(t_q.x(), t_q.y(), t_q.z());
    mat(0,3) = t.x(); mat(1,3) = t.y(); mat(2,3) = t.z();

    memcpy(out_mat4, mat.data(), sizeof(float) * 16);
}

void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3) {
    if (!dq || !out_mat3) return;
    Matrix3f m = to_eigen_quat(&dq->real).toRotationMatrix();
    memcpy(out_mat3, m.data(), sizeof(float) * 9);
}

void dualquat_normalize(dualquat_t* io) {
    if (!io) return;
    Quaternionf real = to_eigen_quat(&io->real);
    Quaternionf dual = to_eigen_quat(&io->dual);

    float norm = real.norm();
    if (norm < 1e-8f) {
        quat_identity(&io->real);
        quat_set(&io->dual, 0, 0, 0, 0);
        return;
    }
    real.normalize();
    dual.coeffs() /= norm;

    from_eigen_quat(&io->real, real);
    from_eigen_quat(&io->dual, dual);
}

void dualquat_unit(dualquat_t* out, const dualquat_t* dq) {
    if (!out || !dq) return;
    *out = *dq;
    dualquat_normalize(out);
}

void dualquat_add(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    quat_add(&out->real, &a->real, &b->real);
    quat_add(&out->dual, &a->dual, &b->dual);
}

void dualquat_sub(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    quat_sub(&out->real, &a->real, &b->real);
    quat_sub(&out->dual, &a->dual, &b->dual);
}

void dualquat_scale(dualquat_t* out, const dualquat_t* a, float s) {
    quat_scale(&out->real, &a->real, s);
    quat_scale(&out->dual, &a->dual, s);
}

float dualquat_dot(const dualquat_t* a, const dualquat_t* b) {
    return quat_dot(&a->real, &b->real) + quat_dot(&a->dual, &b->dual);
}

float dualquat_length(const dualquat_t* dq) {
    float r = quat_length(&dq->real);
    float d = quat_length(&dq->dual);
    return sqrtf(r * r + d * d);
}

void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq) {
    quat_conjugate(&out->real, &dq->real);
    quat_conjugate(&out->dual, &dq->dual);
}

void dualquat_inverse(dualquat_t* out, const dualquat_t* dq) {
    quat_t rinv, tmp;
    quat_conjugate(&rinv, &dq->real);

    quat_mul(&tmp, &dq->dual, &rinv);
    quat_mul(&tmp, &rinv, &tmp);
    quat_scale(&tmp, &tmp, -1.0f);

    out->real = rinv;
    out->dual = tmp;
}


void dualquat_mul(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    Quaternionf ra = to_eigen_quat(&a->real);
    Quaternionf rb = to_eigen_quat(&b->real);
    Quaternionf da = to_eigen_quat(&a->dual);
    Quaternionf db = to_eigen_quat(&b->dual);

    from_eigen_quat(&out->real, ra * rb);

    Eigen::Vector4f dual_sum = (ra * db).coeffs() + (da * rb).coeffs();
    from_eigen_quat(&out->dual, Quaternionf(dual_sum));
}


void dualquat_align(dualquat_t* out, const dualquat_t* dq) {
    if (dq->real.w >= 0.0f) *out = *dq;
    else {
        quat_scale(&out->real, &dq->real, -1);
        quat_scale(&out->dual, &dq->dual, -1);
    }
}

void dualquat_lerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t) {
    dualquat_t b_aligned;
    dualquat_align(&b_aligned, b);

    dualquat_t temp1, temp2;
    dualquat_scale(&temp1, a, 1 - t);
    dualquat_scale(&temp2, &b_aligned, t);
    dualquat_add(&temp1, &temp1, &temp2);
    dualquat_normalize(&temp1);

    *out = temp1;
}

void dualquat_nlerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t) {
    dualquat_lerp(out, a, b, t);
}

void dualquat_slerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t) {
    dualquat_t b_aligned;
    dualquat_align(&b_aligned, b);

    quat_t r;
    quat_slerp(&r, &a->real, &b_aligned.real, t);

    quat_t aconj, bconj;
    quat_conjugate(&aconj, &a->real);
    quat_conjugate(&bconj, &b_aligned.real);

    quat_t ta_q, tb_q;
    quat_mul(&ta_q, &a->dual, &aconj);
    quat_mul(&tb_q, &b_aligned.dual, &bconj);

    vec3_t ta = { ta_q.x * 2, ta_q.y * 2, ta_q.z * 2 };
    vec3_t tb = { tb_q.x * 2, tb_q.y * 2, tb_q.z * 2 };

    vec3_t tpos = {
        (1 - t) * ta.x + t * tb.x,
        (1 - t) * ta.y + t * tb.y,
        (1 - t) * ta.z + t * tb.z
    };

    quat_t p, d;
    quat_set(&p, 0, tpos.x, tpos.y, tpos.z);
    quat_mul(&d, &p, &r);
    quat_scale(&d, &d, 0.5f);

    out->real = r;
    out->dual = d;
}

void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2) {
    dualquat_t b_aligned;
    dualquat_align(&b_aligned, b);

    dualquat_t tmp1, tmp2;
    dualquat_scale(&tmp1, a, w1);
    dualquat_scale(&tmp2, &b_aligned, w2);
    dualquat_add(&tmp1, &tmp1, &tmp2);
    dualquat_normalize(&tmp1);

    *out = tmp1;
}

void dualquat_apply_to_point_inplace(const dualquat_t* dq, vec3_t* io_point) {
    if (!dq || !io_point) return;
    vec3_t temp = *io_point;
    dualquat_apply_to_point(dq, &temp, io_point);
}

void dualquat_apply_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out) {
    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);

    Vector3f v(in->x, in->y, in->z);
    Vector3f r = real * v;

    Quaternionf t_q = dual * real.conjugate();
    Vector3f t = 2.0f * Vector3f(t_q.x(), t_q.y(), t_q.z());

    Vector3f final = r + t;
    out->x = final.x(); out->y = final.y(); out->z = final.z();
}

void dualquat_apply_inverse_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out) {
    dualquat_t inv;
    dualquat_inverse(&inv, dq);
    dualquat_apply_to_point(&inv, in, out);
}
