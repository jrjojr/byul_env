#include "internal/dualquat.h"
#include "internal/quat.h"
#include "internal/vec3.h"
#include "internal/quat.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstring>
#include <functional>

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

// -----------------------------
// 생성 / 해제 / 복사
// -----------------------------

dualquat_t* dualquat_new(void) {
    dualquat_t* dq = new dualquat_t;
    dq->real = *quat_new();
    dq->dual = *quat_new_full(0.0f, 0.0f, 0.0f, 0.0f);
    return dq;
}

void dualquat_free(dualquat_t* dq) {
    delete dq;
}

void dualquat_copy(dualquat_t* dst, const dualquat_t* src) {
    if (!dst || !src) return;
    dst->real = src->real;
    dst->dual = src->dual;
}

// -----------------------------
// 비교 / 해시
// -----------------------------

int dualquat_equal(const dualquat_t* a, const dualquat_t* b) {
    return quat_equal(&a->real, &b->real) && quat_equal(&a->dual, &b->dual);
}

uint32_t dualquat_hash(const dualquat_t* dq) {
    return quat_hash(&dq->real) ^ (quat_hash(&dq->dual) << 1);
}

// -----------------------------
// 생성 및 추출
// -----------------------------

void dualquat_from_quat_vec(dualquat_t* out, const quat_t* rot, const vec3_t* vec) {
    if (!out || !rot || !vec) return;
    float w, x, y, z;
    quat_get(rot, &w, &x, &y, &z);
    Quaternionf real(w, x, y, z);
    Vector3f t(vec->x, vec->y, vec->z);
    Quaternionf dual(0.0f, t.x(), t.y(), t.z());

    from_eigen_quat(&out->real, real);
    
    // from_eigen_quat(&out->dual, (dual * real) * 0.5f);
    quat_t temp;
    from_eigen_quat(&temp, dual * real);
    quat_scale(&out->dual, &temp, 0.5f);


}

void dualquat_to_quat_vec(const dualquat_t* dq, quat_t* rot, vec3_t* vec) {
    if (!dq || !rot || !vec) return;
    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);

    Quaternionf dq_conj = real.conjugate();
    dual = dual * dq_conj;
    Vector3f t = 2.0f * Vector3f(dual.x(), dual.y(), dual.z());

    quat_set(rot, real.w(), real.x(), real.y(), real.z());
    vec->x = t.x(); vec->y = t.y(); vec->z = t.z();
}

// -----------------------------
// 연산
// -----------------------------

void dualquat_normalize(dualquat_t* out, const dualquat_t* dq) {
    if (!out || !dq) return;
    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);
    float norm = real.norm();

    from_eigen_quat(&out->real, real.normalized());
    from_eigen_quat(&out->dual, dual.normalized());

}

void dualquat_mul(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    if (!out || !a || !b) return;
    Quaternionf ra = to_eigen_quat(&a->real);
    Quaternionf rb = to_eigen_quat(&b->real);
    Quaternionf da = to_eigen_quat(&a->dual);
    Quaternionf db = to_eigen_quat(&b->dual);

    from_eigen_quat(&out->real, ra * rb);

    // from_eigen_quat(&out->dual, ra * db + da * rb);
    Quaternionf ra_db = ra * db;
    Quaternionf da_rb = da * rb;
    quat_t qrd, qdr, qout;
    from_eigen_quat(&qrd, ra_db);
    from_eigen_quat(&qdr, da_rb);
    quat_add(&qout, &qrd , &qdr);
    Quaternionf q = to_eigen_quat(&qout);
    from_eigen_quat(&out->dual, q);
}

void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq) {
    if (!out || !dq) return;
    from_eigen_quat(&out->real, to_eigen_quat(&dq->real).conjugate());
    from_eigen_quat(&out->dual, to_eigen_quat(&dq->dual).conjugate());
}

// -----------------------------
// 변환 / 적용
// -----------------------------

void dualquat_transform_point(const dualquat_t* dq, vec3_t* io_point) {
    if (!dq || !io_point) return;
    Quaternionf real = to_eigen_quat(&dq->real);
    Quaternionf dual = to_eigen_quat(&dq->dual);

    Vector3f v(io_point->x, io_point->y, io_point->z);
    Vector3f r = real * v;

    Quaternionf t_q = dual * real.conjugate();
    Vector3f t = 2.0f * Vector3f(t_q.x(), t_q.y(), t_q.z());

    Vector3f final = r + t;
    io_point->x = final.x(); io_point->y = final.y(); io_point->z = final.z();
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

void dualquat_from_mat4(dualquat_t* out, const float* mat4x4) {
    if (!out || !mat4x4) return;
    Eigen::Map<const Matrix4f> m(mat4x4);
    Matrix3f rot = m.block<3,3>(0,0);
    Vector3f t = m.block<3,1>(0,3);
    Quaternionf real(rot);
    Quaternionf p(0, t.x(), t.y(), t.z());
    Quaternionf dual = (p * real);

    quat_t q;
    from_eigen_quat(&q, dual);

    from_eigen_quat(&out->real, real);
    // from_eigen_quat(&out->dual, dual);
    quat_scale(&out->dual, &q, 0.5);
}

void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3) {
    if (!dq || !out_mat3) return;
    Matrix3f m = to_eigen_quat(&dq->real).toRotationMatrix();
    memcpy(out_mat3, m.data(), sizeof(float) * 9);
}

void dualquat_from_mat3(dualquat_t* out, const float* mat3) {
    if (!out || !mat3) return;
    Eigen::Map<const Matrix3f> m(mat3);
    Quaternionf q(m);
    from_eigen_quat(&out->real, q);
    from_eigen_quat(&out->dual, Quaternionf(0, 0, 0, 0));
}

// -----------------------------
// 보간
// -----------------------------
void dualquat_lerp(dualquat_t* out, const dualquat_t* a, const dualquat_t* b, float t) {
    if (!out || !a || !b) return;

    // 회전 보간: r = normalize((1 - t) * a.real + t * b.real)
    quat_t qa_scaled, qb_scaled, qsum, qnorm;
    quat_scale(&qa_scaled, &a->real, 1.0f - t);
    quat_scale(&qb_scaled, &b->real, t);
    quat_add(&qsum, &qa_scaled, &qb_scaled);
    quat_normalize(&qnorm, &qsum);

    // 위치 보간
    quat_t aconj, bconj;
    quat_conjugate(&aconj, &a->real);
    quat_conjugate(&bconj, &b->real);

    quat_t ta_q, tb_q;
    quat_mul(&ta_q, &a->dual, &aconj);
    quat_mul(&tb_q, &b->dual, &bconj);

    vec3_t ta = { ta_q.x * 2.0f, ta_q.y * 2.0f, ta_q.z * 2.0f };
    vec3_t tb = { tb_q.x * 2.0f, tb_q.y * 2.0f, tb_q.z * 2.0f };

    vec3_t tpos = {
        (1.0f - t) * ta.x + t * tb.x,
        (1.0f - t) * ta.y + t * tb.y,
        (1.0f - t) * ta.z + t * tb.z
    };

    quat_t p, d;
    quat_set(&p, 0.0f, tpos.x, tpos.y, tpos.z);
    quat_mul(&d, &p, &qnorm);
    quat_scale(&d, &d, 0.5f);

    out->real = qnorm;
    out->dual = d;
}

void dualquat_slerp(dualquat_t* out, const dualquat_t* a, const dualquat_t* b, float t) {
    if (!out || !a || !b) return;

    // 회전 SLERP: r = slerp(a.real, b.real, t)
    quat_t r;
    quat_slerp(&r, &a->real, &b->real, t);

    // 위치 보간
    quat_t aconj, bconj;
    quat_conjugate(&aconj, &a->real);
    quat_conjugate(&bconj, &b->real);

    quat_t ta_q, tb_q;
    quat_mul(&ta_q, &a->dual, &aconj);
    quat_mul(&tb_q, &b->dual, &bconj);

    vec3_t ta = { ta_q.x * 2.0f, ta_q.y * 2.0f, ta_q.z * 2.0f };
    vec3_t tb = { tb_q.x * 2.0f, tb_q.y * 2.0f, tb_q.z * 2.0f };

    vec3_t tpos = {
        (1.0f - t) * ta.x + t * tb.x,
        (1.0f - t) * ta.y + t * tb.y,
        (1.0f - t) * ta.z + t * tb.z
    };

    quat_t p, d;
    quat_set(&p, 0.0f, tpos.x, tpos.y, tpos.z);
    quat_mul(&d, &p, &r);
    quat_scale(&d, &d, 0.5f);

    out->real = r;
    out->dual = d;
}

// -----------------------------
// 기초 연산 (add/sub/scale/dot/length)
// -----------------------------

void dualquat_add(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    if (!out || !a || !b) return;
    quat_add(&out->real, &a->real, &b->real);
    quat_add(&out->dual, &a->dual, &b->dual);
}

void dualquat_sub(dualquat_t* out, const dualquat_t* a, const dualquat_t* b) {
    if (!out || !a || !b) return;
    quat_sub(&out->real, &a->real, &b->real);
    quat_sub(&out->dual, &a->dual, &b->dual);
}

void dualquat_scale(dualquat_t* out, const dualquat_t* a, float scalar) {
    if (!out || !a) return;
    quat_scale(&out->real, &a->real, scalar);
    quat_scale(&out->dual, &a->dual, scalar);
}

float dualquat_dot(const dualquat_t* a, const dualquat_t* b) {
    if (!a || !b) return 0.0f;
    return quat_dot(&a->real, &b->real) + quat_dot(&a->dual, &b->dual);
}

float dualquat_length(const dualquat_t* a) {
    if (!a) return 0.0f;
    float r = quat_length(&a->real);
    float d = quat_length(&a->dual);
    return sqrtf(r * r + d * d);
}

// -----------------------------
// dualquat_inverse: conj(real) + ε·(-conj(dual))
// -----------------------------

// void dualquat_inverse(dualquat_t* out, const dualquat_t* dq) {
//     if (!out || !dq) return;
//     quat_t real_conj, dual_conj, dual_neg;
//     quat_conjugate(&real_conj, &dq->real);
//     quat_conjugate(&dual_conj, &dq->dual);
//     quat_scale(&dual_neg, &dual_conj, -1.0f);
//     out->real = real_conj;
//     out->dual = dual_neg;
// }

void dualquat_inverse(dualquat_t* out, const dualquat_t* dq) {
    if (!out || !dq) return;

    // r⁻¹ = conjugate(r) (unit quaternion 가정)
    quat_t rinv, rinv2, tmp;
    quat_conjugate(&rinv, &dq->real);

    // dual = -rinv * dual * rinv
    quat_mul(&tmp, &dq->dual, &rinv);
    quat_mul(&tmp, &rinv, &tmp); // rinv * dual * rinv
    quat_scale(&tmp, &tmp, -1.0f);

    out->real = rinv;
    out->dual = tmp;
}


// -----------------------------
// dualquat_align: real.w ≥ 0로 뒤집기
// -----------------------------

void dualquat_align(dualquat_t* out, const dualquat_t* dq) {
    if (!out || !dq) return;
    if (dq->real.w >= 0.0f) {
        *out = *dq;
    } else {
        quat_scale(&out->real, &dq->real, -1.0f);
        quat_scale(&out->dual, &dq->dual, -1.0f);
    }
}

// -----------------------------
// dualquat_blend_weighted: out = normalize(w1·a + w2·b)
// -----------------------------
void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2) {

    if (!out || !a || !b) return;

    quat_t ar, br, dr, sumr;
    quat_t ad, bd, sumd;

    // 회전 부분 선형 합
    quat_scale(&ar, &a->real, w1);
    quat_scale(&br, &b->real, w2);
    quat_add(&sumr, &ar, &br);
    quat_normalize(&dr, &sumr);  // ⬅ 회전만 정규화

    // 듀얼 부분 선형 합 (정규화하지 않음!)
    quat_scale(&ad, &a->dual, w1);
    quat_scale(&bd, &b->dual, w2);
    quat_add(&sumd, &ad, &bd);

    out->real = dr;
    out->dual = sumd;  // 정규화 제거
}

// -----------------------------
// dualquat_identity: 단위 회전 + 위치 0
// -----------------------------

void dualquat_identity(dualquat_t* out) {
    if (!out) return;
    quat_identity(&out->real);
    quat_set(&out->dual, 0.0f, 0.0f, 0.0f, 0.0f);
}