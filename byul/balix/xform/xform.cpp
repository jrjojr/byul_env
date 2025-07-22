#include "internal/xform.h"
#include "internal/dualquat.h"
#include "internal/quat.h"
#include <iostream>
#include <cmath>
#include <iomanip>
#include "internal/common.h"

// ---------------------------------------------------------
// 📌 내부 유틸
// ---------------------------------------------------------

static inline void xform_clamp_position(vec3_t* pos) {
    pos->x = fminf(fmaxf(pos->x, XFORM_POS_MIN), XFORM_POS_MAX);
    pos->y = fminf(fmaxf(pos->y, XFORM_POS_MIN), XFORM_POS_MAX);
    pos->z = fminf(fmaxf(pos->z, XFORM_POS_MIN), XFORM_POS_MAX);
}

// ---------------------------------------------------------
// 📌 생성 / 소멸
// ---------------------------------------------------------

void xform_init(xform_t* out) {
    if (!out) return;
    dualquat_identity(&out->dq);
    out->scale = {1.0f, 1.0f, 1.0f};
}

// ---------------------------------------------------------
// 📌 기본 변환 (라디안)
// ---------------------------------------------------------

void xform_init_axis_angle(
    xform_t* out, const vec3_t* pos, const vec3_t* axis, float radians) {
    xform_init(out);
    vec3_t clamped_pos = *pos;
    xform_clamp_position(&clamped_pos);
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_init_quat_vec(&out->dq, &r, &clamped_pos);
}

void xform_init_euler(
    xform_t* out, const vec3_t* pos,
    float yaw, float pitch, float roll, euler_order_t order) {
    xform_init(out);
    vec3_t clamped_pos = *pos;
    xform_clamp_position(&clamped_pos);
    quat_t r;
    quat_init_euler(&r, yaw, pitch, roll, order);
    dualquat_init_quat_vec(&out->dq, &r, &clamped_pos);
}

// ---------------------------------------------------------
// 📌 기본 변환 (도 단위)
// ---------------------------------------------------------

void xform_init_axis_angle_deg(
    xform_t* out, const vec3_t* pos, const vec3_t* axis, float degrees) {
    xform_init_axis_angle(out, pos, axis, degrees * (M_PI / 180.0f));
}

void xform_init_euler_deg(
    xform_t* out, const vec3_t* pos,
    float yaw_deg, float pitch_deg, float roll_deg, euler_order_t order) {
    xform_init_euler(out, pos,
        yaw_deg * (M_PI / 180.0f),
        pitch_deg * (M_PI / 180.0f),
        roll_deg * (M_PI / 180.0f),
        order);
}

// ---------------------------------------------------------
// 📌 복사 / 비교
// ---------------------------------------------------------

void xform_assign(xform_t* out, const xform_t* src) {
    if (!out || !src) return;
    out->dq = src->dq;
    out->scale = src->scale;
}

bool xform_equal(const xform_t* a, const xform_t* b) {
    return dualquat_equal(&a->dq, &b->dq) &&
           vec3_equal(&a->scale, &b->scale);
}

// ---------------------------------------------------------
// 📌 위치 / 회전 / 스케일
// ---------------------------------------------------------

void xform_get_position(const xform_t* xf, vec3_t* out) {
    dualquat_to_quat_vec(&xf->dq, nullptr, out);
}

void xform_set_position(xform_t* xf, const vec3_t* pos) {
    vec3_t clamped_pos = *pos;
    xform_clamp_position(&clamped_pos);
    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    dualquat_init_quat_vec(&xf->dq, &r, &clamped_pos);
}

void xform_get_axis_angle(
    const xform_t* xf, vec3_t* out_axis, float* out_radians) {
    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    quat_to_axis_angle(&r, out_axis, out_radians);
}

void xform_get_axis_angle_deg(
    const xform_t* xf, vec3_t* out_axis, float* out_degrees) {
    float rad;
    xform_get_axis_angle(xf, out_axis, &rad);
    *out_degrees = rad * (180.0f / M_PI);
}

void xform_set_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians) {
    vec3_t pos;
    dualquat_to_quat_vec(&xf->dq, nullptr, &pos);
    xform_clamp_position(&pos);
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_init_quat_vec(&xf->dq, &r, &pos);
}

void xform_set_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees) {
    xform_set_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

void xform_set_euler(
    xform_t* xf, float yaw, float pitch, float roll, euler_order_t order) {
    vec3_t pos;
    dualquat_to_quat_vec(&xf->dq, nullptr, &pos);
    xform_clamp_position(&pos);
    quat_t r;
    quat_init_euler(&r, yaw, pitch, roll, order);
    dualquat_init_quat_vec(&xf->dq, &r, &pos);
}

void xform_set_euler_deg(xform_t* xf, 
    float yaw_deg, float pitch_deg, float roll_deg, euler_order_t order) {

    xform_set_euler(xf,
        yaw_deg * (M_PI / 180.0f),
        pitch_deg * (M_PI / 180.0f),
        roll_deg * (M_PI / 180.0f),
        order);
}

void xform_get_euler(const xform_t* xf, 
    float* out_yaw, float* out_pitch, float* out_roll, euler_order_t order) {

    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    quat_to_euler(&r, out_yaw, out_pitch, out_roll, order);
}

void xform_get_euler_deg(const xform_t* xf, 
    float* out_yaw_deg, float* out_pitch_deg, float* out_roll_deg, 
    euler_order_t order) {

    xform_get_euler(xf, out_yaw_deg, out_pitch_deg, out_roll_deg, order);
    *out_yaw_deg   *= (180.0f / M_PI);
    *out_pitch_deg *= (180.0f / M_PI);
    *out_roll_deg  *= (180.0f / M_PI);
}

void xform_set_scale(xform_t* xf, float sx, float sy, float sz) {
    xf->scale = {sx, sy, sz};
}

void xform_get_scale(const xform_t* xf, vec3_t* out_scale) {
    *out_scale = xf->scale;
}

// ---------------------------------------------------------
// 📌 이동 / 회전
// ---------------------------------------------------------

void xform_translate(xform_t* xf, const vec3_t* delta_world) {
    vec3_t pos;
    xform_get_position(xf, &pos);
    pos.x += delta_world->x;
    pos.y += delta_world->y;
    pos.z += delta_world->z;
    xform_clamp_position(&pos);
    xform_set_position(xf, &pos);
}

void xform_translate_local(xform_t* xf, const vec3_t* delta_local) {
    // 현재 회전 추출
    quat_t rot;
    dualquat_to_quat_vec(&xf->dq, &rot, nullptr);

    // delta_local을 월드 좌표로 변환
    vec3_t delta_world;
    quat_apply_to_vec3(&rot, delta_local, &delta_world);

    // 월드 이동 처리 (클램프 포함)
    xform_translate(xf, &delta_world);
}

void xform_rotate_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians) {
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_t dq_rot;
    dualquat_init_quat_vec(&dq_rot, &r, nullptr);
    dualquat_mul(&xf->dq, &dq_rot, &xf->dq);
}

void xform_rotate_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees) {
    xform_rotate_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians) {
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_t dq_rot;
    dualquat_init_quat_vec(&dq_rot, &r, nullptr);
    dualquat_mul(&xf->dq, &xf->dq, &dq_rot);
}

void xform_rotate_local_axis_angle_deg(
    xform_t* xf, const vec3_t* axis, float degrees) {
    xform_rotate_local_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

// ---------------------------------------------------------
// 📌 Inverse / Mul
// ---------------------------------------------------------

void xform_inverse(xform_t* out, const xform_t* src) {
    dualquat_inverse(&out->dq, &src->dq);
    out->scale = {
        1.0f / src->scale.x, 1.0f / src->scale.y, 1.0f / src->scale.z};
}

void xform_mul(xform_t* out, const xform_t* a, const xform_t* b) {
    dualquat_mul(&out->dq, &a->dq, &b->dq);
    out->scale = {
        a->scale.x * b->scale.x, 
        a->scale.y * b->scale.y, 
        a->scale.z * b->scale.z};
}

// ---------------------------------------------------------
// 📌 LookAt, AlignVectors
// ---------------------------------------------------------

void xform_look_at(
    xform_t* out, const vec3_t* eye, const vec3_t* target, const vec3_t* up) {
    vec3_t f, r, u;
    vec3_sub(&f, target, eye);
    vec3_unit(&f, &f);
    vec3_cross(&r, up, &f);
    vec3_unit(&r, &r);
    vec3_cross(&u, &f, &r);
    quat_t q;
    quat_init_axes(&q, &r, &u, &f);
    vec3_t clamped_eye = *eye;
    xform_clamp_position(&clamped_eye);
    dualquat_init_quat_vec(&out->dq, &q, &clamped_eye);
}

void xform_align_vectors(
    xform_t* out, const vec3_t* from, const vec3_t* to) {
    quat_t q;
    quat_init_two_vector(&q, from, to);
    dualquat_init_quat_vec(&out->dq, &q, nullptr);
}

// ---------------------------------------------------------
// 📌 적용
// ---------------------------------------------------------

void xform_apply_to_point(
    const xform_t* xf, const vec3_t* local, vec3_t* out_world) {
    dualquat_apply_to_point(&xf->dq, local, out_world);
}

void xform_apply_to_point_inverse(
    const xform_t* xf, const vec3_t* world, vec3_t* out_local) {
    dualquat_t inv;
    dualquat_inverse(&inv, &xf->dq);
    dualquat_apply_to_point(&inv, world, out_local);
}

void xform_apply_to_direction(
    const xform_t* xf, const vec3_t* local_dir, vec3_t* out_dir) {
    vec3_t temp;
    dualquat_apply_to_point(&xf->dq, local_dir, &temp);
    vec3_unit(out_dir, &temp);
}

void xform_apply_to_direction_inverse(
    const xform_t* xf, const vec3_t* world_dir, vec3_t* out_local_dir) {
    dualquat_t inv;
    dualquat_inverse(&inv, &xf->dq);
    dualquat_apply_to_point(&inv, world_dir, out_local_dir);
    vec3_unit(out_local_dir, out_local_dir);
}

// ---------------------------------------------------------
// 📌 보간
// ---------------------------------------------------------

void xform_lerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;

    vec3_t pos_a, pos_b;
    xform_get_position(a, &pos_a);
    xform_get_position(b, &pos_b);

    vec3_t pos_mid = {
        (1.0f - t) * pos_a.x + t * pos_b.x,
        (1.0f - t) * pos_a.y + t * pos_b.y,
        (1.0f - t) * pos_a.z + t * pos_b.z
    };
    xform_clamp_position(&pos_mid);

    quat_t rot_a, rot_b, rot_mid;
    dualquat_to_quat_vec(&a->dq, &rot_a, nullptr);
    dualquat_to_quat_vec(&b->dq, &rot_b, nullptr);
    quat_slerp(&rot_mid, &rot_a, &rot_b, t);

    dualquat_init_quat_vec(&out->dq, &rot_mid, &pos_mid);
}

void xform_slerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    dualquat_slerp(&out->dq, &a->dq, &b->dq, t);
}

void xform_nlerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    dualquat_nlerp(&out->dq, &a->dq, &b->dq, t);
}

// ---------------------------------------------------------
// 📌 계층 변환
// ---------------------------------------------------------

void xform_apply(xform_t* out, const xform_t* parent, const xform_t* local) {
    dualquat_mul(&out->dq, &parent->dq, &local->dq);
}

void xform_apply_inverse(xform_t* out, 
    const xform_t* parent, const xform_t* world) {
    dualquat_t inv_parent;
    dualquat_inverse(&inv_parent, &parent->dq);
    dualquat_mul(&out->dq, &inv_parent, &world->dq);
}

// ---------------------------------------------------------
// 📌 행렬 변환
// ---------------------------------------------------------

void xform_to_mat4(const xform_t* xf, float* out_mat4_16) {
    dualquat_to_mat4(&xf->dq, out_mat4_16);
}

// ---------------------------------------------------------
// 📌 디버깅
// ---------------------------------------------------------

void xform_print(const xform_t* xf) {
    vec3_t pos;
    xform_get_position(xf, &pos);
    float yaw, pitch, roll;
    xform_get_euler(xf, &yaw, &pitch, &roll, EULER_ORDER_ZYX);
    std::cout << "[XFORM] Pos: (" << pos.x << ", " << pos.y << ", " << pos.z
              << ") Rot(YPR rad): (" << yaw << ", " << pitch << ", " << roll
              << ") Scale: (" << xf->scale.x << ", " << xf->scale.y << ", " 
              << xf->scale.z << ")\n";
}
