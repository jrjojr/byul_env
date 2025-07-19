#include "internal/xform.h"
#include "internal/dualquat.h"
#include "internal/quat.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>


// 생성
void xform_init(xform_t* out) {
    dualquat_identity(&out->dq);
}

void xform_init_axis_angle(xform_t* out,
    const vec3_t* pos, const vec3_t* axis, float radians) {

    xform_init(out);

    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_init_quat_vec(&out->dq, &r, pos);
}

void xform_init_euler(xform_t* out,
    const vec3_t* pos,
    float yaw, float pitch, float roll,
    euler_order_t order){

    xform_init(out);

    quat_t r;
    quat_init_euler(&r, yaw, pitch, roll, order);
    dualquat_init_quat_vec(&out->dq, &r, pos);
}

void xform_copy(xform_t* out,const xform_t* src) {
    if (!src) return;
    out->dq = src->dq;
}

bool xform_equal(const xform_t* a, const xform_t* b) {
    return dualquat_equal(&a->dq, &b->dq);
}

void xform_get_position(const xform_t* xf, vec3_t* out) {
    dualquat_to_quat_vec(&xf->dq, nullptr, out);
}

void xform_set_position(xform_t* xf, const vec3_t* pos) {
    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    dualquat_init_quat_vec(&xf->dq, &r, pos);
}

void xform_get_axis_angle(
    const xform_t* xf, vec3_t* out_axis, float* out_radians) {

    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    quat_to_axis_angle(&r, out_axis, out_radians);
}

void xform_set_axis_angle(xform_t* xf, const vec3_t* axis, float radians) {
    vec3_t pos;
    dualquat_to_quat_vec(&xf->dq, nullptr, &pos);
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);
    dualquat_init_quat_vec(&xf->dq, &r, &pos);
}

void xform_set_euler(
    xform_t* xf,
    float yaw, float pitch, float roll,
    euler_order_t order){

    vec3_t pos;
    dualquat_to_quat_vec(&xf->dq, nullptr, &pos);
    quat_t r;
    quat_init_euler(&r, yaw, pitch, roll, order);
    dualquat_init_quat_vec(&xf->dq, &r, &pos);
}

void xform_get_euler(
    const xform_t* xf,
    float* out_yaw, float* out_pitch, float* out_roll,
    euler_order_t order){

    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    quat_to_euler(&r, out_yaw, out_pitch, out_roll, order);
}

void xform_translate(xform_t* xf, const vec3_t* delta_world) {
    if (!xf || !delta_world) return;

    // 1. 이동만 포함하는 듀얼 쿼터니언 생성 (회전 없음)
    dualquat_t dq_delta;
    dualquat_init_quat_vec(&dq_delta, nullptr, delta_world);

    // 2. 선행 곱 → 월드 기준 이동
    dualquat_mul(&xf->dq, &dq_delta, &xf->dq);
}


void xform_translate_local(xform_t* xf, const vec3_t* delta_local) {
    dualquat_t dq_delta;
    dualquat_init_quat_vec(&dq_delta, nullptr, delta_local);
    dualquat_mul(&xf->dq, &xf->dq, &dq_delta);  // local 기준이므로 후행곱
}

void xform_rotate_axis_angle(xform_t* xf, const vec3_t* axis, float radians) {
    quat_t r;
    quat_init_axis_angle(&r, axis, radians);

    dualquat_t dq_rot;
    dualquat_init_quat_vec(&dq_rot, &r, nullptr);        

    dualquat_mul(&xf->dq, &dq_rot, &xf->dq);  // 앞에 곱하므로 월드 기준 회전
}

void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians) {

    quat_t r;
    quat_init_axis_angle(&r, axis, radians);

    dualquat_t dq_rot;
    dualquat_init_quat_vec(&dq_rot, &r, nullptr);

    // 로컬 기준 회전 → 기존 dq 뒤에 곱함
    dualquat_mul(&xf->dq, &xf->dq, &dq_rot);
}

void xform_apply_to_point(
    const xform_t* xf, const vec3_t* local, vec3_t* out_world) {

    dualquat_apply_to_point(&xf->dq, local, out_world);
}

void xform_apply_to_direction(
    const xform_t* xf, const vec3_t* local_dir, vec3_t* out_dir) {

    vec3_t temp;
    dualquat_apply_to_point(&xf->dq, local_dir, &temp);
    vec3_unit(out_dir, &temp);  // 방향은 반드시 단위벡터로
}

void xform_to_mat4(const xform_t* xf, float* out_mat4_16) {
    dualquat_to_mat4(&xf->dq, out_mat4_16);
}

void xform_lerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;
    dualquat_lerp(&out->dq, &a->dq, &b->dq, t);
}

void xform_slerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;
    dualquat_slerp(&out->dq, &a->dq, &b->dq, t);
}

void xform_apply_inverse(xform_t* out, const xform_t* parent, const xform_t* world) {
    if (!out || !parent || !world) return;

    dualquat_t inv_parent;
    dualquat_inverse(&inv_parent, &parent->dq);       // parent^{-1}
    dualquat_mul(&out->dq, &inv_parent, &world->dq);   // inv_parent * world
}

void xform_apply(xform_t* out, const xform_t* parent, const xform_t* local) {
    if (!out || !parent || !local) return;

    // out = parent * local
    dualquat_mul(&out->dq, &parent->dq, &local->dq);
}