#include "internal/xform.h"
#include "internal/dualquat.h"
#include "internal/quat.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>


// 생성
xform_t* xform_new_identity(void) {
    xform_t* xf = (xform_t*)malloc(sizeof(xform_t));
    if (xf) dualquat_identity(&xf->dq);
    return xf;
}

xform_t* xform_new_from_axis_angle(
    const vec3_t* pos, const vec3_t* axis, float radians) {

    xform_t* xf = xform_new_identity();
    if (!xf) return NULL;

    quat_t r;
    quat_from_axis_angle(&r, axis, radians);
    dualquat_from_quat_vec(&xf->dq, &r, pos);
    return xf;
}

// xform_t* xform_new_from_euler(
//     const vec3_t* pos, float yaw, float pitch, float roll) {

//     xform_t* xf = xform_new_identity();
//     if (!xf) return NULL;

//     quat_t r;
//     quat_from_euler(&r, yaw, pitch, roll, EULER_ORDER_ZYX);
//     dualquat_from_quat_vec(&xf->dq, &r, pos);
//     return xf;
// }

xform_t* xform_new_from_euler(
    const vec3_t* pos,
    float yaw, float pitch, float roll,
    euler_order_t order){

    xform_t* xf = xform_new_identity();
    if (!xf) return NULL;

    quat_t r;
    quat_from_euler(&r, yaw, pitch, roll, order);
    dualquat_from_quat_vec(&xf->dq, &r, pos);
    return xf;
}

xform_t* xform_clone(const xform_t* src) {
    if (!src) return NULL;
    xform_t* copy = (xform_t*)malloc(sizeof(xform_t));
    if (copy) memcpy(copy, src, sizeof(xform_t));
    return copy;
}

void xform_free(xform_t* xf) {
    if (xf) free(xf);
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
    dualquat_from_quat_vec(&xf->dq, &r, pos);
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
    quat_from_axis_angle(&r, axis, radians);
    dualquat_from_quat_vec(&xf->dq, &r, &pos);
}

void xform_set_euler(
    xform_t* xf,
    float yaw, float pitch, float roll,
    euler_order_t order){

    vec3_t pos;
    dualquat_to_quat_vec(&xf->dq, nullptr, &pos);
    quat_t r;
    quat_from_euler(&r, yaw, pitch, roll, order);
    dualquat_from_quat_vec(&xf->dq, &r, &pos);
}

void xform_get_euler(
    const xform_t* xf,
    float* out_yaw, float* out_pitch, float* out_roll,
    euler_order_t order){

    quat_t r;
    dualquat_to_quat_vec(&xf->dq, &r, nullptr);
    quat_to_euler(&r, out_yaw, out_pitch, out_roll, order);
}

// void xform_translate(xform_t* xf, const vec3_t* delta_world) {
//     if (!xf || !delta_world) return;

//     // 1. 단위 회전 쿼터니언 사용 (진짜 회전 없음 보장)
//     quat_t identity_rot;
//     quat_identity(&identity_rot);
//     dualquat_t dq_rot;
//     dualquat_from_quat_vec(&dq_rot, &identity_rot, nullptr);

//     // 2. 이동만 포함하는 듀얼쿼터니언 생성
//     dualquat_t dq_delta;
//     dualquat_from_quat_vec(&dq_delta,nullptr, delta_world);

//     // 3. 선행 곱 → 월드 기준 이동
//     dualquat_mul(&xf->dq, &dq_rot, &dq_delta);
// }

void xform_translate(xform_t* xf, const vec3_t* delta_world) {
    if (!xf || !delta_world) return;

    // 1. 이동만 포함하는 듀얼 쿼터니언 생성 (회전 없음)
    dualquat_t dq_delta;
    dualquat_from_quat_vec(&dq_delta, nullptr, delta_world);

    // 2. 선행 곱 → 월드 기준 이동
    dualquat_mul(&xf->dq, &dq_delta, &xf->dq);
}


void xform_translate_local(xform_t* xf, const vec3_t* delta_local) {
    dualquat_t dq_delta;
    dualquat_from_quat_vec(&dq_delta, nullptr, delta_local);
    dualquat_mul(&xf->dq, &xf->dq, &dq_delta);  // local 기준이므로 후행곱
}


void xform_rotate_axis_angle(xform_t* xf, const vec3_t* axis, float radians) {
    quat_t r;
    quat_from_axis_angle(&r, axis, radians);

    dualquat_t dq_rot;
    dualquat_from_quat_vec(&dq_rot, &r, nullptr);        

    dualquat_mul(&xf->dq, &dq_rot, &xf->dq);  // 앞에 곱하므로 월드 기준 회전
}

void xform_rotate_local_axis_angle(
    xform_t* xf, const vec3_t* axis, float radians) {

    quat_t r;
    quat_from_axis_angle(&r, axis, radians);

    dualquat_t dq_rot;
    dualquat_from_quat_vec(&dq_rot, &r, nullptr);

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
    vec3_normalize(out_dir, &temp);  // 방향은 반드시 단위벡터로
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