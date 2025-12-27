#include "xform.h"
#include "quat.h"
#include "vec3.h"
#include "dualquat.h"
#include "scalar.h"
#include <iostream>
#include <cmath>
#include <iomanip>

static void xform_to_dualquat(const xform_t* xf, dualquat_t* dq_out) {
    if (!xf || !dq_out) return;
    dualquat_init_quat_vec(dq_out, &xf->rot, &xf->pos);
}

static void xform_from_dualquat(xform_t* xf, 
    const dualquat_t* dq_in, const vec3_t* scale) {

    if (!xf || !dq_in) return;
    quat_t q;
    vec3_t p;
    dualquat_to_quat_vec(dq_in, &q, &p);
    xf->rot = q;
    xf->pos = p;
    if (scale) xf->scale = *scale;
}

static inline void xform_clamp_position(vec3_t* pos) {
    pos->x = fminf(fmaxf(pos->x, XFORM_MIN_POS), XFORM_MAX_POS);
    pos->y = fminf(fmaxf(pos->y, XFORM_MIN_POS), XFORM_MAX_POS);
    pos->z = fminf(fmaxf(pos->z, XFORM_MIN_POS), XFORM_MAX_POS);
}

void xform_init(xform_t* out) {
    if (!out) return;
    out->pos = {0.0f, 0.0f, 0.0f};
    quat_identity(&out->rot);
    out->scale = {1.0f, 1.0f, 1.0f};
}

void xform_init_axis_angle(xform_t* out, 
    const vec3_t* pos, const vec3_t* axis, float radians) {

    xform_init(out);
    if (pos) out->pos = *pos;
    xform_clamp_position(&out->pos);
    quat_init_axis_angle(&out->rot, axis, radians);
}

void xform_init_axis_deg(xform_t* out, 
    const vec3_t* pos, const vec3_t* axis, float degrees) {

    xform_init_axis_angle(out, pos, axis, degrees * (M_PI / 180.0f));
}

void xform_init_euler(xform_t* out, const vec3_t* pos,
        float yaw, float pitch, float roll, euler_order_t order) {

    xform_init(out);
    if (pos) out->pos = *pos;
    xform_clamp_position(&out->pos);
    quat_init_euler(&out->rot, yaw, pitch, roll, order);
}

void xform_init_euler_deg(xform_t* out, const vec3_t* pos,
        float yaw_deg, float pitch_deg, float roll_deg, euler_order_t order) {

    xform_init_euler(out, pos,
        yaw_deg * (M_PI / 180.0f),
        pitch_deg * (M_PI / 180.0f),
        roll_deg * (M_PI / 180.0f),
        order);
}

void xform_assign(xform_t* out, const xform_t* src) {
    if (!out || !src) return;
    *out = *src;
}

bool xform_equal(const xform_t* a, const xform_t* b) {
    return vec3_equal(&a->pos, &b->pos) &&
           quat_equal(&a->rot, &b->rot) &&
           vec3_equal(&a->scale, &b->scale);
}

void xform_get_position(const xform_t* xf, vec3_t* out) {
    if (!out || !xf) return;
    *out = xf->pos;
}

void xform_set_position(xform_t* xf, const vec3_t* pos) {
    if (!xf || !pos) return;
    xf->pos = *pos;
    xform_clamp_position(&xf->pos);
}

void xform_get_axis_angle(const xform_t* xf, 
    vec3_t* out_axis, float* out_radians) {

    if (!xf) return;
    quat_to_axis_angle(&xf->rot, out_axis, out_radians);
}

void xform_get_axis_deg(const xform_t* xf, 
    vec3_t* out_axis, float* out_degrees) {

    float rad = 0.0f;
    xform_get_axis_angle(xf, out_axis, &rad);
    if (out_degrees) *out_degrees = rad * (180.0f / M_PI);
}

void xform_set_axis_angle(xform_t* xf, const vec3_t* axis, float radians) {
    if (!xf) return;
    quat_init_axis_angle(&xf->rot, axis, radians);
}

void xform_set_axis_deg(xform_t* xf, const vec3_t* axis, float degrees) {
    xform_set_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

void xform_set_euler(xform_t* xf, 
    float yaw, float pitch, float roll, euler_order_t order) {

    if (!xf) return;
    quat_init_euler(&xf->rot, yaw, pitch, roll, order);
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

    if (!xf) return;
    quat_to_euler(&xf->rot, out_yaw, out_pitch, out_roll, order);
}

void xform_get_euler_deg(const xform_t* xf, 
    float* out_yaw_deg, float* out_pitch_deg, float* out_roll_deg, 
    euler_order_t order) {

    xform_get_euler(xf, out_yaw_deg, out_pitch_deg, out_roll_deg, order);
    if (out_yaw_deg)   *out_yaw_deg   *= (180.0f / M_PI);
    if (out_pitch_deg) *out_pitch_deg *= (180.0f / M_PI);
    if (out_roll_deg)  *out_roll_deg  *= (180.0f / M_PI);
}

void xform_set_scale(xform_t* xf, float sx, float sy, float sz) {
    if (!xf) return;
    xf->scale = {sx, sy, sz};
}

void xform_get_scale(const xform_t* xf, vec3_t* out_scale) {
    if (!xf || !out_scale) return;
    *out_scale = xf->scale;
}

void xform_translate(xform_t* xf, const vec3_t* delta_world) {
    if (!xf || !delta_world) return;
    xf->pos.x += delta_world->x;
    xf->pos.y += delta_world->y;
    xf->pos.z += delta_world->z;
    xform_clamp_position(&xf->pos);
}

void xform_translate_local(xform_t* xf, const vec3_t* delta_local) {
    if (!xf || !delta_local) return;
    vec3_t delta_world;
    quat_apply_to_vec3(&xf->rot, delta_local, &delta_world);
    xform_translate(xf, &delta_world);
}

void xform_rotate_axis_angle(xform_t* xf, const vec3_t* axis, float radians) {
    if (!xf) return;
    quat_t q;
    quat_init_axis_angle(&q, axis, radians);
    quat_mul(&xf->rot, &q, &xf->rot);
}

void xform_rotate_axis_deg(xform_t* xf, 
    const vec3_t* axis, float degrees) {

    xform_rotate_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

void xform_rotate_local_axis_angle(xform_t* xf, 
    const vec3_t* axis, float radians) {

    if (!xf) return;
    quat_t q;
    quat_init_axis_angle(&q, axis, radians);
    quat_mul(&xf->rot, &xf->rot, &q);
}

void xform_rotate_local_axis_deg(xform_t* xf, 
    const vec3_t* axis, float degrees) {

    xform_rotate_local_axis_angle(xf, axis, degrees * (M_PI / 180.0f));
}

void xform_inverse(xform_t* out, const xform_t* src) {
    if (!out || !src) return;
    quat_inverse(&out->rot, &src->rot);
    out->scale.x = 1.0f / src->scale.x;
    out->scale.y = 1.0f / src->scale.y;
    out->scale.z = 1.0f / src->scale.z;

    vec3_t inv_pos;
    vec3_scale(&inv_pos, &src->pos, -1.0f);
    quat_apply_to_vec3(&out->rot, &inv_pos, &out->pos);
}

void xform_mul(xform_t* out, const xform_t* a, const xform_t* b) {
    if (!out || !a || !b) return;
    quat_mul(&out->rot, &a->rot, &b->rot);
    out->scale.x = a->scale.x * b->scale.x;
    out->scale.y = a->scale.y * b->scale.y;
    out->scale.z = a->scale.z * b->scale.z;

    vec3_t temp;
    quat_apply_to_vec3(&a->rot, &b->pos, &temp);
    vec3_add(&out->pos, &a->pos, &temp);
}

void xform_look_at(xform_t* out, 
    const vec3_t* eye, const vec3_t* target, const vec3_t* up) {

    if (!out || !eye || !target || !up) return;
    vec3_t f, r, u;
    vec3_sub(&f, target, eye);
    vec3_unit(&f, &f);
    vec3_cross(&r, up, &f);
    vec3_unit(&r, &r);
    vec3_cross(&u, &f, &r);
    quat_init_axes(&out->rot, &r, &u, &f);
    out->pos = *eye;
    xform_clamp_position(&out->pos);
}

/**
 * @brief Constructs a transform from a forward and up direction.
 *
 * @param out Output transform.
 * @param forward Forward direction vector.
 * @param up Up direction vector.
 */
void xform_from_forward(
    xform_t* out,
    const vec3_t* forward,
    const vec3_t* up)
{
    vec3_t zaxis = *forward;
    vec3_normalize(&zaxis);

    vec3_t xaxis;
    vec3_cross(&xaxis, up, &zaxis);
    vec3_normalize(&xaxis);

    vec3_t yaxis;
    vec3_cross(&yaxis, &zaxis, &xaxis);

    quat_init_axes(&out->rot, &xaxis, &yaxis, &zaxis);
    vec3_zero(&out->pos);
    vec3_init_full(&out->scale, 1.0f, 1.0f, 1.0f);
}


void xform_align_vectors(xform_t* out, 
    const vec3_t* from, const vec3_t* to) {

    if (!out || !from || !to) return;
    quat_init_two_vector(&out->rot, from, to);
    out->pos = {0, 0, 0};
}

void xform_apply_to_point(const xform_t* xf, const vec3_t* local, 
    vec3_t* out_world) {

    if (!xf || !local || !out_world) return;
    quat_apply_to_vec3(&xf->rot, local, out_world);
    vec3_add(out_world, out_world, &xf->pos);
}

void xform_apply_to_point_inverse(const xform_t* xf, const vec3_t* world, 
    vec3_t* out_local) {

    if (!xf || !world || !out_local) return;
    vec3_t temp;
    vec3_sub(&temp, world, &xf->pos);
    quat_t inv;
    quat_inverse(&inv, &xf->rot);
    quat_apply_to_vec3(&inv, &temp, out_local);
}

void xform_apply_to_direction(const xform_t* xf, const vec3_t* local_dir, 
    vec3_t* out_dir) {

    if (!xf || !local_dir || !out_dir) return;
    quat_apply_to_vec3(&xf->rot, local_dir, out_dir);
    vec3_unit(out_dir, out_dir);
}

void xform_apply_to_direction_inverse(const xform_t* xf, 
    const vec3_t* world_dir, vec3_t* out_local_dir) {

    if (!xf || !world_dir || !out_local_dir) return;
    quat_t inv;
    quat_inverse(&inv, &xf->rot);
    quat_apply_to_vec3(&inv, world_dir, out_local_dir);
    vec3_unit(out_local_dir, out_local_dir);
}

void xform_lerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;

    vec3_lerp(&out->pos, &a->pos, &b->pos, t);

    quat_slerp(&out->rot, &a->rot, &b->rot, t);

    vec3_lerp(&out->scale, &a->scale, &b->scale, t);
}

void xform_slerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;

    dualquat_t dq_a, dq_b, dq_res;
    dualquat_init_quat_vec(&dq_a, &a->rot, &a->pos);
    dualquat_init_quat_vec(&dq_b, &b->rot, &b->pos);

    dualquat_slerp(&dq_res, &dq_a, &dq_b, t);

    quat_t rot;
    vec3_t pos;
    dualquat_to_quat_vec(&dq_res, &rot, &pos);

    out->pos = pos;
    out->rot = rot;
    vec3_lerp(&out->scale, &a->scale, &b->scale, t);
}

void xform_nlerp(xform_t* out, const xform_t* a, const xform_t* b, float t) {
    if (!out || !a || !b) return;

    dualquat_t dq_a, dq_b, dq_res;
    dualquat_init_quat_vec(&dq_a, &a->rot, &a->pos);
    dualquat_init_quat_vec(&dq_b, &b->rot, &b->pos);

    dualquat_nlerp(&dq_res, &dq_a, &dq_b, t);

    quat_t rot;
    vec3_t pos;
    dualquat_to_quat_vec(&dq_res, &rot, &pos);

    out->pos = pos;
    out->rot = rot;
    vec3_lerp(&out->scale, &a->scale, &b->scale, t);
}

void xform_apply(xform_t* out, const xform_t* parent, const xform_t* local) {
    xform_mul(out, parent, local);
}

void xform_apply_inverse(xform_t* out, 
    const xform_t* parent, const xform_t* world) {

    xform_t inv_parent;
    xform_inverse(&inv_parent, parent);
    xform_mul(out, &inv_parent, world);
}

void xform_to_mat4(const xform_t* xf, float* out_mat4_16) {
    if (!xf || !out_mat4_16) return;
    float mat_rot[16];
    quat_to_mat4(&xf->rot, mat_rot);

    out_mat4_16[0]  = mat_rot[0] * xf->scale.x;
    out_mat4_16[1]  = mat_rot[1] * xf->scale.x;
    out_mat4_16[2]  = mat_rot[2] * xf->scale.x;
    out_mat4_16[3]  = 0.0f;

    out_mat4_16[4]  = mat_rot[4] * xf->scale.y;
    out_mat4_16[5]  = mat_rot[5] * xf->scale.y;
    out_mat4_16[6]  = mat_rot[6] * xf->scale.y;
    out_mat4_16[7]  = 0.0f;

    out_mat4_16[8]  = mat_rot[8] * xf->scale.z;
    out_mat4_16[9]  = mat_rot[9] * xf->scale.z;
    out_mat4_16[10] = mat_rot[10] * xf->scale.z;
    out_mat4_16[11] = 0.0f;

    out_mat4_16[12] = xf->pos.x;
    out_mat4_16[13] = xf->pos.y;
    out_mat4_16[14] = xf->pos.z;
    out_mat4_16[15] = 1.0f;
}

void xform_print(const xform_t* xf) {
    if (!xf) return;
    float yaw, pitch, roll;
    xform_get_euler(xf, &yaw, &pitch, &roll, EULER_ORDER_ZYX);
    std::cout << "[XFORM] Pos: (" 
        << xf->pos.x << ", " << xf->pos.y << ", " << xf->pos.z
        << ") Rot(YPR rad): (" << yaw << ", " << pitch << ", " << roll
        << ") Scale: (" 
        << xf->scale.x << ", " << xf->scale.y << ", " << xf->scale.z << ")\n";
}

void xform_rotate_around_pivot(
    xform_t* xf,
    const quat_t* q,
    const vec3_t* pivot)
{
    vec3_t relative;
    vec3_sub(&relative, &xf->pos, pivot);              // local = pos - pivot

    vec3_t rotated;
    quat_rotate_vector(q, &relative, &rotated);

    vec3_add(&xf->pos, &rotated, pivot);               // world = rotated + pivot

    // 2. rot (q * old)
    quat_mul(&xf->rot, q, &xf->rot);                   // rot = q * old
    quat_normalize(&xf->rot);                          
}

void xform_slerp_pivot(
    xform_t* out,
    const xform_t* a,
    const xform_t* b,
    float t,
    const vec3_t* pivot)
{
    quat_t rot_interp;
    quat_slerp(&rot_interp, &a->rot, &b->rot, t);

    vec3_t offset;
    vec3_sub(&offset, &a->pos, pivot);               // offset = a->pos - pivot

    vec3_t rotated_offset;
    quat_apply_to_vec3(&rot_interp, &offset, &rotated_offset);

    vec3_add(&out->pos, pivot, &rotated_offset);     // new_pos = pivot + rotated_offset

    out->rot = rot_interp;
    vec3_lerp(&out->scale, &a->scale, &b->scale, t);
}

void xform_look_at_pivot(
    xform_t* xf,
    const vec3_t* target,
    const vec3_t* up,
    const vec3_t* pivot)
{
    vec3_t dir;
    vec3_sub(&dir, target, pivot);
    vec3_normalize(&dir);
    xform_from_forward(xf, &dir, up);
    vec3_assign(&xf->pos, pivot);
}

void xform_rotate_local_around_pivot(
    xform_t* xf,
    const quat_t* q,
    const vec3_t* pivot)
{
    vec3_t local_offset;
    vec3_sub(&local_offset, &xf->pos, pivot);
    vec3_t rotated_offset;
    quat_apply_to_vec3(q, &local_offset, &rotated_offset);
    vec3_add(&xf->pos, pivot, &rotated_offset);

    quat_t new_rot;
    quat_mul(&new_rot, q, &xf->rot);
    quat_assign(&xf->rot, &new_rot);
}

void xform_look_to(
    xform_t* xf,
    const vec3_t* direction,
    const vec3_t* up)
{
    vec3_t norm_dir;
    vec3_assign(&norm_dir, direction);
    vec3_normalize(&norm_dir);
    xform_from_forward(xf, &norm_dir, up);
}

void xform_align_axis(
    xform_t* xf,
    int axis_index,
    const vec3_t* target_dir)
{
    vec3_t current_axis;
    switch (axis_index) {
        case 0: quat_get_right(&xf->rot, &current_axis); break;
        case 1: quat_get_up(&xf->rot, &current_axis); break;
        case 2: quat_get_forward(&xf->rot, &current_axis); break;
        default: return; // Invalid axis
    }

    quat_t q_align;
    quat_init_two_vector(&q_align, &current_axis, target_dir);
    quat_t new_rot;
    quat_mul(&new_rot, &q_align, &xf->rot);
    quat_assign(&xf->rot, &new_rot);
}

void xform_reflect(
    xform_t* out,
    const xform_t* src,
    const vec3_t* plane_point,
    const vec3_t* plane_normal)
{
    // Reflect position
    vec3_t diff;
    vec3_sub(&diff, &src->pos, plane_point);
    float dist = 2.0f * vec3_dot(&diff, plane_normal);
    vec3_t reflect_offset;
    vec3_scale(&reflect_offset, plane_normal, dist);
    vec3_sub(&out->pos, &src->pos, &reflect_offset);

    // Reflect rotation
    vec3_t x, y, z;
    quat_get_right(&src->rot, &x);
    quat_get_up(&src->rot, &y);
    quat_get_forward(&src->rot, &z);

    vec3_reflect(&x, &x, plane_normal);
    vec3_reflect(&y, &y, plane_normal);
    vec3_reflect(&z, &z, plane_normal);

    quat_init_axes(&out->rot, &x, &y, &z);

    // Copy scale
    vec3_assign(&out->scale, &src->scale);
}
