#include "numeq_kalman.h"
#include <cmath>
#include <cstring> // memset

void kalman_init(kalman_filter_t* kf) {
    if (!kf) return;
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = 0.01f;
    kf->r = 1.0f;
    kf->k = 0.0f;
}

void kalman_init_full(kalman_filter_t* kf, float init_x, float init_p,
                               float process_noise, float measurement_noise) {
    if (!kf) return;
    kf->x = init_x;
    kf->p = init_p;
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->k = 0.0f;
}

void kalman_assign(kalman_filter_t* dst, const kalman_filter_t* src) {
    if (!dst || !src) return;
    *dst = *src;
}

void kalman_time_update(kalman_filter_t* kf) {
    if (!kf) return;
    // x' = x 
    // P' = P + Q 
    kf->p += kf->q;
}

float kalman_measurement_update(kalman_filter_t* kf, float measured) {
    if (!kf) return 0.0f;
    // K = P' / (P' + R)
    kf->k = kf->p / (kf->p + kf->r);

    // x = x' + K (z - x')
    kf->x = kf->x + kf->k * (measured - kf->x);

    // P = (1 - K) P'
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

void kalman_vec3_init(kalman_filter_vec3_t* kf) {
    if (!kf) return;
    kf->position = {0, 0, 0};
    kf->velocity = {0, 0, 0};
    kf->error_p = {1, 1, 1};
    kf->q = 0.01f;
    kf->r = 1.0f;
    kf->dt = 0.1f;
}

void kalman_vec3_init_full(kalman_filter_vec3_t* kf,
                                    const vec3_t* init_pos,
                                    const vec3_t* init_vel,
                                    float process_noise,
                                    float measurement_noise,
                                    float dt) {
    if (!kf) return;
    kf->position = init_pos ? *init_pos : vec3_t{0, 0, 0};
    kf->velocity = init_vel ? *init_vel : vec3_t{0, 0, 0};
    kf->error_p = {1, 1, 1};
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->dt = (dt > 0) ? dt : 0.1f;
}

void kalman_vec3_assign(kalman_filter_vec3_t* dst,
                           const kalman_filter_vec3_t* src) {
    if (!dst || !src) return;
    *dst = *src;
}

void kalman_vec3_time_update(kalman_filter_vec3_t* kf) {
    if (!kf) return;

    // x' = x + v * dt
    kf->position.x += kf->velocity.x * kf->dt;
    kf->position.y += kf->velocity.y * kf->dt;
    kf->position.z += kf->velocity.z * kf->dt;

    // P' = P + Q
    kf->error_p.x += kf->q;
    kf->error_p.y += kf->q;
    kf->error_p.z += kf->q;
}

void kalman_vec3_measurement_update(kalman_filter_vec3_t* kf,
                                             const vec3_t* measured_pos) {
    if (!kf || !measured_pos) return;

    float kx = kf->error_p.x / (kf->error_p.x + kf->r);
    float ky = kf->error_p.y / (kf->error_p.y + kf->r);
    float kz = kf->error_p.z / (kf->error_p.z + kf->r);

    float px = kf->position.x;
    float py = kf->position.y;
    float pz = kf->position.z;

    kf->position.x = px + kx * (measured_pos->x - px);
    kf->position.y = py + ky * (measured_pos->y - py);
    kf->position.z = pz + kz * (measured_pos->z - pz);

    kf->velocity.x = (kf->position.x - px) / kf->dt;
    kf->velocity.y = (kf->position.y - py) / kf->dt;
    kf->velocity.z = (kf->position.z - pz) / kf->dt;

    kf->error_p.x = (1.0f - kx) * kf->error_p.x;
    kf->error_p.y = (1.0f - ky) * kf->error_p.y;
    kf->error_p.z = (1.0f - kz) * kf->error_p.z;
}

void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                                  float future_dt,
                                  vec3_t* out_predicted_pos) {
    if (!kf || !out_predicted_pos) return;
    *out_predicted_pos = {
        kf->position.x + kf->velocity.x * future_dt,
        kf->position.y + kf->velocity.y * future_dt,
        kf->position.z + kf->velocity.z * future_dt
    };
}
