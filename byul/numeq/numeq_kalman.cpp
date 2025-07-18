#include "internal/numeq_kalman.h"
#include "internal/vec3.hpp"

// ---------------------------------------------------------
// 스칼라 Kalman 필터
// ---------------------------------------------------------

void kalman_reset(kalman_filter_t* kf, float init_x, float init_p,
                  float process_noise, float measurement_noise) {
    kf->x = init_x;
    kf->p = init_p;
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->k = 0.0f;
}

void kalman_predict(kalman_filter_t* kf) {
    // 오차 공분산 증가 (모델 불확실성 증가)
    kf->p += kf->q;
}

float kalman_update(kalman_filter_t* kf, float measured) {
    // 칼만 이득 계산
    kf->k = kf->p / (kf->p + kf->r);

    // 상태 업데이트
    kf->x += kf->k * (measured - kf->x);

    // 공분산 감소
    kf->p *= (1.0f - kf->k);

    return kf->x;
}

// ---------------------------------------------------------
// 벡터 Kalman 필터 (위치 + 속도)
// ---------------------------------------------------------

void kalman_vec3_reset(kalman_filter_vec3_t* kf,
                       const vec3_t* init_pos,
                       const vec3_t* init_vel,
                       float process_noise,
                       float measurement_noise,
                       float dt) {
    kf->position = *init_pos;
    kf->velocity = *init_vel;
    kf->error_p = Vec3(process_noise, process_noise, process_noise);  // 초기 오차
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->dt = dt;
}

void kalman_vec3_predict(kalman_filter_vec3_t* kf) {
    // 위치 예측: p = p + v * dt
    Vec3 p(kf->position);
    Vec3 v(kf->velocity);
    kf->position = p + v * kf->dt;

    // 오차 공분산 증가
    Vec3 p_error(kf->error_p);
    kf->error_p = p_error + Vec3(kf->q, kf->q, kf->q);
}

void kalman_vec3_update(kalman_filter_vec3_t* kf,
                        const vec3_t* measured_pos) {
    Vec3 z(*measured_pos); // 측정값
    Vec3 x(kf->position);  // 추정값

    Vec3 e_p(kf->error_p);
    Vec3 k = e_p / (e_p + Vec3(kf->r, kf->r, kf->r)); // 칼만 이득

    Vec3 corrected = x + k * (z - x);
    kf->position = corrected;

    kf->error_p = (Vec3(1.0f, 1.0f, 1.0f) - k) * e_p;

    // 속도 추정: (새 위치 - 이전 위치) / dt
    kf->velocity = (Vec3(kf->position) - x) / kf->dt;
}

void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                         float future_dt,
                         vec3_t* out_predicted_pos) {
    Vec3 p(kf->position);
    Vec3 v(kf->velocity);
    *out_predicted_pos = p + v * future_dt;
}

void kalman_vec3_copy(kalman_filter_vec3_t* dst,
                      const kalman_filter_vec3_t* src) {
    *dst = *src;
}
