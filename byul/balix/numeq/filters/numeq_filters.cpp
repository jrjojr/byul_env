#include "numeq_filters.h"
#include <cstring>  // memset
#include <cstdio>   // 디버깅용 (필요 시)

// =========================================================
// 내부 get_state 함수 (Kalman 전용)
// =========================================================
static void kalman_vec3_get_state_internal(void* filter, vec3_t* out_pos, vec3_t* out_vel) {
    if (!filter) return;
    kalman_filter_vec3_t* kf = (kalman_filter_vec3_t*)filter;
    if (out_pos) *out_pos = kf->position;
    if (out_vel) *out_vel = kf->velocity;
}

// =========================================================
// Kalman Filter 어댑터 함수
// =========================================================
filter_interface_t make_kalman_vec3_interface(kalman_filter_vec3_t* kf) {
    filter_interface_t iface;
    iface.filter_state = kf;
    iface.time_update = (filter_time_update_func)kalman_vec3_time_update;
    iface.measurement_update = (filter_measurement_update_func)kalman_vec3_measurement_update;
    iface.get_state = kalman_vec3_get_state_internal;
    return iface;
}

// =========================================================
// (추가 예정) EKF 어댑터 함수
// =========================================================
// 예시로 구조만 정의
/*
filter_interface_t make_ekf_vec3_interface(ekf_filter_vec3_t* ekf) {
    filter_interface_t iface;
    iface.filter_state = ekf;
    iface.time_update = (filter_time_update_func)ekf_vec3_time_update;
    iface.measurement_update = (filter_measurement_update_func)ekf_vec3_measurement_update;
    iface.get_state = ekf_vec3_get_state_internal;
    return iface;
}
*/

// =========================================================
// (추가 예정) UKF 어댑터 함수
// =========================================================
// 예시로 구조만 정의
/*
filter_interface_t make_ukf_vec3_interface(ukf_filter_vec3_t* ukf) {
    filter_interface_t iface;
    iface.filter_state = ukf;
    iface.time_update = (filter_time_update_func)ukf_vec3_time_update;
    iface.measurement_update = (filter_measurement_update_func)ukf_vec3_measurement_update;
    iface.get_state = ukf_vec3_get_state_internal;
    return iface;
}
*/
