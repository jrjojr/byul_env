#include "numeq_filters.h"

static void kalman_vec3_get_state_internal(void* filter, vec3_t* out_pos, vec3_t* out_vel) {
    if (!filter) return;
    kalman_filter_vec3_t* kf = (kalman_filter_vec3_t*)filter;
    if (out_pos) *out_pos = kf->position;
    if (out_vel) *out_vel = kf->velocity;
}

filter_interface_t make_kalman_vec3_interface(kalman_filter_vec3_t* kf) {
    filter_interface_t iface;
    iface.filter_state = kf;
    iface.time_update = (filter_time_update_func)kalman_vec3_time_update;
    iface.measurement_update = (filter_measurement_update_func)kalman_vec3_measurement_update;
    iface.get_state = kalman_vec3_get_state_internal;
    return iface;
}
