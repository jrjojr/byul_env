#include "internal/trajectory.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------
// linear_state_t 유틸리티
// ---------------------------------------------------------
void linear_state_init(linear_state_t* out) {
    if (!out) return;
    vec3_zero(&out->position);
    vec3_zero(&out->velocity);
    vec3_zero(&out->acceleration);
}

void linear_state_init_full(linear_state_t* out,
                            const vec3_t* position,
                            const vec3_t* velocity,
                            const vec3_t* acceleration) {
    if (!out) return;
    out->position = *position;
    out->velocity = *velocity;
    out->acceleration = *acceleration;
}

void linear_state_copy(linear_state_t* out, const linear_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// attitude_state_t 유틸리티
// ---------------------------------------------------------
void attitude_state_init(attitude_state_t* out) {
    if (!out) return;
    quat_identity(&out->orientation);
    vec3_zero(&out->angular_velocity);
    vec3_zero(&out->angular_acceleration);
}

void attitude_state_init_full(attitude_state_t* out,
                              const quat_t* orientation,
                              const vec3_t* angular_velocity,
                              const vec3_t* angular_acceleration) {
    if (!out) return;
    out->orientation = *orientation;
    out->angular_velocity = *angular_velocity;
    out->angular_acceleration = *angular_acceleration;
}

void attitude_state_copy(attitude_state_t* out, const attitude_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// motion_state_t 유틸리티
// ---------------------------------------------------------
void motion_state_init(motion_state_t* out) {
    if (!out) return;
    linear_state_init(&out->linear);
    attitude_state_init(&out->angular);
}

void motion_state_init_full(motion_state_t* out,
                            const vec3_t* position,
                            const vec3_t* velocity,
                            const vec3_t* acceleration,
                            const quat_t* orientation,
                            const vec3_t* angular_velocity,
                            const vec3_t* angular_acceleration) {
    if (!out) return;
    linear_state_init_full(&out->linear, position, velocity, acceleration);
    attitude_state_init_full(&out->angular, 
        orientation, angular_velocity, angular_acceleration);
}

void motion_state_copy(motion_state_t* out, const motion_state_t* src) {
    if (!out || !src) return;
    *out = *src;
}

trajectory_t* trajectory_create_full(int capacity) {
    if (capacity <= 0) return nullptr;

    trajectory_t* traj = new trajectory_t;
    traj->samples = new trajectory_sample_t[capacity];
    traj->count = 0;
    traj->capacity = capacity;

    return traj;
}

// 기본 capacity = 100개 기본값
trajectory_t* trajectory_create(){
    return trajectory_create_full(100);
}

void trajectory_free(trajectory_t* traj){
    if (!traj) return;
    delete[] traj->samples;
    traj->samples = nullptr;
    traj->count = 0;
    traj->capacity = 0;    
}

void trajectory_destroy(trajectory_t* traj){
    if (!traj) return;
    trajectory_free(traj);
    delete traj;
}

void trajectory_copy(trajectory_t* out, const trajectory_t* src) {
    if (!out || !src) return;
    if (out->capacity < src->count) {
        delete[] out->samples;
        out->samples = new trajectory_sample_t[src->capacity];
        out->capacity = src->capacity;
    }
    out->count = src->count;
    memcpy(out->samples, src->samples,
           sizeof(trajectory_sample_t) * src->count);
}

trajectory_t* trajectory_clone(const trajectory_t* src) {
    if (!src) return nullptr;

    trajectory_t* traj = trajectory_create_full(src->capacity);
    traj->count = src->count;

    if (src->count > 0 && src->samples) {
        memcpy(traj->samples, src->samples,
               sizeof(trajectory_sample_t) * src->count);
    }

    return traj;
}

// ---------------------------------------------------------
// trajectory 초기화 (0으로 세팅)
// ---------------------------------------------------------
void trajectory_clear(trajectory_t* traj) {
    if (!traj) return;
    traj->count = 0;
    if (traj->samples) {
        memset(traj->samples, 0,
               sizeof(trajectory_sample_t) * traj->capacity);
    }
}

// ---------------------------------------------------------
// trajectory 샘플 추가
// ---------------------------------------------------------
bool trajectory_add_sample(
    trajectory_t* traj, float t, const motion_state_t* state) {

    if (!traj || !state || traj->count >= traj->capacity) return false;
    trajectory_sample_t& sample = traj->samples[traj->count++];
    sample.t = t;
    sample.state = *state;
    return true;
}



int trajectory_length(const trajectory_t* traj) {
    return (traj ? traj->count : 0);
}

int trajectory_capacity(const trajectory_t* traj) {
    return (traj ? traj->capacity : 0);
}

// ---------------------------------------------------------
// 문자열 버퍼로 trajectory 출력
// ---------------------------------------------------------
char* trajectory_to_string(
    const trajectory_t* traj, char* buffer, size_t size) {

    if (!traj || !buffer || size == 0) return buffer;
    size_t offset = 0;
    int n = snprintf(buffer + offset, size - offset,
        "---- Trajectory Samples (count=%d) ----\n", traj->count);
    if (n < 0) return buffer;
    offset += static_cast<size_t>(n);

    for (int i = 0; i < traj->count && offset < size; ++i) {
        const trajectory_sample_t* s = &traj->samples[i];
        const vec3_t* p = &s->state.linear.position;
        const vec3_t* v = &s->state.linear.velocity;

        n = snprintf(buffer + offset, size - offset,
            " t=%.3f  pos=(%.3f, %.3f, %.3f)  vel=(%.3f, %.3f, %.3f)\n",
            s->t, p->x, p->y, p->z, v->x, v->y, v->z);
        if (n < 0) break;
        offset += static_cast<size_t>(n);
    }
    return buffer;
}

// ---------------------------------------------------------
// 콘솔로 trajectory 출력
// ---------------------------------------------------------
void trajectory_print(const trajectory_t* traj) {
    if (!traj) return;
    printf("---- Trajectory Samples (count=%d) ----\n", traj->count);
    printf("    t(s)        pos(x,y,z)              vel(x,y,z)\n");
    printf("-----------------------------------------------------------\n");

    for (int i = 0; i < traj->count; ++i) {
        const trajectory_sample_t* s = &traj->samples[i];
        const vec3_t* p = &s->state.linear.position;
        const vec3_t* v = &s->state.linear.velocity;
        printf(" %6.3f   (%.3f, %.3f, %.3f)   (%.3f, %.3f, %.3f)\n",
               s->t, p->x, p->y, p->z, v->x, v->y, v->z);
    }
    printf("-----------------------------------------------------------\n");
}

// ---------------------------------------------------------
// 위치 벡터 리스트 추출
// ---------------------------------------------------------
int trajectory_get_positions(
    const trajectory_t* traj, vec3_t* out_list, int max) {

    if (!traj || !out_list || max <= 0) return 0;
    int count = (traj->count < max) ? traj->count : max;
    for (int i = 0; i < count; ++i) {
        out_list[i] = traj->samples[i].state.linear.position;
    }
    return count;
}

// ---------------------------------------------------------
// 속력 리스트 추출
// ---------------------------------------------------------
int trajectory_get_speeds(
    const trajectory_t* traj, float* out_list, int max) {

    if (!traj || !out_list || max <= 0) return 0;
    int count = (traj->count < max) ? traj->count : max;
    for (int i = 0; i < count; ++i) {
        const vec3_t* v = &traj->samples[i].state.linear.velocity;
        out_list[i] = std::sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
    }
    return count;
}

// ---------------------------------------------------------
// 특정 시간의 타겟 위치를 보간
// ---------------------------------------------------------
bool trajectory_sample_position(
    const trajectory_t* traj, float t, vec3_t* out_pos) {
        
    if (!traj || traj->count < 1 || !out_pos) return false;
    if (t <= traj->samples[0].t) {
        *out_pos = traj->samples[0].state.linear.position;
        return true;
    }
    if (t >= traj->samples[traj->count - 1].t) {
        *out_pos = traj->samples[traj->count - 1].state.linear.position;
        return true;
    }
    for (int i = 0; i < traj->count - 1; i++) {
        const trajectory_sample_t* s1 = &traj->samples[i];
        const trajectory_sample_t* s2 = &traj->samples[i + 1];
        if (t >= s1->t && t <= s2->t) {
            float alpha = (t - s1->t) / (s2->t - s1->t);
            vec3_lerp(out_pos, &s1->state.linear.position, 
                &s2->state.linear.position, alpha);
            return true;
        }
    }
    return false;
}
