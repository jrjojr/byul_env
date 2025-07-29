#include "trajectory.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>

trajectory_t* trajectory_create_full(int capacity) {
    if (capacity <= 0) return nullptr;

    trajectory_t* traj = new trajectory_t;
    traj->samples = new trajectory_sample_t[capacity];
    traj->count = 0;
    traj->capacity = capacity;

    return traj;
}

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

void trajectory_init(trajectory_t* traj) {
    if (!traj) return;

    if (traj->samples && traj->capacity > 0) {
        delete[] traj->samples;
    }
    traj->capacity = 100;
    traj->samples = new trajectory_sample_t[traj->capacity];
    traj->count = 0;
}

void trajectory_init_full(trajectory_t* traj, int capacity) {
    if (!traj || capacity <= 0) return;
    if (traj->samples) {
        delete[] traj->samples;
    }
    traj->capacity = capacity;
    traj->samples = new trajectory_sample_t[traj->capacity];
    traj->count = 0;
}

void trajectory_destroy(trajectory_t* traj){
    if (!traj) return;
    trajectory_free(traj);
    delete traj;
}

void trajectory_assign(trajectory_t* out, const trajectory_t* src) {
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

trajectory_t* trajectory_copy(const trajectory_t* src) {
    if (!src) return nullptr;

    trajectory_t* traj = trajectory_create_full(src->capacity);
    traj->count = src->count;

    if (src->count > 0 && src->samples) {
        memcpy(traj->samples, src->samples,
               sizeof(trajectory_sample_t) * src->count);
    }

    return traj;
}

void trajectory_clear(trajectory_t* traj) {
    if (!traj) return;
    traj->count = 0;
    if (traj->samples) {
        memset(traj->samples, 0,
               sizeof(trajectory_sample_t) * traj->capacity);
    }
}

void trajectory_resize(trajectory_t* traj, int new_cap) {
    if (!traj || new_cap <= 0) return;

    if (new_cap == traj->capacity) return;

    trajectory_sample_t* new_samples = new trajectory_sample_t[new_cap];

    int copy_count = (traj->count < new_cap) ? traj->count : new_cap;
    for (int i = 0; i < copy_count; ++i) {
        new_samples[i] = traj->samples[i];
    }

    delete[] traj->samples;

    traj->samples = new_samples;
    traj->capacity = new_cap;
    traj->count = copy_count;
}

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

int trajectory_get_positions(
    const trajectory_t* traj, vec3_t* out_list, int max) {

    if (!traj || !out_list || max <= 0) return 0;
    int count = (traj->count < max) ? traj->count : max;
    for (int i = 0; i < count; ++i) {
        out_list[i] = traj->samples[i].state.linear.position;
    }
    return count;
}

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

bool trajectory_interpolate_position(
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

bool trajectory_estimate_velocity(
    const trajectory_t* traj, float t, vec3_t* out_vel) {

    if (!traj || traj->count < 2 || !out_vel) return false;

    if (t <= traj->samples[0].t) {
        *out_vel = traj->samples[0].state.linear.velocity;
        return true;
    }
    if (t >= traj->samples[traj->count - 1].t) {
        *out_vel = traj->samples[traj->count - 1].state.linear.velocity;
        return true;
    }

    for (int i = 0; i < traj->count - 1; ++i) {
        const trajectory_sample_t* s1 = &traj->samples[i];
        const trajectory_sample_t* s2 = &traj->samples[i + 1];
        if (t >= s1->t && t <= s2->t) {
            float dt = s2->t - s1->t;
            if (dt <= 1e-6f) {
                *out_vel = s1->state.linear.velocity;
                return true;
            }
            // velocity (p2 - p1) / dt
            vec3_sub(out_vel, &s2->state.linear.position, &s1->state.linear.position);
            vec3_scale(out_vel, out_vel, 1.0f / dt);
            return true;
        }
    }
    return false;
}

bool trajectory_estimate_acceleration(
    const trajectory_t* traj, float t, vec3_t* out_acc) {

    if (!traj || traj->count < 3 || !out_acc) return false;

    if (t <= traj->samples[0].t) {
        *out_acc = traj->samples[0].state.linear.acceleration;
        return true;
    }
    if (t >= traj->samples[traj->count - 1].t) {
        *out_acc = traj->samples[traj->count - 1].state.linear.acceleration;
        return true;
    }

    for (int i = 1; i < traj->count - 1; ++i) {
        const trajectory_sample_t* s0 = &traj->samples[i - 1];
        const trajectory_sample_t* s1 = &traj->samples[i];
        const trajectory_sample_t* s2 = &traj->samples[i + 1];
        if (t >= s0->t && t <= s2->t) {
            float dt0 = s1->t - s0->t;
            float dt1 = s2->t - s1->t;

            vec3_t v0, v1;
            vec3_sub(&v0, &s1->state.linear.position, &s0->state.linear.position);
            vec3_scale(&v0, &v0, 1.0f / dt0);
            vec3_sub(&v1, &s2->state.linear.position, &s1->state.linear.position);
            vec3_scale(&v1, &v1, 1.0f / dt1);

            // accel : (v1 - v0) / ((dt0 + dt1)/2)
            vec3_sub(out_acc, &v1, &v0);
            float avg_dt = 0.5f * (dt0 + dt1);
            vec3_scale(out_acc, out_acc, 1.0f / avg_dt);
            return true;
        }
    }
    return false;
}
