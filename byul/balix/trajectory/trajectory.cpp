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

    traj->impact_time = 0.0f;
    traj->impact_pos = {};

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

    traj->impact_time = 0.0f;
    traj->impact_pos = {};    
}

void trajectory_init(trajectory_t* traj) {
    if (!traj) return;

    if (traj->samples && traj->capacity > 0) {
        delete[] traj->samples;
    }
    traj->capacity = 100;
    traj->samples = new trajectory_sample_t[traj->capacity];
    traj->count = 0;

    traj->impact_time = 0.0f;
    traj->impact_pos = {};    
}

void trajectory_init_full(trajectory_t* traj, int capacity) {
    if (!traj || capacity <= 0) return;
    if (traj->samples) {
        delete[] traj->samples;
    }
    traj->capacity = capacity;
    traj->samples = new trajectory_sample_t[traj->capacity];
    traj->count = 0;

    traj->impact_time = 0.0f;
    traj->impact_pos = {};    
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
    out->impact_pos = src->impact_pos;
    out->impact_time = src->impact_time;
    memcpy(out->samples, src->samples,
           sizeof(trajectory_sample_t) * src->count);
}

trajectory_t* trajectory_copy(const trajectory_t* src) {
    if (!src) return nullptr;

    trajectory_t* traj = trajectory_create_full(src->capacity);
    traj->count = src->count;
    traj->impact_pos = src->impact_pos;
    traj->impact_time = src->impact_time;    

    if (src->count > 0 && src->samples) {
        memcpy(traj->samples, src->samples,
               sizeof(trajectory_sample_t) * src->count);
    }

    return traj;
}

void trajectory_clear(trajectory_t* traj) {
    if (!traj) return;
    traj->count = 0;

    traj->impact_time = 0.0f;
    traj->impact_pos = {};    
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
    const trajectory_t* traj, size_t size, char* buffer) {

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
    char buf[64];
    n = snprintf(buffer + offset, size - offset,
        "impact time : %f, impact pos : %s\n", traj->impact_time, 
        vec3_to_string(&traj->impact_pos, 64, buf));        
    
    offset += static_cast<size_t>(n);        
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

    char buf[64];
    printf("impact time : %f, impact pos : %s\n", traj->impact_time, 
        vec3_to_string(&traj->impact_pos, 64, buf));
        
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

static inline int clamp_count(int n, int maxc){ return (n < maxc) ? n : maxc; }

int trajectory_export_pos(
    const trajectory_t* traj,
    float* out_times, vec3_t* out_positions,
    int max_count)
{
    if (!traj || traj->count <= 0 || max_count <= 0) return 0;
    const int n = clamp_count(traj->count, max_count);
    for (int i = 0; i < n; ++i) {
        if (out_times)      out_times[i]      = traj->samples[i].t;
        if (out_positions)  out_positions[i]  = traj->samples[i].state.linear.position;
    }
    return n;
}

int trajectory_export_until_impact(
    const trajectory_t* traj,
    float* out_times, vec3_t* out_positions,
    int max_count)
{
    if (!traj || traj->count <= 0 || max_count <= 0) return 0;

    float t_cut = traj->impact_time;
    const bool has_cut = (t_cut > 0.0f);

    int written = 0;
    for (int i = 0; i < traj->count && written < max_count; ++i) {
        const float ti = traj->samples[i].t;
        const vec3_t pi = traj->samples[i].state.linear.position;

        if (!has_cut || ti <= t_cut) {
            if (out_times)     out_times[written]     = ti;
            if (out_positions) out_positions[written] = pi;
            ++written;
        } else {
            // cut between samples[i-1] and samples[i]
            if (i > 0 && written < max_count) {
                const float t0 = traj->samples[i-1].t;
                const float t1 = ti;
                const vec3_t p0 = traj->samples[i-1].state.linear.position;
                const vec3_t p1 = pi;
                const float u = (t1 > t0) ? (t_cut - t0) / (t1 - t0) : 0.0f;
                vec3_t pcut;
                vec3_lerp(&pcut, &p0, &p1, (u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u)));
                if (out_times)     out_times[written]     = t_cut;
                if (out_positions) out_positions[written] = pcut;
                ++written;
            }
            break;
        }
    }
    return written;
}

int trajectory_export_resample_time(
    const trajectory_t* traj,
    float dt,
    float* out_times, vec3_t* out_positions,
    int max_count)
{
    if (!traj || traj->count <= 0 || max_count <= 0) return 0;
    if (dt <= 0.0f) 
        return trajectory_export_until_impact(
            traj, out_times, out_positions, max_count);

    const float t0 = traj->samples[0].t;
    const float tN_full = traj->samples[traj->count - 1].t;
    const float tN = (traj->impact_time > 0.0f && traj->impact_time < tN_full)
                   ? traj->impact_time : tN_full;

    int i = 0; // segment index
    int written = 0;

    for (float t = t0; t <= tN + 1e-6f && written < max_count; t += dt) {
        // advance segment that contains t
        while (i + 1 < traj->count && traj->samples[i+1].t < t) {
            ++i;
        }

        // last exact sample or beyond
        if (i + 1 >= traj->count) {
            const vec3_t p = traj->samples[traj->count - 1].state.linear.position;
            if (out_times)     out_times[written]     = tN;
            if (out_positions) out_positions[written] = p;
            ++written;
            break;
        }

        const float t0s = traj->samples[i].t;
        const float t1s = traj->samples[i+1].t;
        const vec3_t p0 = traj->samples[i].state.linear.position;
        const vec3_t p1 = traj->samples[i+1].state.linear.position;

        float u = (t1s > t0s) ? (t - t0s) / (t1s - t0s) : 0.0f;
        if (u < 0.0f) u = 0.0f; else if (u > 1.0f) u = 1.0f;

        vec3_t p; vec3_lerp(&p, &p0, &p1, u);

        if (out_times)     out_times[written]     = t;
        if (out_positions) out_positions[written] = p;
        ++written;
    }

    // ensure last cut at impact exactly present
    if (traj->impact_time > 0.0f 
        && traj->impact_time < tN_full 
        && written < max_count) {

        // add impact point exactly
        float tcut = traj->impact_time;
        // find segment containing tcut
        int j = 0;
        while (j + 1 < traj->count && traj->samples[j+1].t < tcut) ++j;
        if (j + 1 < traj->count) {
            const float ta = traj->samples[j].t;
            const float tb = traj->samples[j+1].t;
            const vec3_t pa = traj->samples[j].state.linear.position;
            const vec3_t pb = traj->samples[j+1].state.linear.position;
            const float u = (tb > ta) ? (tcut - ta) / (tb - ta) : 0.0f;
            vec3_t pc; vec3_lerp(&pc, &pa, &pb, (u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u)));
            if (out_times)     out_times[written]     = tcut;
            if (out_positions) out_positions[written] = pc;
            ++written;
        }
    }

    return written;
}

int trajectory_export_resample_distance(
    const trajectory_t* traj,
    float spacing,
    float* out_times, vec3_t* out_positions,
    int max_count)
{
    if (!traj || traj->count <= 0 || max_count <= 0) return 0;
    if (spacing <= 0.0f) 
        return trajectory_export_until_impact(
            traj, out_times, out_positions, max_count);

    const int N = traj->count;
    const float tN_full = traj->samples[N - 1].t;
    const float t_cut = (traj->impact_time > 0.0f && traj->impact_time < tN_full)
                      ? traj->impact_time : tN_full;
    // write first point
    int written = 0;
    if (out_times)     out_times[written]     = traj->samples[0].t;
    if (out_positions) out_positions[written] = traj->samples[0].state.linear.position;
    ++written;

    float carry = 0.0f;
    vec3_t last_p = traj->samples[0].state.linear.position;
    float last_t  = traj->samples[0].t;

    for (int i = 1; i < N && written < max_count; ++i) {
        // stop at cut time if needed
        if (traj->samples[i].t > t_cut) {
            // interpolate to exact cut
            const float t0 = traj->samples[i-1].t;
            const float t1 = traj->samples[i].t;
            const vec3_t p0 = traj->samples[i-1].state.linear.position;
            const vec3_t p1 = traj->samples[i].state.linear.position;
            const float ucut = (t1 > t0) ? (t_cut - t0) / (t1 - t0) : 0.0f;
            vec3_t pcut; vec3_lerp(&pcut, &p0, &p1, (ucut < 0.0f ? 0.0f : (ucut > 1.0f ? 1.0f : ucut)));

            // fill remaining spaced points up to cut
            float seg_len = vec3_distance(&last_p, &pcut);
            while (written < max_count && carry + seg_len >= spacing) {
                const float need = spacing - carry;
                const float u = (seg_len > 0.0f) ? (need / seg_len) : 1.0f;
                vec3_t q; vec3_lerp(&q, &last_p, &pcut, u);
                float tq = last_t + (t_cut - last_t) * u;
                if (out_times)     out_times[written]     = tq;
                if (out_positions) out_positions[written] = q;
                ++written;
                // advance within this partial segment
                last_p = q;
                last_t = tq;
                seg_len = vec3_distance(&last_p, &pcut);
                carry = 0.0f;
            }

            // 마지막 지점으로 스냅
            if (written < max_count) {
                if (out_times)     out_times[written]     = t_cut;
                if (out_positions) out_positions[written] = pcut;
                ++written;
            }
            return written;
        }

        // regular segment i-1 -> i
        const vec3_t cur_p = traj->samples[i].state.linear.position;
        const float  cur_t = traj->samples[i].t;

        float seg_len = vec3_distance(&last_p, &cur_p);
        // 채워 넣을 수 있을 만큼 반복
        while (written < max_count && carry + seg_len >= spacing) {
            const float need = spacing - carry;
            const float u = (seg_len > 0.0f) ? (need / seg_len) : 1.0f;

            vec3_t q; vec3_lerp(&q, &last_p, &cur_p, u);
            float tq = last_t + (cur_t - last_t) * u;

            if (out_times)     out_times[written]     = tq;
            if (out_positions) out_positions[written] = q;
            ++written;

            // 다음 간격 계산 준비
            last_p = q;
            last_t = tq;
            seg_len = vec3_distance(&last_p, &cur_p);
            carry = 0.0f;
        }

        // 간격이 남았다면 누적
        carry += seg_len;
        last_p = cur_p;
        last_t = cur_t;
    }

    // cut이 없고 마지막 점이 아직 아니면 마지막 점 보장
    if (written < max_count) {
        if (out_times)     out_times[written]     = last_t;
        if (out_positions) out_positions[written] = last_p;
        ++written;
    }
    return written;
}
