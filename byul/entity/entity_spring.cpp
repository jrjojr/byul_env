#include "internal/entity_spring.h"
#include <cmath>
#include <cstring>

// ---------------------------------------------------------
// 내부 유틸리티
// ---------------------------------------------------------
static inline vec3_t vec3_subtract(const vec3_t& a, const vec3_t& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline float vec3_length(const vec3_t& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static inline vec3_t vec3_normalize(const vec3_t& v) {
    float len = vec3_length(v);
    if (len > 1e-6f)
        return {v.x / len, v.y / len, v.z / len};
    return {0.0f, 0.0f, 0.0f};
}

static inline vec3_t vec3_scale(const vec3_t& v, float s) {
    return {v.x * s, v.y * s, v.z * s};
}

static inline vec3_t vec3_add(const vec3_t& a, const vec3_t& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

// ---------------------------------------------------------
// 스프링 힘 계산 (두 엔티티 간)
// ---------------------------------------------------------
vec3_t entity_dynamic_calc_spring_force(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float k,
    float c,
    float L0)
{
    // 두 위치 간 벡터
    vec3_t delta = vec3_subtract(self->xf.position, other->xf.position);
    float dist = vec3_length(delta);

    // 정규화 방향
    vec3_t dir = vec3_normalize(delta);

    // 상대 속도 계산
    vec3_t rel_vel = vec3_subtract(self->velocity, other->velocity);
    float vel_dot = rel_vel.x * dir.x + rel_vel.y * dir.y + rel_vel.z * dir.z;

    // Hooke's Law + 감쇠
    float displacement = dist - L0;
    float force_mag = -k * displacement - c * vel_dot;

    return vec3_scale(dir, force_mag);
}

// ---------------------------------------------------------
// self 엔티티가 모든 엔티티로부터 받는 총합 스프링 힘
// ---------------------------------------------------------
vec3_t entity_dynamic_calc_spring_total(
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float k,
    float c,
    float L0)
{
    vec3_t total = {0, 0, 0};
    for (int i = 0; i < count; i++) {
        const entity_dynamic_t* other = &others[i];
        if (other == self) continue;
        vec3_t f = entity_dynamic_calc_spring_force(self, other, k, c, L0);
        total = vec3_add(total, f);
    }
    return total;
}

// ---------------------------------------------------------
// 스프링 기반 거리 유지 시뮬레이션
// ---------------------------------------------------------
void entity_dynamic_auto_spring(
    entity_dynamic_t* e, int count,
    entity_spring_traj_t* traj,
    float dt, float k, float c, float L0,
    int steps)
{
    for (int step = 0; step < steps; step++) {
        // 각 엔티티의 스프링 힘 계산
        vec3_t forces[SPRING_MAX_STEP];
        for (int i = 0; i < count; i++) {
            forces[i] = entity_dynamic_calc_spring_total(&e[i], e, count, k, c, L0);
        }

        // 속도 및 위치 업데이트
        for (int i = 0; i < count; i++) {
            // a = F/m
            float inv_mass = (e[i].props.mass > 1e-6f) ? 1.0f / e[i].props.mass : 1.0f;
            vec3_t accel = vec3_scale(forces[i], inv_mass);

            // v = v + a*dt
            e[i].velocity = vec3_add(e[i].velocity, vec3_scale(accel, dt));

            // p = p + v*dt
            e[i].xf.position = vec3_add(e[i].xf.position, vec3_scale(e[i].velocity, dt));

            // 궤적 기록
            if (traj) {
                traj[i].path[traj[i].count] = e[i].xf.position;
                traj[i].force[traj[i].count] = forces[i];
                traj[i].count++;
                if (traj[i].count >= SPRING_MAX_STEP)
                    traj[i].count = SPRING_MAX_STEP - 1;
            }
        }
    }
}
