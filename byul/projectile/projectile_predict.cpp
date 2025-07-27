#include <cmath>
#include <cstdio>
#include "internal/projectile_predict.h"
#include "internal/guidance.h"
#include "internal/common.h"
#include "internal/numeq_filters.h"

// ---------------------------------------------------------
// projectile_result_create
// ---------------------------------------------------------
projectile_result_t* projectile_result_create() {
    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->valid = false;

    res->trajectory = trajectory_create();  // 기본 capacity = 100
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_create_full
// ---------------------------------------------------------
projectile_result_t* projectile_result_create_full(int capacity) {
    if (capacity <= 0) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->valid = false;

    res->trajectory = trajectory_create_full(capacity);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_copy
// ---------------------------------------------------------
projectile_result_t* projectile_result_copy(const projectile_result_t* src) {
    if (!src) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = src->impact_time;
    res->impact_pos = src->impact_pos;
    res->valid = src->valid;

    res->trajectory = trajectory_copy(src->trajectory);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

void projectile_result_reset(projectile_result_t* res)
{
    if (!res) return;

    res->impact_time = 0.0f;
    res->impact_pos = (vec3_t){0.0f, 0.0f, 0.0f};
    res->valid = false;

    if (res->trajectory) {
        trajectory_clear(res->trajectory);  // 내부 데이터만 초기화
    }
}

void projectile_result_resize(projectile_result_t* res, int new_capacity)
{
    if (!res) return;

    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
    }
    res->trajectory = trajectory_create_full(new_capacity);

    // 재할당 후 상태 초기화
    res->impact_time = 0.0f;
    res->impact_pos = (vec3_t){0.0f, 0.0f, 0.0f};
    res->valid = false;
}

void projectile_result_free(projectile_result_t* res)
{
    if (!res) return;

    // trajectory 메모리도 같이 해제
    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
        res->trajectory = NULL;
    }

    // 필드 초기화
    res->impact_time = 0.0f;
    res->impact_pos = (vec3_t){0.0f, 0.0f, 0.0f};
    res->valid = false;
}

// ---------------------------------------------------------
// projectile_result_destroy
// ---------------------------------------------------------
void projectile_result_destroy(projectile_result_t* res) {
    if (!res) return;
    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
        res->trajectory = nullptr;
    }
    delete res;
}

// ---------------------------------------------------------
// 내부 헬퍼 함수
// ---------------------------------------------------------

/**
 * @brief y=0 지점과 충돌하는 정확한 시간을 2차 방정식으로 계산.
 *        pos_prev -> pos_curr 구간에서만 보정 계산.
 */
static bool solve_ground_hit_time_interval(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    float dt,
    float* alpha) // 0 ~ 1
{
    float y0 = pos_prev->y;
    float vy = vel_prev->y;
    float ay = accel->y;

    // y(t) = y0 + vy*t + 0.5*ay*t^2
    if (fabsf(ay) < 1e-6f) {
        if (fabsf(vy) < 1e-6f) return false;
        *alpha = -y0 / (vy * dt);
        return (*alpha >= 0.0f && *alpha <= 1.0f);
    }

    // 0 = y0 + vy*t + 0.5*ay*t^2
    float a = 0.5f * ay;
    float b = vy;
    float c = y0;

    float disc = b*b - 4*a*c;
    if (disc < 0.0f) return false;

    float sqrt_disc = sqrtf(disc);
    float t1 = (-b - sqrt_disc) / (2*a);
    float t2 = (-b + sqrt_disc) / (2*a);

    float t_hit = (t1 >= 0 && t1 <= dt) ? t1 : ((t2 >= 0 && t2 <= dt) ? t2 : -1.0f);
    if (t_hit < 0) return false;

    *alpha = t_hit / dt;
    return true;
}

/**
 * @brief pos_prev -> pos_curr 구간에서 충돌 시점 보정 (지면)
 */
static bool detect_ground_collision_precise(
    const vec3_t* pos_prev,
    const vec3_t* pos_curr,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time)
{
    if (pos_prev->y > 0.0f && pos_curr->y <= 0.0f) {
        float alpha;
        if (!solve_ground_hit_time_interval(pos_prev, vel_prev, accel, dt, &alpha))
            return false;

        *impact_time = t_prev + alpha * dt;
        vec3_lerp(impact_pos, pos_prev, pos_curr, alpha);
        impact_pos->y = 0.0f;
        return true;
    }
    return false;
}

static bool solve_entity_hit_time(
    const vec3_t* rel_p,
    const vec3_t* rel_v,
    const vec3_t* rel_a,
    float R,
    float dt,
    float* t_hit)
{
    // 1/2 a 미리 계산
    vec3_t half_a = *rel_a;
    vec3_scale(&half_a, &half_a, 0.5f);

    // 계수 A, B, C 계산
    // s(t) = p + v t + 0.5 a t²
    // |s(t)|² = A t² + B t + C = R²
    float A = vec3_dot(rel_v, rel_v) +
              2.0f * vec3_dot(rel_v, &half_a) +
              vec3_dot(&half_a, &half_a);

    float B = 2.0f * (vec3_dot(rel_p, rel_v) + vec3_dot(rel_p, &half_a));

    float C = vec3_dot(rel_p, rel_p) - R * R;

    // A가 매우 작은 경우 → 1차 방정식 근사
    if (fabsf(A) < FLOAT_EPSILON) {
        float t;
        if (!numeq_solve_linear(B, C, &t)) return false;
        if (t >= 0.0f && t <= dt) { *t_hit = t; return true; }
        return false;
    }

    // 2차 방정식 해 찾기
    float x1, x2;
    if (!numeq_solve_quadratic(A, B, C, &x1, &x2)) return false;

    // [0, dt] 범위 내 가장 작은 양수 해 선택
    bool found = false;
    float best = dt + 1.0f;

    if (x1 >= 0.0f && x1 <= dt) { best = x1; found = true; }
    if (x2 >= 0.0f && x2 <= dt && (!found || x2 < best)) {
         best = x2; found = true; 
    }

    if (!found) return false;
    *t_hit = best;
    return true;
}

// static bool detect_entity_collision_precise(
//     const vec3_t* proj_pos_prev,
//     const vec3_t* proj_vel_prev,
//     const vec3_t* proj_accel,
//     const vec3_t* target_pos,
//     float target_radius,
//     float dt,
//     float t_prev,
//     vec3_t* impact_pos,
//     float* impact_time)
// {
//     // --- 상대 좌표계 변환 ---
//     vec3_t rel_p = *proj_pos_prev;
//     vec3_sub(&rel_p, &rel_p, target_pos);

//     vec3_t rel_v = *proj_vel_prev;
//     vec3_t rel_a = *proj_accel;

//     // --- 초기 선형 상태 생성 ---
//     linear_state_t state_prev;
//     linear_state_init(&state_prev);
//     state_prev.position = rel_p;
//     state_prev.velocity = rel_v;
//     state_prev.acceleration = rel_a;

//     // --- 이전/현재 거리 ---
//     float d_prev = vec3_length(&rel_p);
//     vec3_t rel_curr;
//     numeq_model_pos_at(dt, &state_prev, NULL, NULL, &rel_curr);
//     float d_curr = vec3_length(&rel_curr);

//     // --- 빠른 배제 조건 ---
//     if (d_prev <= target_radius) return false;   // 이전 프레임에서 이미 충돌
//     if (d_curr > d_prev) return false;           // 점점 멀어짐 → 충돌 불가

//     // --- 충돌 시간 계산 ---
//     float t_local;
//     if (!solve_entity_hit_time(&rel_p, &rel_v, &rel_a, target_radius, dt, &t_local))
//         return false;
//     if (t_local < 0.0f || t_local > dt) return false;

//     // --- 충돌 위치 계산 ---
//     *impact_time = t_prev + t_local;
//     numeq_model_pos_at(t_local, &state_prev, NULL, NULL, impact_pos);
//     vec3_add(impact_pos, impact_pos, target_pos); // 타겟 좌표계에서 월드 좌표로 변환

//     return true;
// }

static bool detect_entity_collision_precise(
    const vec3_t* proj_pos_prev,
    const vec3_t* proj_vel_prev,
    const vec3_t* proj_accel,
    const vec3_t* target_pos,
    float target_radius,
    float dt,
    float t_prev,
    vec3_t* impact_pos,
    float* impact_time)
{
    // --- 상대 좌표계 변환 ---
    vec3_t rel_p = *proj_pos_prev;
    vec3_sub(&rel_p, &rel_p, target_pos);

    vec3_t rel_v = *proj_vel_prev;
    vec3_t rel_a = *proj_accel;

    // --- 초기 선형 상태 생성 ---
    linear_state_t state_prev;
    linear_state_init(&state_prev);
    state_prev.position = rel_p;
    state_prev.velocity = rel_v;
    state_prev.acceleration = rel_a;

    // --- 이전/현재 거리 ---
    float d_prev = vec3_length(&rel_p);
    vec3_t rel_curr;
    numeq_model_pos_at(dt, &state_prev, NULL, NULL, &rel_curr);
    float d_curr = vec3_length(&rel_curr);

    // --- 빠른 배제: 이전 시점이 이미 충돌 상태 ---
    if (d_prev <= target_radius) {
        *impact_time = t_prev;
        *impact_pos = *proj_pos_prev;
        return true;
    }

    // --- 2차 방정식으로 충돌 시점 정확히 계산 ---
    float t_local;
    if (solve_entity_hit_time(&rel_p, &rel_v, &rel_a, target_radius, dt, &t_local)) {
        if (t_local >= 0.0f && t_local <= dt) {
            *impact_time = t_prev + t_local;
            numeq_model_pos_at(t_local, &state_prev, NULL, NULL, impact_pos);
            vec3_add(impact_pos, impact_pos, target_pos); // 타겟 좌표계에서 월드 좌표로 변환
            return true;
        }
    }

    // --- 거리 감소 여부 기반 추가 체크 (샘플 간격이 큰 경우 대비) ---
    if (d_prev > target_radius && d_curr < target_radius) {
        // 보간 실패 시, 선형 보간으로 근사 충돌 시점 추정
        float ratio = (d_prev - target_radius) / (d_prev - d_curr);
        float approx_t = ratio * dt;
        if (approx_t < 0.0f) approx_t = 0.0f;
        if (approx_t > dt) approx_t = dt;

        *impact_time = t_prev + approx_t;
        numeq_model_pos_at(approx_t, &state_prev, NULL, NULL, impact_pos);
        vec3_add(impact_pos, impact_pos, target_pos);
        return true;
    }

    return false;
}


// ---------------------------------------------------------
// projectile_predict()
// ---------------------------------------------------------
bool projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    propulsion_t* propulsion,
    guidance_func guidance_fn)
{
    if (!proj || !out || time_step <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    vec3_t target_pos = entdyn ? entdyn->xf.pos : (vec3_t){0, 0, 0};
    float target_radius = entdyn ? entity_size(&entdyn->base) : 0.0f;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) 
    ? proj->base.props.mass : 1.0f;
    float fuel = propulsion ? propulsion->fuel_remaining : 0.0f;

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / time_step);

    for (int step = 0; step < max_steps; ++step, t += time_step) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        // --------------------------
        // 1) 가속도 계산 (환경 + 추진력)
        // --------------------------
        vec3_t env_accel = {0, 0, 0};
        if (env) {
            entity_dynamic_calc_accel_env(
                &proj->base, &vel_prev, time_step, env, &env_accel);
        }

        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && fuel > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (entdyn) {
                vec3_sub(&guidance, &entdyn->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (entdyn) info.target = *entdyn;
                vec3_t vec;
                const vec3_t* g = guidance_fn(
                    &temp_proj.base, time_step, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            propulsion_update(propulsion, 100.0f, time_step);
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
            fuel -= propulsion->burn_rate * time_step;
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        // --------------------------
        // 2) RK4 적분
        // --------------------------
        integrator_config_t config;
        integrator_config_init_full(
            &config, INTEGRATOR_RK4_ENV,
            time_step, nullptr, env, &proj->base.props, nullptr);
        numeq_integrate(&state, &config);

        // 수평 마찰 보정
        bodyprops_apply_friction(
            &state.linear.velocity, &proj->base.props, time_step);

        // --------------------------
        // 3) trajectory 기록 (후처리 후)
        // --------------------------
        trajectory_add_sample(out->trajectory, t, &state);

        // --------------------------
        // 4) 충돌 감지 + 후처리
        // --------------------------
        bool ground_hit = false;
        // if (detect_ground_collision_precise(&pos_prev, &state.linear.position,
        //     &vel_prev, &state.linear.acceleration,
        //     t, time_step, &out->impact_pos, &out->impact_time)) 
        if (state.linear.position.y <= 0.0f){
            // // 위치를 충돌 지점으로 보정
            // state.linear.position = out->impact_pos;

            // // 반발 계수 적용
            // if (proj->base.props.restitution > 0.0f) {
            //     state.linear.velocity.y 
            //     = -vel_prev.y * proj->base.props.restitution;
            // } else {
            //     state.linear.velocity.y = 0.0f;
            // }

            // 수평 마찰 보정
            // bodyprops_apply_friction(
            //     &state.linear.velocity, &proj->base.props, time_step);

            ground_hit = true;
        }

        // 엔티티 충돌 감지
        if (entdyn) {
            float dist_prev = vec3_distance(&pos_prev, &target_pos);
            float dist_curr = vec3_distance(
                &state.linear.position, &target_pos);

            // if(dist_curr <= target_radius){
                if (detect_entity_collision_precise(
                        &pos_prev, &vel_prev, 
                        &state.linear.acceleration,
                        &target_pos, target_radius,
                        time_step, t-time_step, &out->impact_pos, &out->impact_time))
                {
                    out->valid = true;
                    return true;
                }
            }
        // }

        // --------------------------
        // 5) 정지 조건 체크
        // --------------------------
        // if (ground_hit && vec3_length(&state.linear.velocity) < FLOAT_EPSILON) {
        if(ground_hit){
            if(detect_ground_collision_precise(
                &pos_prev, &state.linear.position,
                &vel_prev, &state.linear.acceleration,
                t, time_step, &out->impact_pos, &out->impact_time)){

                out->valid = true;
                return true;
            }
        }
    }

    out->valid = false;
    return false;
}

bool projectile_predict_with_kalman_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn)
{
    if (!proj || !out || time_step <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    // --- 타겟 정보 ---
    vec3_t target_pos = entdyn ? entdyn->xf.pos : (vec3_t){0, 0, 0};
    float target_radius = entdyn ? entity_size(&entdyn->base) : 0.0f;

    // --- 초기 상태 설정 ---
    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) ? proj->base.props.mass : 1.0f;
    float fuel = propulsion ? propulsion->fuel_remaining : 0.0f;

    // --- Kalman Filter 초기화 ---
    kalman_filter_vec3_t kf;
    kalman_vec3_init_full(
        &kf,
        &state.linear.position,
        &state.linear.velocity,
        0.01f,      // process_noise: 환경 변동성 (바람 등)
        1.0f,       // measurement_noise: 센서 측정 노이즈
        time_step); // dt

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / time_step);

    for (int step = 0; step < max_steps; ++step, t += time_step) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        // --- 환경 가속도 ---
        vec3_t env_accel = {0, 0, 0};
        if (env) {
            numeq_model_accel(&state.linear, env, &proj->base.props, &env_accel);
        }

        // --- 추진력 및 유도 ---
        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && fuel > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (entdyn) {
                vec3_sub(&guidance, &entdyn->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (entdyn) info.target = *entdyn;
                vec3_t vec;
                const vec3_t* g = guidance_fn(&temp_proj.base, time_step, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
            fuel -= propulsion->burn_rate * time_step;
        }

        // 총 가속도 업데이트
        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        // --- Kalman Filter Predict & Measurement Update ---
        kalman_vec3_time_update(&kf);
        kalman_vec3_measurement_update(&kf, &state.linear.position);

        // Kalman 결과를 state에 반영
        state.linear.position = kf.position;
        state.linear.velocity = kf.velocity;

        // trajectory에 보정된 결과 기록
        trajectory_add_sample(out->trajectory, t, &state);

        // --- RK4 적분 ---
        integrator_config_t config;
        integrator_config_init_full(
            &config, INTEGRATOR_MOTION_RK4_ENV,
            time_step, nullptr, env, &proj->base.props, nullptr);

        numeq_integrate(&state, &config);

        // --- 엔티티 충돌 감지 ---
        if (entdyn) {
            float dist_prev = vec3_distance(&pos_prev, &target_pos);
            float dist_curr = vec3_distance(&state.linear.position, &target_pos);

            if (dist_prev > target_radius && dist_curr <= target_radius) {
                if (detect_entity_collision_precise(
                        &pos_prev, &vel_prev, 
                        &state.linear.acceleration,
                        &target_pos, target_radius,
                        time_step, t, &out->impact_pos, &out->impact_time))
                {
                    out->valid = true;
                    return true;
                }
            }
        }

        // --- 지면 충돌 감지 ---
        if (detect_ground_collision_precise(
                &pos_prev, &state.linear.position,
                &vel_prev, &state.linear.acceleration,
                t, time_step, &out->impact_pos, &out->impact_time))
        {
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

bool projectile_predict_with_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn,
    const filter_interface_t* filter_if)  // 인터페이스로 통일
{
    if (!proj || !out || time_step <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    vec3_t target_pos = entdyn ? entdyn->xf.pos : (vec3_t){0, 0, 0};
    float target_radius = entdyn ? entity_size(&entdyn->base) : 0.0f;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) ? proj->base.props.mass : 1.0f;
    float fuel = propulsion ? propulsion->fuel_remaining : 0.0f;

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / time_step);

    for (int step = 0; step < max_steps; ++step, t += time_step) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        // 환경 가속도
        vec3_t env_accel = {0, 0, 0};
        if (env) {
            numeq_model_accel(&state.linear, env, &proj->base.props, &env_accel);
        }

        // 추진력 및 유도
        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && fuel > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (entdyn) {
                vec3_sub(&guidance, &entdyn->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (entdyn) info.target = *entdyn;
                vec3_t vec;
                const vec3_t* g = guidance_fn(&temp_proj.base, time_step, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
            fuel -= propulsion->burn_rate * time_step;
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        // --- 필터 적용 ---
        if (filter_if && filter_if->time_update && filter_if->measurement_update) {
            filter_if->time_update(filter_if->filter_state);
            filter_if->measurement_update(filter_if->filter_state,
                                          &state.linear.position,
                                          &state.linear.velocity);
            if (filter_if->get_state) {
                vec3_t filtered_pos, filtered_vel;
                filter_if->get_state(filter_if->filter_state,
                                     &filtered_pos, &filtered_vel);
                state.linear.position = filtered_pos;
                state.linear.velocity = filtered_vel;
            }
        }

        trajectory_add_sample(out->trajectory, t, &state);

        // RK4 적분
        integrator_config_t config;
        integrator_config_init_full(&config, INTEGRATOR_MOTION_RK4_ENV,
                                    time_step, nullptr, env, &proj->base.props, nullptr);
        config.time_step = time_step;
        numeq_integrate(&state, &config);

        // 충돌 검사 (엔티티, 지면)
        if (entdyn) {
            float dist_prev = vec3_distance(&pos_prev, &target_pos);
            float dist_curr = vec3_distance(&state.linear.position, &target_pos);
            if (dist_prev > target_radius && dist_curr <= target_radius) {
                if (detect_entity_collision_precise(
                        &pos_prev, &vel_prev, 
                        &state.linear.acceleration,
                        &target_pos, target_radius,
                        time_step, t, &out->impact_pos, &out->impact_time))
                {
                    out->valid = true;
                    return true;
                }
            }
        }
        if (detect_ground_collision_precise(&pos_prev, &state.linear.position,
                                            &vel_prev, &state.linear.acceleration,
                                            t, time_step, &out->impact_pos, &out->impact_time))
        {
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

// ---------------------------------------------------------
// 내부 유틸 함수
// ---------------------------------------------------------
static inline float projectile_safe_mass(const projectile_t* proj) {
    return (proj && proj->base.props.mass > 1e-6f) 
    ? proj->base.props.mass : 1.0f;
}

static inline bool projectile_calc_horizontal(
    vec3_t* dir_out, 
    float* R_out, 
    const vec3_t* start, 
    const vec3_t* target) {

    vec3_t diff;
    vec3_sub(&diff, target, start);

    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    if (dir_out) {
        dir_out->x = diff.x / R;
        dir_out->y = 0.0f;
        dir_out->z = diff.z / R;
    }
    if (R_out) *R_out = R;
    return true;
}

// ---------------------------------------------------------
// 발사 파라미터 계산 (환경 미적용)
// ---------------------------------------------------------
bool projectile_calc_launch_param(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force)
{
    if (!out || !proj || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t dir;
    float R;
    if (!projectile_calc_horizontal(&dir, &R, &start, target)) return false;

    float Dy = target->y - start.y;
    float mass = projectile_safe_mass(proj);
    float a0 = initial_force / mass;
    float v0 = sqrtf(2.0f * a0 * R);
    float g = 9.8f;

    float under_sqrt = v0 * v0 * v0 * v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0 * v0 - sqrtf(under_sqrt)) / (g * R));

    // direction = 발사 각도를 포함한 단위 벡터
    out->direction.x = cosf(theta) * dir.x;
    out->direction.y = sinf(theta);
    out->direction.z = cosf(theta) * dir.z;
    vec3_normalize(&out->direction);

    // force와 예상 시간
    out->force = initial_force;
    out->time_to_hit = R / (v0 * cosf(theta));
    return true;
}

// ---------------------------------------------------------
// 발사 파라미터 계산 (환경 적용)
// ---------------------------------------------------------
bool projectile_calc_launch_param_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force)
{
    if (!out || !proj || !env || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t dir;
    float R;
    if (!projectile_calc_horizontal(&dir, &R, &start, target)) return false;

    float Dy = target->y - start.y;
    float mass = projectile_safe_mass(proj);
    float a0 = initial_force / mass;
    float g = fabsf(env->gravity.y) > 1e-6f ? fabsf(env->gravity.y) : 9.8f;

    float v0 = sqrtf(2.0f * a0 * R);
    float under_sqrt = v0 * v0 * v0 * v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0 * v0 - sqrtf(under_sqrt)) / (g * R));

    // 발사 방향: 바람 보정 전
    out->direction.x = cosf(theta) * dir.x;
    out->direction.y = sinf(theta);
    out->direction.z = cosf(theta) * dir.z;
    vec3_normalize(&out->direction);

    // force와 예상 시간 (바람 영향 보정)
    float wind_h 
    = sqrtf(env->wind.x * env->wind.x + env->wind.z * env->wind.z);
    float v_h = v0 * cosf(theta) + wind_h;
    out->force = initial_force;
    out->time_to_hit = R / (v_h > 1e-3f ? v_h : 1e-3f);

    return true;
}

// ---------------------------------------------------------
// 목표 도달 시간(hit_time)이 정해진 경우의 초기 힘 계산
// ---------------------------------------------------------
bool projectile_calc_launch_param_inverse(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float hit_time)
{
    if (!out || !proj || !target || hit_time <= 0.0f) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t delta;
    vec3_sub(&delta, target, &start);

    // y방향 중력 보정 (단순 모델)
    vec3_t gravity_term = { 0.0f, -0.5f * 9.81f * hit_time * hit_time, 0.0f };

    vec3_t required_vel = {
        (delta.x - gravity_term.x) / hit_time,
        (delta.y - gravity_term.y) / hit_time,
        (delta.z - gravity_term.z) / hit_time
    };

    float mass = projectile_safe_mass(proj);
    float required_force = mass * vec3_length(&required_vel);

    vec3_unit(&out->direction, &required_vel);
    out->force = required_force;
    out->time_to_hit = hit_time;
    return true;
}

bool projectile_calc_launch_param_inverse_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float hit_time)
{
    if (!out || !proj || !env || !target || hit_time <= 0.0f) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t delta;
    vec3_sub(&delta, target, &start);

    vec3_t gravity_term = {
        0.0f,
        -0.5f * fabsf(env->gravity.y) * hit_time * hit_time,
        0.0f
    };

    vec3_t required_vel = {
        (delta.x - gravity_term.x - env->wind.x * hit_time) / hit_time,
        (delta.y - gravity_term.y - env->wind.y * hit_time) / hit_time,
        (delta.z - gravity_term.z - env->wind.z * hit_time) / hit_time
    };

    float mass = projectile_safe_mass(proj);
    float required_force = mass * vec3_length(&required_vel);

    vec3_unit(&out->direction, &required_vel);
    out->force = required_force;
    out->time_to_hit = hit_time;
    return true;
}
