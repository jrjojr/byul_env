#include "internal/environ.h"
#include "internal/numeq_model.h"
#include "internal/vec3.h"
#include <math.h>
#include <string.h>
#include "internal/common.h"

// ---------------------------------------------------------
// environ_t 초기화 및 복사
// ---------------------------------------------------------

void environ_init(environ_t* env) {
    if (!env) return;
    env->gravity = (vec3_t){0.0f, -9.8f, 0.0f};
    vec3_zero(&env->wind);
    env->air_density = 1.225f;   // 해수면 기준 (kg/m³)
    env->humidity = 50.0f;       // 기본 습도 [%]
    env->temperature = 20.0f;    // 기본 온도 [°C]
    env->pressure = 101325.0f;   // 해수면 기압 [Pa]
}

void environ_init_full(environ_t* env,
                       const vec3_t* gravity,
                       const vec3_t* wind,
                       float air_density,
                       float humidity,
                       float temperature,
                       float pressure) {
    if (!env) return;
    env->gravity = gravity ? *gravity : (vec3_t){0.0f, -9.8f, 0.0f};
    env->wind = wind ? *wind : (vec3_t){0.0f, 0.0f, 0.0f};
    env->air_density = air_density;
    env->humidity = humidity;
    env->temperature = temperature;
    env->pressure = pressure;
}

void environ_assign(environ_t* out, const environ_t* src) {
    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// 외부 가속도 계산
// ---------------------------------------------------------

void environ_get_accel(vec3_t* out, const environ_t* env) {
    if (!out) return;

    if (!env) {
        *out = (vec3_t){0.0f, -9.81f, 0.0f};
        return;
    }

    out->x = env->gravity.x + env->wind.x;
    out->y = env->gravity.y + env->wind.y;
    out->z = env->gravity.z + env->wind.z;
}

void environ_get_accel_ext(vec3_t* out,
                           const environ_t* env,
                           const bodyprops_t* body,
                           const vec3_t* velocity) {
    if (!out) return;

    // 1. 기본 중력 설정
    vec3_t gravity = {0.0f, -9.81f, 0.0f};
    vec3_t wind = {0.0f, 0.0f, 0.0f};
    float air_density = 1.225f;

    if (env) {
        gravity = env->gravity;
        wind = env->wind;
        air_density = env->air_density;
    }

    // 2. 중력 + 바람 가속도 초기화
    *out = (vec3_t){
        gravity.x + wind.x,
        gravity.y + wind.y,
        gravity.z + wind.z
    };

    // 3. 공기 저항 계산 (drag = 0.5 * Cd * ρ * A * v² / m)
    if (body && velocity && body->mass > 0.0f) {
        float speed = vec3_length(velocity);
        if (speed > 0.0001f) {
            vec3_t drag_dir;
            vec3_unit(&drag_dir, velocity);

            float drag_mag = 0.5f * body->drag_coef *
                             air_density *
                             body->cross_section *
                             speed * speed;

            // drag_accel = -drag * direction / mass
            vec3_t drag_accel;
            vec3_scale(&drag_accel, &drag_dir, -drag_mag / body->mass);

            // 4. 총 가속도에 drag 더하기
            vec3_add(out, out, &drag_accel);
        }
    }
}

// ---------------------------------------------------------
// 기본 환경 함수
// ---------------------------------------------------------

const vec3_t* environ_func_none(
    vec3_t* out_accel,
    float dt,
    void* userdata)
{
    (void)dt; (void)userdata;
    static vec3_t zero = {0, 0, 0};
    if (out_accel) {
        *out_accel = zero;
        return out_accel;
    }
    return &zero;
}

const vec3_t* environ_func_default(
    vec3_t* out_accel,
    float dt,
    void* userdata)
{
    (void)dt; (void)userdata;
    static vec3_t gravity = {0.0f, -9.81f, 0.0f};
    if (out_accel) {
        *out_accel = gravity;
        return out_accel;
    }
    return &gravity;
}

const vec3_t* environ_func_constant(
    vec3_t* out_accel,
    float dt,
    void* userdata)
{
    (void)dt;
    static vec3_t result;
    vec3_t* wind = (vec3_t*)userdata;

    result.x = (wind ? wind->x : 0.0f);
    result.y = (wind ? wind->y : 0.0f) - 9.81f;
    result.z = (wind ? wind->z : 0.0f);

    if (out_accel) {
        *out_accel = result;
        return out_accel;
    }
    return &result;
}

// ---------------------------------------------------------
// 주기적 환경 데이터
// ---------------------------------------------------------

void env_periodic_init(env_periodic_data_t* out) {
    if (!out) return;
    out->base_wind = (vec3_t){0.0f, 0.0f, 0.0f};
    out->gust_amplitude = (vec3_t){0.5f, 0.0f, 0.5f};
    out->gust_frequency = 1.0f;    // 1초 주기
    out->time = 0.0f;
    out->gravity = (vec3_t){0.0f, -9.81f, 0.0f};
}

void env_periodic_init_full(
    env_periodic_data_t* out,
    const vec3_t* base_wind,
    const vec3_t* gust_amp,
    float gust_freq,
    const vec3_t* gravity)
{
    if (!out) return;
    out->base_wind = base_wind ? *base_wind : (vec3_t){0.0f, 0.0f, 0.0f};
    out->gust_amplitude = gust_amp ? *gust_amp : (vec3_t){0.5f, 0.0f, 0.5f};
    out->gust_frequency = (gust_freq < 0.0f) ? 0.0f : gust_freq;
    out->gravity = gravity ? *gravity : (vec3_t){0.0f, -9.81f, 0.0f};
    out->time = 0.0f;
}

void env_periodic_assign(env_periodic_data_t* out, const env_periodic_data_t* src) {
    if (!out || !src) return;
    memcpy(out, src, sizeof(env_periodic_data_t));
}

const vec3_t* environ_func_periodic(
    vec3_t* out_accel,
    float dt,
    void* userdata)
{
    static vec3_t result = {0.0f, -9.81f, 0.0f}; // 기본 중력
    vec3_t* target = out_accel ? out_accel : &result;

    if (!userdata) {
        *target = result;
        return target;
    }

    env_periodic_data_t* env = (env_periodic_data_t*)userdata;
    env->time += dt;

    float phase = 2.0f * (float)M_PI * env->gust_frequency * env->time;

    target->x = env->base_wind.x + env->gust_amplitude.x * sinf(phase);
    target->y = env->gravity.y + env->gust_amplitude.y * sinf(phase);
    target->z = env->base_wind.z + env->gust_amplitude.z * sinf(phase);

    return target;
}
