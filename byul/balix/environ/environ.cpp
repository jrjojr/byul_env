#include "internal/environ.h"
#include "internal/numeq_model.h"
#include "internal/vec3.h"
#include "internal/common.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------
// 공통 보정 계수 계산 (humidity, temp, density, pressure)
// ---------------------------------------------------------
static float environ_calc_factor(const environ_t* env) {
    if (!env) return 1.0f;

    // --- 습도: 역 U자형 (50%에서 최대 효율) ---
    float humidity_norm = (env->humidity - 50.0f) / 50.0f;
    float humidity_factor = 1.0f - 0.3f * (humidity_norm * humidity_norm);
    if (humidity_factor < 0.7f) humidity_factor = 0.7f;

    // --- 온도: U자형 + 비례형 ---
    float temp_norm = (env->temperature - 20.0f) / 40.0f;
    float temp_u = 0.7f + 0.3f * (temp_norm * temp_norm);
    float temp_linear = 1.0f - (fabsf(env->temperature - 20.0f) / 200.0f);
    if (temp_linear < 0.8f) temp_linear = 0.8f;
    float temp_factor = temp_u * temp_linear;

    // --- 공기 밀도 ---
    float air_density_factor = env->air_density / 1.225f;
    if (air_density_factor < 0.8f) air_density_factor = 0.8f;
    if (air_density_factor > 1.2f) air_density_factor = 1.2f;

    // --- 기압 ---
    float pressure_norm = (env->pressure - 101325.0f) / 20000.0f;
    float pressure_u = 1.0f - 0.1f * (pressure_norm * pressure_norm);
    float pressure_linear = 1.0f - (fabsf(env->pressure - 101325.0f) / 200000.0f);
    if (pressure_linear < 0.85f) pressure_linear = 0.85f;
    float pressure_factor = pressure_u * pressure_linear;

    // --- 최종 보정값 ---
    float final_factor = humidity_factor * temp_factor *
                         air_density_factor * pressure_factor;
    if (final_factor < 0.5f) final_factor = 0.5f;

    return final_factor;
}

// ---------------------------------------------------------
// environ_t 초기화 및 복사
// ---------------------------------------------------------
void environ_init(environ_t* env) {
    if (!env) return;
    env->gravity = (vec3_t){0.0f, -9.81f, 0.0f};
    vec3_zero(&env->wind);
    env->air_density = 1.225f;
    env->humidity = 50.0f;
    env->temperature = 20.0f;
    env->pressure = 101325.0f;
    env->environ_fn = environ_calc_gravity;
    env->userdata = NULL;
}

void environ_init_full(environ_t* env,
                           const vec3_t* gravity,
                           const vec3_t* wind,
                           float air_density,
                           float humidity,
                           float temperature,
                           float pressure,
                           environ_func environ_fn,
                           void* userdata) {
    if (!env) return;

    env->gravity = gravity ? *gravity : (vec3_t){0.0f, -9.81f, 0.0f};
    env->wind = wind ? *wind : (vec3_t){0.0f, 0.0f, 0.0f};
    env->air_density = air_density;
    env->humidity = humidity;
    env->temperature = temperature;
    env->pressure = pressure;
    env->environ_fn = environ_fn ? environ_fn : environ_calc_gravity;
    env->userdata = userdata;
}

void environ_assign(environ_t* out, const environ_t* src) {
    if (!src || !out) return;
    *out = *src;
}

// ---------------------------------------------------------
// 외력(accel)에 환경 보정만 적용 (중력 제외)
// ---------------------------------------------------------
void environ_adjust_accel( const environ_t* env, vec3_t* accel) {
    if (!env || !accel) return;

    environ_adjust_accel_gsplit(env, true, accel);
}

void environ_adjust_accel_gsplit(
    const environ_t* env, bool has_gravity, vec3_t* accel)
{
    if (!env || !accel) return;

    vec3_t non_gravity = *accel;

    if (has_gravity) {
        // 중력 포함 → 중력만 분리
        vec3_sub(&non_gravity, &non_gravity, &env->gravity);
    }

    // 외력 보정
    float factor = environ_calc_factor(env);
    non_gravity.x *= factor;
    non_gravity.y *= factor;
    non_gravity.z *= factor;

    // 중력 재합산
    if (has_gravity) {
        vec3_add(accel, &non_gravity, &env->gravity);
    } else {
        *accel = non_gravity;
    }
}

// ---------------------------------------------------------
// 기본 환경 가속도 함수
// ---------------------------------------------------------
const vec3_t* environ_calc_none(const environ_t* env,
                                    float dt,
                                    void* userdata,
                                    vec3_t* out) {
    (void)env; (void)dt; (void)userdata;
    static vec3_t zero = {0, 0, 0};
    if (out) { *out = zero; return out; }
    return &zero;
}

const vec3_t* environ_calc_gravity(const environ_t* env,
                                       float dt,
                                       void* userdata,
                                       vec3_t* out) {
    (void)dt; (void)userdata;
    static vec3_t result;
    result = env ? env->gravity : (vec3_t){0.0f, -9.81f, 0.0f};
    environ_adjust_accel(env, &result);
    if (out) { *out = result; return out; }
    return &result;
}

const vec3_t* environ_calc_gravity_wind(const environ_t* env,
                                            float dt,
                                            void* userdata,
                                            vec3_t* out) {
    (void)dt; (void)userdata;
    static vec3_t result;
    if (env) {
        result.x = env->gravity.x + env->wind.x;
        result.y = env->gravity.y + env->wind.y;
        result.z = env->gravity.z + env->wind.z;
        environ_adjust_accel(env, &result);
    } else {
        result = (vec3_t){0.0f, -9.81f, 0.0f};
    }
    if (out) { *out = result; return out; }
    return &result;
}

// ---------------------------------------------------------
// 주기적 환경 데이터
// ---------------------------------------------------------
void environ_periodic_init(environ_periodic_t* out) {
    if (!out) return;
    out->base_wind = (vec3_t){0.0f, 0.0f, 0.0f};
    out->gust_amplitude = (vec3_t){0.5f, 0.0f, 0.5f};
    out->gust_frequency = 1.0f;
    out->time = 0.0f;
    out->gravity = (vec3_t){0.0f, -9.81f, 0.0f};
}

void environ_periodic_init_full(const vec3_t* base_wind,
                                    const vec3_t* gust_amp,
                                    float gust_freq,
                                    const vec3_t* gravity,
                                    environ_periodic_t* out) {
    if (!out) return;
    out->base_wind = base_wind ? *base_wind : (vec3_t){0.0f, 0.0f, 0.0f};
    out->gust_amplitude = gust_amp ? *gust_amp : (vec3_t){0.5f, 0.0f, 0.5f};
    out->gust_frequency = (gust_freq < 0.0f) ? 0.0f : gust_freq;
    out->gravity = gravity ? *gravity : (vec3_t){0.0f, -9.81f, 0.0f};
    out->time = 0.0f;
}

void environ_periodic_assign(const environ_periodic_t* src,
                                 environ_periodic_t* out) {
    if (!out || !src) return;
    memcpy(out, src, sizeof(environ_periodic_t));
}

const vec3_t* environ_calc_periodic(const environ_t* env,
                                        float dt,
                                        void* userdata,
                                        vec3_t* out) {
    static vec3_t result;
    vec3_t* target = out ? out : &result;

    environ_periodic_t* pdata = (environ_periodic_t*)userdata;
    if (!pdata) {
        *target = env ? env->gravity : (vec3_t){0.0f, -9.81f, 0.0f};
        environ_adjust_accel(env, target);
        return target;
    }

    pdata->time += dt;
    float phase = 2.0f * (float)M_PI * pdata->gust_frequency * pdata->time;

    target->x = pdata->base_wind.x + pdata->gust_amplitude.x * sinf(phase);
    target->y = pdata->gravity.y + pdata->gust_amplitude.y * sinf(phase);
    target->z = pdata->base_wind.z + pdata->gust_amplitude.z * sinf(phase);

    environ_adjust_accel(env, target);
    return target;
}
