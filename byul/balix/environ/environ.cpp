#include "environ.h"
#include "numeq_model.h"
#include "vec3.h"
#include "scalar.h"
#include <math.h>
#include <string.h>

static float environ_calc_factor(const environ_t* env) {
    if (!env) return 1.0f;

    float humidity_norm = (env->humidity - 50.0f) / 50.0f;
    float humidity_factor = 1.0f - 0.3f * (humidity_norm * humidity_norm);
    if (humidity_factor < 0.7f) humidity_factor = 0.7f;

    float temp_norm = (env->temperature - 20.0f) / 40.0f;
    float temp_u = 0.7f + 0.3f * (temp_norm * temp_norm);
    float temp_linear = 1.0f - (fabsf(env->temperature - 20.0f) / 200.0f);
    if (temp_linear < 0.8f) temp_linear = 0.8f;
    float temp_factor = temp_u * temp_linear;

    float air_density_factor = env->air_density / 1.225f;
    if (air_density_factor < 0.8f) air_density_factor = 0.8f;
    if (air_density_factor > 1.2f) air_density_factor = 1.2f;

    float pressure_norm = (env->pressure - 101325.0f) / 20000.0f;
    float pressure_u = 1.0f - 0.1f * (pressure_norm * pressure_norm);
    float pressure_linear = 1.0f - (fabsf(env->pressure - 101325.0f) / 200000.0f);
    if (pressure_linear < 0.85f) pressure_linear = 0.85f;
    float pressure_factor = pressure_u * pressure_linear;

    float final_factor = humidity_factor * temp_factor *
                         air_density_factor * pressure_factor;
    if (final_factor < 0.5f) final_factor = 0.5f;

    return final_factor;
}

void environ_init(environ_t* env) {
    if (!env) return;
    env->gravity = vec3_t{0.0f, -9.81f, 0.0f};
    vec3_zero(&env->wind_vel);
    env->air_density = 1.225f;
    env->humidity = 50.0f;
    env->temperature = 20.0f;
    env->pressure = 101325.0f;
    env->environ_fn = environ_calc_gravity_wind;
    env->userdata = NULL;
}

void environ_init_full(environ_t* env,
                           const vec3_t* gravity,
                           const vec3_t* wind_vel,
                           float air_density,
                           float humidity,
                           float temperature,
                           float pressure,
                           environ_func environ_fn,
                           void* userdata) {
    if (!env) return;

    env->gravity = gravity ? *gravity : vec3_t{0.0f, -9.81f, 0.0f};
    env->wind_vel = wind_vel ? *wind_vel : vec3_t{0.0f, 0.0f, 0.0f};
    env->air_density = air_density;
    env->humidity = humidity;
    env->temperature = temperature;
    env->pressure = pressure;
    env->environ_fn = environ_fn ? environ_fn : environ_calc_gravity_wind;
    env->userdata = userdata;
}

void environ_assign(environ_t* out, const environ_t* src) {
    if (!src || !out) return;
    *out = *src;
}

void environ_apply_wind(
    environ_t* env, const vec3_t* accel, float dt)
{
    if (!env || !accel || dt <= 0.0f) return;
    vec3_madd(&env->wind_vel, &env->wind_vel, accel, dt);
}

void environ_distort_accel( const environ_t* env, vec3_t* accel) {
    if (!env || !accel) return;

    environ_distort_accel_except_gravity(env, true, accel);
}

void environ_distort_accel_except_gravity(
    const environ_t* env, bool include_gravity, vec3_t* accel)
{
    if (!env || !accel) return;

    vec3_t non_gravity = *accel;

    if (include_gravity) {
        vec3_sub(&non_gravity, &non_gravity, &env->gravity);
    }

    float factor = environ_calc_factor(env);
    non_gravity.x *= factor;
    non_gravity.y *= factor;
    non_gravity.z *= factor;

    if (include_gravity) {
        vec3_add(accel, &non_gravity, &env->gravity);
    } else {
        *accel = non_gravity;
    }
}

const vec3_t* environ_calc_none(const environ_t* env,
                                    void* userdata,
                                    vec3_t* out) {
    (void)env;
    (void)userdata;

    static vec3_t zero = {0, 0, 0};
    if (out) { *out = zero; return out; }
    return &zero;
}

const vec3_t* environ_calc_gravity(const environ_t* env,
                                       void* userdata,
                                       vec3_t* out) {
(void)userdata;
    static vec3_t result;
    result = env ? env->gravity : vec3_t{0.0f, -9.81f, 0.0f};
    environ_distort_accel(env, &result);
    if (out) { *out = result; return out; }
    return &result;
}

const vec3_t* environ_calc_gravity_wind(const environ_t* env,
                                            void* userdata,
                                            vec3_t* out) {
    (void)userdata;
    static vec3_t result;
    if (env) {
        result.x = env->gravity.x + env->wind_vel.x;
        result.y = env->gravity.y + env->wind_vel.y;
        result.z = env->gravity.z + env->wind_vel.z;
        environ_distort_accel(env, &result);
    } else {
        result = vec3_t{0.0f, -9.81f, 0.0f};
    }
    if (out) { *out = result; return out; }
    return &result;
}

void environ_periodic_init(environ_periodic_t* out) {
    if (!out) return;
    out->base_wind = vec3_t{0.0f, 0.0f, 0.0f};
    out->gust_amplitude = vec3_t{0.5f, 0.0f, 0.5f};
    out->gust_frequency = 1.0f;
    out->elapsed = 0.0f;
    out->gravity = vec3_t{0.0f, -9.81f, 0.0f};
}

void environ_periodic_init_full(const vec3_t* base_wind,
                                    const vec3_t* gust_amp,
                                    float gust_freq,
                                    const vec3_t* gravity,
                                    environ_periodic_t* out) {
    if (!out) return;
    out->base_wind = base_wind ? *base_wind : vec3_t{0.0f, 0.0f, 0.0f};
    out->gust_amplitude = gust_amp ? *gust_amp : vec3_t{0.5f, 0.0f, 0.5f};
    out->gust_frequency = (gust_freq < 0.0f) ? 0.0f : gust_freq;
    out->gravity = gravity ? *gravity : vec3_t{0.0f, -9.81f, 0.0f};
    out->elapsed = 0.0f;
}

void environ_periodic_assign(const environ_periodic_t* src,
                                 environ_periodic_t* out) {
    if (!out || !src) return;
    memcpy(out, src, sizeof(environ_periodic_t));
}

const vec3_t* environ_calc_periodic(const environ_t* env,
                                        void* userdata,
                                        vec3_t* out) {
    static vec3_t result;
    vec3_t* target = out ? out : &result;

    environ_periodic_t* pdata = (environ_periodic_t*)userdata;
    if (!pdata) {
        *target = env ? env->gravity : vec3_t{0.0f, -9.81f, 0.0f};
        environ_distort_accel(env, target);
        return target;
    }

    pdata->elapsed += pdata->delta_time;
    float phase = 2.0f * (float)M_PI * pdata->gust_frequency * pdata->elapsed;

    target->x = pdata->base_wind.x + pdata->gust_amplitude.x * sinf(phase);
    target->y = pdata->gravity.y + pdata->gust_amplitude.y * sinf(phase);
    target->z = pdata->base_wind.z + pdata->gust_amplitude.z * sinf(phase);

    environ_distort_accel(env, target);
    return target;
}
