#include "internal/propulsion.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// ---------------------------------------------------------
// 초기화 및 상태 복사
// ---------------------------------------------------------
void propulsion_init(propulsion_t* p) {
    if (!p) return;

    // --- 기본 성능 ---
    p->max_thrust = 120.0f;          // 120N (소형 로켓/드론 모터 수준)
    p->current_thrust = 0.0f;
    p->fuel_capacity = 50.0f;        // 50kg 연료 탑재
    p->fuel_remaining = 50.0f;
    p->burn_rate = 0.05f;            // 0.05 kg/s (100N 기준 약 2분 지속 가능)

    // --- 효율 관련 ---
    p->efficiency = 0.7f;           // 70% 효율
    p->thermal_loss = 0.05f;         // 5% 손실
    p->energy_density = 42.0f;       // MJ/kg (케로신 비슷한 값)

    // --- 동적 응답 ---
    p->response_time = 0.8f;         // 목표 추력까지 약 0.8초 걸림
    p->max_thrust_rate = 30.0f;      // 초당 최대 30N 상승
    p->delay_time = 0.2f;            // 제어 명령 후 약 0.2초 지연

    // --- 열 및 마모 ---
    p->heat = 0.0f;
    p->heat_dissipation_rate = 0.3f; // 느린 열 방출
    p->wear_level = 0.0f;

    // --- 컨트롤러 ---
    p->controller = NULL;
    p->active = true;
}

void propulsion_init_full(propulsion_t* p,
                          float max_thrust,
                          float fuel_capacity,
                          float burn_rate,
                          controller_t* ctrl,
                          bool active) {
    if (!p) return;
    propulsion_init(p); // 기본값 초기화
    p->max_thrust = (max_thrust > 0.0f) ? max_thrust : 100.0f;
    p->fuel_capacity = (fuel_capacity > 0.0f) ? fuel_capacity : 100.0f;
    p->fuel_remaining = p->fuel_capacity;
    p->burn_rate = (burn_rate > 0.0f) ? burn_rate : 1.0f;
    p->controller = ctrl;
    p->active = active;
}

void propulsion_assign(propulsion_t* dst, const propulsion_t* src) {
    if (!dst || !src) return;
    memcpy(dst, src, sizeof(propulsion_t));
}

void propulsion_reset(propulsion_t* p) {
    if (!p) return;
    p->current_thrust = 0.0f;
    p->fuel_remaining = p->fuel_capacity;
    p->active = false;
}

// ---------------------------------------------------------
// 동작 및 업데이트
// ---------------------------------------------------------
void propulsion_update(propulsion_t* p, float target_thrust, float dt) {
    if (!p || dt <= 0.0f) return;

    // 비활성 또는 연료 부족
    if (!p->active || p->fuel_remaining <= 0.0f) {
        p->current_thrust = 0.0f;
        p->active = false;
        return;
    }

    // 1. 목표 출력 제한
    if (target_thrust > p->max_thrust) target_thrust = p->max_thrust;
    if (target_thrust < 0.0f) target_thrust = 0.0f;

    // 2. 컨트롤러 계산
    float desired_thrust = target_thrust;
    if (p->controller) {
        float control = controller_compute(p->controller, target_thrust, p->current_thrust, dt);
        if (control > p->max_thrust) control = p->max_thrust;
        if (control < 0.0f) control = 0.0f;
        desired_thrust = control;
    }

    // 3. 효율, 열, 마모 반영
    desired_thrust *= p->efficiency;
    desired_thrust *= (1.0f - p->thermal_loss);
    desired_thrust *= (1.0f - p->wear_level * 0.3f);

    // 4. 추력 변화율 제한
    float max_delta = p->max_thrust_rate * dt;
    float delta = desired_thrust - p->current_thrust;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    p->current_thrust += delta;

    // 5. 연료 소모
    float fuel_needed = p->burn_rate * p->current_thrust * dt;
    if (fuel_needed >= p->fuel_remaining) {
        p->current_thrust = p->fuel_remaining / (p->burn_rate * dt);
        p->fuel_remaining = 0.0f;
        p->active = false;
    } else {
        p->fuel_remaining -= fuel_needed;
    }

    // 6. 열 관리
    p->heat += p->current_thrust * 0.05f;
    p->heat -= p->heat_dissipation_rate * dt;
    if (p->heat < 0.0f) p->heat = 0.0f;

    // 7. 마모 증가
    p->wear_level += 0.0001f * p->current_thrust * dt;
    if (p->wear_level > 1.0f) p->wear_level = 1.0f;
}

// ---------------------------------------------------------
// 정보 조회
// ---------------------------------------------------------
float propulsion_get_thrust(const propulsion_t* p) {
    return (p && p->active) ? p->current_thrust : 0.0f;
}

bool propulsion_is_empty(const propulsion_t* p) {
    return (!p || p->fuel_remaining <= 0.0f);
}

float propulsion_get_fuel_ratio(const propulsion_t* p) {
    if (!p || p->fuel_capacity <= 0.0f) return 0.0f;
    return p->fuel_remaining / p->fuel_capacity;
}

float propulsion_get_max_runtime(const propulsion_t* p) {
    if (!p || p->current_thrust <= 0.0f || p->burn_rate <= 0.0f) return 0.0f;
    return p->fuel_remaining / (p->burn_rate * p->current_thrust);
}

float propulsion_get_remaining_impulse(const propulsion_t* p) {
    return (p && p->burn_rate > 0.0f) ? p->fuel_remaining / p->burn_rate : 0.0f;
}

// ---------------------------------------------------------
// 연료 관리
// ---------------------------------------------------------
void propulsion_refuel(propulsion_t* p, float amount) {
    if (!p || amount <= 0.0f) return;
    p->fuel_remaining += amount;
    if (p->fuel_remaining > p->fuel_capacity)
        p->fuel_remaining = p->fuel_capacity;
}

void propulsion_consume(propulsion_t* p, float amount) {
    if (!p || amount <= 0.0f) return;
    p->fuel_remaining -= amount;
    if (p->fuel_remaining < 0.0f) {
        p->fuel_remaining = 0.0f;
        p->active = false;
    }
}

// ---------------------------------------------------------
// 예측 관련
// ---------------------------------------------------------
float propulsion_predict_runtime(const propulsion_t* p, float desired_thrust) {
    if (!p || desired_thrust <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    return p->fuel_remaining / (p->burn_rate * desired_thrust);
}

float propulsion_predict_empty_time(const propulsion_t* p) {
    if (!p || p->current_thrust <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    return p->fuel_remaining / (p->burn_rate * p->current_thrust);
}

float propulsion_predict_max_thrust(const propulsion_t* p, float duration) {
    if (!p || duration <= 0.0f || p->fuel_remaining <= 0.0f || p->burn_rate <= 0.0f)
        return 0.0f;
    float possible = p->fuel_remaining / (p->burn_rate * duration);
    return (possible > p->max_thrust) ? p->max_thrust : possible;
}

// ---------------------------------------------------------
// 제어기 관리
// ---------------------------------------------------------
void propulsion_set_active(propulsion_t* p, bool active) {
    if (p) p->active = active;
}

void propulsion_attach_controller(propulsion_t* p, controller_t* ctrl) {
    if (p) p->controller = ctrl;
}

void propulsion_detach_controller(propulsion_t* p) {
    if (p) p->controller = NULL;
}

// ---------------------------------------------------------
// 디버깅/로깅
// ---------------------------------------------------------
void propulsion_print(const propulsion_t* p) {
    if (!p) {
        printf("propulsion: (null)\n");
        return;
    }
    printf("Thrust=%.2fN, Fuel=%.2f/%.2fkg, Active=%d\n",
           p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
}

const char* propulsion_to_string(const propulsion_t* p, char* buffer, size_t buffer_size) {
    if (!p || !buffer || buffer_size == 0) return NULL;
    snprintf(buffer, buffer_size,
             "Thrust=%.2fN, Fuel=%.2f/%.2fkg, Active=%d",
             p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
    return buffer;
}

const char* propulsion_to_json(const propulsion_t* p, char* buffer, size_t buffer_size) {
    if (!p || !buffer || buffer_size == 0) return NULL;
    snprintf(buffer, buffer_size,
             "{\"thrust\":%.2f,\"fuel\":%.2f,\"capacity\":%.2f,\"active\":%d}",
             p->current_thrust, p->fuel_remaining, p->fuel_capacity, p->active);
    return buffer;
}
