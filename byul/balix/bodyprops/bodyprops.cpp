#include "bodyprops.h"
#include "numeq_solver.h"

// ---------------------------------------------------------
// bodyprops_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief bodyprops_t 기본값 초기화
 * @param body 초기화할 물체 특성 구조체
 */
void bodyprops_init(bodyprops_t* body) {
    if (!body) return;
    body->mass = 1.0f;        // 기본 1 kg
    body->drag_coef = 0.47f;  // 구체(Cd) 기준값
    body->cross_section = 0.01f;  // 10 cm² (0.01 m²)
    body->restitution = 0.5f; // 기본 반발 계수
    body->friction = 0.1f;    // 기본 마찰 계수
}

/**
 * @brief bodyprops_t를 지정한 값으로 초기화
 * @param body 초기화할 구조체
 * @param mass 질량 [kg]
 * @param drag_coef 공기 저항 계수
 * @param cross_section 단면적 [m²]
 * @param restitution 반발 계수
 * @param friction 마찰 계수
 */
void bodyprops_init_full(bodyprops_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction) {
    if (!body) return;
    body->mass = mass;
    body->drag_coef = drag_coef;
    body->cross_section = cross_section;
    body->restitution = restitution;
    body->friction = friction;
}

/**
 * @brief bodyprops_t 복사
 * @param out 복사 대상
 * @param src 원본
 */
void bodyprops_assign(bodyprops_t* out, const bodyprops_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void bodyprops_apply_friction(vec3_t* velocity,
                         const bodyprops_t* body,
                         float dt)
{
    if (!velocity || !body || dt <= 0.0f) return;

    float factor = 1.0f - body->friction * dt;
    if (factor < 0.0f) factor = 0.0f;  // 마찰이 너무 커서 역방향으로 가지 않게 제한
    vec3_scale(velocity, velocity, factor);
}

float bodyprops_apply_friction_dt(vec3_t* velocity,
                               const bodyprops_t* body,
                               float dt)
{
    if (!velocity || !body || dt <= 0.0f) return 0.0f;

    float v0 = vec3_length(velocity);
    if (v0 <= 1e-5f) {
        *velocity = vec3_t{0, 0, 0};
        return 0.0f;
    }

    float factor = 1.0f - body->friction * dt;

    if (factor >= 0.0f) {
        // 일반 감쇠
        vec3_scale(velocity, velocity, factor);
        return dt;
    } else {
        // 속도가 0이 되는 순간을 방정식으로 계산
        float t_stop = 0.0f;
        if (numeq_solve_linear(body->friction, -1.0f, &t_stop) && t_stop > 0.0f) {
            *velocity = vec3_t{0, 0, 0};
            return (t_stop < dt) ? t_stop : dt;
        }
    }
    return dt;
}



float bodyprops_apply_friction_heat(vec3_t* velocity,
                               const bodyprops_t* body,
                               float dt)
{
    if (!velocity || !body || dt <= 0.0f) return 0.0f;

    float v_prev = vec3_length(velocity);

    // 마찰에 의한 속도 감소
    float factor = 1.0f - body->friction * dt;
    if (factor < 0.0f) factor = 0.0f;
    vec3_scale(velocity, velocity, factor);

    // 감소한 운동에너지 = 열 발생
    float v_new = vec3_length(velocity);
    float delta_ke = 0.5f * body->mass * (v_prev * v_prev - v_new * v_new);
    return delta_ke > 0.0f ? delta_ke : 0.0f;
}
