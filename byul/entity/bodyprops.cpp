#include "internal/bodyprops.h"

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
    body->friction = 0.5f;    // 기본 마찰 계수
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