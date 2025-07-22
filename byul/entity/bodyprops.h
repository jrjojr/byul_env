#ifndef BODYPROPS_H
#define BODYPROPS_H

#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct bodyprops_t
 * @brief 물체(물리 객체)의 고유 특성 값
 *
 * 물체의 질량, 마찰 계수, 공기 저항 계수 등을 포함합니다.
 * 이 값들은 물리 시뮬레이션에서 물체의 거동(운동, 반발, 마찰)에 영향을 줍니다.
 */
typedef struct s_bodyprops {
    float mass;             /**< 질량 (kg) - 기본값 1.0 kg */
    float drag_coef;        /**< 공기 저항 계수 (Cd) */
    float cross_section;    /**< 단면적 (m²) - 공기 저항 계산에 사용 */
    float restitution;      /**< 반발 계수 (0: 흡수, 1: 완전 반사) */
    float friction;         /**< 마찰 계수 (0~1, 0: 마찰 없음) */
} bodyprops_t;

// ---------------------------------------------------------
// bodyprops_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief bodyprops_t 기본값 초기화
 *
 * 이 함수는 bodyprops_t 구조체를 다음의 기본값으로 초기화합니다.
 * - mass = 1.0f (기본 질량 1 kg)
 * - drag_coef = 0.47f (구체의 전형적인 공기 저항 계수)
 * - cross_section = 0.01f (기본 단면적 0.01 m², 약 10cm²)
 * - restitution = 0.5f (중간 수준 반발 계수, 0: 흡수 ~ 1: 완전 반사)
 * - friction = 0.5f (중간 수준 마찰 계수, 0: 마찰 없음 ~ 1: 매우 높은 마찰)
 *
 * @param body 초기화할 물체 특성 구조체 (NULL이면 동작하지 않음)
 */
BYUL_API void bodyprops_init(bodyprops_t* body);

/**
 * @brief bodyprops_t를 지정한 값으로 초기화
 *
 * @param body 초기화할 구조체 (NULL이면 동작하지 않음)
 * @param mass 질량 [kg] (예: 1.0f)
 * @param drag_coef 공기 저항 계수 (예: 0.47f, 구체 기준)
 * @param cross_section 단면적 [m²] (예: 0.01f)
 * @param restitution 반발 계수 (0: 흡수, 1: 완전 반사)
 * @param friction 마찰 계수 (0~1, 0: 없음, 1: 최대)
 */
BYUL_API void bodyprops_init_full(bodyprops_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction);

/**
 * @brief bodyprops_t 복사
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void bodyprops_assign(
    bodyprops_t* out, const bodyprops_t* src);


#ifdef __cplusplus
}
#endif

#endif // BODYPROPS_H
