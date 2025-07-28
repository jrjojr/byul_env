#ifndef BODYPROPS_H
#define BODYPROPS_H

#include "byul_common.h"
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct bodyprops_t
 * @brief 물체(물리 객체)의 고유 특성 값
 *
 * 질량, 마찰, 반발, 공기저항, 단면적
 * 
 */
typedef struct s_bodyprops {
    float mass;             /**< 질량 (kg), 기본값 1.0 */
    float drag_coef;        /**< 공기 저항 계수 (Cd) 모양이 존재해야 의미가 있다. */
    float cross_section;    /**< 단면적 (m²), 항력 계산용  모양이 존재해야 한다 */
    float restitution;      /**< 반발 계수 (0: 흡수, 1: 완전 반사) */
    float friction;         /**< 마찰 계수 (0~1, 0: 없음, 1: 최대) */
} bodyprops_t;

// ---------------------------------------------------------
// bodyprops_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief bodyprops_t 기본값 초기화
 *
 * 기본값:
 * - mass = 1.0f
 * - drag_coef = 0.47f (구체 기준)
 * - cross_section = 0.01f (10cm²)
 * - restitution = 0.5f
 * - friction = 0.1f
 *
 * @param body 초기화할 물체 특성 구조체 (NULL이면 무시)
 */
BYUL_API void bodyprops_init(bodyprops_t* body);

/**
 * @brief bodyprops_t를 지정한 값으로 초기화
 *
 * @param body        초기화할 구조체 (NULL이면 무시)
 * @param mass        질량 [kg]
 * @param drag_coef   공기 저항 계수
 * @param cross_section 단면적 [m²]
 * @param restitution 반발 계수
 * @param friction    마찰 계수
 */
BYUL_API void bodyprops_init_full(bodyprops_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction);

/**
 * @brief bodyprops_t 복사
 *
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void bodyprops_assign(
    bodyprops_t* out, const bodyprops_t* src);

/**
 * @brief 마찰력에 의한 속도 감소를 적용
 *
 * - friction(0~1): 감쇠 계수 (0=감쇠 없음, 1=강한 마찰)
 * - friction은 선형 비율로 적용되며, dt가 커질수록 감소량이 큼
 *
 * @param[in,out] velocity 속도 벡터 (m/s)
 * @param[in] body         물체 특성 (friction 값 사용)
 * @param[in] dt           시간 간격 (초)
 */
BYUL_API void bodyprops_apply_friction(vec3_t* velocity,
                                  const bodyprops_t* body,
                                  float dt);

/**
 * @brief 마찰력에 의한 속도 감소와 열 발생 계산
 *
 * - friction(0~1): 감쇠 계수
 * - 감소한 운동에너지를 Joule 단위로 계산하여 heat로 반환
 *
 * @param[in,out] velocity 속도 벡터 (m/s)
 * @param[in] body         물체 특성 (mass, friction 사용)
 * @param[in] dt           시간 간격 (초)
 * @return 발생한 열량 (Joule)
 */
BYUL_API float bodyprops_apply_friction_heat(vec3_t* velocity,
                                        const bodyprops_t* body,
                                        float dt);


#ifdef __cplusplus
}
#endif

#endif // BODYPROPS_H
