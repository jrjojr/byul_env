#ifndef BODYPROPS_H
#define BODYPROPS_H

#include "byul_config.h"
#include "internal/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @enum shape_type_t
 * @brief 물체의 기본 충돌 모양 타입
 *
 * - SHAPE_SPHERE: 구 형태
 * - SHAPE_BOX: 직육면체(AABB)
 * - SHAPE_CAPSULE: 캡슐 형태
 * - SHAPE_CYLINDER: 원기둥
 * - SHAPE_CUSTOM: 사용자 정의 메쉬
 */
typedef enum e_shape_type {
    SHAPE_SPHERE = 0,
    SHAPE_BOX,
    SHAPE_CAPSULE,
    SHAPE_CYLINDER,
    SHAPE_CUSTOM
} shape_type_t;

/**
 * @struct bodyprops_t
 * @brief 물체(물리 객체)의 고유 특성 값
 *
 * 질량, 마찰, 반발, 공기저항과 함께
 * 충돌 판정을 위한 모양(shape)과 크기(size)를 포함합니다.
 *
 * ### size 필드의 의미
 * - SHAPE_SPHERE: size.x = 직경 (radius = size.x / 2)
 * - SHAPE_BOX:    size = {가로, 세로, 깊이}
 * - SHAPE_CAPSULE: size.x = 반지름, size.y = 캡슐 높이
 */
typedef struct s_bodyprops {
    float mass;             /**< 질량 (kg), 기본값 1.0 */
    float drag_coef;        /**< 공기 저항 계수 (Cd) */
    float cross_section;    /**< 단면적 (m²), 항력 계산용 */
    float restitution;      /**< 반발 계수 (0: 흡수, 1: 완전 반사) */
    float friction;         /**< 마찰 계수 (0~1, 0: 없음, 1: 최대) */
    shape_type_t shape;     /**< 충돌 모양 (Sphere, Box 등) */
    vec3_t size;            /**< 크기 정보 (모양별 의미는 위 설명 참조) */
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
 * - friction = 0.5f
 * - shape = SHAPE_SPHERE
 * - size = {0.1f, 0.1f, 0.1f} (지름 10cm 구체)
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
 * @param shape       모양 타입
 * @param size        크기 정보 (모양별 의미는 @ref bodyprops_t 참조)
 */
BYUL_API void bodyprops_init_full(bodyprops_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction,
                               shape_type_t shape,
                               const vec3_t* size);

/**
 * @brief bodyprops_t 복사
 *
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void bodyprops_assign(
    bodyprops_t* out, const bodyprops_t* src);

#ifdef __cplusplus
}
#endif

#endif // BODYPROPS_H
