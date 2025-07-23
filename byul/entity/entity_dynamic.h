#ifndef ENTITY_DYNAMIC_H
#define ENTITY_DYNAMIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "byul_common.h"
#include "internal/entity.h"
#include "internal/xform.h"
#include "internal/bodyprops.h"
#include "internal/environ.h"

// 전방 선언
typedef struct s_motion_state motion_state_t;

// ---------------------------------------------------------
// 동적 엔티티 구조체
// ---------------------------------------------------------
/**
 * @struct s_entity_dynamic
 * @brief 움직이는 엔티티의 확장 구조체
 *
 * - 기본 엔티티 정보(entity_t)
 * - xform_t (정밀 위치/회전)
 * - 속도(velocity), 회전 속도(angular_velocity)
 * - 물리 속성(bodyprops_t)
 */
typedef struct s_entity_dynamic {
    entity_t base;            ///< 공통 엔티티 속성
    xform_t xf;               ///< 정밀 위치 + 회전 (Transform)
    bodyprops_t props;        ///< 물리 속성 (질량, 마찰 등)
    vec3_t velocity;          ///< 선속도 (m/s)
    vec3_t angular_velocity;  ///< 회전 속도 (rad/s)
} entity_dynamic_t;

// ---------------------------------------------------------
// 함수 선언
// ---------------------------------------------------------

/**
 * @brief entity_dynamic_t를 기본값으로 초기화합니다.
 *
 * **기본값**
 * - base = entity_init()
 * - xf = identity transform
 * - props = bodyprops_init()
 * - velocity = (0,0,0)
 * - angular_velocity = (0,0,0)
 */
BYUL_API void entity_dynamic_init(entity_dynamic_t* d);

/**
 * @brief entity_dynamic_t를 사용자 지정 값으로 초기화합니다.
 *
 * @param[out] d         초기화할 동적 엔티티
 * @param[in]  base      기본 엔티티 정보 (NULL이면 기본값)
 * @param[in]  xf        초기 위치/회전 (NULL이면 기본값)
 * @param[in]  velocity  초기 속도 (NULL이면 (0,0,0))
 * @param[in]  angular   초기 회전 속도 (NULL이면 (0,0,0))
 * @param[in]  props     물리 속성 (NULL이면 기본값)
 */
BYUL_API void entity_dynamic_init_full(
    entity_dynamic_t* d,
    const entity_t* base,
    const xform_t* xf,
    const vec3_t* velocity,
    const vec3_t* angular,
    const bodyprops_t* props
);

/**
 * @brief entity_dynamic_t를 다른 동적 엔티티로 복사합니다.
 *
 * @param[out] dst 복사 대상
 * @param[in]  src 원본
 */
BYUL_API void entity_dynamic_assign(
    entity_dynamic_t* dst, const entity_dynamic_t* src);

/**
 * @brief 동적 엔티티의 이동 및 회전 업데이트
 *
 * - p = p + v * dt
 * - 회전 속도(angular_velocity)를 기반으로 xf에 회전 적용
 * - base.age += dt
 *
 * @param d   동적 엔티티 
 * @param dt  시간 간격 (초)
 */
BYUL_API void entity_dynamic_update(entity_dynamic_t* d, float dt);

/**
 * @brief 현재 속도를 유지했을 때 dt초 후 예상 위치를 계산합니다.
 *
 * @param[in] d   동적 엔티티
 * @param[in] dt  시간 간격 (초)
 * @param[out] out_pos 계산된 위치 (world 좌표)
 */
BYUL_API void entity_dynamic_predict_position(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_pos
);

// ---------------------------------------------------------
// 위치 계산: p(t)
// ---------------------------------------------------------
/**
 * @brief t초 후 예상 위치를 계산합니다.
 *
 * @param d        대상 동적 엔티티
 * @param env      환경 정보
 * @param dt       시간 간격 (초)
 * @param out_pos  계산된 위치 벡터
 */
BYUL_API void entity_dynamic_predict_position_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_pos
);

// ---------------------------------------------------------
// 속도 계산: v(t)
// ---------------------------------------------------------
/**
 * @brief t초 후 예상 속도를 계산합니다.
 *
 * @param d        대상 동적 엔티티
 * @param env      환경 정보
 * @param dt       시간 간격 (초)
 * @param out_vel  계산된 속도 벡터
 */
BYUL_API void entity_dynamic_predict_velocity_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_vel
);

// ---------------------------------------------------------
// 가속도 계산: a(t)
// ---------------------------------------------------------
/**
 * @brief 현재 상태에서 가속도를 계산합니다.
 *
 * @param d         대상 동적 엔티티
 * @param env       환경 정보
 * @param out_accel 계산된 가속도 벡터
 */
BYUL_API void entity_dynamic_predict_accel_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    vec3_t* out_accel
);

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
/**
 * @brief t초 후 전체 상태 (위치 + 속도) 예측
 *
 * @param d         대상 동적 엔티티
 * @param env       환경 정보
 * @param dt        시간 간격 (초)
 * @param out_state 예측된 동적 엔티티 상태
 */
BYUL_API void entity_dynamic_predict_state_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    entity_dynamic_t* out_state
);

// ---------------------------------------------------------
// 공기 저항력 계산
// ---------------------------------------------------------
/**
 * @brief 현재 속도에서 공기 저항 가속도를 계산합니다.
 *
 * @param d             대상 동적 엔티티
 * @param env           환경 정보
 * @param out_drag_accel 계산된 드래그 가속도
 */
BYUL_API void entity_dynamic_drag_accel_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    vec3_t* out_drag_accel
);

// ---------------------------------------------------------
// 최고점 여부 판단 (vy ≈ 0)
// ---------------------------------------------------------
/**
 * @brief vy가 0에 가까우면 최고점으로 판단
 *
 * @param d 대상 동적 엔티티
 * @return true이면 최고점
 */
BYUL_API bool entity_dynamic_is_apex(const entity_dynamic_t* d);

// ---------------------------------------------------------
// 착지 여부 판단
// ---------------------------------------------------------
/**
 * @brief y <= ground_height이면 착지로 판단
 *
 * @param d             대상 동적 엔티티
 * @param ground_height 지면 높이
 * @return true이면 착지
 */
BYUL_API bool entity_dynamic_is_grounded(
    const entity_dynamic_t* d,
    float ground_height
);

// ---------------------------------------------------------
// 충돌 반발 계산
// ---------------------------------------------------------
typedef bool (*entity_dynamic_bounce_func)(
    const vec3_t* velocity_in,
    const vec3_t* normal,
    float restitution,
    void* userdata,
    vec3_t* out_velocity_out
);

BYUL_API void entity_dynamic_set_bounce_func(
    entity_dynamic_bounce_func func,
    void* userdata);

BYUL_API void entity_dynamic_get_bounce_func(
    entity_dynamic_bounce_func* out_func,
    void** out_userdata);

BYUL_API bool entity_dynamic_default_bounce(
    const vec3_t* velocity_in,
    const vec3_t* normal,
    float restitution,
    vec3_t* out_velocity_out);

/**
 * @brief entity_dynamic_t → motion_state_t 변환
 *
 * @param ed   변환할 엔티티
 * @param out  결과 motion_state
 * @param lin_acc 외부에서 계산된 선형 가속도 (NULL 가능)
 * @param ang_acc 외부에서 계산된 각가속도 (NULL 가능)
 */
BYUL_API void entity_dynamic_to_motion_state(
    const entity_dynamic_t* ed,
    motion_state_t* out,
    const vec3_t* lin_acc,
    const vec3_t* ang_acc);
    

/**
 * @brief motion_state_t → entity_dynamic_t 변환
 */
BYUL_API void entity_dynamic_from_motion_state(
    entity_dynamic_t* ed, const motion_state_t* ms);


#ifdef __cplusplus
}
#endif

#endif // ENTITY_DYNAMIC_H
