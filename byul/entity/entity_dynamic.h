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
#include "internal/motion_state.h"

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
    bool is_grounded;        ///< 지면 접촉 여부 (true면 y축 운동 정지)    
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
 * @brief 현재와 과거 상태를 기반으로 평균 가속도를 계산합니다.
 *
 * a = (v_curr - v_prev) / dt
 * dt가 0.0f 이하이면 결과는 (0,0,0)을 반환합니다.
 *
 * @param curr      현재 엔티티 동적 상태
 * @param prev_vel      과거 엔티티 속도
 * @param dt        시간 간격 (초)
 * @param out_accel 계산된 가속도 (m/s²)
 */
BYUL_API void entity_dynamic_calc_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    vec3_t* out_accel
);

BYUL_API void entity_dynamic_calc_accel_env(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,
    vec3_t* out_accel
);

/**
 * @brief 엔티티의 공기 저항력(항력 가속도) 계산
 *
 * @param curr   현재 엔티티 동적 상태
 * @param prev   과거 엔티티 동적 상태
 * @param dt     시간 간격 (초)
 * @param env    환경 정보 (밀도, 바람 등)
 * @param out_drag_accel 계산된 항력 가속도 (m/s²)
 */
BYUL_API void entity_dynamic_calc_drag_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,
    vec3_t* out_drag_accel
);

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
 * @brief 환경(env)을 고려한 동적 엔티티의 이동 및 회전 업데이트
 *
 * - 위치/속도는 중력, 항력, 바람을 포함한 가속도로 적분
 * - 회전은 angular_velocity 기반으로 업데이트
 * - base.age += dt
 *
 * @param d    동적 엔티티
 * @param env  환경 정보
 * @param dt   시간 간격 (초)
 */
BYUL_API void entity_dynamic_update_env(
    entity_dynamic_t* d,
    const environ_t* env,
    float dt);


// ---------------------------------------------------------
// 위치 계산: p(t) = p0 + v0 * dt
// ---------------------------------------------------------
/**
 * @brief 단순 등속도 이동으로 t초 후 예상 위치를 계산합니다.
 *
 * **환경(env) 영향을 고려하지 않고**, 
 * 현재 속도(`d->velocity`)와 마찰(`d->props.friction`)만 적용합니다.
 * 
 * @note
 * - 중력이나 항력 등 외력은 계산에 포함되지 않습니다.
 * - 일반적인 NPC나 오브젝트의 기본 이동에 사용됩니다.
 *
 * @param d        대상 동적 엔티티
 * @param dt       시간 간격 (초)
 * @param out_pos  계산된 위치 벡터
 */
BYUL_API void entity_dynamic_calc_position(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_pos
);

// ---------------------------------------------------------
// 속도 계산: v(t) = v0 (가속도 없음)
// ---------------------------------------------------------
/**
 * @brief t초 후 예상 속도를 계산합니다. (등속도 가정)
 *
 * **가속도가 없는 단순 등속도 운동**을 가정하여,
 * 현재 속도(`d->velocity`)와 마찰(`d->props.friction`)만 적용해 
 * t초 후 속도를 계산합니다.
 *
 * @note
 * - 추진기나 외부 가속도가 없는 일반 엔티티에 적합합니다.
 * - 환경 영향(env) 및 중력은 고려되지 않습니다.
 *
 * @param d        대상 동적 엔티티
 * @param dt       시간 간격 (초)
 * @param out_vel  계산된 속도 벡터
 */
BYUL_API void entity_dynamic_calc_velocity(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_vel
);

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
/**
 * @brief t초 후 엔티티의 위치 및 속도 상태를 예측합니다. (등속도 가정)
 *
 * `entity_dynamic_calc_position()`과 `entity_dynamic_calc_velocity()`를 함께 사용하여
 * **가속도가 없는 등속도 운동** 기반으로 
 * t초 후 위치와 속도를 `linear_state_t` 구조체로 반환합니다.
 *
 * @note
 * - 환경(env), 중력, 항력은 고려되지 않습니다.
 * - NPC나 일반 오브젝트의 단순 이동에 적합합니다.
 *
 * @param d         대상 동적 엔티티
 * @param dt        시간 간격 (초)
 * @param out_state 예측된 선형 상태 (위치 + 속도)
 */
BYUL_API void entity_dynamic_calc_state(
    const entity_dynamic_t* d,
    float dt,
    linear_state_t* out_state
);

// ---------------------------------------------------------
// 위치 계산: p(t) with env
// ---------------------------------------------------------
/**
 * @brief t초 후 예상 위치를 계산합니다. (환경 영향 포함)
 *
 * **중력(env->gravity), 바람(env->wind), 항력(Drag)** 등을 고려하여  
 * 등가속도 운동 공식 `p = p₀ + v₀·t + 0.5·a·t²`을 기반으로 위치를 예측합니다.
 *
 * @note
 * - 발사체나 물리 연산이 필요한 엔티티에 사용됩니다.
 * - `entity_dynamic_calc_position()`과 달리 가속도가 항상 존재할 수 있습니다.
 *
 * @param d        대상 동적 엔티티
 * @param env      환경 정보 (중력, 바람, 항력 등)
 * @param dt       시간 간격 (초)
 * @param out_pos  계산된 위치 벡터
 */
BYUL_API void entity_dynamic_calc_position_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_pos
);

// ---------------------------------------------------------
// 속도 계산: v(t) with env
// ---------------------------------------------------------
/**
 * @brief t초 후 예상 속도를 계산합니다. (환경 영향 포함)
 *
 * **중력(env->gravity)** 및 **항력(Drag)**의 영향을 받아 
 * 속도가 시간에 따라 감소/증가하는 효과를 포함합니다.
 *
 * @param d        대상 동적 엔티티
 * @param env      환경 정보
 * @param dt       시간 간격 (초)
 * @param out_vel  계산된 속도 벡터
 */
BYUL_API void entity_dynamic_calc_velocity_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_vel
);

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
/**
 * @brief t초 후 전체 상태(위치 + 속도)를 예측합니다. (환경 영향 포함)
 *
 * 위치(`entity_dynamic_calc_position_env`)와 속도(`entity_dynamic_calc_velocity_env`)를  
 * 동시에 계산하여 `linear_state_t` 구조체로 반환합니다.
 *
 * @param d         대상 동적 엔티티
 * @param env       환경 정보
 * @param dt        시간 간격 (초)
 * @param out_state 예측된 동적 엔티티 상태
 */
BYUL_API void entity_dynamic_calc_state_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    linear_state_t* out_state
);

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

BYUL_API bool entity_dynamic_bounce(
    const entity_dynamic_t* d,
    const vec3_t* normal,
    vec3_t* out_velocity_out);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_DYNAMIC_H
