#pragma once
#include "internal/xform.h"
#include "internal/vec3.h"
#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------
// 콜백 타입 정의
// ----------------------------
typedef void (*projectile_hit_cb)(const struct s_projectile* proj, void* userdata);
typedef const vec3_t* (*projectile_guidance_func)(const struct s_projectile* proj, float dt, void* userdata);
typedef void (*projectile_environ_func)(vec3_t* out_accel, const struct s_projectile* proj, float dt, void* userdata);

// ----------------------------
// 발사체 기본 구조체
// ----------------------------
typedef struct s_projectile {
    xform_t xf;                  ///< 위치 + 회전(자세)
    vec3_t velocity;             ///< 현재 속도
    vec3_t angular_velocity;     ///< 회전 속도 (회전축은 xf.rotation 기준)
    
    float mass;                  ///< 발사체 질량 (kg 단위)
    float age;                   ///< 경과 시간 (초)
    float lifetime;              ///< 최대 생존 시간 (초)

    projectile_type_t type;      ///< 발사체 유형
    int32_t projectile_id;       ///< 고유 식별자
    void* owner;                 ///< 소유 객체 (예: NPC, 무기 등)

    projectile_hit_cb on_hit;    ///< 충돌 또는 종료 콜백
    void* hit_userdata;          ///< 콜백용 사용자 데이터
} projectile_t;

// ----------------------------
// 추진 시스템 구조체
// ----------------------------
typedef struct s_propulsion {
    float max_thrust;            ///< 최대 추진력 (N)
    float current_thrust;        ///< 현재 출력 (controller로 계산된 값)
    float fuel_capacity;         ///< 총 연료량 (kg)
    float fuel_remaining;        ///< 남은 연료량 (kg)
    float burn_rate;             ///< 초당 연료 소모량 (kg/s)

    controller_t* controller;    ///< PID, MPC, Bang-Bang 등 제어 장치
    bool active;                 ///< 현재 추진 중인지 여부
} propulsion_t;

// ----------------------------
// 예측 결과 구조체
// ----------------------------
typedef struct s_projectile_result {
    float impact_time;           ///< 예측 충돌 시각 (초 단위)
    vec3_t impact_pos;           ///< 예측 충돌 위치 (월드 좌표)
    bool valid;                  ///< 예측 결과의 유효 여부 (true면 충돌 발생)

    trajectory_t* trajectory;    ///< 예측 궤적 데이터 (동적 메모리 할당)
} projectile_result_t;

// ----------------------------
// 발사체 예측 함수
// ----------------------------
/**
 * @brief 발사체 및 미사일 궤적 예측 함수
 *
 * propulsion, guidance, env_fn을 포함한 모든 요소를 고려하여 발사체의
 * 충돌 시점과 궤적을 예측합니다.
 *
 * - 발사체 모드: propulsion = NULL
 * - 미사일 모드: propulsion != NULL
 *
 * @param[out] out 예측 결과 (impact_time, impact_pos, trajectory 포함)
 * @param[in] proj 발사체 기본 정보 (s_projectile)
 * @param[in] propulsion 추진력 및 연료 정보 (NULL이면 발사체)
 * @param[in] guidance_fn 유도 함수 (NULL이면 projectile_guidance_none)
 * @param[in] guidance_userdata 유도 함수용 데이터 (target_info_t*, trajectory 등)
 * @param[in] max_time 최대 시뮬레이션 시간 (초)
 * @param[in] time_step 시뮬레이션 간격 (초)
 * @param[in] env_fn 외부 환경 함수 (NULL이면 projectile_env_none)
 * @param[in] env_userdata 환경 함수용 데이터
 */
BYUL_API void projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    const propulsion_t* propulsion,
    projectile_guidance_func guidance_fn,
    void* guidance_userdata,
    float max_time,
    float time_step,
    projectile_environ_func env_fn,
    void* env_userdata
);

#ifdef __cplusplus
}
#endif
