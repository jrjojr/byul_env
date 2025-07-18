#ifndef PROJECTILE_H
#define PROJECTILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_config.h"
#include "internal/numeq.h"
#include "internal/xform.h"
#include "internal/trajectory.h"
#include "internal/controller.h"

// ---------------------------------------------------------
// 탄도체 유형 구분
// ---------------------------------------------------------
typedef enum e_projectile_type {
    PROJECTILE_TYPE_SHELL,
    PROJECTILE_TYPE_MISSILE
} projectile_type_t;

// ---------------------------------------------------------
// 구조체 선언
// ---------------------------------------------------------
typedef struct s_projectile projectile_t;
typedef struct s_shell shell_t;
typedef struct s_missile missile_t;

// ---------------------------------------------------------
// 함수 포인터 타입 정의
// ---------------------------------------------------------
typedef void (*projectile_hit_cb)(const projectile_t* proj, void* userdata);
typedef const vec3_t* (*projectile_environ_func)(
    const projectile_t* proj, float dt, void* userdata);

typedef const vec3_t* (*projectile_guidance_func)(
    const projectile_t* proj, float dt, void* userdata);

// ---------------------------------------------------------
// 공통 탄도체 구조체
// ---------------------------------------------------------
struct s_projectile {
    xform_t xf;          // 위치 + 회전
    vec3_t velocity;               // 현재 속도
    vec3_t acceleration;           // 외부 가속도
    vec3_t angular_velocity;       // 자전 속도

    float age;                     // 경과 시간
    float lifetime;                // 최대 생존 시간

    projectile_type_t type;        // 탄도체 유형
    int32_t projectile_id;         // 고유 식별자
    void* owner;                   // 소유 객체 (예: NPC 등)

    projectile_hit_cb on_hit;      // 충돌 또는 종료 콜백
    void* hit_userdata;            // 콜백용 사용자 데이터
};

// ---------------------------------------------------------
// 포탄 (비유도)
// ---------------------------------------------------------
struct s_shell {
    projectile_t base;

    float drag_coef;                       // 공기 저항 계수

    projectile_environ_func env_fn;       // 외부 환경 함수
    void* env_userdata;                   // 환경 함수 사용자 데이터
};

// ---------------------------------------------------------
// 미사일 (유도)
// ---------------------------------------------------------
struct s_missile {
    projectile_t base;

    vec3_t thrust;                         // 추진력
    float fuel;                            // 연료량

    controller_t* controller;              // 제어기 (PID, MPC, Bang-Bang)

    projectile_guidance_func guidance_fn;  // 유도 함수
    void* guidance_userdata;               // 유도용 사용자 데이터

    projectile_environ_func env_fn;        // 외부 환경 함수
    void* env_userdata;                    // 환경 함수 사용자 데이터
};

// ---------------------------------------------------------
// 프레임별 업데이트 함수
// ---------------------------------------------------------
void shell_update(shell_t* shell, float dt);
void missile_update(missile_t* missile, float dt);

// ---------------------------------------------------------
// 기본 콜백 및 함수 구현
// ---------------------------------------------------------

// 충돌 콜백: 아무 일도 하지 않음 (디버그용)
void projectile_default_hit_cb(const projectile_t* proj, void* userdata);

// 환경 함수: 바람 없음
const vec3_t* projectile_env_none(const projectile_t* proj, 
    float dt, void* userdata);

// 환경 함수: 일정한 바람 (X 방향)
const vec3_t* projectile_env_constant(const projectile_t* proj, 
    float dt, void* userdata);

// 유도 함수: 유도 없음
const vec3_t* projectile_guidance_none(const projectile_t* proj, 
    float dt, void* userdata);

// 유도 함수: 고정 타겟 향해 직선 유도 (userdata = const vec3_t* target_pos)
const vec3_t* projectile_guidance_to_target(const projectile_t* proj, 
    float dt, void* userdata);


// 유도 함수: 목표 위치와 속도를 고려한 리드 유도
// userdata = const target_info_t* (목표의 position, velocity)
typedef struct {
    vec3_t position;
    vec3_t velocity;
} target_info_t;

const vec3_t* projectile_guidance_lead(
    const projectile_t* proj, float dt, void* userdata);

typedef struct {
    const trajectory_t* trajectory; // 타겟의 궤적 (시간별 위치 데이터)
    float current_time;             // 현재 시간
} target_traj_info_t;

// 유도 함수: 타겟의 trajectory 기반 예측
const vec3_t* projectile_guidance_from_trajectory(
    const projectile_t* proj, float dt, void* userdata);


void projectile_apply_rotation(projectile_t* proj, float dt);    

// ---------------------------------------------------------
// 탄도 예측 구조체 및 API
// ---------------------------------------------------------
typedef struct s_projectile_result {
    float impact_time;      // 예측 충돌 시각
    vec3_t impact_pos;      // 예측 충돌 위치
    bool valid;             // 예측 유효 여부

    trajectory_t trajectory; // 충돌 전까지의 궤적 전체
} projectile_result_t;

typedef struct s_projectile_predictor {
    vec3_t start_pos;
    vec3_t start_velocity;
    vec3_t gravity;

    projectile_environ_func env_fn;
    void* env_userdata;

    float ground_height;
    float max_time;
    float time_step;
} projectile_predictor_t;

// 예측 함수 (out_result에 기록, bool로 성공 여부 반환)
bool projectile_predict(const projectile_predictor_t* predictor,
                        projectile_result_t* out_result);

typedef struct s_missile_predictor {
    vec3_t start_pos;            ///< 초기 위치
    vec3_t start_velocity;       ///< 초기 속도
    vec3_t gravity;              ///< 중력 가속도
    vec3_t thrust;               ///< 추진력 크기 및 최대 방향

    float fuel;                  ///< 연료량
    controller_t* controller;        // 제어기 추가 (PID, MPC 등)    

    projectile_guidance_func guidance_fn; ///< 유도 함수
    void* guidance_userdata;

    projectile_environ_func env_fn;       ///< 외부 환경 함수
    void* env_userdata;

    float ground_height;         ///< 지면 충돌 기준 높이
    float max_time;              ///< 최대 시뮬레이션 시간
    float time_step;             ///< 적분 시간 간격

    integrator_type_t integrator_type;  ///< ✅ 적분 방식 (Euler, RK4 등)
} missile_predictor_t;

typedef struct s_missile_state {
    motion_state_t motion; // 일반 운동 상태
    float fuel;            // 미사일 전용
} missile_state_t;

bool projectile_predict_missile(const missile_predictor_t* p,
                                 projectile_result_t* out);


#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
