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
// 🔧 초기화 및 복사 함수
// ---------------------------------------------------------

/**
 * @brief projectile_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - 위치/회전: 단위 xform
 * - velocity/acceleration/angular_velocity: (0,0,0)
 * - age: 0
 * - lifetime: 10초 최소 0보다 커야 함
 * - type: PROJECTILE_TYPE_SHELL
 * - projectile_id: -1
 * - owner: NULL
 * - on_hit: projectile_default_hit_cb
 * - hit_userdata: NULL
 *
 * @param[out] out 초기화할 projectile_t 포인터
 */
BYUL_API void projectile_init(projectile_t* out);

/**
 * @brief projectile_t를 주어진 값으로 초기화합니다.
 *
 * @param[out] out 초기화할 projectile_t 포인터
 * @param[in] type 탄도체 유형 (PROJECTILE_TYPE_SHELL / PROJECTILE_TYPE_MISSILE)
 * @param[in] lifetime 최대 생존 시간 (초)
 */
BYUL_API void projectile_init_full(projectile_t* out, 
    projectile_type_t type, float lifetime);

/**
 * @brief projectile_t를 다른 projectile_t로 복사합니다.
 *
 * @param[out] out 대상 projectile_t
 * @param[in] src 원본 projectile_t
 */
BYUL_API void projectile_assign(projectile_t* out, const projectile_t* src);


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
// 🧩 Shell 초기화 및 복사
// ---------------------------------------------------------

/**
 * @brief shell_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - base: projectile_init() 호출
 * - drag_coef: 0.0f (공기 저항 없음)
 * - env_fn: projectile_env_none
 * - env_userdata: NULL
 *
 * @param[out] shell 초기화할 shell_t 포인터
 */
BYUL_API void shell_init(shell_t* shell);

/**
 * @brief shell_t를 주어진 값으로 초기화합니다.
 *
 * @param[out] shell 초기화할 shell_t 포인터
 * @param[in] drag_coef 공기 저항 계수
 * @param[in] env_fn 외부 환경 함수 (NULL이면 projectile_env_none)
 * @param[in] env_userdata 환경 함수용 사용자 데이터
 */
BYUL_API void shell_init_full(shell_t* shell, float drag_coef,
            projectile_environ_func env_fn, void* env_userdata);

/**
 * @brief shell_t를 다른 shell_t로 복사합니다.
 *
 * @param[out] out 대상 shell_t
 * @param[in] src 원본 shell_t
 */
BYUL_API void shell_assign(shell_t* out, const shell_t* src);


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
// 🚀 Missile 초기화 및 복사
// ---------------------------------------------------------

/**
 * @brief missile_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - base: projectile_init() 호출
 * - thrust: (0, 0, 0)
 * - fuel: 0
 * - controller: NULL
 * - guidance_fn: projectile_guidance_none
 * - guidance_userdata: NULL
 * - env_fn: projectile_env_none
 * - env_userdata: NULL
 *
 * @param[out] missile 초기화할 missile_t 포인터
 */
BYUL_API void missile_init(missile_t* missile);

/**
 * @brief missile_t를 주어진 값으로 초기화합니다.
 *
 * @param[out] missile 초기화할 missile_t 포인터
 * @param[in] thrust 추진력 벡터
 * @param[in] fuel 연료량
 * @param[in] controller 제어기 포인터 (NULL 가능)
 * @param[in] guidance_fn 유도 함수 (NULL이면 projectile_guidance_none)
 * @param[in] guidance_userdata 유도 함수 사용자 데이터
 * @param[in] env_fn 외부 환경 함수 (NULL이면 projectile_env_none)
 * @param[in] env_userdata 환경 함수 사용자 데이터
 */
BYUL_API void missile_init_full(missile_t* missile,
                                const vec3_t* thrust,
                                float fuel,
                                controller_t* controller,
                                projectile_guidance_func guidance_fn,
                                void* guidance_userdata,
                                projectile_environ_func env_fn,
                                void* env_userdata);

/**
 * @brief missile_t를 다른 missile_t로 복사합니다.
 *
 * @param[out] out 대상 missile_t
 * @param[in] src 원본 missile_t
 */
BYUL_API void missile_assign(missile_t* out, const missile_t* src);

// ---------------------------------------------------------
// 프레임별 업데이트 함수
// ---------------------------------------------------------
BYUL_API void shell_update(shell_t* shell, float dt);
BYUL_API void missile_update(missile_t* missile, float dt);

// ---------------------------------------------------------
// 기본 콜백 및 함수 구현
// ---------------------------------------------------------

// 충돌 콜백: 아무 일도 하지 않음 (디버그용)
BYUL_API void projectile_default_hit_cb(const projectile_t* proj, void* userdata);

/**
 * @brief 환경 함수: 외부 힘 없음 (중력/바람 미적용)
 *
 * 완전히 자유로운 운동 시뮬레이션에 사용됩니다.
 * 가속도 벡터 (0, 0, 0)을 반환합니다.
 *
 * @param[in] proj 현재 탄도체 (사용하지 않음)
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata 추가 데이터 (NULL)
 * @return 항상 (0, 0, 0) 벡터를 반환
 */
BYUL_API const vec3_t* projectile_env_none(const projectile_t* proj, 
    float dt, void* userdata);

/**
 * @brief 환경 함수: 기본 중력 적용
 *
 * 표준 중력 (-9.81 m/s²)을 Y축 방향으로 적용합니다.
 * 바람 및 추가 외부 힘은 포함되지 않습니다.
 *
 * @param[in] proj 현재 탄도체 (사용하지 않음)
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata 추가 데이터 (NULL)
 * @return 중력 벡터 (0, -9.81, 0)
 */    
BYUL_API const vec3_t* projectile_env_default(
    const projectile_t* proj, float dt, void* userdata);

/**
 * @brief 환경 함수: 일정한 바람 + 중력
 *
 * 고정된 바람(외력)과 중력을 함께 적용합니다.
 * userdata로 vec3_t* 타입의 바람 벡터를 전달할 수 있습니다.
 *
 * @param[in] proj 현재 탄도체 (사용하지 않음)
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata const vec3_t* (고정 바람 벡터)
 * @return 바람 + 중력 합산 가속도 벡터
 */
BYUL_API const vec3_t* projectile_env_constant(const projectile_t* proj, 
    float dt, void* userdata);

/**
 * @struct env_dynamic_data_t
 * @brief 동적 환경 데이터 구조체
 *
 * 시간에 따라 변동하는 바람(외력)을 표현하기 위한 데이터 구조체입니다.
 * projectile_env_dynamic 함수의 userdata로 전달됩니다.
 */    
typedef struct {
    vec3_t base_wind;    ///< 기본 바람 벡터
    float gust_strength; ///< 바람의 변동 강도 (진폭)
    float time;          ///< 내부 시간 (업데이트 시 누적)
} env_dynamic_data_t;

/**
 * @brief 환경 함수: 동적 바람 + 중력
 *
 * 시간에 따라 변화하는 바람(예: sin 파형)을 생성하고 중력과 합산합니다.
 * userdata는 env_dynamic_data_t* 타입이며, time 값은 dt만큼 증가합니다.
 *
 * @param[in] proj 현재 탄도체 (사용하지 않음)
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata env_dynamic_data_t* (동적 환경 데이터)
 * @return 동적 바람 + 중력의 합산 가속도 벡터
 */
BYUL_API const vec3_t* projectile_env_dynamic(
    const projectile_t* proj, float dt, void* userdata);

/**
 * @brief 유도 함수: 유도 없음
 *
 * 미사일이나 포탄이 별도의 유도 없이 현재 방향 그대로 직진합니다.
 * 항상 (0, 0, 0) 벡터를 반환합니다.
 *
 * @param[in] proj 현재 탄도체 포인터
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata 추가 데이터 (사용하지 않음)
 * @return 항상 NULL 또는 (0,0,0) 벡터 포인터
 */
BYUL_API const vec3_t* projectile_guidance_none(const projectile_t* proj, 
    float dt, void* userdata);


/**
 * @brief 유도 함수: 고정 타겟 향해 직선 유도
 * 유도 함수: 고정 타겟 향해 직선 유도 (userdata = const vec3_t* target_pos)
 * 지정된 목표 위치를 향해 직선 경로를 따라가도록 유도합니다.
 * userdata는 const vec3_t* 타입의 목표 위치 포인터여야 합니다.
 *
 * @param[in] proj 현재 탄도체 포인터
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata const vec3_t* (목표 위치)
 * @return 목표 방향 벡터 포인터
 */
BYUL_API const vec3_t* projectile_guidance_to_target(const projectile_t* proj, 
    float dt, void* userdata);

/**
 * @struct target_info_t
 * @brief 리드 유도용 목표 정보
 *
 * 목표의 위치와 속도를 기반으로 리드(선도) 유도를 계산하기 위한 데이터 구조체입니다.
 */
typedef struct {
    vec3_t position;
    vec3_t velocity;
} target_info_t;

/**
 * @brief 유도 함수: 목표 위치와 속도를 고려한 리드 유도
 *
 * 목표의 이동 속도를 고려하여 예측된 위치를 향해 유도 벡터를 계산합니다.
 * userdata는 target_info_t* 타입이어야 합니다.
 *
 * @param[in] proj 현재 탄도체 포인터
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata target_info_t* (목표 정보)
 * @return 유도 벡터 포인터
 */
BYUL_API const vec3_t* projectile_guidance_lead(
    const projectile_t* proj, float dt, void* userdata);

/**
 * @struct target_traj_info_t
 * @brief trajectory 기반 유도 데이터
 *
 * 타겟의 궤적(trajectory_t)과 현재 시간 정보를 포함하는 구조체입니다.
 */    
typedef struct {
    const trajectory_t* trajectory; // 타겟의 궤적 (시간별 위치 데이터)
    float current_time;             // 현재 시간
} target_traj_info_t;

/**
 * @brief 유도 함수: 타겟의 trajectory 기반 예측
 *
 * 타겟의 미래 궤적 데이터를 기반으로 유도 벡터를 계산합니다.
 * userdata는 target_traj_info_t* 타입이어야 합니다.
 *
 * @param[in] proj 현재 탄도체 포인터
 * @param[in] dt   시간 간격 (초)
 * @param[in] userdata target_traj_info_t* (타겟 궤적 정보)
 * @return 유도 벡터 포인터
 */
BYUL_API const vec3_t* projectile_guidance_from_trajectory(
    const projectile_t* proj, float dt, void* userdata);

/**
 * @brief 탄도체 회전 적용
 *
 * projectile_t의 angular_velocity를 기반으로 xform_t에 회전을 적용합니다.
 * 이 함수는 프레임 업데이트 시 회전 방향을 갱신하는 데 사용됩니다.
 *
 * @param[in,out] proj 회전을 적용할 projectile_t 포인터
 * @param[in] dt       시간 간격 (초)
 */
BYUL_API void projectile_apply_rotation(projectile_t* proj, float dt);    

// ---------------------------------------------------------
// 탄도 예측 구조체 및 API
// ---------------------------------------------------------

/**
 * @struct projectile_result_t
 * @brief 탄도체 궤적 예측 결과를 저장하는 구조체.
 *
 * 이 구조체는 탄도체가 이동하는 동안 충돌 예상 시각과 위치,
 * 예측 궤적(trajectory)을 포함합니다.
 * trajectory_t는 예측된 궤적의 각 시점 데이터를 관리합니다.
 */
typedef struct s_projectile_result {
    float impact_time;       ///< 예측 충돌 시각 (초 단위)
    vec3_t impact_pos;       ///< 예측 충돌 위치 (월드 좌표)
    bool valid;              ///< 예측 결과의 유효 여부 (true면 충돌이 발생함)

    trajectory_t* trajectory; ///< 예측 궤적 데이터 (동적 메모리 할당됨)
} projectile_result_t;

/**
 * @brief 기본 projectile_result_t 객체를 생성합니다.
 *
 * 기본 용량(capacity = 100)을 갖는 trajectory를 내부적으로 생성하고,
 * impact_time, impact_pos, valid 값을 초기화합니다.
 *
 * @return 생성된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_create();

/**
 * @brief 지정된 capacity로 trajectory를 생성하는 projectile_result_t 객체를 생성합니다.
 *
 * @param capacity trajectory에 할당할 최대 샘플 개수
 * @return 생성된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_create_full(int capacity);

/**
 * @brief 기존 projectile_result_t 객체를 복제합니다.
 *
 * @param src 원본 projectile_result_t (NULL 불가)
 * @return 복제된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note trajectory도 깊은 복사(Deep Copy)됩니다.
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_assign(
    const projectile_result_t* src);

/**
 * @brief projectile_result_t 객체와 내부 trajectory를 해제합니다.
 *
 * @param res 해제할 projectile_result_t 포인터 (NULL 가능)
 * @note trajectory_destroy()가 내부적으로 호출되며 res 자체도 free됩니다.
 */
BYUL_API void projectile_result_destroy(projectile_result_t* res);

typedef struct s_projectile_predictor {
    vec3_t start_pos;         ///< 초기 위치
    vec3_t start_velocity;    ///< 초기 속도

    projectile_environ_func env_fn; ///< 외부 환경 함수 (중력+바람 등)
    void* env_userdata;             ///< 환경 함수 데이터

    float ground_height;      ///< 지면 높이
    float max_time;           ///< 최대 예측 시간
    float time_step;          ///< 적분 간격
} projectile_predictor_t;

// ---------------------------------------------------------
// 🔮 projectile_predictor_t 초기화 및 복사
// ---------------------------------------------------------

/**
 * @brief projectile_predictor_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - start_pos = (0,0,0)
 * - start_velocity = (0,0,0)
 * - env_fn = projectile_env_none
 * - env_userdata = NULL
 * - ground_height = 0.0f
 * - max_time = 10.0f
 * - time_step = 0.01f
 *
 * @param[out] out 초기화할 projectile_predictor_t
 */
BYUL_API void projectile_predictor_init(projectile_predictor_t* out);

/**
 * @brief projectile_predictor_t를 주어진 값으로 초기화합니다.
 *
 * @param[out] out 초기화할 predictor
 * @param[in] start_pos 시작 위치
 * @param[in] start_velocity 시작 속도
 * @param[in] ground_height 지면 높이
 * @param[in] max_time 최대 시뮬레이션 시간
 * @param[in] time_step 시뮬레이션 간격
 * @param[in] env_fn 환경 함수 (NULL이면 projectile_env_none)
 * @param[in] env_userdata 환경 데이터
 */
BYUL_API void projectile_predictor_init_full(
    projectile_predictor_t* out,
    const vec3_t* start_pos,
    const vec3_t* start_velocity,
    float ground_height,
    float max_time,
    float time_step,
    projectile_environ_func env_fn,
    void* env_userdata
);

/**
 * @brief projectile_predictor_t를 복사합니다.
 *
 * @param[out] out 대상 predictor
 * @param[in] src 원본 predictor
 */
BYUL_API void projectile_predictor_assign(projectile_predictor_t* out,
                                        const projectile_predictor_t* src);

// 예측 함수 (out_result에 기록, bool로 성공 여부 반환)
BYUL_API bool projectile_predict(const projectile_predictor_t* predictor,
                        projectile_result_t* out_result);

typedef struct s_missile_predictor {
    vec3_t start_pos;            ///< 초기 위치
    vec3_t start_velocity;       ///< 초기 속도
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

// ---------------------------------------------------------
// 🚀 missile_predictor_t 초기화 및 복사
// ---------------------------------------------------------

/**
 * @brief missile_predictor_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - start_pos = (0,0,0)
 * - start_velocity = (0,0,0)
 * - thrust = (0,0,0)
 * - fuel = 0
 * - controller = NULL
 * - guidance_fn = projectile_guidance_none
 * - guidance_userdata = NULL
 * - env_fn = projectile_env_none
 * - env_userdata = NULL
 * - ground_height = 0
 * - max_time = 10초
 * - time_step = 0.01초
 * - integrator_type = INTEGRATOR_EULER
 *
 * @param[out] out 초기화할 missile_predictor_t 포인터
 */
BYUL_API void missile_predictor_init(missile_predictor_t* out);

/**
 * @brief missile_predictor_t를 주어진 값으로 초기화합니다.
 *
 * @param[out] out 초기화할 missile_predictor_t 포인터
 * @param[in] start_pos 초기 위치
 * @param[in] start_velocity 초기 속도
 * @param[in] thrust 추진력 벡터
 * @param[in] fuel 연료량
 * @param[in] controller 제어기
 * @param[in] guidance_fn 유도 함수 (NULL이면 projectile_guidance_none)
 * @param[in] guidance_userdata 유도 데이터
 * @param[in] env_fn 외부 환경 함수 (NULL이면 projectile_env_none)
 * @param[in] env_userdata 환경 데이터
 * @param[in] ground_height 지면 높이
 * @param[in] max_time 최대 시뮬레이션 시간
 * @param[in] time_step 적분 간격
 * @param[in] integrator_type 적분 방식
 */
BYUL_API void missile_predictor_init_full(
    missile_predictor_t* out,
    const vec3_t* start_pos,
    const vec3_t* start_velocity,
    const vec3_t* thrust,
    float fuel,
    controller_t* controller,
    projectile_guidance_func guidance_fn,
    void* guidance_userdata,
    projectile_environ_func env_fn,
    void* env_userdata,
    float ground_height,
    float max_time,
    float time_step,
    integrator_type_t integrator_type
);

/**
 * @brief missile_predictor_t를 복사합니다.
 *
 * @param[out] out 대상 predictor
 * @param[in] src 원본 predictor
 */
BYUL_API void missile_predictor_assign(missile_predictor_t* out,
                                     const missile_predictor_t* src);

typedef struct s_missile_state {
    motion_state_t motion; // 일반 운동 상태
    float fuel;            // 미사일 전용
} missile_state_t;

BYUL_API bool projectile_predict_missile(const missile_predictor_t* p,
                                 projectile_result_t* out);


#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
