// 로켓엔진, 제트엔진을 사용하는 모든 물체들 미사일, 비행기 등을 포함해야 겠다.
// 단순히 발사체 총이나, 돌멩이로 하기에는 
// 유도 시스템과 추진기 시스템이 너무 아깝다

// 자동차의 내연기관 엔진과는 다르다. 어쩌면 내연기관 엔진도 포함될수 있겠다.

#ifndef PROJECTILE_H
#define PROJECTILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_common.h"
#include "internal/entity_dynamic.h"
#include "internal/projectile_common.h"
#include "internal/guidance.h"
#include "internal/propulsion.h"
#include "internal/projectile_predict.h"

// 전방 선언
// typedef struct s_projectile projectile_t;

/*
 * @return `true`이면 계산 성공, `false`이면 해당 힘으로 목표에 도달할 수 없음
 외부의 힘 즉 중력이 없으니까 발사체가 그냥 직진해야 하네
 이건 그냥 테스트 용으로 사용해야 할정도인가 중력일 없으면 그냥 직선으로 진행하면 끝
 자기가 같고있는 bodyprops의 마찰계수만이 동작해서 시간이 지나면 속도가 0이 되는건가
 속도가 0이 되어도 그냥 공중에 떠있어야 하네 중력이 없으니까
 일반 사용자는 shell_projectile_launch_env를 사용해야 한다.
 중력이 있어야 실제 상황이 발생하지 
 계속 공중에 떠있다가 자기 수명이 다하면 그냥 사라져야 겠네 
 수명은 모든 엔티티가 갖고 있는 특성이다 발사체 전용이 아니다.
 */
BYUL_API bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* target, // 현재 내가 보고 있는 방향, 여기로 발사한다, dir이 더 정확한 표현인가?
    float initial_force, // 초기의 외부의 힘 발사체에 가해질...
    projectile_result_t* out
);

/**
 * @brief 환경 정보를 고려하여 포탄이 target 위치에 도달하기 위한 
 * 초기 발사 속도와 예상 비행 시간을 계산합니다.
 *
 * 중력, 바람, 공기 저항 등 환경 요소를 반영하여 궤적을 얻는다.
 *
 * @param[out] out           계산 결과 (목표까지의 궤적 내가 보고 있는 방향으로 발사했지만 결과는 다른곳이겠지)
 * @param[in]  proj          발사체 정보 (시작 위치, 질량 등)
 * @param[in]  env           환경 정보 (중력, 바람, 공기 저항 등)
 * @param[in]  target        목표 지점의 세계 좌표, 목표라기보단 내가 보고 있는 방향이 맞겠다
 * @param[in]  initial_force 발사 순간에 적용되는 힘 (뉴턴, N)
 *                           - 1 N = 1 kg × 1 m/s² (뉴턴의 정의)
 *                           - 평균 권장값:
 *                             * 1 kg 발사체 → 약 10~100 N (10~30 m/s 초기 속도)
 *                             * 10 kg 발사체 → 약 500~5000 N (20~100 m/s 초기 속도)
 *
 * @return `true`이면 계산 성공, `false`이면 주어진 힘과 환경으로 목표에 도달할 수 없음
 */
BYUL_API bool projectile_launch_env(
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force,
    projectile_result_t* out    
);

typedef struct s_shell_projectile {
    projectile_t proj;
    float explosion_radius;
} shell_projectile_t;

BYUL_API void shell_projectile_init(shell_projectile_t* shell);
BYUL_API void shell_projectile_init_full(shell_projectile_t* shell,
    projectile_t* proj,
    float explosion_radius
);

/*
 * @return `true`이면 계산 성공, `false`이면 해당 힘으로 목표에 도달할 수 없음
 외부의 힘 즉 중력이 없으니까 발사체가 그냥 직진해야 하네
 이건 그냥 테스트 용으로 사용해야 할정도인가 중력일 없으면 그냥 직선으로 진행하면 끝
 자기가 같고있는 bodyprops의 마찰계수만이 동작해서 시간이 지나면 속도가 0이 되는건가
 속도가 0이 되어도 그냥 공중에 떠있어야 하네 중력이 없으니까
 일반 사용자는 shell_projectile_launch_env를 사용해야 한다.
 중력이 있어야 실제 상황이 발생하지
 */
BYUL_API bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* target,
    float initial_force,
    projectile_result_t* out
);

// projectile_launch_env와 같다 다를게 없네 단지 목표에 도달하거나 바닥에 추락하면 
// 폭발 반경이 있겠네
BYUL_API bool shell_projectile_launch_env(
    const shell_projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force,
    projectile_result_t* out    
);

typedef struct s_missile {
    shell_projectile_t shell;
    propulsion_t propulsion;
    guidance_func guidance_fn;
    void* guidance_userdata;
} missile_t;

// 추진기는 미사일의 기본 속성이고,
// 부가적으로 유도장치가 붙을수도 있고 없을 수도 있고
// 유도장치가 있으면 외부 환경이 변해도 또는 자기자신의 마찰계수로 인해 속도가
// 줄어도 연료가 떨어질때까지는 속도를 유지한다. 그래서 내가 보고있는 방향으로 계속 진행
// 만약 중력의 영향을 받고 싶으면 missile_launch_env 사용해라. 이게 기본값
// 특수 목적으로 외부환경의 영향 즉 중력의 영향을 받지않느 상황에서는
// missile_launch 사용한다 아주 특수한 상황이다 자기자신의 마찰계수로 인해 속도는 줄어든다.
// 
BYUL_API void missile_init(missile_t* missile);
BYUL_API void missile_init_full(missile_t* missile,
    shell_projectile_t* shell,
    propulsion_t* propulsion,
    guidance_func guidance_fn,
    void* guidance_userdata
);

/*
 * @return `true`이면 계산 성공, `false`이면 목표에 도달할 수 없음
 미사일은 추진기로 발사한다 외부의 힘이 아니라
 일반 발사체와 많이 다르다
 목표는 내가 보고 있는 방향으로 해야하나, 아니면 추진기와 유도기능이 없으면 내가 보고 있는
 방향이 확실하다.
 하지만 미사일은 추진기와 유도장치가 있다 그래도 내가 보고있는방향이라고 해야 하나
 단지 차이는 발사체는 내가 보고 있는 방향으로 발사해도 외부의 힘때문에 도착할때의 위치가
 내가 보고 있는 방향이 아니라 다른 곳이겠지
 미사일은 내가 보고있는 방향 그대로 인가. 추진기와 유도장치가 있다면
 추진기만 있을때에는 일반 발사체와 같겠네 외부의 힘에 의해 실제 도착한곳이 다르겠지
 대부분이 내가 보고있는 방향에 정확히 도착하지 않는다 유도 장치가 없으면
 만약 유도 장치가 있으면 내가 보고있는 방향으로 정확히 일직선으로 가야하나
 추진기가 외부의 힘을 상쇄할거니까
 하지만 추진기 연료가 떨어지면 일반 발사체가 되는거지
 일반 미사일 발사는 환경이 적용되지 않으면 단지 중력도 무시하고 나의 연료가 
 떨어질때까지 일직선으로 진행한다 연료가 떨어지면 바닥에 떨어지지 않는다
 외부의 힘 즉 중력이 없으니까 중력을 받고 싶으면 missile_launch_env함수를 사용해라
 */
BYUL_API bool missile_launch(
    const missile_t* missile,
    const vec3_t* target,
    projectile_result_t* out
);

/*
 * @return `true`이면 계산 성공, `false`이면 목표에 도달할 수 없음
 미사일은 추진기로 발사한다 외부의 힘이 아니라
 missile_launch와 같다 하지만 외부의 힘이 작용해서 중력같은게 ...
 무조건 내가 보고 있는 방향을로 직진하지 못한다 
 추진기로 자세제어를 해서 직진성을 유지해야 한다. 유도장치와 합작해서 직진성 유지
 유도 장치가 있어야만 목표에 정확히 도착한다.
  */
BYUL_API bool missile_launch_env(
    const missile_t* missile,
    const environ_t* env,    
    const vec3_t* target,
    projectile_result_t* out
);

// 미사일과 같지만 target이 entity_dynamic이 된다.
// entity_dynamic이면 유도장치가 entdyn 의 속도를 보고 추적한다.
BYUL_API bool patriot_launch(
    const missile_t* missile,
    const entity_dynamic_t* target,
    projectile_result_t* out
);

// 미사일과 같지만 target이 entity_dynamic이 된다.
// entity_dynamic이면 유도장치가 entdyn 의 속도를 보고 추적한다.
BYUL_API bool patriot_launch_accel(
    const missile_t* missile,
    const entity_dynamic_t* target,
    projectile_result_t* out
);

// 미사일과 같지만 target이 entity_dynamic이 된다.
// entity_dynamic이면 유도장치가 entdyn 의 속도를 보고 추적한다.
// 외부 환경 즉 중력의 영향을 받는다
BYUL_API bool patriot_launch_env(
    const missile_t* missile,
    const environ_t* env,
    const entity_dynamic_t* target,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 항공 발사체 (비행기)
// ---------------------------------------------------------
typedef struct s_aerial_projectile {
    projectile_t proj;          ///< 기본 발사체
    float wing_area;            ///< 날개 면적 (m²)
    float lift_coefficient;     ///< 양력 계수
    float drag_coefficient;     ///< 항력 계수
} aerial_projectile_t;

BYUL_API void aerial_projectile_init(aerial_projectile_t* ap);
BYUL_API void aerial_projectile_init_full(
    aerial_projectile_t* ap,
    projectile_t* proj,
    float wing_area,
    float lift_coeff,
    float drag_coeff,
    propulsion_t* propulsion,
    guidance_func guidance_fn,
    void* guidance_userdata
);

/*
 * @return `true`이면 계산 성공, `false`이면 목표에 도달할 수 없음
 공중유닛은 추진기로 발사한다 외부의 힘이 아니라
 미사일과 다른점은 추진을 아주 천천히 해야한다는 거다 미사일처럼 
 순간 가속하면 내부의 조종사와 승무원등의 사람이 죽는다.
 추진기 가 전문 추진기가 있어야겠다 제트엔진이라고 해야 하나
 */
BYUL_API bool aerial_projectile_launch(
    const aerial_projectile_t* aerial,
    const vec3_t* target,
    projectile_result_t* out
);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
