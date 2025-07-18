#include "internal/controller.h"
#include "internal/numeq_mpc.h"
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------
// 내부 구현체 구조체
// ---------------------------------------------------------

// PID 구현체
typedef struct s_pid_impl {
    pid_controller_t pid;
    float output_limit;
} pid_impl_t;

// Bang-Bang 구현체
typedef struct s_bangbang_impl {
    float max_output;
} bangbang_impl_t;

// MPC 구현체
typedef struct s_mpc_impl {
    mpc_config_t config;   // MPC 설정
    vec3_t target;         // 목표 속도/위치 (x방향 기준)
    environment_t env;     // 환경 정보 (필요 시 세팅)
    body_properties_t body;// 물리 속성 (질량 등)
} mpc_impl_t;

// Forward declarations
static void   controller_mpc_reset(controller_t* ctrl);
static float  controller_mpc_compute(controller_t* ctrl, const vec3_t* target);

// ---------------------------------------------------------
// PID 컨트롤러 함수
// ---------------------------------------------------------
static float controller_pid_compute(
    controller_t* ctrl, float target, float measured, float dt) {

    pid_impl_t* impl = (pid_impl_t*)ctrl->impl;
    if (!impl) return 0.0f;

    // PID dt 업데이트
    impl->pid.dt = dt;
    float output = pid_update(&impl->pid, target, measured);

    // 출력 제한
    if (impl->output_limit > 0.0f) {
        if (output > impl->output_limit) output = impl->output_limit;
        if (output < -impl->output_limit) output = -impl->output_limit;
    }
    return output;
}

static void controller_pid_reset(controller_t* ctrl) {
    pid_impl_t* impl = (pid_impl_t*)ctrl->impl;
    if (impl) pid_reset(&impl->pid);
}

// ---------------------------------------------------------
// Bang-Bang 컨트롤러 함수
// ---------------------------------------------------------
static float controller_bangbang_compute(
    controller_t* ctrl, float target, float measured, float dt) {

    (void)dt;
    bangbang_impl_t* impl = (bangbang_impl_t*)ctrl->impl;
    if (!impl) return 0.0f;

    // Bang-Bang 로직: 목표보다 크면 -max, 작으면 +max
    return (measured < target) ? impl->max_output : -impl->max_output;
}

static void controller_bangbang_reset(controller_t* ctrl) {
    (void)ctrl;
    // Bang-Bang은 내부 상태 없음
}

// ---------------------------------------------------------
// MPC 컨트롤러 함수
// ---------------------------------------------------------
static float controller_mpc_compute(
    controller_t* ctrl, float target, float measured, float dt) 
{
    mpc_impl_t* impl = (mpc_impl_t*)ctrl->impl;
    if (!impl) return 0.0f;

    // -----------------------------------------------------
    // 1. 현재 상태 설정 (x축 속도만 측정값으로 사용)
    // -----------------------------------------------------
    motion_state_t current_state = motion_state_t{};
    current_state.linear.position = {0.0f, 0.0f, 0.0f};
    current_state.linear.velocity = {measured, 0.0f, 0.0f};
    current_state.linear.acceleration = {0.0f, 0.0f, 0.0f};
    quat_t oq;
    quat_identity(&oq);
    current_state.angular.orientation = oq;
    current_state.angular.angular_velocity = {0.0f, 0.0f, 0.0f};
    current_state.angular.angular_acceleration = {0.0f, 0.0f, 0.0f};

    // -----------------------------------------------------
    // 2. 목표 상태 설정 (target 위치를 x축에 설정)
    // -----------------------------------------------------
    motion_state_t target_state = motion_state_t{};
    target_state.linear.position = {target, 0.0f, 0.0f};
    target_state.linear.velocity = {0.0f, 0.0f, 0.0f};
    target_state.linear.acceleration = {0.0f, 0.0f, 0.0f};

    quat_identity(&oq);    
    target_state.angular.orientation = oq;
    target_state.angular.angular_velocity = {0.0f, 0.0f, 0.0f};
    target_state.angular.angular_acceleration = {0.0f, 0.0f, 0.0f};

    // -----------------------------------------------------
    // 3. MPC 계산 실행
    // -----------------------------------------------------
    mpc_output_t out = {0};
    bool ok = numeq_mpc_solve(
        &current_state,
        &target_state,
        &impl->env,
        &impl->body,
        &impl->config,
        &out,
        NULL,  // trajectory는 디버깅 시만
        numeq_mpc_cost_default,
        &impl->config  // userdata로 config 전달
    );

    // -----------------------------------------------------
    // 4. 결과 처리
    // -----------------------------------------------------
    if (ok) {
        return out.desired_accel.x;  // x방향 가속도만 출력
    }
    return 0.0f;
}


static void controller_mpc_reset(controller_t* ctrl) {
    mpc_impl_t* impl = (mpc_impl_t*)ctrl->impl;
    impl->target = (vec3_t){0.0f, 0.0f, 0.0f};
}

// ---------------------------------------------------------
// 컨트롤러 생성 및 해제
// ---------------------------------------------------------

controller_t* controller_create_pid(
    float kp, float ki, float kd, float dt, float output_limit) {

    controller_t* ctrl = (controller_t*)malloc(sizeof(controller_t));
    if (!ctrl) return NULL;
    memset(ctrl, 0, sizeof(controller_t));

    pid_impl_t* impl = (pid_impl_t*)malloc(sizeof(pid_impl_t));
    if (!impl) { free(ctrl); return NULL; }
    memset(impl, 0, sizeof(pid_impl_t));

    pid_init(&impl->pid, kp, ki, kd, dt);
    impl->output_limit = output_limit;

    ctrl->type = CONTROLLER_PID;
    ctrl->impl = impl;
    ctrl->compute = controller_pid_compute;
    ctrl->reset = controller_pid_reset;
    ctrl->userdata = NULL;

    return ctrl;
}

controller_t* controller_create_bangbang(float max_output) {
    controller_t* ctrl = (controller_t*)malloc(sizeof(controller_t));
    if (!ctrl) return NULL;
    memset(ctrl, 0, sizeof(controller_t));

    bangbang_impl_t* impl = (bangbang_impl_t*)malloc(sizeof(bangbang_impl_t));
    if (!impl) { free(ctrl); return NULL; }
    impl->max_output = max_output;

    ctrl->type = CONTROLLER_BANGBANG;
    ctrl->impl = impl;
    ctrl->compute = controller_bangbang_compute;
    ctrl->reset = controller_bangbang_reset;
    ctrl->userdata = NULL;

    return ctrl;
}

controller_t* controller_create_mpc(
    const mpc_config_t*      config,
    const environment_t*     env,
    const body_properties_t* body) {

    // Allocate controller and impl
    controller_t* ctrl = (controller_t*)malloc(sizeof(controller_t));
    if (!ctrl) return NULL;
    mpc_impl_t* impl = (mpc_impl_t*)malloc(sizeof(mpc_impl_t));
    if (!impl) {
        free(ctrl);
        return NULL;
    }
    // Initialize implementation data
    memcpy(&impl->config, config, sizeof(mpc_config_t));
    memcpy(&impl->env, env,     sizeof(environment_t));
    memcpy(&impl->body, body,   sizeof(body_properties_t));
    impl->target = (vec3_t){0.0f, 0.0f, 0.0f};

    // Assign controller interface
    ctrl->type    = CONTROLLER_MPC;
    ctrl->impl    = impl;
    ctrl->compute = controller_mpc_compute;
    ctrl->reset   = controller_mpc_reset;
    return ctrl;
}

void controller_destroy(controller_t* ctrl) {
    if (!ctrl) return;
    if (ctrl->impl) {
        free(ctrl->impl);
        ctrl->impl = NULL;
    }
    free(ctrl);
}
