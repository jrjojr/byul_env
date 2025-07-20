#include "internal/controller.h"
#include "internal/numeq_mpc.h"
#include <stdlib.h>
#include <string.h>

// Forward declarations
static void   controller_mpc_reset(controller_t* ctrl);
static float  controller_mpc_compute(controller_t* ctrl, const vec3_t* target);

// ---------------------------------------------------------
// 내부 구현체 구조체
// ---------------------------------------------------------

// ---------------------- PID ----------------------
void pid_impl_init(pid_impl_t* impl) {
    if (!impl) return;
    pid_init(&impl->pid);        // PID 기본 초기화 (Kp=1, Ki=0, Kd=0)
    impl->output_limit = 0.0f;
}

void pid_impl_init_full(pid_impl_t* impl,
                                 float kp, float ki, float kd, float dt,
                                 float output_limit) {
    if (!impl) return;
    pid_init_full(&impl->pid, kp, ki, kd, dt);
    impl->output_limit = output_limit;
}

void pid_impl_assign(pid_impl_t* dst, const pid_impl_t* src) {
    if (!dst || !src) return;
    dst->pid = src->pid;
    dst->output_limit = src->output_limit;
}

// ---------------------- Bang-Bang ----------------------
void bangbang_impl_init(bangbang_impl_t* impl) {
    if (!impl) return;
    impl->max_output = 1.0f;
}

void bangbang_impl_init_full(bangbang_impl_t* impl,
                                      float max_output) {
    if (!impl) return;
    impl->max_output = max_output;
}

void bangbang_impl_assign(bangbang_impl_t* dst,
                                 const bangbang_impl_t* src) {
    if (!dst || !src) return;
    dst->max_output = src->max_output;
}

// ---------------------- MPC ----------------------
void mpc_impl_init(mpc_impl_t* impl) {
    if (!impl) return;
    mpc_config_init(&impl->config);
    motion_state_init(&impl->target);
    environment_init(&impl->env);
    body_properties_init(&impl->body);
    impl->cost_fn = numeq_mpc_cost_default;
}

void mpc_impl_init_full(mpc_impl_t* impl,
                                 const mpc_config_t* cfg,
                                 const motion_state_t* target,
                                 const environment_t* env,
                                 const body_properties_t* body,
                                 mpc_cost_func cost_fn) {
    if (!impl) return;
    if (cfg) mpc_config_assign(&impl->config, cfg);
    else mpc_config_init(&impl->config);

    if (target) motion_state_assign(&impl->target, target);
    else motion_state_init(&impl->target);

    if (env) environment_assign(&impl->env, env);
    else environment_init(&impl->env);

    if (body) body_properties_assign(&impl->body, body);
    else body_properties_init(&impl->body);

    // 비용 함수 설정
    impl->cost_fn = (cost_fn) ? cost_fn : numeq_mpc_cost_default;
}

void mpc_impl_assign(mpc_impl_t* dst, const mpc_impl_t* src) {
    if (!dst || !src) return;
    dst->config = src->config;
    dst->target = src->target;
    dst->env = src->env;
    dst->body = src->body;
    dst->cost_fn = src->cost_fn;
}


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
    motion_state_t current_state;
    motion_state_init(&current_state);

    // -----------------------------------------------------
    // 2. 목표 상태 설정 (target 위치를 x축에 설정)
    // -----------------------------------------------------
    motion_state_t target_state;
    motion_state_init(&target_state);
    target_state.linear.position = {target, 0.0f, 0.0f};

    // -----------------------------------------------------
    // 3. MPC 계산 실행
    // -----------------------------------------------------
    mpc_output_t out;
    bool ok = numeq_mpc_solve(
        &current_state,
        &target_state,
        &impl->env,
        &impl->body,
        &impl->config,
        &out,
        NULL,  // trajectory는 디버깅 시만
        impl->cost_fn,
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
    if (!ctrl || !ctrl->impl) return;

    mpc_impl_t* impl = (mpc_impl_t*)ctrl->impl;

    // 선형 상태 초기화
    impl->target.linear.position = (vec3_t){0.0f, 0.0f, 0.0f};
    impl->target.linear.velocity = (vec3_t){0.0f, 0.0f, 0.0f};
    impl->target.linear.acceleration = (vec3_t){0.0f, 0.0f, 0.0f};

    // 각 상태 초기화
    quat_t oq;
    quat_identity(&oq);
    impl->target.angular.orientation = oq; // 단위 쿼터니언
    impl->target.angular.angular_velocity = (vec3_t){0.0f, 0.0f, 0.0f};
    impl->target.angular.angular_acceleration = (vec3_t){0.0f, 0.0f, 0.0f};
}


// ---------------------------------------------------------
// 컨트롤러 생성 및 해제
// ---------------------------------------------------------

controller_t* controller_create_pid(
    float kp, float ki, float kd, float dt, float output_limit) {

    controller_t* ctrl = new controller_t;

    // pid_impl_t* impl = new pid_impl_t;

    pid_impl_t* impl = (pid_impl_t*)malloc(sizeof(pid_impl_t));
    if (!impl) {
        // 메모리 할당 실패 처리
        return NULL;
    }    

    pid_impl_init(impl);
    impl->output_limit = output_limit;

    ctrl->type = CONTROLLER_PID;
    ctrl->impl = impl;
    ctrl->compute = controller_pid_compute;
    ctrl->reset = controller_pid_reset;
    ctrl->userdata = NULL;

    return ctrl;
}

controller_t* controller_create_bangbang(float max_output) {
    controller_t* ctrl = new controller_t;

    // bangbang_impl_t* impl = new bangbang_impl_t;
    bangbang_impl_t* impl = (bangbang_impl_t*)malloc(sizeof(bangbang_impl_t));

    bangbang_impl_init_full(impl, max_output);

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
    const body_properties_t* body) 
{
    // Allocate controller and impl
    controller_t* ctrl = new controller_t;

    motion_state_t target;
    motion_state_init(&target);
    // mpc_impl_t* impl = new mpc_impl_t;
    mpc_impl_t* impl = (mpc_impl_t*)malloc(sizeof(mpc_impl_t));

    mpc_impl_init_full(
        impl, config, &target, env, body, numeq_mpc_cost_default);

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
        ctrl->impl = nullptr;
    }
    delete ctrl;
}
