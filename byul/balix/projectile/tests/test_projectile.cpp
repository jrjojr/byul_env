#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
#include "internal/projectile.h"

#include "internal/common.h"
}

int main(int argc, char** argv) {
#ifdef _WIN32
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(65001);                          // UTF-8 출력용
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // UTF-8 로케일
#else
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // 리눅스/맥에서도 설정
#endif

    std::cout << u8"🌟 UTF-8 콘솔 코드페이지로 전환하고 테스트 시작!\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);
    int res = context.run();

    if (context.shouldExit()) {
        std::cout << u8"🌙 테스트 끝! 콘솔 코드페이지 원래대로 복구했습니다.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp);                // 원래 코드페이지 복원
        setlocale(LC_ALL, "");                          // 기본 로케일로 복귀
#endif
        return res;
    }

    std::cout << u8"🌙 테스트 종료. 콘솔 상태 복원 완료.\n";
#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, "");                              // 로케일 복원
#endif

    return res;
}

// ---------------------------------------------------------
// 테스트 콜백: on_hit 감지용
// ---------------------------------------------------------
static bool hit_called = false;
void test_hit_cb(const projectile_t* proj, void* userdata) {
    (void)proj; (void)userdata;
    hit_called = true;
}

// ---------------------------------------------------------
// Shell 테스트: 단순 중력만 적용
// ---------------------------------------------------------
TEST_CASE("Shell basic gravity") {
    shell_t shell{};
    shell.base.type = PROJECTILE_TYPE_SHELL;
    shell.base.projectile_id = 1;
    shell.base.velocity = {0, 0, 0};
    shell.base.acceleration = {0, -9.8f, 0};  // 중력
    shell.base.lifetime = 5.0f;
    shell.drag_coef = 0.0f;

    shell.env_fn = projectile_env_none;
    shell.base.on_hit = test_hit_cb;

    xform_t* x0 = xform_new_identity();        // ✅ 위치 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    shell.base.xf = *x0;
    xform_free(x0);

    hit_called = false;

    for (int i = 0; i < 100; ++i) {
        shell_update(&shell, 0.1f);
    }

    CHECK(shell.base.age >= shell.base.lifetime);
    CHECK(hit_called == true);

    vec3_t pos;
    xform_get_position(&shell.base.xf, &pos);
    CHECK(pos.y < 0.0f); // 중력으로 낙하했는지 확인
}

// ---------------------------------------------------------
// Missile 테스트: 타겟 유도 방향
// ---------------------------------------------------------
TEST_CASE("Missile guidance to target") {
    vec3_t target = {10, 0, 0};

    missile_t missile{};
    missile.base.type = PROJECTILE_TYPE_MISSILE;
    missile.base.projectile_id = 2;
    missile.base.velocity = {0, 0, 0};
    missile.base.acceleration = {0, 0, 0};
    missile.thrust = {5, 0, 0};
    missile.fuel = 10.0f;
    missile.guidance_fn = projectile_guidance_to_target;
    missile.guidance_userdata = &target;

    xform_t* x0 = xform_new_identity();        // ✅ 위치 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    missile.base.xf = *x0;
    xform_free(x0);

    for (int i = 0; i < 10; ++i)
        missile_update(&missile, 0.1f);

    vec3_t pos;
    xform_get_position(&missile.base.xf, &pos);

    CHECK(pos.x > 0.5f);  // 유도 방향으로 이동했는지 확인
    CHECK(missile.fuel < 10.0f);  // 연료가 소모되었는지
}

// ---------------------------------------------------------
// 회전 적용 테스트
// ---------------------------------------------------------
TEST_CASE("Projectile angular velocity applies rotation") {
    projectile_t proj{};
    proj.angular_velocity = {0, 1.0f, 0}; // Y축 자전

    xform_t* x0 = xform_new_identity();        // ✅ 회전 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    proj.xf = *x0;
    xform_free(x0);

    vec3_t forward = {0, 0, 1};
    vec3_t world_before, world_after;

    xform_apply_to_direction(&proj.xf, &forward, &world_before);

    // 180도 회전 (pi 라디안)
    projectile_apply_rotation(&proj, 3.14159f);

    xform_apply_to_direction(&proj.xf, &forward, &world_after);

    CHECK(world_before.z > 0.99f); // 원래 전방
    CHECK(world_after.z < -0.99f); // 회전 후 반대 방향
}

TEST_CASE("Projectile prediction with gravity only") {
    projectile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {5, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;

    projectile_result_t result{};
    bool ok = projectile_predict(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 1.0f);
    CHECK(result.impact_pos.y <= 0.0f);
    CHECK(result.impact_pos.x > 0.0f); // 전방으로 나아감
}

TEST_CASE("Missile prediction: basic vertical fall with thrust") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 10.0f, 0};  // 위로 향하는 thrust
    pred.fuel = 0.5f;  // 짧게만 작동
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;

    projectile_result_t result{};
    bool ok = projectile_predict_missile(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 0.5f); // 중력과 thrust에 의해 낙하 지연됨
}

static float expect_y(const char* method) {
    if (strcmp(method, "euler") == 0) return 0.0f;
    if (strcmp(method, "semi") == 0) return -9.8f;
    if (strcmp(method, "rk4") == 0) return -4.9f;
    return 0.0f;
}

// ---------------------------------------------------------
// Helper: trajectory 유효성 검사
// ---------------------------------------------------------
static void validate_trajectory(const projectile_result_t& result,
                                float start_height,
                                bool expect_impact) {
    CHECK(result.trajectory.count > 0);
    CHECK(result.trajectory.samples[0].state.linear.position.y == doctest::Approx(start_height));

    if (expect_impact) {
        CHECK(result.trajectory.samples[result.trajectory.count - 1]
              .state.linear.position.y <= doctest::Approx(0.0f));
    }
}

// ---------------------------------------------------------
// TEST: Gravity-only projectile prediction
// ---------------------------------------------------------
TEST_CASE("Projectile prediction with trajectory samples") {
    projectile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {5, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 2000);

    bool ok = projectile_predict(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 1.0f);
    CHECK(result.impact_pos.y <= 0.0f);
    CHECK(result.impact_pos.x > 0.0f);

    validate_trajectory(result, 10.0f, true);

    trajectory_free(&result.trajectory);
}

// ---------------------------------------------------------
// TEST: Missile prediction with limited thrust (impact expected)
// ---------------------------------------------------------
TEST_CASE("Missile prediction: impact with limited thrust") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 10.0f, 0};  // 중력을 약간 상쇄
    pred.fuel = 0.5f;             // 짧은 thrust
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;
    pred.integrator_type = INTEGRATOR_EULER;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 2000);

    bool ok = projectile_predict_missile(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 0.5f);
    CHECK(result.impact_pos.y <= 0.0f);

    validate_trajectory(result, 10.0f, true);

    trajectory_free(&result.trajectory);
}

// ---------------------------------------------------------
// Helper: trajectory의 마지막 Y값 가져오기
// ---------------------------------------------------------
static float last_y(const projectile_result_t& result) {
    if (result.trajectory.count == 0) return 0.0f;
    return result.trajectory.samples[result.trajectory.count - 1].state.linear.position.y;
}

// ---------------------------------------------------------
// TEST: Missile prediction with SEMI-IMPLICIT integrator
// ---------------------------------------------------------
TEST_CASE("Missile prediction with SEMI-IMPLICIT integrator") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 10.0f, 0};
    pred.fuel = 1.0f;
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 2.0f;
    pred.time_step = 0.01f;
    pred.integrator_type = INTEGRATOR_SEMI_IMPLICIT;  // Semi-Implicit

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 2000);

    bool ok = projectile_predict_missile(&pred, &result);
    CHECK(result.trajectory.count > 0);

    // 마지막 높이가 초기보다 낮아져야 함 (중력 영향)
    CHECK(last_y(result) <= 10.0f);

    // trajectory 유효성 검증
    validate_trajectory(result, 10.0f, ok);

    trajectory_free(&result.trajectory);
}

// ---------------------------------------------------------
// TEST: Missile prediction with RK4 integrator
// ---------------------------------------------------------
TEST_CASE("Missile prediction with RK4 integrator") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 10.0f, 0};
    pred.fuel = 1.0f;
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 2.0f;
    pred.time_step = 0.01f;
    pred.integrator_type = INTEGRATOR_RK4;  // RK4

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 2000);

    bool ok = projectile_predict_missile(&pred, &result);
    CHECK(result.trajectory.count > 0);

    // RK4는 Semi-Implicit보다 조금 더 정확하게 추정됨
    CHECK(last_y(result) <= 10.0f);

    // trajectory 유효성 검증
    validate_trajectory(result, 10.0f, ok);

    trajectory_free(&result.trajectory);
}

#include <iostream>
#include <iomanip>

static void print_trajectory(const projectile_result_t& result) {
    std::cout << "---- Trajectory Samples (" << result.trajectory.count << " points) ----\n";
    std::cout << "   t(s)    pos(x,y,z)          vel(x,y,z) \n";
    std::cout << "-------------------------------------------------------\n";

    for (int i = 0; i < result.trajectory.count; ++i) {
        const trajectory_sample_t& s = result.trajectory.samples[i];
        if (result.valid && s.t > result.impact_time) break; // 충돌 후 중단

        const vec3_t& pos = s.state.linear.position;
        const vec3_t& vel = s.state.linear.velocity;
        std::cout << std::fixed << std::setprecision(3)
                  << "  " << s.t << "  "
                  << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")  "
                  << "(" << vel.x << ", " << std::max(0.0f, vel.y) << ", " << vel.z << ")\n";
    }
    std::cout << "-------------------------------------------------------\n";
}


// ---------------------------------------------------------
// TEST: Print projectile trajectory
// ---------------------------------------------------------
TEST_CASE("Print projectile trajectory") {
    projectile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {5, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 3.0f;
    pred.time_step = 0.2f; // 샘플링 간격을 크게 해서 출력 줄 수를 줄임

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 100); // 100 샘플 예약

    bool ok = projectile_predict(&pred, &result);

    CHECK(ok == true);
    print_trajectory(result);

    trajectory_free(&result.trajectory);
}

// ---------------------------------------------------------
// TEST: Print missile trajectory
// ---------------------------------------------------------
TEST_CASE("Print missile trajectory") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 15.0f, 0};
    pred.fuel = 1.0f;
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 3.0f;
    pred.time_step = 0.2f;
    pred.integrator_type = INTEGRATOR_EULER;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 100);

    bool ok = projectile_predict_missile(&pred, &result);

    CHECK(result.trajectory.count > 0);
    print_trajectory(result);

    trajectory_free(&result.trajectory);
}

TEST_CASE("Print projectile trajectory with stop at impact") {
    projectile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {5, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 5.0f;
    pred.time_step = 0.2f;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 200);

    bool ok = projectile_predict(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);

    // 충돌 시점 이전 trajectory만 출력
    print_trajectory(result);

    trajectory_free(&result.trajectory);
}

// 테스트용 타겟 정보
static target_info_t make_target(float x, float y, float z, float vx, float vy, float vz) {
    target_info_t target;
    target.position = {x, y, z};
    target.velocity = {vx, vy, vz};
    return target;
}

TEST_CASE("Guidance Lead: Static Target") {
    projectile_t missile{};
    missile.velocity = {10, 0, 0}; // 미사일이 x 방향으로 10 m/s

    target_info_t target = make_target(100, 0, 0, 0, 0, 0);

    const vec3_t* dir = projectile_guidance_lead(&missile, 0.1f, &target);

    CHECK(dir != nullptr);
    CHECK(dir->x > 0.99f); // 거의 x 방향
    CHECK(fabs(dir->y) < 0.01f);
    CHECK(fabs(dir->z) < 0.01f);

    std::cout << "[Static Target] Guidance Dir = ("
              << dir->x << ", " << dir->y << ", " << dir->z << ")\n";
}

TEST_CASE("Guidance Lead: Moving Target") {
    projectile_t missile{};
    missile.velocity = {10, 0, 0}; // 미사일이 x 방향으로 10 m/s

    // 목표가 x=100, y=0 위치에서 x축 방향으로 5 m/s로 이동
    target_info_t target = make_target(100, 0, 0, 5, 0, 0);

    const vec3_t* dir = projectile_guidance_lead(&missile, 0.1f, &target);

    CHECK(dir != nullptr);
    CHECK(dir->x > 0.95f); // x 방향 성분이 큰 값이어야 함
    CHECK(fabs(dir->y) < 0.05f);
    CHECK(fabs(dir->z) < 0.01f);

    std::cout << "[Moving Target] Guidance Dir = ("
              << dir->x << ", " << dir->y << ", " << dir->z << ")\n";
}

TEST_CASE("Guidance Lead: Diagonal Target") {
    projectile_t missile{};
    missile.velocity = {20, 0, 0}; // 미사일이 x 방향으로 20 m/s

    // 목표가 x=100, y=50, z=0 위치에서 (5,2,0) m/s로 이동
    target_info_t target = make_target(100, 50, 0, 5, 2, 0);

    const vec3_t* dir = projectile_guidance_lead(&missile, 0.1f, &target);

    CHECK(dir != nullptr);
    CHECK(dir->x > 0.7f);  // x 성분이 가장 커야 함
    CHECK(dir->y > 0.3f);  // y 방향도 포함되어야 함

    std::cout << "[Diagonal Target] Guidance Dir = ("
              << dir->x << ", " << dir->y << ", " << dir->z << ")\n";
}

// ---------------------------------------------
// 비선형 타겟 trajectory 생성 (사인파 경로)
// ---------------------------------------------
static void build_target_sine_trajectory(trajectory_t* traj, float duration, float dt) {
    trajectory_init(traj, (int)(duration / dt) + 1);
    motion_state_t state = {0};

    float t = 0.0f;
    for (int i = 0; t <= duration; t += dt, i++) {
        // 타겟은 X축으로 직진하면서 Y축으로 사인 곡선을 그립니다.
        state.linear.position.x = 5.0f * t;            // x = 5 * t
        state.linear.position.y = 5.0f * std::sin(0.5f * t); // y = 5 * sin(0.5 * t)
        state.linear.position.z = 0.0f;
        state.linear.velocity = {5.0f, 2.5f * std::cos(0.5f * t), 0.0f}; // 속도 벡터
        state.linear.acceleration = {0.0f, 0.0f, 0.0f};

        trajectory_add_sample(traj, t, &state);
    }
}

// ---------------------------------------------
// 유도 함수: 타겟 trajectory 기반 테스트
// ---------------------------------------------
TEST_CASE("Missile guidance with nonlinear target trajectory") {
    // 1. 타겟 trajectory 생성
    trajectory_t target_traj;
    build_target_sine_trajectory(&target_traj, 10.0f, 0.5f);

    // 2. 미사일 초기화
    missile_predictor_t pred{};
    pred.start_pos = {0, 0, 0};
    pred.start_velocity = {10, 0, 0};  // 초기 속도 x방향 10 m/s
    pred.gravity = {0, 0, 0};          // 중력 없음 (단순 시뮬레이션)
    pred.thrust = {10.0f, 0, 0};       // 기본 추진력
    pred.fuel = 3.0f;
    pred.guidance_fn = projectile_guidance_from_trajectory; // trajectory 기반 유도
    target_traj_info_t target_info = target_traj_info_t{&target_traj, 0.0f};
    pred.guidance_userdata = &target_info;
    pred.env_fn = projectile_env_none;
    pred.ground_height = -100.0f;      // 지면 체크 불필요
    pred.max_time = 10.0f;
    pred.time_step = 0.2f;
    pred.integrator_type = INTEGRATOR_EULER;

    // 3. 미사일 trajectory 예측
    projectile_result_t result;
    trajectory_init(&result.trajectory, 1000);

    bool ok = projectile_predict_missile(&pred, &result);

    CHECK(result.trajectory.count > 0);

    CHECK_MESSAGE(result.trajectory.count > 0, "Missile trajectory must have samples");

    // 4. trajectory 출력 (디버깅)
    std::cout << "Missile trajectory (tracking sine target):" << std::endl;
    for (int i = 0; i < result.trajectory.count; i++) {
        const trajectory_sample_t& s = result.trajectory.samples[i];
        std::cout << "t=" << s.t
                  << " pos=(" << s.state.linear.position.x << ", "
                               << s.state.linear.position.y << ", "
                               << s.state.linear.position.z << ")\n";
    }

    trajectory_free(&result.trajectory);
    trajectory_free(&target_traj);
}

// 속도 계산 헬퍼 함수
static float sample_speed(const trajectory_sample_t& sample) {
    const vec3_t& v = sample.state.linear.velocity;
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

TEST_CASE("Missile control test with PID controller") {
    // PID 컨트롤러 생성 (목표 속력 제어)
    controller_t* pid_ctrl = controller_create_pid(
        2.0f, 0.5f, 0.1f, 0.1f, 25.0f  // kp, ki, kd, dt, output_limit
    );

    missile_predictor_t pred{};
    pred.start_pos = {0, 0, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, 0, 0};
    pred.thrust = {30.0f, 0, 0};
    pred.fuel = 5.0f;
    pred.controller = pid_ctrl;
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = -100.0f;
    pred.max_time = 5.0f;
    pred.time_step = 0.1f;
    pred.integrator_type = INTEGRATOR_EULER;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 1000);

    bool ok = projectile_predict_missile(&pred, &result);
    CHECK(result.trajectory.count > 0);

    std::cout << "[PID Controller] Missile speed samples:\n";
    for (int i = 0; i < result.trajectory.count; ++i) {
        float speed = sample_speed(result.trajectory.samples[i]);
        std::cout << "t=" << result.trajectory.samples[i].t << " speed=" << speed << "\n";
    }

    trajectory_free(&result.trajectory);
    controller_destroy(pid_ctrl);
}

TEST_CASE("Missile control test with Bang-Bang controller") {
    // Bang-Bang 컨트롤러 생성 (목표 속력 20m/s를 위해 +/- max_output 사용)
    controller_t* bb_ctrl = controller_create_bangbang(25.0f); // max_output=25

    missile_predictor_t pred{};
    pred.start_pos = {0, 0, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, 0, 0};
    pred.thrust = {25.0f, 0, 0};
    pred.fuel = 5.0f;
    pred.controller = bb_ctrl;
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = -100.0f;
    pred.max_time = 5.0f;
    pred.time_step = 0.1f;
    pred.integrator_type = INTEGRATOR_EULER;

    projectile_result_t result{};
    trajectory_init(&result.trajectory, 1000);

    bool ok = projectile_predict_missile(&pred, &result);
    CHECK(result.trajectory.count > 0);

    std::cout << "[Bang-Bang Controller] Missile speed samples:\n";
    for (int i = 0; i < result.trajectory.count; ++i) {
        float speed = sample_speed(result.trajectory.samples[i]);
        std::cout << "t=" << result.trajectory.samples[i].t << " speed=" << speed << "\n";
    }

    trajectory_free(&result.trajectory);
    controller_destroy(bb_ctrl);
}


// ---------------------------------------------------------
// 헬퍼: 미사일 predictor 동적 생성
// ---------------------------------------------------------
static missile_predictor_t* create_missile_predictor(controller_t* ctrl) {
    missile_predictor_t* pred = new missile_predictor_t();
    pred->start_pos       = {0.0f, 0.0f, 0.0f};
    pred->start_velocity  = {0.0f, 0.0f, 0.0f};
    pred->gravity         = {0.0f, -9.81f, 0.0f};
    pred->thrust          = {30.0f, 0.0f, 0.0f};
    pred->fuel            = 5.0f;
    pred->controller      = ctrl;
    pred->guidance_fn     = projectile_guidance_none;
    pred->env_fn          = projectile_env_none;
    pred->ground_height   = -100.0f;
    pred->max_time        = 5.0f;
    pred->time_step       = 0.1f;
    pred->integrator_type = INTEGRATOR_EULER;
    return pred;
}

// ---------------------------------------------------------
// 🛠 테스트 기본 설정 함수들 (동적 생성)
// ---------------------------------------------------------
static mpc_config_t* setup_mpc_config() {
    mpc_config_t* config = new mpc_config_t;
    config->horizon_sec       = 2.0f;   // 2초 예측
    config->step_dt           = 0.05f;  // 50ms 스텝
    config->max_accel         = 10.0f;  // 최대 선형 가속도
    config->max_speed         = 30.0f;  // 최대 속도
    config->weight_distance   = 1.0f;   // 거리 오차 가중치
    config->weight_accel      = 0.1f;   // 가속도 가중치
    config->max_iter          = 100;
    config->output_trajectory = true;
    config->candidate_step    = 5.0f;
    return config;
}

static environment_t* setup_environment() {
    environment_t* env = new environment_t;
    env->gravity = {0.0f, -9.81f, 0.0f};  // 기본 중력
    env->wind    = {0.0f, 0.0f, 0.0f};    // 바람 없음
    return env;
}

static body_properties_t* setup_body_properties() {
    body_properties_t* body = new body_properties_t;
    body->mass          = 50.0f;   // 50kg 미사일
    body->drag_coef     = 0.1f;    // 기본 드래그
    body->cross_section = 0.05f;   // 단면적
    return body;
}

TEST_CASE("Missile control test with MPC controller") {
    mpc_config_t* mpc_config = setup_mpc_config();
    environment_t* env       = setup_environment();
    body_properties_t* body  = setup_body_properties();

    controller_t* mpc_ctrl = controller_create_mpc(mpc_config, env, body);
    REQUIRE(mpc_ctrl != nullptr);

    missile_predictor_t* pred = create_missile_predictor(mpc_ctrl);

    projectile_result_t result{};
    REQUIRE(trajectory_init(&result.trajectory, 1000) == true);

    bool ok = projectile_predict_missile(pred, &result);
    CHECK_MESSAGE(ok, "[MPC] projectile_predict_missile() returned false");

    if (ok) {
        CHECK(result.trajectory.count > 0);
        std::cout << "[MPC Controller] Missile speed samples:\n";

        bool near_target = false;
        for (int i = 0; i < result.trajectory.count; ++i) {
            float speed = sample_speed(result.trajectory.samples[i]);
            std::cout << "t=" << result.trajectory.samples[i].t
                      << " speed=" << speed << "\n";
            if (speed > 15.0f && speed < 25.0f) {
                near_target = true;
            }
        }
        CHECK_MESSAGE(near_target, "MPC failed to keep speed within expected range");
    } else {
        std::cout << "[DEBUG] Missile MPC returned false. Check config or candidate generation.\n";
    }

    trajectory_free(&result.trajectory);
    controller_destroy(mpc_ctrl);

    // 메모리 해제
    delete pred;
    delete mpc_config;
    delete env;
    delete body;
}
