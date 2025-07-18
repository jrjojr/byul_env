#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/trajectory.h"
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
// 헬퍼 함수: motion_state_t 기본값 생성
// ---------------------------------------------------------
static motion_state_t make_motion_state(float px, float py, float pz) {
    motion_state_t state{};
    state.linear.position = {px, py, pz};
    state.linear.velocity = {0.0f, 0.0f, 0.0f};
    state.linear.acceleration = {0.0f, 0.0f, 0.0f};
    state.angular.orientation = {0.0f, 0.0f, 0.0f, 1.0f};  // 단위 쿼터니언
    state.angular.angular_velocity = {0.0f, 0.0f, 0.0f};
    state.angular.angular_acceleration = {0.0f, 0.0f, 0.0f};
    return state;
}

// ---------------------------------------------------------
// 테스트 케이스
// ---------------------------------------------------------

TEST_CASE("trajectory_init allocates memory and initializes") {
    trajectory_t traj{};
    CHECK(trajectory_init(&traj, 5) == true);
    CHECK(traj.samples != nullptr);
    CHECK(traj.capacity == 5);
    CHECK(traj.count == 0);
    trajectory_free(&traj);
    CHECK(traj.samples == nullptr);
}

TEST_CASE("trajectory_add_sample stores motion states") {
    trajectory_t traj{};
    REQUIRE(trajectory_init(&traj, 3) == true);

    motion_state_t s1 = make_motion_state(1.0f, 2.0f, 3.0f);
    motion_state_t s2 = make_motion_state(4.0f, 5.0f, 6.0f);

    CHECK(trajectory_add_sample(&traj, 0.1f, &s1) == true);
    CHECK(traj.count == 1);
    CHECK(traj.samples[0].t == doctest::Approx(0.1f));
    CHECK(traj.samples[0].state.linear.position.x == doctest::Approx(1.0f));

    CHECK(trajectory_add_sample(&traj, 0.2f, &s2) == true);
    CHECK(traj.count == 2);
    CHECK(traj.samples[1].state.linear.position.y == doctest::Approx(5.0f));

    // capacity 초과
    motion_state_t s3 = make_motion_state(7.0f, 8.0f, 9.0f);
    CHECK(trajectory_add_sample(&traj, 0.3f, &s3) == true);
    motion_state_t s4 = make_motion_state(10.0f, 11.0f, 12.0f);
    CHECK(trajectory_add_sample(&traj, 0.4f, &s4) == false);

    trajectory_free(&traj);
}

TEST_CASE("trajectory_clear resets the sample count") {
    trajectory_t traj{};
    REQUIRE(trajectory_init(&traj, 3) == true);

    motion_state_t s = make_motion_state(1.0f, 1.0f, 1.0f);
    CHECK(trajectory_add_sample(&traj, 0.1f, &s) == true);
    CHECK(traj.count == 1);

    trajectory_clear(&traj);
    CHECK(traj.count == 0);
    trajectory_free(&traj);
}
