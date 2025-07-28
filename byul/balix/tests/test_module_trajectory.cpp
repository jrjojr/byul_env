#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "trajectory.h"
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

TEST_CASE("trajectory_create allocates memory and initializes") {
    trajectory_t* traj = trajectory_create_full(5);
    CHECK(traj != nullptr);
    CHECK(traj->samples != nullptr);
    CHECK(traj->capacity == 5);
    CHECK(traj->count == 0);
    trajectory_destroy(traj);    
}

TEST_CASE("trajectory_add_sample stores motion states") {
    trajectory_t* traj = trajectory_create_full(3);
    REQUIRE(traj != nullptr);

    motion_state_t s1 = make_motion_state(1.0f, 2.0f, 3.0f);
    motion_state_t s2 = make_motion_state(4.0f, 5.0f, 6.0f);

    CHECK(trajectory_add_sample(traj, 0.1f, &s1) == true);
    CHECK(traj->count == 1);
    CHECK(traj->samples[0].t == doctest::Approx(0.1f));
    CHECK(traj->samples[0].state.linear.position.x == doctest::Approx(1.0f));

    CHECK(trajectory_add_sample(traj, 0.2f, &s2) == true);
    CHECK(traj->count == 2);
    CHECK(traj->samples[1].state.linear.position.y == doctest::Approx(5.0f));

    // capacity 초과
    motion_state_t s3 = make_motion_state(7.0f, 8.0f, 9.0f);
    CHECK(trajectory_add_sample(traj, 0.3f, &s3) == true);
    motion_state_t s4 = make_motion_state(10.0f, 11.0f, 12.0f);
    CHECK(trajectory_add_sample(traj, 0.4f, &s4) == false);

    trajectory_destroy(traj);
}

TEST_CASE("trajectory_clear resets the sample count") {
    trajectory_t* traj = trajectory_create_full(3);
    REQUIRE(traj != nullptr);

    motion_state_t s = make_motion_state(1.0f, 1.0f, 1.0f);
    CHECK(trajectory_add_sample(traj, 0.1f, &s) == true);
    CHECK(traj->count == 1);

    trajectory_clear(traj);
    CHECK(traj->count == 0);
    trajectory_destroy(traj);
}
