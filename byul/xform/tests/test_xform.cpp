#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
}

#include "internal/vec3.hpp"
#include "internal/common.h"


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

TEST_CASE("xform: identity transform") {
    xform_t* xf = xform_new_identity();
    vec3_t pos;
    xform_get_position(xf, &pos);
    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
    xform_free(xf);
}

TEST_CASE("xform: axis-angle roundtrip") {
    vec3_t pos = {1.0f, 2.0f, 3.0f};
    vec3_t axis = {0.0f, 1.0f, 0.0f};
    float radians = 3.1415926f / 2.0f;

    xform_t* xf = xform_new_from_axis_angle(&pos, &axis, radians);

    vec3_t got_pos;
    vec3_t got_axis;
    float got_radians;

    xform_get_position(xf, &got_pos);
    xform_get_axis_angle(xf, &got_axis, &got_radians);

    CHECK(got_pos.x == doctest::Approx(pos.x));
    CHECK(got_pos.y == doctest::Approx(pos.y));
    CHECK(got_pos.z == doctest::Approx(pos.z));
    CHECK(got_radians == doctest::Approx(radians));
    CHECK(got_axis.y == doctest::Approx(1.0f));

    xform_free(xf);
}

TEST_CASE("xform: translate and apply") {
    xform_t* xf = xform_new_identity();

    vec3_t delta = {5.0f, 0.0f, 0.0f};
    xform_translate(xf, &delta);

    vec3_t local = {1.0f, 0.0f, 0.0f};
    vec3_t world;

    xform_apply_to_point(xf, &local, &world);

    CHECK(world.x == doctest::Approx(6.0f));
    CHECK(world.y == doctest::Approx(0.0f));
    CHECK(world.z == doctest::Approx(0.0f));

    xform_free(xf);
}

TEST_CASE("xform_new_identity works correctly") {
    // TODO: implement test for xform_new_identity
    xform_t* xf = xform_new_identity(); 
    CHECK(xf != nullptr);
    xform_free(xf);
}

TEST_CASE("xform_new_from_axis_angle works correctly") {
    // TODO: implement test for xform_new_from_axis_angle
    vec3_t pos = {1, 2, 3};
    vec3_t axis = {0, 1, 0};
    xform_t* xf = xform_new_from_axis_angle(&pos, &axis, 3.14f);
    CHECK(xf != nullptr);
    xform_free(xf);
}

TEST_CASE("xform_equal works correctly") {
    xform_t* a = xform_new_identity();
    xform_t* b = xform_new_identity();
    CHECK(xform_equal(a, b));
    xform_free(a);
    xform_free(b);
}
TEST_CASE("xform: identity and clone") {
    xform_t* xf = xform_new_identity();
    REQUIRE(xf != nullptr);

    xform_t* copy = xform_clone(xf);
    CHECK(copy != nullptr);
    CHECK(xform_equal(xf, copy));

    xform_free(xf);
    xform_free(copy);
}

TEST_CASE("xform: from axis-angle and roundtrip") {
    vec3_t pos = {1, 2, 3};
    vec3_t axis = {0, 1, 0};
    float radians = 3.14159f;

    xform_t* xf = xform_new_from_axis_angle(&pos, &axis, radians);
    REQUIRE(xf != nullptr);

    vec3_t got_pos, got_axis;
    float got_radians;
    xform_get_position(xf, &got_pos);
    xform_get_axis_angle(xf, &got_axis, &got_radians);

    CHECK(got_pos.x == doctest::Approx(pos.x));
    CHECK(got_pos.y == doctest::Approx(pos.y));
    CHECK(got_pos.z == doctest::Approx(pos.z));

    CHECK(got_radians == doctest::Approx(radians));
    CHECK(got_axis.y == doctest::Approx(1.0f));

    xform_free(xf);
}

TEST_CASE("xform: from euler and roundtrip") {
    vec3_t pos = {0, 0, 0};
    float yaw = 1.0f, pitch = 0.5f, roll = 0.25f;

    xform_t* xf = xform_new_from_euler(&pos, yaw, pitch, roll, EULER_ORDER_ZYX);
    REQUIRE(xf != nullptr);

    float got_yaw, got_pitch, got_roll;
    xform_get_euler(xf, &got_yaw, &got_pitch, &got_roll, EULER_ORDER_ZYX);

    CHECK(got_yaw == doctest::Approx(yaw));
    CHECK(got_pitch == doctest::Approx(pitch));
    CHECK(got_roll == doctest::Approx(roll));

    xform_free(xf);
}

TEST_CASE("xform: position setter and getter") {
    xform_t* xf = xform_new_identity();
    vec3_t p = {5, 10, 15};
    xform_set_position(xf, &p);

    vec3_t got;
    xform_get_position(xf, &got);
    CHECK(got.x == doctest::Approx(5.0f));
    CHECK(got.y == doctest::Approx(10.0f));
    CHECK(got.z == doctest::Approx(15.0f));
    xform_free(xf);
}

TEST_CASE("xform: translation") {
    xform_t* xf = xform_new_identity();
    vec3_t delta = {0, 0, 5};
    xform_set_euler(xf, 0, 3.14159f / 2.0f, 0, EULER_ORDER_ZYX); // 90도 yaw 회전

    xform_translate(xf, &delta);

    vec3_t pos;
    xform_get_position(xf, &pos);
    CHECK(pos.x == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.z == doctest::Approx(5.0f).epsilon(0.01));

    xform_free(xf);
}

TEST_CASE("xform: translate local") {
    xform_t* xf = xform_new_identity();
    vec3_t delta = {0, 0, 5};
    xform_set_euler(xf, 0, 3.14159f / 2.0f, 0, EULER_ORDER_ZYX); // 90도 yaw 회전

    xform_translate_local(xf, &delta);

    vec3_t pos;
    xform_get_position(xf, &pos);
    CHECK(pos.x == doctest::Approx(5.0f).epsilon(0.01));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.01));

    xform_free(xf);
}

TEST_CASE("xform: apply to point and direction") {
    xform_t* xf = xform_new_identity();
    vec3_t move = {5, 0, 0};
    xform_translate(xf, &move);

    vec3_t local = {1, 0, 0};
    vec3_t world = {0};
    xform_apply_to_point(xf, &local, &world);
    CHECK(world.x == doctest::Approx(6.0f));

    vec3_t dir;
    xform_apply_to_direction(xf, &local, &dir);
    CHECK(dir.x == doctest::Approx(1.0f));

    xform_free(xf);
}

TEST_CASE("xform: local translate with yaw 90deg") {
    xform_t* xf = xform_new_identity();
    REQUIRE(xf != nullptr);

    // Yaw 90도 회전 (Z축 기준 회전)
    float yaw_deg = 90.0f;
    float yaw_rad = 3.1415926f / 2.0f;
    xform_set_euler(xf, 0.0f, yaw_rad, 0.0f, EULER_ORDER_ZYX);

    // 로컬 Z축 +5 이동 → 월드 -X 방향 이동 기대
    vec3_t delta_local = {0.0f, 0.0f, 5.0f};
    xform_translate_local(xf, &delta_local);

    // 위치 확인
    vec3_t pos;
    xform_get_position(xf, &pos);

    // 회전값 확인용
    float yaw, pitch, roll;
    xform_get_euler(xf, &yaw, &pitch, &roll, EULER_ORDER_ZYX);

    // std::cout << "yaw (rad): " << yaw << ", pitch: " << pitch << ", roll: " << roll << std::endl;
    // std::cout << "translated position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;

    CHECK(pos.x == doctest::Approx(5.0f).epsilon(0.01f));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.01f));

    xform_free(xf);
}
TEST_CASE("quat rotation: yaw 90deg rotates +Z to -X") {
    quat_t q;
    quat_from_euler(&q, M_PI / 2.0f, 0.0f, 0.0f, EULER_ORDER_ZYX);

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_apply_to_vec3(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(-1.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("quat rotation: yaw 90deg rotates +Z to -X v1") {
    quat_t q;
    quat_from_euler(&q, 0.0f, M_PI / 2.0f, 0.0f, EULER_ORDER_ZYX); // yaw=90도

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_apply_to_vec3(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(1.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("quat rotation: yaw -90deg rotates +Z to -X v2") {
    quat_t q;
    quat_from_euler(&q, 0.0f, -M_PI / 2.0f, 0.0f, EULER_ORDER_ZYX); // yaw = -90도

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_apply_to_vec3(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(-1.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}


TEST_CASE("xform: translate after euler rotation affects world, not local") {
    xform_t* xf = xform_new_identity();

    // 1. 오일러 회전: Yaw 90도 (Z축 회전)
    xform_set_euler(xf, M_PI_2, 0, 0, EULER_ORDER_ZYX);  // Yaw = 90도

    // 2. 이동 벡터 (월드 기준으로 이동)
    vec3_t delta = {0, 0, 5}; // Z+ 방향

    // 3. 월드 기준 이동 → 회전과 상관없이 Z축으로 이동해야 함
    xform_translate(xf, &delta);

    // 4. 위치 확인
    vec3_t pos;
    xform_get_position(xf, &pos);

    // ✅ Z축으로 이동했는지 확인 (월드 기준 이동이기 때문에 Z만 변해야 함)
    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(5.0f));  // ← 회전과 무관하게 Z+ 방향

    xform_free(xf);
}
TEST_CASE("xform: set_euler → get_euler roundtrip (ZYX)") {
    xform_t* xf = xform_new_identity();

    float yaw_in = M_PI_2;     // 90도
    float pitch_in = M_PI / 4; // 45도
    float roll_in = M_PI / 6;  // 30도

    xform_set_euler(xf, yaw_in, pitch_in, roll_in, EULER_ORDER_ZYX);

    float yaw_out, pitch_out, roll_out;
    xform_get_euler(xf, &yaw_out, &pitch_out, &roll_out, EULER_ORDER_ZYX);

    // ⚠️ 오차는 쿼터니언 변환 과정에서 약간 있을 수 있음 (0.01 라디안 허용)
    CHECK(yaw_out == doctest::Approx(yaw_in).epsilon(0.01));
    CHECK(pitch_out == doctest::Approx(pitch_in).epsilon(0.01));
    CHECK(roll_out == doctest::Approx(roll_in).epsilon(0.01));

    xform_free(xf);
}
TEST_CASE("xform_lerp should interpolate position and rotation linearly") {
    vec3_t pos_a = {0, 0, 0};
    vec3_t axis_a = {0, 1, 0}; // Y축
    float rad_a = 0.0f;

    vec3_t pos_b = {10, 0, 0};
    vec3_t axis_b = {0, 1, 0}; // Y축
    float rad_b = (float)M_PI;

    xform_t* a = xform_new_from_axis_angle(&pos_a, &axis_a, rad_a);
    xform_t* b = xform_new_from_axis_angle(&pos_b, &axis_b, rad_b);

    xform_t* mid = xform_new_identity();
    xform_lerp(mid, a, b, 0.5f);

    vec3_t mid_pos;
    xform_get_position(mid, &mid_pos);
    // CHECK(vec3_equal(&mid_pos, &(vec3_t){5, 0, 0}));
    vec3_t v3 =  vec3_t{5,0,0};
    CHECK(vec3_equal(&mid_pos, &v3));

    vec3_t axis;
    float rad;
    xform_get_axis_angle(mid, &axis, &rad);
    CHECK(float_equal(rad, (float)M_PI / 2));

    xform_free(a);
    xform_free(b);
    xform_free(mid);
}

TEST_CASE("xform_slerp should interpolate position linearly and rotation via slerp") {
    vec3_t pos_a = {0, 0, 0};
    vec3_t axis_a = {0, 1, 0}; // Y축
    float rad_a = 0.0f;

    vec3_t pos_b = {10, 0, 0};
    vec3_t axis_b = {0, 1, 0};
    float rad_b = (float)M_PI;

    xform_t* a = xform_new_from_axis_angle(&pos_a, &axis_a, rad_a);
    xform_t* b = xform_new_from_axis_angle(&pos_b, &axis_b, rad_b);

    xform_t* mid = xform_new_identity();
    xform_slerp(mid, a, b, 0.5f);

    vec3_t mid_pos;
    xform_get_position(mid, &mid_pos);
    vec3_t v3= vec3_t{5,0,0};
    CHECK(vec3_equal(&mid_pos, &v3));

    vec3_t axis;
    float rad;
    xform_get_axis_angle(mid, &axis, &rad);
    CHECK(float_equal(rad, (float)M_PI / 2));

    xform_free(a);
    xform_free(b);
    xform_free(mid);
}