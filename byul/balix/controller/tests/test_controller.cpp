#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
#include "internal/projectile.h"
#include "internal/controller.h"

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

    context.setOption("success", true);      // 성공한 테스트도 출력
    context.setOption("durations", true);    // 각 테스트 케이스 시간 출력    

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

TEST_CASE("PID controller basic response") {
    controller_t* pid_ctrl = controller_create_pid(1.0f, 0.1f, 0.01f, 0.01f, 10.0f);
    REQUIRE(pid_ctrl != nullptr);

    // 목표값과 측정값 차이가 10이면, 양수 출력이 나와야 함
    float output = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output > 0.0f);

    // 리셋 후 상태가 초기화되는지 확인
    controller_reset(pid_ctrl);
    float output_after_reset = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output_after_reset > 0.0f);

    controller_destroy(pid_ctrl);
}

TEST_CASE("Bang-Bang controller switching") {
    controller_t* bang_ctrl = controller_create_bangbang(5.0f);
    REQUIRE(bang_ctrl != nullptr);

    // 측정값이 목표보다 작으면 +max_output
    float output1 = controller_compute(bang_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(output1 == doctest::Approx(5.0f));

    // 측정값이 목표보다 크면 -max_output
    float output2 = controller_compute(bang_ctrl, 10.0f, 15.0f, 0.01f);
    CHECK(output2 == doctest::Approx(-5.0f));

    controller_destroy(bang_ctrl);
}

TEST_CASE("MPC controller basic output") {
    mpc_config_t config;
    mpc_config_init(&config);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    controller_t* mpc_ctrl = controller_create_mpc(&config, &env, &body);
    REQUIRE(mpc_ctrl != nullptr);

    // 기본적으로 MPC는 유효한 float 값을 반환해야 함
    float output = controller_compute(mpc_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(std::isfinite(output));

    controller_reset(mpc_ctrl);
    controller_destroy(mpc_ctrl);
}
