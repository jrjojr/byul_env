#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

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

    // --- 기본 옵션 강제 설정 ---
    // context.setOption("success", true);      // 성공한 테스트도 모두 출력
    // context.setOption("no-skip", true);      // 스킵된 테스트도 출력
    // context.setOption("reporters", "console");
    // context.setOption("no-colors", true);
    // context.setOption("durations", true);  // 각 테스트 실행 시간 출력 (원하면 활성화)

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
