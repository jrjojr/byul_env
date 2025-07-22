#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char** argv) {
    // --- 전역 버퍼링 해제 (printf, cout 모두 실시간 출력) ---
    setvbuf(stdout, NULL, _IONBF, 0);
    std::cout.setf(std::ios::unitbuf);  // std::cout 자동 flush

#ifdef _WIN32
    // --- Windows 콘솔을 UTF-8로 전환 ---
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(CP_UTF8);
    setlocale(LC_ALL, ""); // Windows의 현재 로케일 설정

    std::wcout.imbue(std::locale("")); // wide string 출력도 로케일 적용
#else
    // --- 리눅스/맥 UTF-8 로케일 설정 ---
    setlocale(LC_ALL, "ko_KR.UTF-8");
#endif

    std::cout << u8"🌟 UTF-8 콘솔 코드페이지로 전환하고 테스트 시작!\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);

    // 필요한 경우 옵션 활성화
    // context.setOption("success", true);    // 성공한 테스트도 모두 출력
    // context.setOption("durations", true);  // 테스트 케이스 시간 출력

    int res = context.run();

    if (context.shouldExit()) {
        std::cout << u8"🌙 테스트 끝! 콘솔 코드페이지 원래대로 복구했습니다.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp); // 원래 코드페이지로 복구
        setlocale(LC_ALL, "");
#endif
        return res;
    }

    std::cout << u8"🌙 테스트 종료. 콘솔 상태 복원 완료.\n";

#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, ""); // 원래 로케일로 복구
#endif

    return res;
}
