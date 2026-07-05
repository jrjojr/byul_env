#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

int main(int argc, char** argv) {
#ifdef _WIN32
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(65001);                          // Set console to UTF-8 output
    setlocale(LC_ALL, "en_US.UTF-8");                   // Set UTF-8 locale
#else
    setlocale(LC_ALL, "en_US.UTF-8");                   // Linux/Mac locale setting
#endif

    std::cout << "UTF-8 console code page set. Starting tests...\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);

    // --- Forced default options ---
    // context.setOption("success", true);      // Show all successful tests
    // context.setOption("no-skip", true);      // Show skipped tests
    // context.setOption("reporters", "console");
    // context.setOption("no-colors", true);
    // context.setOption("durations", true);    // Show execution time for each test

    int res = context.run();

    if (context.shouldExit()) {
        std::cout << "Tests finished. Console code page restored.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp);                // Restore original code page
        setlocale(LC_ALL, "");                          // Restore default locale
#endif
        return res;
    }

    std::cout << "Tests complete. Console state restored.\n";
#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, "");                              // Restore locale
#endif

    return res;
}
