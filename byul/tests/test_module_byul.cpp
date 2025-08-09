#include "doctest.h"

#include "byul.h"

#include <string>

TEST_CASE("byul_version_string returns correct version") {
    const char* version = byul_version_string();
    CHECK(version != nullptr);
    CHECK(std::string(version) == BYUL_VERSION_STRING);
}

TEST_CASE("byul_print_version does not crash") {
    // 이 테스트는 출력만 수행, 내용 검증은 하지 않음
    // 단지 호출이 정상적으로 수행되는지만 확인
    CHECK_NOTHROW(byul_print_version());
}
