#include "doctest.h"

#include "byul.h"

#include <string>

TEST_CASE("byul_version_string returns correct version") {
    const char* version = byul_version_string();
    CHECK(version != nullptr);
    CHECK(std::string(version) == BYUL_VERSION_STRING);
}

TEST_CASE("byul_print_version does not crash") {
    CHECK_NOTHROW(byul_print_version());
}
