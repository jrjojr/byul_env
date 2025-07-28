# parse_exports.cmake
# Usage: cmake -P parse_exports.cmake <exports.txt> <output.def>

# CMAKE_ARGC: 전체 인자 개수
# CMAKE_ARGV3: 첫 번째 사용자 인자
# CMAKE_ARGV4: 두 번째 사용자 인자
if(CMAKE_ARGC LESS 5)
    message(FATAL_ERROR "Usage: cmake -P parse_exports.cmake <exports.txt> <output.def>")
endif()

set(EXPORTS_TXT "${CMAKE_ARGV3}")
set(DEF_FILE "${CMAKE_ARGV4}")

message(STATUS "EXPORTS_TXT=${EXPORTS_TXT}, DEF_FILE=${DEF_FILE}")

file(READ "${EXPORTS_TXT}" EXPORT_CONTENTS)

# 추출된 심볼 리스트
set(EXPORT_SYMBOLS "")

# 각 줄을 순회하면서 심볼 라인만 추출
foreach(line IN LISTS EXPORT_CONTENTS)
# message("EXPORT_CONTENTS : ${EXPORT_CONTENTS}")
    string(REGEX MATCH "^[ \t]*[0-9]+[ \t]+[0-9A-F]+[ \t]+[0-9A-F]+[ \t]+([A-Za-z0-9_@?]+)" match "${line}")
    if(match)
        string(REGEX REPLACE "^[ \t]*[0-9]+[ \t]+[0-9A-F]+[ \t]+[0-9A-F]+[ \t]+" "" symbol "${line}")
        message("line : ${line} ")
        list(APPEND EXPORT_SYMBOLS ${symbol})
    endif()
endforeach()

# DEF 파일에 심볼 추가
message(" 추가된 심볼들")
file(APPEND "${DEF_FILE}" "\n")
foreach(symbol IN LISTS EXPORT_SYMBOLS)
message("symbol : ${symbol} ")
    file(APPEND "${DEF_FILE}" "${symbol}\n")
endforeach()
