cmake_minimum_required(VERSION 3.20)
project(test_byul)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# ─────────────────────────────────────────────
# 테스트 소스 초기화 (main.cpp는 항상 포함)
# ─────────────────────────────────────────────
set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
set(TEST_INC_DIRS "")  # 모든 테스트 헤더 디렉토리를 저장할 변수

# ─────────────────────────────────────────────
# 특정 모듈의 모든 서브 디렉토리에서 test_module_*.cpp 검색
# ─────────────────────────────────────────────
function(add_module MODULE_NAME)
    string(TOUPPER ${MODULE_NAME} UPPER_NAME)
    option(TEST_${UPPER_NAME} "Enable ${MODULE_NAME} module tests" ON)

    set(MODULE_DIR "${CMAKE_SOURCE_DIR}/${MODULE_NAME}")

    if(TEST_${UPPER_NAME})
        if(EXISTS "${MODULE_DIR}")
            # 재귀적으로 test_module_*.cpp 수집
            file(GLOB_RECURSE MODULE_TESTS
                "${MODULE_DIR}/**/test_module_*.cpp")

            if(MODULE_TESTS)
                message(STATUS "[TEST] Adding tests from ${MODULE_NAME}:")
                foreach(TEST_SRC ${MODULE_TESTS})
                    message(STATUS "   ${TEST_SRC}")
                endforeach()
                list(APPEND TEST_SOURCES ${MODULE_TESTS})
                set(TEST_SOURCES "${TEST_SOURCES}" PARENT_SCOPE)
            else()
                message(STATUS "[TEST] No test_module_*.cpp found in ${MODULE_NAME}")
            endif()

            # MODULE_DIR 내부 모든 헤더 (*.h, *.hpp) 경로 추가
            file(GLOB_RECURSE MODULE_HEADERS
                "${MODULE_DIR}/**/*.h"
                "${MODULE_DIR}/**/*.hpp")
            foreach(HDR ${MODULE_HEADERS})
                get_filename_component(HDR_DIR ${HDR} DIRECTORY)
                list(APPEND TEST_INC_DIRS ${HDR_DIR})
            endforeach()

            list(REMOVE_DUPLICATES TEST_INC_DIRS)
            set(TEST_INC_DIRS "${TEST_INC_DIRS}" PARENT_SCOPE)
        else()
            message(STATUS "[TEST] ${MODULE_NAME} directory does not exist")
        endif()
    else()
        message(STATUS "[TEST] ${MODULE_NAME} tests disabled (TEST_${UPPER_NAME}=OFF)")
    endif()
endfunction()

# ─────────────────────────────────────────────
# 테스트할 모듈 등록
# ─────────────────────────────────────────────
add_module(common)
add_module(console)
add_module(navsys)
add_module(balix)
add_module(entity)

# ─────────────────────────────────────────────
# 테스트 실행 파일 생성
# ─────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${TEST_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE byul)

# ─────────────────────────────────────────────
# 외부 include 경로 설정
# ─────────────────────────────────────────────
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
    ${TEST_INC_DIRS}
)

# ─────────────────────────────────────────────
# ASan 옵션 설정 (Unix-like만)
# ─────────────────────────────────────────────
if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "${PROJECT_NAME} No apply ASan for Windows because error.")
else()
    message(STATUS "${PROJECT_NAME} Applying ASan + LSan for Unix-like")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
        -g
        -O1
    )
    target_link_options(${PROJECT_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
    )
endif()

# ─────────────────────────────────────────────
# CTest 등록
# ─────────────────────────────────────────────
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

# ─────────────────────────────────────────────
# 최종 확인 메시지
# ─────────────────────────────────────────────
message(STATUS "[TEST] Final source list:")
foreach(SRC ${TEST_SOURCES})
    message(STATUS "   ${SRC}")
endforeach()

message(STATUS "[TEST] Final include directories:")
foreach(DIR ${TEST_INC_DIRS})
    message(STATUS "   ${DIR}")
endforeach()
