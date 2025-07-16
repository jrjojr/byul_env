cmake_minimum_required(VERSION 3.20)

project(test_gpu)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# ------------------- 테스트 실행 파일 -------------------
add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    test_module_dualnumber.cpp
    test_module_rotator.cpp
    # test_module_dualquat.cpp
)

# 컴파일·링크 옵션에 ASan 추가
# win11과 우분투는 다르다
if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS 
    "${PROJECT_NAME} No apply ASan for Windows because error.")

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

# 외부 include 경로 설정
set(EXTERNAL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_INCLUDE_DIR = ${EXTERNAL_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_INCLUDE_DIR}
)

# ------------------- 정적 라이브러리 연결 -------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/gpu
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    gpu
)

# coord는 STATIC 라이브러리이므로 필요 시 심볼 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# ------------------- CTest -------------------
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
