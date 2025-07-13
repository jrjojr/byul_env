cmake_minimum_required(VERSION 3.10)

project(test_core)

# ------------------- 테스트 실행 파일 -------------------
add_executable(${PROJECT_NAME} test_core.cpp)

# ------------------- include 경로 설정 -------------------
set(EXTERNAL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_INCLUDE_DIR = ${EXTERNAL_INCLUDE_DIR}")

# 포함 디렉토리
# core.h, doctest.h 등 경로 포함
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}                         # 프로젝트 루트
    ${CMAKE_CURRENT_SOURCE_DIR}                 # 현재 테스트 디렉토리
    ${CMAKE_BINARY_DIR}/include/byul     # 빌드시 생성되는 헤더
    ${EXTERNAL_INCLUDE_DIR}                     
)

# ------------------- 라이브러리 링크 -------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/core
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    core
    m # 수학함수 fabsf 등
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# ------------------- CTest -------------------
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

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
