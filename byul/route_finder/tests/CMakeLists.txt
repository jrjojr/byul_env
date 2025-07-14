# CMakeLists.txt for the test project

cmake_minimum_required(VERSION 3.20)

# Set the project name
project(test_route_finder)

# 외부 include 경로 설정
set(EXTERNAL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_INCLUDE_DIR = ${EXTERNAL_INCLUDE_DIR}")

# Enable testing
enable_testing()

# Add the test executable
add_executable(${PROJECT_NAME} 
    ${PROJECT_NAME}.cpp
    )

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/coord
    )

# Ensure glib.h can be included
target_link_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_BINARY_DIR}/route_finder
    ${CMAKE_BINARY_DIR}/console
    )

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
    route_finder
    console
    )

# Add a test goal CTest
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

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
