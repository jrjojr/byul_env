cmake_minimum_required(VERSION 3.20)

project(balix)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# ✅ C++17 이상 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # gnu++17이 아닌 순수 c++17 사용

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
    environ.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})


# 모듈 추가
add_subdirectory(numal)
add_subdirectory(numeq)
add_subdirectory(xform)
add_subdirectory(trajectory)
add_subdirectory(controller)
add_subdirectory(entity_dynamic)

# include 경로
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    numal
    numeq
    xform
    trajectory
    controller
    entity_dynamic
)

# add_subdirectory(tests)
