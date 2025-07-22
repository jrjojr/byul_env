cmake_minimum_required(VERSION 3.20)

project(balix)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# ─────────────────────────────────────────────
# ⛔ 루트에서만 설정되는 항목 (중복 제거)
# ─────────────────────────────────────────────
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF) # gnu++17이 아닌 순수 c++17 사용

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 루트에서 이미 설정
# set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 루트에서 이미 설정

# ─────────────────────────────────────────────
# 소스 파일 설정
# ─────────────────────────────────────────────
set(SOURCES
    ${PROJECT_NAME}.cpp

    environ/environ.cpp    

    numal/dualnumber.cpp

    numal/dualquat.cpp

    numal/dualquat.hpp
    numal/numal.cpp

    numal/quat.cpp

    numal/quat.hpp
    numal/vec3.cpp

    numal/vec3.hpp

    numeq/numeq_integrator.cpp

    numeq/numeq_kalman.cpp

    numeq/numeq_model.cpp

    numeq/numeq_mpc.cpp

    numeq/numeq_pid_vec3.cpp

    numeq/numeq_pid.cpp

    numeq/numeq_solver.cpp

    numeq/numeq.cpp


    xform/xform.cpp

    trajectory/trajectory.cpp
    motion_state/motion_state.cpp

    controller/controller.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

# ─────────────────────────────────────────────
# include 경로
# ─────────────────────────────────────────────
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
)

# ─────────────────────────────────────────────
# 하위 모듈 링크
# ─────────────────────────────────────────────
set(LIBS
    # environ
    # numal
    # numeq
    # xform
    # trajectory
    # controller
    common
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

# ─────────────────────────────────────────────
# 병합 관련 (루트에서 처리 가능)
# ─────────────────────────────────────────────
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# include(merge_static_libs)
# merge_static_libraries(${PROJECT_NAME} ${LIBS})

add_subdirectory(tests)


# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)