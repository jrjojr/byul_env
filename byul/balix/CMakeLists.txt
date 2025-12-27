cmake_minimum_required(VERSION 3.20)

project(balix)

add_subdirectory(collision)

# ─────────────────────────────────────────────
# 소스 파일 설정
# ─────────────────────────────────────────────
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/balix.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/bodyprops/bodyprops.cpp    
    ${CMAKE_CURRENT_SOURCE_DIR}/environ/environ.cpp    

    ${numal_SOURCES}
    ${collision_SOURCES}

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_integrator.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/filters/numeq_filters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/filters/numeq_kalman.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_model_motion.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_mpc.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_pid_vec3.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_pid.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_solver.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/xform/xform.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory/trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_state/motion_state.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/controller/controller.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/balix.h

    ${CMAKE_CURRENT_SOURCE_DIR}/bodyprops/bodyprops.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/environ/environ.h    

    ${numal_HEADERS}
    ${collision_HEADERS}

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_integrator.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/filters/numeq_filters.h
    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/filters/numeq_kalman.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_model_motion.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_mpc.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_pid_vec3.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_pid.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq_solver.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numeq/numeq.h

    ${CMAKE_CURRENT_SOURCE_DIR}/xform/xform.h

    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory/trajectory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_state/motion_state.h

    ${CMAKE_CURRENT_SOURCE_DIR}/controller/controller.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

# ─────────────────────────────────────────────
# include 경로
# ─────────────────────────────────────────────
target_include_directories(${PROJECT_NAME} PRIVATE
    ${EXTERNAL_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal
    ${CMAKE_SOURCE_DIR}/byul_tick
    
    ${CMAKE_SOURCE_DIR}/navsys
    ${CMAKE_SOURCE_DIR}/navsys/coord

    ${CMAKE_SOURCE_DIR}/balix/bodyprops
    ${CMAKE_SOURCE_DIR}/balix/controller
    ${CMAKE_SOURCE_DIR}/balix/environ
    ${CMAKE_SOURCE_DIR}/balix/motion_state
    
    ${CMAKE_SOURCE_DIR}/balix/numeq
    ${CMAKE_SOURCE_DIR}/balix/numeq/filters

    ${CMAKE_SOURCE_DIR}/balix/tests
    ${CMAKE_SOURCE_DIR}/balix/trajectory
    ${CMAKE_SOURCE_DIR}/balix/xform    

    ${CMAKE_SOURCE_DIR}/balix/collision
)

# ─────────────────────────────────────────────
# 하위 모듈 링크
# ─────────────────────────────────────────────
set(LIBS
    # environ
    # numeq
    # xform
    # trajectory
    # controller
    core
    numal    
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

# install(FILES ${${PROJECT_NAME}_HEADERS}
#     DESTINATION include/${CMAKE_PROJECT_NAME}
# )