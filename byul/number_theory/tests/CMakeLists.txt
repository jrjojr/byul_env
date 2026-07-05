cmake_minimum_required(VERSION 3.20)

project(test_number_theory)

set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_module_number_theory.cpp

CACHE INTERNAL "${PROJECT_NAME} source files" FORCE
)

add_executable(${PROJECT_NAME}
    ${${PROJECT_NAME}_SOURCES}
)

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/number_theory
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EXTERNAL_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    number_theory
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

include(${CMAKE_SOURCE_DIR}/cmake/project_manager.cmake)
byul_apply_test_platform_settings(${PROJECT_NAME})
byul_install_target(${PROJECT_NAME})

enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
