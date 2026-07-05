cmake_minimum_required(VERSION 3.20)

project(number_theory)

set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_prime32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_prime_segmented_u32.c

CACHE INTERNAL "${PROJECT_NAME} source files" FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/number_theory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_prime32.h
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_prime_segmented_u32.h

CACHE INTERNAL "${PROJECT_NAME} header files" FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

add_subdirectory(tests)
