cmake_minimum_required(VERSION 3.20)

# Project name
project(rng)

# set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
# message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

# 소스 파일 설정
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/rng.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/roll.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/shuffle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/distributions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rng_fill.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/rng.h
    ${CMAKE_CURRENT_SOURCE_DIR}/roll.h
    ${CMAKE_CURRENT_SOURCE_DIR}/shuffle.h
    ${CMAKE_CURRENT_SOURCE_DIR}/distributions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/rng_fill.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)



# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${EXTERNAL_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE

)

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# install(FILES ${${PROJECT_NAME}_HEADERS}
#     DESTINATION include/${CMAKE_PROJECT_NAME}
# )