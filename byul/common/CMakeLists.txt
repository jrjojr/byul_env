cmake_minimum_required(VERSION 3.20)

# Project name
project(common)

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

# 소스 파일 설정
set(SOURCES
    float_common.cpp
    hashset.cpp
)

set(HEADERS
    float_common.h
    hashset.h
)

# 코드 인텔리센스에 도움이 된다 vscode에서...
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 정적 라이브러리를 동적인 라이브러리에 추가
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE

)

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

install(FILES ${HEADERS}
    DESTINATION include/${CMAKE_PROJECT_NAME}
)