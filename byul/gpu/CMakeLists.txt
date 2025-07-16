cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(gpu)

# SDL2 찾기
find_package(SDL2 REQUIRED)

enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 외부 include 경로 설정
set(EXTERNAL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_INCLUDE_DIR = ${EXTERNAL_INCLUDE_DIR}")


# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
    main.cpp
    gpu_compute.cpp
    ${EXTERNAL_INCLUDE_DIR}/glad/src/gl.c    
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_executable(${PROJECT_NAME} ${SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_INCLUDE_DIR}
    ${EXTERNAL_INCLUDE_DIR}/glad/include
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/numeq
)

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE 
    m
    numeq
    ${SDL2_LIBRARIES} GL    
)

# add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)