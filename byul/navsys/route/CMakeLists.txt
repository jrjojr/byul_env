cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(route)

# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${CMAKE_SOURCE_DIR}/coord
    ${CMAKE_SOURCE_DIR}/common
)

target_link_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_BINARY_DIR}/coord
    ${CMAKE_BINARY_DIR}/common
)

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE ${GLIB_LIBRARIES}
    coord
    common
)

# 컴파일러 플래그 추가
target_compile_options(${PROJECT_NAME} PRIVATE ${GLIB_CFLAGS_OTHER})

target_compile_definitions(${PROJECT_NAME} PRIVATE ${GLIB_CFLAGS_OTHER})

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)