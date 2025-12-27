cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(ground)

enable_testing()


# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")


# 소스 파일 설정
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ground.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/ground.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}/core

    ${CMAKE_SOURCE_DIR}/numal

    ${CMAKE_SOURCE_DIR}/balix/bodyprops
    ${CMAKE_SOURCE_DIR}/balix/environ

    ${CMAKE_SOURCE_DIR}/navsys/coord

    ${EXTERNAL_DIR}    
)

# target_link_directories(${PROJECT_NAME} PRIVATE 
#     ${CMAKE_BINARY_DIR}/numal
#     ${CMAKE_BINARY_DIR}/xform
#     ${CMAKE_BINARY_DIR}/numeq
#     ${CMAKE_BINARY_DIR}/controller
# )

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE 
    numal
    navsys
    balix
)

# 수학 라이브러리 링크 (리눅스/유닉스 전용)
if(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# install(FILES ${${PROJECT_NAME}_HEADERS}
#     DESTINATION include/${CMAKE_PROJECT_NAME}
# )