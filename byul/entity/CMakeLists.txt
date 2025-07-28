cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(entity)

enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
    entity_dynamic.cpp
    entity_dynamic_coord.cpp
    entity_spring.cpp
)

set(HEADERS
    ${PROJECT_NAME}.h
    entity_dynamic.h
    entity_dynamic_coord.h
    entity_spring.h
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_BINARY_DIR}/include/byul
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common

    ${CMAKE_SOURCE_DIR}/navsys/coord

    ${CMAKE_SOURCE_DIR}/balix/numal
    ${CMAKE_SOURCE_DIR}/balix/xform
    ${CMAKE_SOURCE_DIR}/balix/bodyprops
    ${CMAKE_SOURCE_DIR}/balix/environ
    ${CMAKE_SOURCE_DIR}/balix/motion_state
    ${CMAKE_SOURCE_DIR}/balix/trajectory
    ${CMAKE_SOURCE_DIR}/balix/numeq
)

# target_link_directories(${PROJECT_NAME} PRIVATE 
#     ${CMAKE_BINARY_DIR}/navsys
# )

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE 
    navsys
    balix
)

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

install(FILES ${HEADERS}
    DESTINATION include/${CMAKE_PROJECT_NAME}
)