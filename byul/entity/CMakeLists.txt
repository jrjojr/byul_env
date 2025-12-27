cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(entity)

enable_testing()


# 소스 파일 설정
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/entity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_dynamic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_dynamic_coord.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_spring.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_interaction.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/entity.h
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_dynamic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_dynamic_coord.h
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_spring.h
    ${CMAKE_CURRENT_SOURCE_DIR}/entity_interaction.h

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

    ${CMAKE_SOURCE_DIR}/navsys/coord

    ${CMAKE_SOURCE_DIR}/numal
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
    numal
    balix
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# install(FILES ${${PROJECT_NAME}_HEADERS}
#     DESTINATION include/${CMAKE_PROJECT_NAME}
# )