cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(console)

# 소스 파일 설정
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/console.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dstar_lite_console.cpp    

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/console.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dstar_lite_console.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal
    ${CMAKE_SOURCE_DIR}/byul_tick
    
    ${CMAKE_SOURCE_DIR}/navsys/coord
    ${CMAKE_SOURCE_DIR}/navsys/route
    ${CMAKE_SOURCE_DIR}/navsys/navgrid
    ${CMAKE_SOURCE_DIR}/navsys/dstar_lite
    ${CMAKE_SOURCE_DIR}/navsys/route_finder

)

# target_link_directories(${PROJECT_NAME} PRIVATE 
#     ${CMAKE_BINARY_DIR}/coord
#     ${CMAKE_BINARY_DIR}/navgrid
#     ${CMAKE_BINARY_DIR}/route
#     ${CMAKE_BINARY_DIR}/core
# )

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE
    core
    numal
    navsys
)

# add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# install(FILES ${${PROJECT_NAME}_HEADERS}
#     DESTINATION include/${CMAKE_PROJECT_NAME}
# )
