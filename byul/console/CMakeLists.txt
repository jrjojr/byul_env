cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(console)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
    dstar_lite_console.cpp    
)

set(HEADERS
    ${PROJECT_NAME}.h
    dstar_lite_console.h
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 스태틱 라이브러리 생성
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# 헤더 파일 디렉토리 추가
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/navsys/coord
    ${CMAKE_SOURCE_DIR}/navsys/route
    ${CMAKE_SOURCE_DIR}/navsys/navgrid
    ${CMAKE_SOURCE_DIR}/navsys/dstar_lite
    ${CMAKE_SOURCE_DIR}/navsys/route_finder
    ${CMAKE_CURRENT_SOURCE_DIR}    
)

# target_link_directories(${PROJECT_NAME} PRIVATE 
#     ${CMAKE_BINARY_DIR}/coord
#     ${CMAKE_BINARY_DIR}/navgrid
#     ${CMAKE_BINARY_DIR}/route
#     ${CMAKE_BINARY_DIR}/common
# )

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE
    common
    navsys
)

# add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

install(FILES ${HEADERS}
    DESTINATION include/${CMAKE_PROJECT_NAME}
)