cmake_minimum_required(VERSION 3.20)

# 프로젝트 이름과 언어 설정
project(maze)

enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 소스 파일 설정
set(SOURCES
    ${PROJECT_NAME}.cpp
    maze_common.cpp
    maze_recursive.cpp 
    maze_prim.cpp   
    maze_binary.cpp
    maze_eller.cpp
    maze_aldous_broder.cpp
    maze_wilson.cpp
    maze_hunt_and_kill.cpp
    maze_sidewinder.cpp
    maze_recursive_division.cpp
    maze_kruskal.cpp
    maze_room.cpp
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
)

target_link_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_BINARY_DIR}/coord
    ${CMAKE_BINARY_DIR}/navgrid
)

# 링크 라이브러리 설정
target_link_libraries(${PROJECT_NAME} PRIVATE ${GLIB_LIBRARIES}
    common
    coord
    navgrid
    m
)

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)