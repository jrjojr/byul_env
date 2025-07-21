cmake_minimum_required(VERSION 3.20)

project(navsys)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# ✅ C++17 이상 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # gnu++17이 아닌 순수 c++17 사용

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

# navsys는 소스 없는 허브 라이브러리
add_library(${PROJECT_NAME} INTERFACE)

# 모듈 추가
add_subdirectory(coord)
add_subdirectory(route)
add_subdirectory(navgrid)

add_subdirectory(cost_coord_pq)
add_subdirectory(route_finder)
add_subdirectory(dstar_lite)

add_subdirectory(maze)
add_subdirectory(route_carver)
add_subdirectory(obstacle)

# include 경로
target_include_directories(${PROJECT_NAME} INTERFACE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    coord 
    route 
    navgrid 

    cost_coord_pq 
    route_finder 
    dstar_lite

    maze
    route_carver     
    obstacle 
)

# add_subdirectory(tests)
