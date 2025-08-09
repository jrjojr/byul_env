cmake_minimum_required(VERSION 3.20)

project(maze)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/maze.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_common.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/maze_aldous_broder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_binary.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_eller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_hunt_and_kill.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/maze_kruskal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_prim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_recursive_division.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_recursive.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_room_blend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_sidewinder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_wilson.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/maze.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_common.h

    ${CMAKE_CURRENT_SOURCE_DIR}/maze_recursive.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_prim.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_binary.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_eller.h    

    ${CMAKE_CURRENT_SOURCE_DIR}/maze_aldous_broder.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_wilson.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_hunt_and_kill.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_sidewinder.h    

    ${CMAKE_CURRENT_SOURCE_DIR}/maze_recursive_division.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_kruskal.h    
    ${CMAKE_CURRENT_SOURCE_DIR}/maze_room_blend.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
    
    ${CMAKE_SOURCE_DIR}/common

    ${CMAKE_SOURCE_DIR}/navsys/coord
    ${CMAKE_SOURCE_DIR}/navsys/route
    ${CMAKE_SOURCE_DIR}/navsys/navgrid
    ${CMAKE_SOURCE_DIR}/navsys/obstacle
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    common
    coord
    route
    navgrid
    obstacle
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
