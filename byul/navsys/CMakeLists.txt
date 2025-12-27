cmake_minimum_required(VERSION 3.20)

project(navsys)

add_subdirectory(coord)
add_subdirectory(route)
add_subdirectory(navgrid)
add_subdirectory(obstacle)
add_subdirectory(maze)
add_subdirectory(route_finder)
add_subdirectory(dstar_lite)

set(${PROJECT_NAME}_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/navsys.cpp

    ${coord_SOURCES}
    ${route_SOURCES}
    ${navgrid_SOURCES}
    ${obstacle_SOURCES}
    ${maze_SOURCES}
    ${route_finder_SOURCES}
    ${dstar_lite_SOURCES}
    
    ${CMAKE_CURRENT_SOURCE_DIR}/route_carver/route_carver.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/navsys.h

    ${coord_HEADERS}
    ${route_HEADERS}
    ${navgrid_HEADERS}
    ${obstacle_HEADERS}
    ${maze_HEADERS}
    ${route_finder_HEADERS}
    ${dstar_lite_HEADERS}

    ${CMAKE_CURRENT_SOURCE_DIR}/route_carver/route_carver.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC 
    ${${PROJECT_NAME}_SOURCES} 
)

# ─────────────────────────────────────────────
# include 경로
# ─────────────────────────────────────────────
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}

    ${CMAKE_SOURCE_DIR}/byul_tick

    ${CMAKE_CURRENT_SOURCE_DIR}/coord
    ${CMAKE_CURRENT_SOURCE_DIR}/route
    ${CMAKE_CURRENT_SOURCE_DIR}/navgrid
    ${CMAKE_CURRENT_SOURCE_DIR}/obstacle
    ${CMAKE_CURRENT_SOURCE_DIR}/route_finder
    ${CMAKE_CURRENT_SOURCE_DIR}/maze
    ${CMAKE_CURRENT_SOURCE_DIR}/dstar_lite

    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal

    ${CMAKE_SOURCE_DIR}/navsys/route_carver
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    byul_tick
    numal
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
