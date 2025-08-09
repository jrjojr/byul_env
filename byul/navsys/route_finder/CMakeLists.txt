cmake_minimum_required(VERSION 3.20)

project(route_finder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/astar.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/bfs.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/dfs.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/dijkstra.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/fast_marching.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/fringe_search.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/greedy_best_first.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/ida_star.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/route_finder_common.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/route_finder.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/rta_star.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/sma_star.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/weighted_astar.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/astar.h

    ${CMAKE_CURRENT_SOURCE_DIR}/bfs.h

    ${CMAKE_CURRENT_SOURCE_DIR}/dfs.h

    ${CMAKE_CURRENT_SOURCE_DIR}/dijkstra.h

    ${CMAKE_CURRENT_SOURCE_DIR}/fast_marching.h

    ${CMAKE_CURRENT_SOURCE_DIR}/fringe_search.h

    ${CMAKE_CURRENT_SOURCE_DIR}/greedy_best_first.h

    ${CMAKE_CURRENT_SOURCE_DIR}/ida_star.h

    ${CMAKE_CURRENT_SOURCE_DIR}/route_finder_common.h

    ${CMAKE_CURRENT_SOURCE_DIR}/route_finder.h

    ${CMAKE_CURRENT_SOURCE_DIR}/rta_star.h

    ${CMAKE_CURRENT_SOURCE_DIR}/sma_star.h

    ${CMAKE_CURRENT_SOURCE_DIR}/weighted_astar.h

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
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    common
    coord
    route
    navgrid
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
