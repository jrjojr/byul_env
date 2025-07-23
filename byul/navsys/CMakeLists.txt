cmake_minimum_required(VERSION 3.20)

project(navsys)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# ─────────────────────────────────────────────
# ⛔ 루트에서만 설정해도 되는 것들 (코멘트 처리)
# ─────────────────────────────────────────────
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF) # gnu++17이 아닌 순수 c++17 사용

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 루트에서 이미 설정
# set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 루트에서 이미 설정

# ─────────────────────────────────────────────
# 소스 파일
# ─────────────────────────────────────────────
set(SOURCES
    ${PROJECT_NAME}.cpp

    coord/coord.cpp
    coord/coord_hash.cpp
    coord/coord_list.cpp
    coord/cost_coord_pq.cpp

    route/route.cpp

    navgrid/navgrid.cpp

    route_carver/route_carver.cpp

    dstar_lite/dstar_lite_key.cpp
    dstar_lite/dstar_lite_pqueue.cpp
    dstar_lite/dstar_lite.cpp

    route_finder/route_finder.cpp    
    route_finder/route_finder_common.cpp
    route_finder/astar.cpp
    route_finder/bfs.cpp
    route_finder/dfs.cpp
    route_finder/dijkstra.cpp
    route_finder/fast_marching.cpp
    route_finder/fringe_search.cpp
    route_finder/greedy_best_first.cpp
    route_finder/ida_star.cpp
    route_finder/rta_star.cpp
    route_finder/sma_star.cpp
    route_finder/weighted_astar.cpp

    obstacle/obstacle_common.cpp
    obstacle/obstacle.cpp    

    maze/maze_aldous_broder.cpp
    maze/maze_binary.cpp
    maze/maze_common.cpp
    maze/maze_eller.cpp
    maze/maze_hunt_and_kill.cpp
    maze/maze_kruskal.cpp
    maze/maze_prim.cpp
    maze/maze_recursive_division.cpp
    maze/maze_recursive.cpp
    maze/maze_room.cpp
    maze/maze_sidewinder.cpp
    maze/maze_wilson.cpp
    maze/maze.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

# ─────────────────────────────────────────────
# include 경로
# ─────────────────────────────────────────────
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
)

# ─────────────────────────────────────────────
# 하위 모듈 링크
# ─────────────────────────────────────────────
set(LIBS

)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

# ─────────────────────────────────────────────
# 병합 관련 (루트에서 처리 가능)
# ─────────────────────────────────────────────
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# include(merge_static_libs)
# merge_static_libraries(${PROJECT_NAME} ${LIBS})

add_subdirectory(tests)


# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)