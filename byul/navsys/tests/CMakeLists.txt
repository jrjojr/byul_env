cmake_minimum_required(VERSION 3.20)

project(test_navsys)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# 소스 파일 설정
set(SOURCES
    main.cpp
    test_module_coord.cpp
    test_module_coord_list.cpp
    test_module_coord_hash.cpp

    test_module_cost_coord_pq.cpp
    test_module_dstar_lite_pqueue.cpp
    test_module_dstar_lite.cpp
    test_module_maze_room.cpp
    test_module_maze.cpp
    test_module_navgrid.cpp
    test_module_obstacle.cpp
    test_module_route_finder.cpp
    test_module_route.cpp
    test_module_route_carver.cpp
)

# ------------------- 테스트 실행 파일 -------------------
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

# 컴파일·링크 옵션에 ASan 추가
# win11과 우분투는 다르다
if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS 
    "${PROJECT_NAME} No apply ASan for Windows because error.")

else()
    message(STATUS "${PROJECT_NAME} Applying ASan + LSan for Unix-like")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
        -g
        -O1
    )
    target_link_options(${PROJECT_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
    )
endif()

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
)

# ------------------- 정적 라이브러리 연결 -------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/navsys
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    navsys
    console
)

# STATIC 라이브러리이 필요 시 심볼 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# ------------------- CTest -------------------
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
