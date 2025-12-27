cmake_minimum_required(VERSION 3.20)

project(coord)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/coord.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/coord_hash.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/coord_list.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_coord_pq.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/coord.h
    ${CMAKE_CURRENT_SOURCE_DIR}/coord_hash.h
    ${CMAKE_CURRENT_SOURCE_DIR}/coord_list.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_coord_pq.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    core
    numal
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
