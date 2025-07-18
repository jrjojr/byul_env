cmake_minimum_required(VERSION 3.20)

project(coord)

set(SOURCES
    ${PROJECT_NAME}.cpp
    coord_list.cpp
    coord_hash.cpp
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul    
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    m
)

add_subdirectory(tests)

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
