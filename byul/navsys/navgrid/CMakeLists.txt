cmake_minimum_required(VERSION 3.20)

project(navgrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/navgrid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/navcell.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/navgrid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/navcell.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
    
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/navsys/coord
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    core
    coord
)

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
