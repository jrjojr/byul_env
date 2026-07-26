cmake_minimum_required(VERSION 3.20)

project(route)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/route.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/route_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/route_trace.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/route.h

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})
byul_register_production_target(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
    
    ${CMAKE_SOURCE_DIR}/numal

    ${CMAKE_SOURCE_DIR}/navsys
    ${CMAKE_SOURCE_DIR}/navsys/coord
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    numal
    coord
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
