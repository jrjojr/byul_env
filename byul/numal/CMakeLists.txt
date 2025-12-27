cmake_minimum_required(VERSION 3.20)

project(numal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PROJECT_NAME}_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/scalar.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/dualnumber.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/dualquat.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/geom.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/plane.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/numal.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/quat.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/vec3.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/scalar.h

    ${CMAKE_CURRENT_SOURCE_DIR}/dualnumber.h

    ${CMAKE_CURRENT_SOURCE_DIR}/dualquat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dualquat.hpp

    ${CMAKE_CURRENT_SOURCE_DIR}/geom.h

    ${CMAKE_CURRENT_SOURCE_DIR}/plane.h

    ${CMAKE_CURRENT_SOURCE_DIR}/numal.h

    ${CMAKE_CURRENT_SOURCE_DIR}/quat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/quat.hpp

    ${CMAKE_CURRENT_SOURCE_DIR}/vec3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/vec3.hpp

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${EXTERNAL_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}    
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE 

)

# 수학 라이브러리 링크 (리눅스/유닉스 전용)
if(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()

add_subdirectory(tests)

# win11 에서는 무조건 설정해야 한다 dll과 static 간에 문제때문에 링크가 안된다.
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)
