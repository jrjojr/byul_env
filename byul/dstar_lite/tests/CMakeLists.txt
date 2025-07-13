# CMakeLists.txt for the test project

cmake_minimum_required(VERSION 3.10)

# Set the project name
project(test_dstar_lite)

set(EXTERNAL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_INCLUDE_DIR = ${EXTERNAL_INCLUDE_DIR}")

# Add the test executable
add_executable(${PROJECT_NAME} 
    test_main.cpp
    test_module_dstar_lite_pqueue.cpp
    test_dstar_lite.cpp
    )


target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_INCLUDE_DIR}    
    )

# Ensure glib.h can be included
target_link_directories(${PROJECT_NAME} PRIVATE 
    # ${CMAKE_BINARY_DIR}/lib
    ${CMAKE_BINARY_DIR}/coord
    ${CMAKE_BINARY_DIR}/route
    ${CMAKE_BINARY_DIR}/map
    ${CMAKE_BINARY_DIR}/route_finder
    ${CMAKE_BINARY_DIR}/dstar_lite
    ${CMAKE_BINARY_DIR}/console    

    )

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
    coord
    route
    map
    route_finder
    dstar_lite
    console

    )

# Add a test goal CTest
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# 컴파일·링크 옵션에 ASan 추가
# win11과 우분투는 다르다
if (CMAKE_SYSTEM_NAME MATCHES "Windows")
    if (MSVC)
        message(STATUS "${PROJECT_NAME} MSVC: Enabling ASan (/fsanitize=address)")
        target_compile_options(${PROJECT_NAME} PRIVATE /fsanitize=address /Zi /Od)
        target_link_options(${PROJECT_NAME} PRIVATE /fsanitize=address /INCREMENTAL:NO)

        # 추천: MSVC에서는 Release 대신 Debug로 빌드해야 정상 작동
        set_target_properties(${PROJECT_NAME} PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreadedDebug"
        )
    else()
        message(STATUS "${PROJECT_NAME} ASan not configured: unknown compiler on Windows")
    endif()

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
