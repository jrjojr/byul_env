cmake_minimum_required(VERSION 3.20)
project(byul)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
enable_testing()

# ─────────────────────────────────────────────
# 기본 설정
# ─────────────────────────────────────────────
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

# ─────────────────────────────────────────────
# DLL 빌드 설정 (Windows)
# ─────────────────────────────────────────────
if(CMAKE_SYSTEM_NAME STREQUAL "Windows" OR WIN32)
    message(STATUS "Windows build or cross-compilation enabled")

    # 정적 링크
    # 이유는 우분투에서 빌드해서 실행할때는 이미 모든 so가 이미 준비되어있다
    # 우분투 자체에 이미 설치되어있다
    # 하지만 win11용으로 빌드할때는 libstdc++.dll이 필요하다 따로 관리할수 없으니
    # 통합해서 다른 dll이 필요없게 만들어야 한다.
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")

    target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--subsystem,windows")

    if(WIN32)
        set(HOME_DIR "$ENV{USERPROFILE}")
    else()
        set(HOME_DIR "${CMAKE_SOURCE_DIR}/../../jaryo/incoming")
    endif()

    if(MSVC)
        add_compile_options("$<$<COMPILE_LANGUAGE:C>:/utf-8>")
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:/utf-8>")
    endif()

else()
    set(HOME_DIR "$ENV{HOME}")
    set(CMAKE_INSTALL_PREFIX "${HOME_DIR}/${PROJECT_NAME}" 
        CACHE PATH "Install path" FORCE)
endif()

# ─────────────────────────────────────────────
# DLL 정의
# ─────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_EXPORTS)

# ─────────────────────────────────────────────
# 빌드 경로 설정
# ─────────────────────────────────────────────
set(${PROJECT_NAME}_BUILD_INCLUDE_DIR 
    ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME})
set(${PROJECT_NAME}_BUILD_INTERNAL_DIR 
    ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}/internal)
set(${PROJECT_NAME}_BUILD_LIB_DIR ${CMAKE_BINARY_DIR}/lib)

file(MAKE_DIRECTORY ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})
file(MAKE_DIRECTORY ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})
file(MAKE_DIRECTORY ${${PROJECT_NAME}_BUILD_LIB_DIR})

# ─────────────────────────────────────────────
# 헤더 복사 매크로
# ─────────────────────────────────────────────
macro(copy_headers module)
    file(GLOB_RECURSE HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/${module}/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/${module}/*.hpp")
    foreach(hdr ${HEADERS})
        file(COPY ${hdr} DESTINATION ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})
    endforeach()
endmacro()

# ─────────────────────────────────────────────
# 모듈 추가
# ─────────────────────────────────────────────
add_subdirectory(common)
add_subdirectory(console)
add_subdirectory(navsys)
add_subdirectory(numal)
add_subdirectory(numeq)
add_subdirectory(xform)
add_subdirectory(balix)
add_subdirectory(gpu_comp_tester)

# ─────────────────────────────────────────────
# 헤더 복사
# ─────────────────────────────────────────────
copy_headers(common)
copy_headers(console)
copy_headers(navsys)
copy_headers(numal)
copy_headers(numeq)
copy_headers(xform)
copy_headers(balix)
copy_headers(gpu_comp_tester)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h 
    DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})
file(COPY byul_config.h DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})

# ─────────────────────────────────────────────
# include 경로 설정
# ─────────────────────────────────────────────
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}
    ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})

# ─────────────────────────────────────────────
# 라이브러리 링크 (navsys만 whole-archive)
# ─────────────────────────────────────────────
target_link_directories(${PROJECT_NAME} PRIVATE 
    ${${PROJECT_NAME}_BUILD_LIB_DIR}
    ${CMAKE_BINARY_DIR}/common
    ${CMAKE_BINARY_DIR}/navsys
    ${CMAKE_BINARY_DIR}/numal    
    ${CMAKE_BINARY_DIR}/numeq
    ${CMAKE_BINARY_DIR}/xform
    ${CMAKE_BINARY_DIR}/balix
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    -Wl,--whole-archive
    navsys

    numal
    numeq
    xform
    balix

    -Wl,--no-whole-archive    
)

# ─────────────────────────────────────────────
# 설치 관련
# ─────────────────────────────────────────────
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in
    ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(DIRECTORY ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.in
               ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)

install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION 
    ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
