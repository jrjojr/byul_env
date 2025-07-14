cmake_minimum_required(VERSION 3.20)

project(byul)

enable_testing()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)

# Windows 빌드 또는 크로스 컴파일 감지
if(CMAKE_SYSTEM_NAME STREQUAL "Windows" OR WIN32)
    message(STATUS "Windows build or cross-compilation enabled")

    # DLL 관련 속성
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        PREFIX ""
        OUTPUT_NAME "byul"
        IMPORT_PREFIX "lib"
        IMPORT_SUFFIX ".a")

    # 정적 링크 + GUI 서브시스템
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")
    target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--subsystem,windows")

    # 홈 디렉토리 설정
    if(WIN32)
        # 실제 Windows에서 빌드 중 (MSVC or MinGW natively)
        set(HOME_DIR "$ENV{USERPROFILE}")
    else()
        # 우분투에서 Windows 크로스 컴파일 중
        # 사용자가 지정한 경로를 사용 절대 경로 /home으로는 권한이 제한된다
        # 상대 경로를 사용한다.
        set(HOME_DIR "${CMAKE_SOURCE_DIR}/../../jaryo/incoming")

    endif()

    # MSVC 전용 옵션
    if(MSVC)
        add_compile_options("$<$<COMPILE_LANGUAGE:C>:/utf-8>")
        add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:/utf-8>")
    endif()

else()
    # 일반 Linux 빌드
    set(HOME_DIR "$ENV{HOME}")
    set(CMAKE_INSTALL_PREFIX "${HOME_DIR}/${PROJECT_NAME}" 
        CACHE PATH "Install path" FORCE)    
endif()

# uninstall 타겟
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in
    ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)

# 빌드 디렉토리 설정
set(${PROJECT_NAME}_BUILD_INCLUDE_DIR 
    ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME})

set(${PROJECT_NAME}_BUILD_INTERNAL_DIR 
    ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}/internal)

set(${PROJECT_NAME}_BUILD_LIB_DIR ${CMAKE_BINARY_DIR}/lib)
file(MAKE_DIRECTORY ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})
file(MAKE_DIRECTORY ${${PROJECT_NAME}_BUILD_LIB_DIR})

# 헤더 복사 매크로
macro(copy_headers module)
    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/${module}/*.h")
    foreach(hdr ${HEADERS})
        file(COPY ${hdr} DESTINATION ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})
    endforeach()
endmacro()

macro(copy_route_finder_headers module)
    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/${module}/modules/*.h")
    foreach(hdr ${HEADERS})
        file(COPY ${hdr} DESTINATION ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})
    endforeach()
endmacro()

# 모듈 추가
add_subdirectory(core)
add_subdirectory(coord)
add_subdirectory(map)
add_subdirectory(route)
add_subdirectory(cost_coord_pq)
add_subdirectory(route_finder)
add_subdirectory(dstar_lite)
add_subdirectory(maze)
add_subdirectory(route_carver)
add_subdirectory(obstacle)
add_subdirectory(console)

# 헤더 복사
copy_headers(core)
copy_headers(coord)
copy_headers(map)
copy_headers(route)
copy_headers(cost_coord_pq)
copy_headers(maze)
copy_headers(route_carver)
copy_headers(obstacle)
copy_headers(console)
copy_headers(route_finder)
copy_route_finder_headers(route_finder)
copy_headers(dstar_lite)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h 
    DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})

file(COPY byul_config.h DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})

# include 경로
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}
    ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})

target_link_directories(${PROJECT_NAME} PRIVATE 
    ${PROJECT_NAME}_BUILD_LIB_DIR
    ${CMAKE_BINARY_DIR}/coord
    ${CMAKE_BINARY_DIR}/map
    ${CMAKE_BINARY_DIR}/route
    ${CMAKE_BINARY_DIR}/dstar_lite
    ${CMAKE_BINARY_DIR}/route_finder
    ${CMAKE_BINARY_DIR}/cost_coord_pq
    ${CMAKE_BINARY_DIR}/maze
    ${CMAKE_BINARY_DIR}/route_carver
    ${CMAKE_BINARY_DIR}/obstacle
    ${CMAKE_BINARY_DIR}/console)

target_link_libraries(${PROJECT_NAME} PUBLIC
    -Wl,--whole-archive
    coord 
    map 
    route 
    cost_coord_pq 
    dstar_lite
    route_carver 
    route_finder 
    maze 
    obstacle 
    console
    -Wl,--no-whole-archive)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> 
    ${${PROJECT_NAME}_BUILD_LIB_DIR}/lib${PROJECT_NAME}.a)

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

# # 정적으로 링크 (권장) 이러면 dll 하나만 있으면 된다.
# set(CMAKE_SHARED_LINKER_FLAGS 
#     "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")


# add_subdirectory(tests)
