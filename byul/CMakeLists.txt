cmake_minimum_required(VERSION 3.20)
project(byul)

# ─────────────────────────────────────────────
# 루트 전용: 빌드 타입 메시지 출력
# ─────────────────────────────────────────────
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ─────────────────────────────────────────────
# 루트 전용: CTest 활성화
# 하위 디렉토리에서는 enable_testing() 호출 필요 없음
# ─────────────────────────────────────────────
enable_testing()

# ─────────────────────────────────────────────
# 기본 설정 (루트에서 전역 적용)
# ─────────────────────────────────────────────
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 루트 전용: VSCode 등에서 컴파일 DB 제공
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 루트 전용: 정적 라이브러리 -> 동적 라이브러리 링크 지원
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────
# 루트 전용: 최종 DLL/Shared 라이브러리 생성
# ─────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_EXPORTS)

# ─────────────────────────────────────────────
# 루트 전용: DLL 빌드 관련 옵션 (Windows)
# ─────────────────────────────────────────────
if(CMAKE_SYSTEM_NAME STREQUAL "Windows" OR WIN32)
    message(STATUS "Windows build or cross-compilation enabled")

    # 루트 전용: 정적 링킹 플래그 전역 설정이라 다른 dll이 영향을 받는다 사용하지마
    # set(CMAKE_SHARED_LINKER_FLAGS
    #     "${CMAKE_SHARED_LINKER_FLAGS} -static-libgcc -static-libstdc++")

    # 정확하게 현재 프로젝트만 적용한다.
    # set_target_properties(${PROJECT_NAME} PROPERTIES
    #     LINK_FLAGS "-static-libstdc++ -static-libgcc"
    # )

    # 루트 전용: Windows 서브시스템 설정 dll 에서는 절대 콘솔
    # --subsystem,windows가 DLL에 적용되면 
    # main()/DllMain() 진입점 설정이 꼬일 수 있어요.
    # 이 설정은 콘솔/GUI 동작과 무관하게 라이브러리에서 불필요하며, 
    # 종종 test_byul.exe 실행 시 콘솔이 열리지 않는 문제를 유발합니다.
    # target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--subsystem,windows")

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
    # 루트 전용: 설치 경로 설정
    set(CMAKE_INSTALL_PREFIX "${HOME_DIR}/${PROJECT_NAME}" 
        CACHE PATH "Install path" FORCE)
endif()

# ─────────────────────────────────────────────
# 루트 전용: 빌드 경로 관리
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
# 루트 전용: 헤더 복사 매크로 정의
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
# 루트 전용: 하위 모듈 추가
# ─────────────────────────────────────────────
add_subdirectory(common)
add_subdirectory(console)
add_subdirectory(navsys)
add_subdirectory(balix)
add_subdirectory(gpu_comp_tester)
add_subdirectory(entity)
add_subdirectory(projectile)

# ─────────────────────────────────────────────
# 루트 전용: 헤더 복사
# ─────────────────────────────────────────────
copy_headers(common)
copy_headers(console)
copy_headers(navsys)
copy_headers(balix)
copy_headers(gpu_comp_tester)
copy_headers(entity)
copy_headers(projectile)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h 
    DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})
file(COPY byul_config.h DESTINATION ${${PROJECT_NAME}_BUILD_INCLUDE_DIR})

# ─────────────────────────────────────────────
# include 경로 설정 (필요 시 활성화)
# ─────────────────────────────────────────────
# target_include_directories(${PROJECT_NAME} PRIVATE
#     ${CMAKE_SOURCE_DIR}
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${${PROJECT_NAME}_BUILD_INCLUDE_DIR}
#     ${${PROJECT_NAME}_BUILD_INTERNAL_DIR})

# ─────────────────────────────────────────────
# 루트 전용: 라이브러리 링크 (whole-archive)
# ─────────────────────────────────────────────
target_link_libraries(${PROJECT_NAME} PRIVATE
    -Wl,--whole-archive
    common
    console
    navsys
    balix
    entity
    -Wl,--no-whole-archive    
)

# ─────────────────────────────────────────────
# 루트 전용: 설치 관련 설정
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


# ─────────────────────────────────────────────
# Windows MinGW DLL 자동 복사 (선택 사항)
# ─────────────────────────────────────────────
if (WIN32 AND NOT MSVC)
    # 교차 빌드 시 필요한 DLL을 자동 복사
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "/usr/lib/gcc/x86_64-w64-mingw32/13-win32/libstdc++-6.dll"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "/usr/lib/gcc/x86_64-w64-mingw32/13-win32/libgcc_s_seh-1.dll"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "/usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )
endif()    

set(MANDATORY_DLL
    ${CMAKE_BINARY_DIR}/libstdc++-6.dll
    ${CMAKE_BINARY_DIR}/libgcc_s_seh-1.dll
    ${CMAKE_BINARY_DIR}/libwinpthread-1.dll
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    install(FILES
        ${MANDATORY_DLL}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()

add_subdirectory(tests)

