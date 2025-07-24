cmake_minimum_required(VERSION 3.20)

project(byul VERSION 1.0 LANGUAGES CXX)

# ─────────────────────────────────────────────
# 빌드 타입 및 기본 설정
# ─────────────────────────────────────────────
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile DB (VSCode IntelliSense)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DLL 빌드시 Position Independent Code 필수
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Debug 모드 정의
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# ─────────────────────────────────────────────
# 테스트 지원
# ─────────────────────────────────────────────
enable_testing()

# ─────────────────────────────────────────────
# 공용 출력 경로 설정
# ─────────────────────────────────────────────
set(BYUL_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME})
set(BYUL_INTERNAL_DIR ${BYUL_INCLUDE_DIR}/internal)
set(BYUL_LIB_DIR ${CMAKE_BINARY_DIR}/lib)

file(MAKE_DIRECTORY ${BYUL_INCLUDE_DIR})
file(MAKE_DIRECTORY ${BYUL_INTERNAL_DIR})
file(MAKE_DIRECTORY ${BYUL_LIB_DIR})

# ─────────────────────────────────────────────
# 모듈 추가 (STATIC 라이브러리)
# ─────────────────────────────────────────────
add_subdirectory(common)
add_subdirectory(console)
add_subdirectory(navsys)
add_subdirectory(balix)
add_subdirectory(entity)
add_subdirectory(projectile)
add_subdirectory(gpu_comp_tester)

# ─────────────────────────────────────────────
# 최종 DLL 생성
# ─────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_EXPORTS)

# tests는 ${PROJECT_NAME} 라이브러리가 필요하다
add_subdirectory(tests)

set(SUB_MODULES
    common
    console
    navsys
    balix
    entity
    projectile
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(${PROJECT_NAME} PRIVATE
        -Wl,--whole-archive
        ${SUB_MODULES}
        -Wl,--no-whole-archive
    )
elseif (MSVC)
    foreach(module ${SUB_MODULES})
        set(MSVC_WHOLEARCHIVE_FLAGS "${MSVC_WHOLEARCHIVE_FLAGS} /WHOLEARCHIVE:${module}")
    endforeach()
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${MSVC_WHOLEARCHIVE_FLAGS}")
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SUB_MODULES})
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SUB_MODULES})
endif()

# ─────────────────────────────────────────────
# 헤더 설치 및 복사
# ─────────────────────────────────────────────
set(HEADER_MODULES
    common
    console
    navsys
    balix
    entity
    projectile
    gpu_comp_tester    
)

foreach(module ${HEADER_MODULES})
    file(GLOB_RECURSE HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/${module}/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/${module}/*.hpp"
    )
    foreach(hdr ${HEADERS})
        configure_file(${hdr} ${BYUL_INTERNAL_DIR} COPYONLY)
    endforeach()    
endforeach()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h 
    DESTINATION ${BYUL_INCLUDE_DIR})
file(COPY byul_common.h DESTINATION ${BYUL_INCLUDE_DIR})

# ─────────────────────────────────────────────
# 플랫폼별 설정 및 DLL 처리
# ─────────────────────────────────────────────
message(STATUS "CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
message(STATUS "WIN32 = ${WIN32}")
message(STATUS "UNIX = ${UNIX}")
message(STATUS "CMAKE_CXX_COMPILER_ID = ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "TARGET_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")

if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Ubuntu에서 MinGW로 Win11 교차 컴파일
    set(HOME_DIR "$ENV{HOME}/cross_win")
    message(STATUS "🔧 Cross compile for Win11 on Ubuntu (HOME_DIR=${HOME_DIR})")

    set(MINGW_DLL_PATH "/usr/lib/gcc/x86_64-w64-mingw32/13-win32")
    set(MINGW_PTHREAD_DLL "/usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll")

elseif (WIN32)
    # Win11 네이티브 빌드
    set(HOME_DIR "$ENV{USERPROFILE}")
    message(STATUS "🏠 Windows build (HOME_DIR=${HOME_DIR})")

    message(STATUS "${PROJECT_NAME} No ASan for Windows (unsupported).")

    if (NOT MSVC)
        # MSYS2 MinGW 경로를 사용해야 한다면 여기서 수정 가능
        set(MINGW_DLL_PATH "C:/msys64/mingw64/bin")
        set(MINGW_PTHREAD_DLL "C:/msys64/mingw64/bin/libwinpthread-1.dll")
    endif()

    # Windows: exe와 dll이 항상 같은 bin 디렉토리에 있어야 함
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}
    )    

elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Ubuntu 네이티브 빌드
    set(HOME_DIR "$ENV{HOME}")
    message(STATUS "🐧 Ubuntu build (HOME_DIR=${HOME_DIR})")

    # ASan/LSan 적용
    message(STATUS "${PROJECT_NAME} Applying ASan + LSan for Linux")
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak)

    # # RPATH 설정: ../lib 경로에서 .so 찾기
    #  현재 프로젝트가 dll이면 설정하지만 실행파일에서 찾아야지
    # set_target_properties(${PROJECT_NAME} PROPERTIES
    #     BUILD_RPATH "\$ORIGIN/../"
    #     INSTALL_RPATH "\$ORIGIN/../lib"        
    # )

else()
    message(FATAL_ERROR "⚠️ Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()

# ─────────────────────────────────────────────
# MinGW DLL 복사/설치 (Windows 또는 Cross-compile 공통)
# ─────────────────────────────────────────────
if (DEFINED MINGW_DLL_PATH)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MINGW_DLL_PATH}/libstdc++-6.dll"
            "${CMAKE_BINARY_DIR}/bin/libstdc++-6.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MINGW_DLL_PATH}/libgcc_s_seh-1.dll"
            "${CMAKE_BINARY_DIR}/bin/libgcc_s_seh-1.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MINGW_PTHREAD_DLL}"
            "${CMAKE_BINARY_DIR}/bin/libwinpthread-1.dll"
    )

    set(MANDATORY_DLL
        ${CMAKE_BINARY_DIR}/bin/libstdc++-6.dll
        ${CMAKE_BINARY_DIR}/bin/libgcc_s_seh-1.dll
        ${CMAKE_BINARY_DIR}/bin/libwinpthread-1.dll
    )
    install(FILES ${MANDATORY_DLL} DESTINATION bin)
endif()

set(CMAKE_INSTALL_PREFIX "${HOME_DIR}/${PROJECT_NAME}" 
    CACHE PATH "Install path" FORCE)

# ─────────────────────────────────────────────
# 공통 결과물 설치
# ─────────────────────────────────────────────
if(WIN32)
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
    )
else()
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
    )
endif()

install(DIRECTORY ${BYUL_INCLUDE_DIR}/
    DESTINATION include)

# ─────────────────────────────────────────────
# uninstall 타겟
# ─────────────────────────────────────────────
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in
    ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)

# ─────────────────────────────────────────────
# ZIP 패키징 설정
# ─────────────────────────────────────────────
set(PACKAGE_DIR "${CMAKE_BINARY_DIR}/package_tmp")

message(STATUS "PACKAGE_DIR : ${PACKAGE_DIR}")
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

# ZIP 명령어 찾기
find_program(ZIP_EXECUTABLE zip)
if(NOT ZIP_EXECUTABLE)
    message(FATAL_ERROR "zip 명령을 찾을 수 없습니다. \n  MSYS2/MinGW 환경에서 'pacman -S zip'으로 설치해주세요.")
endif()

set(INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# 드라이브 문자 제거 (예: C:/Users/critl/byul → Users/critl/byul)
string(REGEX REPLACE "^[A-Za-z]:/" "" INSTALL_PREFIX_NO_DRIVE "${INSTALL_PREFIX}")

message(STATUS "CMAKE_INSTALL_PREFIX(드라이브 제거): ${INSTALL_PREFIX_NO_DRIVE}")

message(STATUS "package_tmp/INSTALL_PREFIX_NO_DRIVE : package_tmp/${INSTALL_PREFIX_NO_DRIVE}")
set (ZIP_INNER "${INSTALL_PREFIX_NO_DRIVE}")
message(STATUS "ZIP_INNER : ${ZIP_INNER}")

# ─────────────────────────────────────────────
# package_tmp 삭제
# ─────────────────────────────────────────────
add_custom_target(package_clean
    COMMAND ${CMAKE_COMMAND} -E echo "[CLEAN] Removing ${PACKAGE_DIR}..."
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${PACKAGE_DIR}"
    COMMENT "[CLEAN] package_tmp deleted"
)

# ─────────────────────────────────────────────
# 설치 단계 (CMake Install)
# ─────────────────────────────────────────────
add_custom_target(package_install
    COMMAND ${CMAKE_COMMAND} -E echo "[INSTALL] Creating ${PACKAGE_DIR}..."
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PACKAGE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env DESTDIR="${PACKAGE_DIR}"
        ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_install.cmake
    DEPENDS package_clean
    COMMENT "[INSTALL] package_tmp created and files installed"
)

# ─────────────────────────────────────────────
# ZIP 생성 단계
# ─────────────────────────────────────────────
add_custom_target(package_zip
    COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Removing old zip file..."
    COMMAND ${CMAKE_COMMAND} -E remove -f "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.zip"
    COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Compressing byul/bin folder..."
    COMMAND ${ZIP_EXECUTABLE} -r "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.zip" .
        WORKING_DIRECTORY "${PACKAGE_DIR}/${ZIP_INNER}/../"
    DEPENDS package_install
    COMMENT "[ZIP] ${PROJECT_NAME}.zip created"
)
