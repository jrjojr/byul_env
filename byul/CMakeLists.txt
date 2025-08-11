cmake_minimum_required(VERSION 3.20)

project(byul VERSION 1.0 LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Build type and basic settings
# -----------------------------------------------------------------------------
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile DB (VSCode IntelliSense)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Position Independent Code required when building DLLs
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Debug mode definition
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    message(STATUS "DEBUG mode enabled")
endif()

# -----------------------------------------------------------------------------
# MSVC specific settings
# -----------------------------------------------------------------------------
if (MSVC)
    message(STATUS "MSVC build detected")

    # Standard exception handling and multi-threaded compilation
    add_compile_options(/EHsc /MP /W3)

    # DLL, LIB, EXE output directories
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME})
endif()

# -----------------------------------------------------------------------------
# Testing support
# -----------------------------------------------------------------------------
enable_testing()

# -----------------------------------------------------------------------------
# Add modules (STATIC libraries)
# -----------------------------------------------------------------------------
add_subdirectory(common)
add_subdirectory(console)
add_subdirectory(navsys)
add_subdirectory(balix)
add_subdirectory(entity)
add_subdirectory(projectile)
add_subdirectory(gpu_comp_tester)
add_subdirectory(byul_tick)
add_subdirectory(ground)

set(${PROJECT_NAME}_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/byul.cpp

    ${byul_tick_SOURCES}
    ${common_SOURCES}
    ${console_SOURCES}
    ${navsys_SOURCES}
    ${balix_SOURCES}
    ${entity_SOURCES}
    ${projectile_SOURCES}
    ${ground_SOURCES}


CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/byul.h
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_common.h

    ${byul_tick_HEADERS}
    ${common_HEADERS}
    ${console_HEADERS}
    ${navsys_HEADERS}
    ${balix_HEADERS}
    ${entity_HEADERS}
    ${projectile_HEADERS}
    ${ground_HEADERS}

CACHE INTERNAL "${PROJECT_NAME} header files"  FORCE
)

# -----------------------------------------------------------------------------
# DLL 생성
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})

target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_EXPORTS)

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

include(${CMAKE_SOURCE_DIR}/cmake/project_manager.cmake)
extract_header_dirs(common_HEADERS  COMMON_HDR_DIRS)
extract_header_dirs(console_HEADERS  CONSOLE_HDR_DIRS)

extract_header_dirs(navsys_HEADERS  NAVSYS_HDR_DIRS)
extract_header_dirs(balix_HEADERS  BALIX_HDR_DIRS)

extract_header_dirs(entity_HEADERS  ENTITY_HDR_DIRS)
extract_header_dirs(projectile_HEADERS  PROJ_HDR_DIRS)

# -----------------------------------------------------------------------------
# include 디렉토리 추가 (최상위 소스 + 하위 헤더 경로)
# -----------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}

    ${CMAKE_SOURCE_DIR}/ground
    
    ${EXTERNAL_DIR}

    ${CMAKE_SOURCE_DIR}/byul_tick
    
    ${COMMON_HDR_DIRS}
    ${CONSOLE_HDR_DIRS}

    ${NAVSYS_HDR_DIRS}
    ${BALIX_HDR_DIRS}
    ${ENTITY_HDR_DIRS}
    ${PROJ_HDR_DIRS}
)

add_subdirectory(tests)

# -----------------------------------------------------------------------------
# Platform-specific settings and DLL handling
# -----------------------------------------------------------------------------
message(STATUS "CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
message(STATUS "WIN32 = ${WIN32}")
message(STATUS "UNIX = ${UNIX}")
message(STATUS "CMAKE_CXX_COMPILER_ID = ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "TARGET_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")

if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Cross compile for Win11 on Ubuntu using MinGW
    set(HOME_DIR "$ENV{HOME}/cross_win")
    message(STATUS "Cross compile for Win11 on Ubuntu (HOME_DIR=${HOME_DIR})")

    set(MINGW_DLL_PATH "/usr/lib/gcc/x86_64-w64-mingw32/13-win32")
    set(MINGW_PTHREAD_DLL "/usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll")

elseif (WIN32)
    # Windows native build
    set(HOME_DIR "$ENV{USERPROFILE}")
    message(STATUS "Windows build (HOME_DIR=${HOME_DIR})")

    message(STATUS "${PROJECT_NAME} No ASan for Windows (unsupported).")

    if (NOT MSVC)
        # MSYS2 MinGW path configuration
        set(MINGW_DLL_PATH "C:/msys64/mingw64/bin")
        set(MINGW_PTHREAD_DLL "C:/msys64/mingw64/bin/libwinpthread-1.dll")
    endif()

    # Windows: exe and dll must be in the same bin directory
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}
    )    

elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(HOME_DIR "$ENV{HOME}")
    message(STATUS "Ubuntu build (HOME_DIR=${HOME_DIR})")

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "${PROJECT_NAME} Applying ASan + LSan (Debug only)")
        target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
        target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak)
    else()
        message(STATUS "${PROJECT_NAME} Release mode: no sanitizer")
        target_compile_options(${PROJECT_NAME} PRIVATE -O3 -DNDEBUG -g0)
        target_link_options(${PROJECT_NAME} PRIVATE -s)
    endif()
    
else()
    message(FATAL_ERROR "Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()

# -----------------------------------------------------------------------------
# MinGW DLL copy/install (Windows or cross-compile)
# -----------------------------------------------------------------------------
if (NOT MSVC AND DEFINED MINGW_DLL_PATH)
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

# -----------------------------------------------------------------------------
# Install common outputs
# -----------------------------------------------------------------------------
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

install(FILES ${${PROJECT_NAME}_HEADERS}
    DESTINATION include/${CMAKE_PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Uninstall target
# -----------------------------------------------------------------------------
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in
    ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)

# -----------------------------------------------------------------------------
# ZIP packaging
# -----------------------------------------------------------------------------
set(PACKAGE_DIR "${CMAKE_BINARY_DIR}/package_tmp")

message(STATUS "PACKAGE_DIR : ${PACKAGE_DIR}")
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

# Find ZIP command
find_program(ZIP_EXECUTABLE zip)
if(NOT ZIP_EXECUTABLE)
    message(FATAL_ERROR "zip command not found. Please install 'zip' (MSYS2/MinGW: pacman -S zip).")
endif()

set(INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Remove drive letter (e.g., C:/Users/critl/byul -> Users/critl/byul)
string(REGEX REPLACE "^[A-Za-z]:/" "" INSTALL_PREFIX_NO_DRIVE "${INSTALL_PREFIX}")

message(STATUS "CMAKE_INSTALL_PREFIX (drive removed): ${INSTALL_PREFIX_NO_DRIVE}")

message(STATUS "package_tmp/INSTALL_PREFIX_NO_DRIVE : package_tmp/${INSTALL_PREFIX_NO_DRIVE}")
set (ZIP_INNER "${INSTALL_PREFIX_NO_DRIVE}")
message(STATUS "ZIP_INNER : ${ZIP_INNER}")

# -----------------------------------------------------------------------------
# Remove package_tmp
# -----------------------------------------------------------------------------
add_custom_target(package_clean
    COMMAND ${CMAKE_COMMAND} -E echo "[CLEAN] Removing ${PACKAGE_DIR}..."
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${PACKAGE_DIR}"
    COMMENT "[CLEAN] package_tmp deleted"
)

# -----------------------------------------------------------------------------
# Install step (CMake Install)
# -----------------------------------------------------------------------------
add_custom_target(package_install
    COMMAND ${CMAKE_COMMAND} -E echo "[INSTALL] Creating ${PACKAGE_DIR}..."
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PACKAGE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env DESTDIR="${PACKAGE_DIR}"
        ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_install.cmake
    DEPENDS package_clean
    COMMENT "[INSTALL] package_tmp created and files installed"
)

# -----------------------------------------------------------------------------
# ZIP creation step
# -----------------------------------------------------------------------------
add_custom_target(package_zip
    COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Removing old zip file..."
    COMMAND ${CMAKE_COMMAND} -E remove -f "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.zip"
    COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Compressing ${PROJECT_NAME}/bin folder..."
    COMMAND ${ZIP_EXECUTABLE} -r "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.zip" .
        WORKING_DIRECTORY "${PACKAGE_DIR}/${ZIP_INNER}/../"
    DEPENDS package_install
    COMMENT "[ZIP] ${PROJECT_NAME}.zip created"
)
