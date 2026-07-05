cmake_minimum_required(VERSION 3.20)

project(byul VERSION 1.0 LANGUAGES C CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project_manager.cmake)
byul_apply_common_settings()

# -----------------------------------------------------------------------------
# Testing support
# -----------------------------------------------------------------------------
enable_testing()

# -----------------------------------------------------------------------------
# Add modules
# -----------------------------------------------------------------------------
add_subdirectory(core)
add_subdirectory(numal)
add_subdirectory(rng)
add_subdirectory(number_theory)
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

    ${numal_SOURCES}
    ${byul_tick_SOURCES}
    ${core_SOURCES}
    ${number_theory_SOURCES}
    ${console_SOURCES}
    ${navsys_SOURCES}
    ${balix_SOURCES}
    ${entity_SOURCES}
    ${projectile_SOURCES}
    ${ground_SOURCES}

CACHE INTERNAL "${PROJECT_NAME} source files" FORCE
)

set(${PROJECT_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/byul.h
    ${CMAKE_CURRENT_SOURCE_DIR}/byul_config.h

    ${numal_HEADERS}
    ${byul_tick_HEADERS}
    ${core_HEADERS}
    ${number_theory_HEADERS}
    ${console_HEADERS}
    ${navsys_HEADERS}
    ${balix_HEADERS}
    ${entity_HEADERS}
    ${projectile_HEADERS}
    ${ground_HEADERS}

CACHE INTERNAL "${PROJECT_NAME} header files" FORCE
)

# -----------------------------------------------------------------------------
# Shared library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_EXPORTS)

set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

byul_extract_header_dirs(BYUL_HEADER_DIRS
    core_HEADERS
    number_theory_HEADERS
    numal_HEADERS
    byul_tick_HEADERS
    console_HEADERS
    navsys_HEADERS
    balix_HEADERS
    entity_HEADERS
    projectile_HEADERS
    ground_HEADERS
)

# -----------------------------------------------------------------------------
# Include directories
# -----------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EXTERNAL_DIR}
    ${BYUL_HEADER_DIRS}
)

add_subdirectory(tests)

# -----------------------------------------------------------------------------
# Platform-specific settings and DLL handling
# -----------------------------------------------------------------------------
byul_apply_library_platform_settings(${PROJECT_NAME})
byul_copy_mingw_runtime(${PROJECT_NAME} MANDATORY_DLL)

set(CMAKE_INSTALL_PREFIX "${HOME_DIR}/${PROJECT_NAME}"
    CACHE PATH "Install path" FORCE)

# -----------------------------------------------------------------------------
# Install common outputs
# -----------------------------------------------------------------------------
byul_install_target(${PROJECT_NAME})

install(FILES ${${PROJECT_NAME}_HEADERS}
    DESTINATION include/${CMAKE_PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Uninstall and ZIP packaging
# -----------------------------------------------------------------------------
byul_add_uninstall_target(${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in)
byul_add_package_zip_target(${PROJECT_NAME})
