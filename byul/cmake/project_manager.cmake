if(NOT DEFINED PROJECT_MANAGER_HELP_SHOWN)
    message(STATUS "=======================================================")
    message(STATUS " [CMake Project Manager 사용법]")
    message(STATUS "   1. init_project(<프로젝트명>)")
    message(STATUS "   2. add_src(<파일명>) / add_hdr(<파일명>)")
    message(STATUS "   3. add_srcs(<파일1> <파일2> ...) / add_hdrs(...)")
    message(STATUS "   4. add_sub(<모듈명>)")
    message(STATUS "   5. ${PROJECT_NAME}_SOURCES / ${PROJECT_NAME}_HEADERS 활용")
    message(STATUS "=======================================================")
    set(PROJECT_MANAGER_HELP_SHOWN ON CACHE INTERNAL "Help message displayed")
endif()

# ---------------------------------------------------------
# 공통 빌드 기본값
# ---------------------------------------------------------
macro(byul_apply_common_settings)
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_definitions(DEBUG)
        message(STATUS "DEBUG mode enabled")
    endif()

    if(MSVC)
        message(STATUS "MSVC build detected")
        add_compile_options(/EHsc /MP /W3)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME})
    endif()
endmacro()

# ---------------------------------------------------------
# 헤더 목록에서 include 디렉토리 추출
# ---------------------------------------------------------
function(byul_extract_header_dirs out_var)
    set(dirs "")

    foreach(header_list_var ${ARGN})
        if(DEFINED ${header_list_var})
            foreach(header ${${header_list_var}})
                get_filename_component(header_dir "${header}" DIRECTORY)
                list(APPEND dirs "${header_dir}")
            endforeach()
        endif()
    endforeach()

    list(REMOVE_DUPLICATES dirs)
    set(${out_var} "${dirs}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------
# 플랫폼별 출력 디렉토리
# ---------------------------------------------------------
function(byul_set_target_output_dirs target_name)
    if(WIN32)
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
            ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${target_name}
        )
    else()
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
            ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${target_name}
        )
    endif()
endfunction()

# ---------------------------------------------------------
# 라이브러리 타겟 플랫폼 설정
# ---------------------------------------------------------
function(byul_apply_library_platform_settings target_name)
    message(STATUS "CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
    message(STATUS "WIN32 = ${WIN32}")
    message(STATUS "UNIX = ${UNIX}")
    message(STATUS "CMAKE_CXX_COMPILER_ID = ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}")
    message(STATUS "TARGET_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")

    if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(HOME_DIR "$ENV{HOME}/cross_win" PARENT_SCOPE)
        set(MINGW_DLL_PATH "/usr/lib/gcc/x86_64-w64-mingw32/13-win32" PARENT_SCOPE)
        set(MINGW_PTHREAD_DLL "/usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll" PARENT_SCOPE)
        message(STATUS "Cross compile for Win11 on Ubuntu (HOME_DIR=$ENV{HOME}/cross_win)")

    elseif(WIN32)
        set(HOME_DIR "$ENV{USERPROFILE}" PARENT_SCOPE)
        message(STATUS "Windows build (HOME_DIR=$ENV{USERPROFILE})")
        message(STATUS "${target_name} No ASan for Windows (unsupported).")

        if(NOT MSVC)
            set(MINGW_DLL_PATH "C:/msys64/mingw64/bin" PARENT_SCOPE)
            set(MINGW_PTHREAD_DLL "C:/msys64/mingw64/bin/libwinpthread-1.dll" PARENT_SCOPE)
        endif()

        byul_set_target_output_dirs(${target_name})

    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(HOME_DIR "$ENV{HOME}" PARENT_SCOPE)
        message(STATUS "Ubuntu build (HOME_DIR=$ENV{HOME})")

        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            message(STATUS "${target_name} Applying ASan + LSan (Debug only)")
            target_compile_options(${target_name} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
            target_link_options(${target_name} PRIVATE -fsanitize=address -fsanitize=leak)
        else()
            message(STATUS "${target_name} Release mode: no sanitizer")
            target_compile_options(${target_name} PRIVATE -O3 -DNDEBUG -g0)
            target_link_options(${target_name} PRIVATE -s)
        endif()

    else()
        message(FATAL_ERROR "Unknown platform: ${CMAKE_SYSTEM_NAME}")
    endif()
endfunction()

# ---------------------------------------------------------
# 테스트 타겟 플랫폼 설정
# ---------------------------------------------------------
function(byul_apply_test_platform_settings target_name)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(STATUS "${target_name}: Applying ASan + LSan for Linux")
        target_compile_options(${target_name} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
        target_link_options(${target_name} PRIVATE -fsanitize=address -fsanitize=leak)
        set_target_properties(${target_name} PROPERTIES
            BUILD_RPATH "\$ORIGIN/../lib"
            INSTALL_RPATH "\$ORIGIN/../lib"
        )
        byul_set_target_output_dirs(${target_name})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        message(STATUS "${target_name}: No ASan for Windows (unsupported).")
        byul_set_target_output_dirs(${target_name})
    else()
        message(FATAL_ERROR "Unknown platform: ${CMAKE_SYSTEM_NAME}")
    endif()
endfunction()

# ---------------------------------------------------------
# install / uninstall / package 공통 처리
# ---------------------------------------------------------
function(byul_install_target target_name)
    if(WIN32)
        install(TARGETS ${target_name}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION bin
            ARCHIVE DESTINATION lib/${target_name}
        )
    else()
        install(TARGETS ${target_name}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib/${target_name}
        )
    endif()
endfunction()

function(byul_add_uninstall_target template_file)
    configure_file(
        ${template_file}
        ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)
endfunction()

function(byul_copy_mingw_runtime target_name out_var)
    if(NOT MSVC AND DEFINED MINGW_DLL_PATH)
        add_custom_command(TARGET ${target_name} POST_BUILD
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

        set(runtime_files
            ${CMAKE_BINARY_DIR}/bin/libstdc++-6.dll
            ${CMAKE_BINARY_DIR}/bin/libgcc_s_seh-1.dll
            ${CMAKE_BINARY_DIR}/bin/libwinpthread-1.dll
        )
        install(FILES ${runtime_files} DESTINATION bin)
        set(${out_var} "${runtime_files}" PARENT_SCOPE)
    else()
        set(${out_var} "" PARENT_SCOPE)
    endif()
endfunction()

function(byul_get_package_tags out_platform out_toolchain)
    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(system_name "macos")
    else()
        string(TOLOWER "${CMAKE_SYSTEM_NAME}" system_name)
    endif()
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" processor_name)

    if(processor_name MATCHES "^(amd64|x86_64|x64)$")
        set(processor_name "x86_64")
    elseif(processor_name MATCHES "^(aarch64|arm64)$")
        set(processor_name "arm64")
    endif()

    if(MSVC)
        set(toolchain_name "msvc")
    elseif(MINGW)
        set(toolchain_name "mingw")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(toolchain_name "gcc")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(toolchain_name "clang")
    else()
        string(TOLOWER "${CMAKE_CXX_COMPILER_ID}" toolchain_name)
    endif()

    set(${out_platform} "${system_name}-${processor_name}" PARENT_SCOPE)
    set(${out_toolchain} "${toolchain_name}" PARENT_SCOPE)
endfunction()

function(byul_add_sdk_zip_target package_name)
    set(PACKAGE_DIR "${CMAKE_BINARY_DIR}/package_tmp")
    byul_get_package_tags(PACKAGE_PLATFORM_TAG PACKAGE_TOOLCHAIN_TAG)
    set(PACKAGE_FILE_NAME
        "${package_name}-sdk-${PROJECT_VERSION}-${PACKAGE_PLATFORM_TAG}-${PACKAGE_TOOLCHAIN_TAG}-$<LOWER_CASE:$<CONFIG>>.zip"
    )

    message(STATUS "PACKAGE_DIR : ${PACKAGE_DIR}")
    message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

    set(INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
    string(REGEX REPLACE "^[A-Za-z]:/" "" INSTALL_PREFIX_NO_DRIVE "${INSTALL_PREFIX}")

    message(STATUS "CMAKE_INSTALL_PREFIX (drive removed): ${INSTALL_PREFIX_NO_DRIVE}")
    message(STATUS "package_tmp/INSTALL_PREFIX_NO_DRIVE : package_tmp/${INSTALL_PREFIX_NO_DRIVE}")

    set(ZIP_INNER "${INSTALL_PREFIX_NO_DRIVE}")
    message(STATUS "ZIP_INNER : ${ZIP_INNER}")

    if(CMAKE_CONFIGURATION_TYPES)
        set(PACKAGE_BUILD_TARGET ALL_BUILD)
        set(PACKAGE_BUILD_CONFIG_ARGS --config $<CONFIG>)
    else()
        set(PACKAGE_BUILD_TARGET all)
        set(PACKAGE_BUILD_CONFIG_ARGS)
    endif()

    add_custom_target(sdk_build
        COMMAND ${CMAKE_COMMAND} -E echo "[BUILD] Building all targets before packaging..."
        COMMAND ${CMAKE_COMMAND} -E env MAKEFLAGS=
            ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
            --target ${PACKAGE_BUILD_TARGET}
            ${PACKAGE_BUILD_CONFIG_ARGS}
            --parallel
        COMMENT "[BUILD] all targets are up to date"
    )

    add_custom_target(sdk_zip
        COMMAND ${CMAKE_COMMAND} -E echo "[CLEAN] Removing ${PACKAGE_DIR}..."
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${PACKAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E echo "[INSTALL] Creating ${PACKAGE_DIR}..."
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PACKAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E env DESTDIR="${PACKAGE_DIR}"
            ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_CONFIG_NAME=$<CONFIG>
            -P ${CMAKE_BINARY_DIR}/cmake_install.cmake
        COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Removing old zip file..."
        COMMAND ${CMAKE_COMMAND} -E remove -f "${CMAKE_BINARY_DIR}/${PACKAGE_FILE_NAME}"
        COMMAND ${CMAKE_COMMAND} -E echo "[ZIP] Compressing ${package_name}/bin folder..."
        COMMAND ${CMAKE_COMMAND} -E chdir "${PACKAGE_DIR}/${ZIP_INNER}/../"
            ${CMAKE_COMMAND} -E tar cf
            "${CMAKE_BINARY_DIR}/${PACKAGE_FILE_NAME}" --format=zip .
        DEPENDS sdk_build
        COMMENT "[ZIP] ${PACKAGE_FILE_NAME} created"
    )
endfunction()

function(byul_add_grid_zip_target target_name grid_version grid_root)
    if(CMAKE_CROSSCOMPILING)
        message(STATUS
            "grid_zip is unavailable while cross-compiling; "
            "cx_Freeze must run on the target operating system"
        )
        return()
    endif()

    byul_get_package_tags(PACKAGE_PLATFORM_TAG PACKAGE_TOOLCHAIN_TAG)
    set(GRID_BUILD_ROOT "${CMAKE_BINARY_DIR}/grid_package")
    set(GRID_OUTPUT "${GRID_BUILD_ROOT}/byul_grid")
    set(GRID_PACKAGE_FILE_NAME
        "byul-grid-${grid_version}-${PACKAGE_PLATFORM_TAG}-$<LOWER_CASE:$<CONFIG>>.zip"
    )

    if(WIN32)
        set(GRID_BUILD_COMMAND
            cmd /c "${grid_root}/build_byul_grid.bat"
        )
    else()
        set(GRID_BUILD_COMMAND
            sh "${grid_root}/build_byul_grid.sh"
        )
    endif()

    add_custom_target(grid_build
        COMMAND ${CMAKE_COMMAND} -E env
            "BYUL_LIBRARY_PATH=$<TARGET_FILE:${target_name}>"
            ${GRID_BUILD_COMMAND}
            --output "${GRID_OUTPUT}"
            --on-locked cancel
        DEPENDS ${target_name}
        COMMENT "[GRID] Building the standalone BYUL Grid package"
        VERBATIM
    )

    add_custom_target(grid_zip
        COMMAND ${CMAKE_COMMAND} -E remove -f
            "${CMAKE_BINARY_DIR}/${GRID_PACKAGE_FILE_NAME}"
        COMMAND ${CMAKE_COMMAND} -E tar cf
            "${CMAKE_BINARY_DIR}/${GRID_PACKAGE_FILE_NAME}"
            --format=zip byul_grid
        WORKING_DIRECTORY "${GRID_BUILD_ROOT}"
        DEPENDS grid_build
        COMMENT "[GRID] ${GRID_PACKAGE_FILE_NAME} created"
        VERBATIM
    )
endfunction()

function(byul_add_help_target)
    add_custom_target(byul_help
        COMMAND ${CMAKE_COMMAND} -E echo "BYUL build and distribution targets:"
        COMMAND ${CMAKE_COMMAND} -E echo "  sdk_build  - update SDK libraries and test executables"
        COMMAND ${CMAKE_COMMAND} -E echo "  sdk_zip    - build, stage CMake install rules, and create byul-sdk-*.zip"
        COMMAND ${CMAKE_COMMAND} -E echo "  grid_build - build the standalone BYUL Grid directory with cx_Freeze"
        COMMAND ${CMAKE_COMMAND} -E echo "  grid_zip   - build and create the portable byul-grid-*.zip"
        COMMAND ${CMAKE_COMMAND} -E echo "  uninstall  - remove files recorded by the last standard CMake install"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "Standard local SDK install (does not create a ZIP):"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --install BUILD_DIR --config Release"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --install BUILD_DIR --config Release --prefix INSTALL_DIR"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "Distribution examples:"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --build BUILD_DIR --target sdk_zip --config Release"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --build BUILD_DIR --target grid_zip --config Release"
        COMMENT "Show BYUL target help"
        VERBATIM
    )
endfunction()

# ---------------------------------------------------------
# 프로젝트 초기화
# ---------------------------------------------------------
macro(init_project proj_name)
    # set(PROJECT_NAME ${proj_name})
    set(${proj_name}_SOURCES "")
    set(${proj_name}_HEADERS "")
    set(${proj_name}_HEADERS "")
    message(STATUS "Initialized project: ${proj_name}")
endmacro()

# ---------------------------------------------------------
# 단일 소스/헤더 추가
# ---------------------------------------------------------
macro(add_src src_file)
    list(APPEND ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${src_file})
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_SOURCES)
    message(STATUS "Added source: ${src_file}")
endmacro()

macro(add_hdr hdr_file)
    list(APPEND ${PROJECT_NAME}_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/${hdr_file})
    get_filename_component(header_dir "${CMAKE_CURRENT_SOURCE_DIR}/${hdr_file}" DIRECTORY)
    list(APPEND HEADERS_DIR ${header_dir})
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)
    list(REMOVE_DUPLICATES HEADERS_DIR)
    message(STATUS "Added header: ${hdr_file}")
endmacro()

# ---------------------------------------------------------
# 다중 소스/헤더 추가
# ---------------------------------------------------------
macro(add_srcs)
    foreach(src_file ${ARGV})
        add_src(${src_file})
    endforeach()
endmacro()

macro(add_hdrs)
    foreach(hdr_file ${ARGV})
        add_hdr(${hdr_file})
    endforeach()
endmacro()

# ---------------------------------------------------------
# 하위 모듈 통합
# ---------------------------------------------------------
macro(add_sub module_name)
    set(MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${module_name})

    if (DEFINED ${module_name}_SOURCES)
        list(APPEND ${PROJECT_NAME}_SOURCES ${${module_name}_SOURCES})
    endif()

    if (DEFINED ${module_name}_HEADERS)
        list(APPEND ${PROJECT_NAME}_HEADERS ${${module_name}_HEADERS})
        foreach(header ${${module_name}_HEADERS})
            get_filename_component(header_dir "${header}" DIRECTORY)
            list(APPEND HEADERS_DIR "${header_dir}")
        endforeach()
    endif()

    list(REMOVE_DUPLICATES ${PROJECT_NAME}_SOURCES)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS_DIR)

    message(STATUS "Added submodule: ${module_name}")
endmacro()

# headers 리스트에서 디렉토리 경로를 추출하고 중복 제거
# if (DEFINED ${module_name}_HEADERS)
#     list(APPEND ${PROJECT_NAME}_HEADERS ${${module_name}_HEADERS})
#     extract_header_dirs("${module_name}_HEADERS" TMP_HEADER_DIRS)
#     list(APPEND HEADERS_DIR ${TMP_HEADER_DIRS})
# endif()

# list(REMOVE_DUPLICATES ${PROJECT_NAME}_SOURCES)
# list(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)
# list(REMOVE_DUPLICATES HEADERS_DIR)
function(extract_header_dirs header_list out_var)
    message(DEPRECATION "extract_header_dirs() is deprecated. Use byul_extract_header_dirs().")
    set(dirs "")
    foreach(header ${${header_list}})
        get_filename_component(header_dir "${header}" DIRECTORY)
        list(APPEND dirs "${header_dir}")
    endforeach()
    list(REMOVE_DUPLICATES dirs)
    set(${out_var} "${dirs}" PARENT_SCOPE)
endfunction()

# ─────────────────────────────────────────────
# 현재 디렉토리 기준 모든 test_module_*.cpp 찾기
# ─────────────────────────────────────────────
function(collect_all_test_sources OUT_SOURCES OUT_INC_DIRS)
    set(TEST_SOURCES "")
    set(TEST_INC_DIRS "")

    # 현재 디렉토리와 하위 디렉토리의 모든 test_module_*.cpp 검색
    file(GLOB_RECURSE FOUND_TESTS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/**/test_module_*.cpp")

    foreach(TEST_SRC ${FOUND_TESTS})
        message(STATUS "[TEST] Found: ${TEST_SRC}")
        list(APPEND TEST_SOURCES ${TEST_SRC})

        # 테스트 소스의 헤더 경로 추출
        get_filename_component(TEST_DIR ${TEST_SRC} DIRECTORY)
        list(APPEND TEST_INC_DIRS ${TEST_DIR})
    endforeach()

    # 중복 제거
    list(REMOVE_DUPLICATES TEST_SOURCES)
    list(REMOVE_DUPLICATES TEST_INC_DIRS)

    set(${OUT_SOURCES} "${TEST_SOURCES}" PARENT_SCOPE)
    set(${OUT_INC_DIRS} "${TEST_INC_DIRS}" PARENT_SCOPE)
endfunction()
