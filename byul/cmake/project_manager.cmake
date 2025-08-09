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
