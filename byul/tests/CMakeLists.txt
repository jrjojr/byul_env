cmake_minimum_required(VERSION 3.20)
project(test_byul)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/project_manager.cmake)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# ─────────────────────────────────────────────
# 테스트 소스 초기화
# ─────────────────────────────────────────────
set(TEST_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  
)

set(TEST_INC_DIRS "")

# ─────────────────────────────────────────────
# 모듈별 테스트 소스 및 헤더 추가
# ─────────────────────────────────────────────
function(add_module MODULE_NAME)
    string(TOUPPER ${MODULE_NAME} UPPER_NAME)
    option(TEST_${UPPER_NAME} "Enable ${MODULE_NAME} module tests" ON)
    set(MODULE_DIR "${CMAKE_SOURCE_DIR}/${MODULE_NAME}")

    if(TEST_${UPPER_NAME} AND EXISTS "${MODULE_DIR}")
        file(GLOB_RECURSE MODULE_TESTS CONFIGURE_DEPENDS "${MODULE_DIR}/**/test_module_*.cpp")
        if(MODULE_TESTS)
            message(STATUS "[TEST] ${MODULE_NAME}:")
            foreach(TEST_SRC ${MODULE_TESTS})
                message(STATUS "   ${TEST_SRC}")
            endforeach()
            list(APPEND TEST_SOURCES ${MODULE_TESTS})
            set(TEST_SOURCES "${TEST_SOURCES}" PARENT_SCOPE)
        else()
            message(STATUS "[TEST] ${MODULE_NAME}: No test_module_*.cpp found")
        endif()

        file(GLOB_RECURSE MODULE_HEADERS CONFIGURE_DEPENDS
            "${MODULE_DIR}/**/*.h" "${MODULE_DIR}/**/*.hpp")
        foreach(HDR ${MODULE_HEADERS})
            get_filename_component(HDR_DIR ${HDR} DIRECTORY)
            list(APPEND TEST_INC_DIRS ${HDR_DIR})
        endforeach()
        list(REMOVE_DUPLICATES TEST_INC_DIRS)
        set(TEST_INC_DIRS "${TEST_INC_DIRS}" PARENT_SCOPE)
    endif()
endfunction()

# 테스트 모듈 등록
add_module(core)
add_module(numal)
add_module(number_theory)
add_module(console)
add_module(navsys)
add_module(balix)
add_module(entity)
add_module(projectile)
add_module(ground)

# ─────────────────────────────────────────────
# 테스트 실행 파일 생성
# ─────────────────────────────────────────────
add_executable(${PROJECT_NAME} 
    ${TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_module_byul.cpp    
)

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME}: EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${EXTERNAL_DIR}
    ${TEST_INC_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal
    ${CMAKE_SOURCE_DIR}/number_theory
    ${CMAKE_SOURCE_DIR}/byul_tick
    ${CMAKE_SOURCE_DIR}/console
    ${CMAKE_SOURCE_DIR}/navsys
    ${CMAKE_SOURCE_DIR}/ground
    ${CMAKE_SOURCE_DIR}/balix
    ${CMAKE_SOURCE_DIR}/entity
    ${CMAKE_SOURCE_DIR}/projectile    
)

target_link_directories(${PROJECT_NAME} PRIVATE 
    # byul
    # ${CMAKE_BINARY_DIR}/core
    # ${CMAKE_BINARY_DIR}/console
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    byul
    # core
    # console
)

# ─────────────────────────────────────────────
# 플랫폼별 설정
# ─────────────────────────────────────────────
byul_apply_test_platform_settings(${PROJECT_NAME})

# ─────────────────────────────────────────────
# 공통 결과물 설치
# ─────────────────────────────────────────────
byul_install_target(${PROJECT_NAME})

# ─────────────────────────────────────────────
# CTest 등록
# ─────────────────────────────────────────────
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

# ─────────────────────────────────────────────
# 디버그 메시지
# ─────────────────────────────────────────────
message(STATUS "[TEST] Final source list:")
foreach(SRC ${TEST_SOURCES})
    message(STATUS "   ${SRC}")
endforeach()

message(STATUS "[TEST] Final include directories:")
foreach(DIR ${TEST_INC_DIRS})
    message(STATUS "   ${DIR}")
endforeach()
