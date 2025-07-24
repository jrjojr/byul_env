cmake_minimum_required(VERSION 3.20)
project(test_byul)

# ─────────────────────────────────────────────
# 기본 설정
# ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────
# 테스트 소스 초기화
# ─────────────────────────────────────────────
set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
set(TEST_INC_DIRS "")

# ─────────────────────────────────────────────
# 모듈별 테스트 소스 및 헤더 추가
# ─────────────────────────────────────────────
function(add_module MODULE_NAME)
    string(TOUPPER ${MODULE_NAME} UPPER_NAME)
    option(TEST_${UPPER_NAME} "Enable ${MODULE_NAME} module tests" ON)
    set(MODULE_DIR "${CMAKE_SOURCE_DIR}/${MODULE_NAME}")

    if(TEST_${UPPER_NAME} AND EXISTS "${MODULE_DIR}")
        file(GLOB_RECURSE MODULE_TESTS "${MODULE_DIR}/**/test_module_*.cpp")
        if(MODULE_TESTS)
            message(STATUS "[TEST] Adding tests from ${MODULE_NAME}:")
            foreach(TEST_SRC ${MODULE_TESTS})
                message(STATUS "   ${TEST_SRC}")
            endforeach()
            list(APPEND TEST_SOURCES ${MODULE_TESTS})
            set(TEST_SOURCES "${TEST_SOURCES}" PARENT_SCOPE)
        else()
            message(STATUS "[TEST] No test_module_*.cpp found in ${MODULE_NAME}")
        endif()

        file(GLOB_RECURSE MODULE_HEADERS "${MODULE_DIR}/**/*.h" "${MODULE_DIR}/**/*.hpp")
        foreach(HDR ${MODULE_HEADERS})
            get_filename_component(HDR_DIR ${HDR} DIRECTORY)
            list(APPEND TEST_INC_DIRS ${HDR_DIR})
        endforeach()
        list(REMOVE_DUPLICATES TEST_INC_DIRS)
        set(TEST_INC_DIRS "${TEST_INC_DIRS}" PARENT_SCOPE)
    endif()
endfunction()

# 테스트 모듈 등록
add_module(common)
add_module(console)
add_module(navsys)
add_module(balix)
add_module(entity)

# ─────────────────────────────────────────────
# 테스트 실행 파일 생성
# ─────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${TEST_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE byul)

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
    ${TEST_INC_DIRS}
)

# ─────────────────────────────────────────────
# 플랫폼별 설정
# ─────────────────────────────────────────────
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux: ASan/LSan 적용
    message(STATUS "${PROJECT_NAME} Applying ASan + LSan for Linux")
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak)

    # RPATH 설정: ../lib 경로에서 .so 찾기
    message("ORIGIN : \$ORIGIN" )
    message("ORIGIN : $ORIGIN" )
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "\$ORIGIN/../"
        INSTALL_RPATH "\$ORIGIN/../lib"
    )

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "${PROJECT_NAME} No ASan for Windows (unsupported).")

else()
    message(FATAL_ERROR "⚠️ Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()

# ─────────────────────────────────────────────
# CTest 등록
# ─────────────────────────────────────────────
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

# ─────────────────────────────────────────────
# 결과물 설치
# ─────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

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
