cmake_minimum_required(VERSION 3.20)

project(gpu_comp_tester LANGUAGES C CXX)

# ----------------------------------------
# 📁 외부 디렉토리 정의
# ----------------------------------------
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

# ----------------------------------------
# 🛠️ 공통 설정
# ----------------------------------------
enable_testing()

# GLSL 셰이더 런타임 디렉토리 (define으로 넘김)
set(SHADER_DIR "${CMAKE_INSTALL_PREFIX}/glsl")
message(STATUS "SHADER_DIR : ${SHADER_DIR}")
# ----------------------------------------
# 📦 소스 목록
# ----------------------------------------
set(${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gpu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gpu_compute.cpp

CACHE INTERNAL "${PROJECT_NAME} source files"  FORCE
)

# Generate the platform-neutral OpenGL loader from the pinned glad revision
# only when the GPU tool is actually built.
set(GLAD_SOURCE_DIR "${EXTERNAL_DIR}/glad-2.0.8")
if(NOT EXISTS "${GLAD_SOURCE_DIR}/glad/__main__.py")
    message(FATAL_ERROR
        "Vendored glad source is missing. Run: "
        "git submodule update --init external/glad-2.0.8"
    )
endif()

find_package(Python COMPONENTS Interpreter QUIET)
if(Python_Interpreter_FOUND)
    set(GLAD_PYTHON_COMMAND "${Python_EXECUTABLE}")
elseif(WIN32)
    set(GLAD_PYTHON_COMMAND py -3)
else()
    set(GLAD_PYTHON_COMMAND python3)
endif()

set(GLAD_GENERATED_DIR "${CMAKE_BINARY_DIR}/external/glad/generated")
set(GLAD_GENERATED_SOURCE "${GLAD_GENERATED_DIR}/src/gl.c")
set(GLAD_GENERATED_HEADER "${GLAD_GENERATED_DIR}/include/glad/gl.h")
set(GLAD_KHR_HEADER "${GLAD_GENERATED_DIR}/include/KHR/khrplatform.h")

add_custom_command(
    OUTPUT
        "${GLAD_GENERATED_SOURCE}"
        "${GLAD_GENERATED_HEADER}"
        "${GLAD_KHR_HEADER}"
    COMMAND ${GLAD_PYTHON_COMMAND} -m glad
        --out-path "${GLAD_GENERATED_DIR}"
        --api gl:core=4.6
        --reproducible
        c
    WORKING_DIRECTORY "${GLAD_SOURCE_DIR}"
    DEPENDS
        "${GLAD_SOURCE_DIR}/glad/__main__.py"
        "${GLAD_SOURCE_DIR}/glad/files/gl.xml"
        "${GLAD_SOURCE_DIR}/glad/files/khrplatform.h"
    COMMENT "Generating reproducible OpenGL 4.6 loader with glad 2.0.8"
    VERBATIM
)

add_library(byul_glad STATIC
    "${GLAD_GENERATED_SOURCE}"
    "${GLAD_GENERATED_HEADER}"
    "${GLAD_KHR_HEADER}"
)
target_include_directories(byul_glad PUBLIC
    "${GLAD_GENERATED_DIR}/include"
)
target_link_libraries(byul_glad PUBLIC ${CMAKE_DL_LIBS})

# ----------------------------------------
# 🔧 바이너리 생성
# ----------------------------------------
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

if(WIN32)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/gpu_comp_tester.rc
    )
endif()

# ─────────────────────────────────────────────
# 플랫폼별 설정
# ─────────────────────────────────────────────
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "${PROJECT_NAME}: Applying ASan + LSan for Linux")
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak -g -O1)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address -fsanitize=leak)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "\$ORIGIN/../lib"
        INSTALL_RPATH "\$ORIGIN/../lib"
    )
    # ubuntu: 실행파일 : bin, so : lib 디렉토리에 있어야 함
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}
    )    
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "${PROJECT_NAME}: No ASan for Windows (unsupported).")

    # Windows: exe와 dll이 항상 같은 bin 디렉토리에 있어야 함
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}
    )

else()
    message(FATAL_ERROR "⚠️ Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()

# ----------------------------------------
# 🧭 SDL3 source dependency shared by every platform
# ----------------------------------------
set(SDL3_SOURCE_DIR "${EXTERNAL_DIR}/SDL3-3.2.18/source")
if(NOT EXISTS "${SDL3_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "Vendored SDL3 source is missing. Run: "
        "git submodule update --init external/SDL3-3.2.18/source"
    )
endif()

set(SDL_TESTS OFF CACHE BOOL "Build SDL tests" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "Build SDL examples" FORCE)
add_subdirectory(
    "${SDL3_SOURCE_DIR}"
    "${CMAKE_BINARY_DIR}/external/SDL3"
    EXCLUDE_FROM_ALL
)

target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SDL3=1)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3)
message(STATUS "Using vendored SDL3 source: ${SDL3_SOURCE_DIR}")

if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:SDL3::SDL3>"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )
endif()

# ----------------------------------------
# 🔍 include 디렉토리
# ----------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/numal

    ${CMAKE_SOURCE_DIR}/navsys/dstar_lite

    ${CMAKE_SOURCE_DIR}/balix/numal
    ${CMAKE_SOURCE_DIR}/balix/trajectory

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
)

# ----------------------------------------
# 🔗 라이브러리 링크
# ----------------------------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/balix
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(${PROJECT_NAME} PRIVATE
        byul_glad
        opengl32
        balix
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        byul_glad
        GL
        balix
        m
    )
endif()

# ----------------------------------------
# 🧾 컴파일 define
# ----------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    BYUL_STATIC
)

# ----------------------------------------
# 📁 GLSL 셰이더 파일
# ----------------------------------------
set(GPU_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glsl)
set(GPU_SHADER_FILES
    ${GPU_SHADER_DIR}/clock.comp
    ${GPU_SHADER_DIR}/nlerp.comp
    ${GPU_SHADER_DIR}/pulse.comp
    ${GPU_SHADER_DIR}/lissajous.comp
    ${GPU_SHADER_DIR}/shader.vert
    ${GPU_SHADER_DIR}/shader.frag
)

# ----------------------------------------
# GLSL 셰이더 복사 (CMAKE_BINARY_DIR/glsl)
# ----------------------------------------
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/glsl)

file(COPY ${GPU_SHADER_FILES}
    DESTINATION ${CMAKE_BINARY_DIR}/glsl
)
