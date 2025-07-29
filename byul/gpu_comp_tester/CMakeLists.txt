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
set(SOURCES
    main.cpp
    gpu.cpp
    gpu_compute.cpp
    ${EXTERNAL_DIR}/glad/src/gl.c
)

# ----------------------------------------
# 🔧 바이너리 생성
# ----------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})

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
# 🧭 플랫폼 확인 및 SDL2 / SDL3 자동 분기
# ----------------------------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(SDL2 REQUIRED)
    set(SDL_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS})
    set(SDL_LIBRARIES ${SDL2_LIBRARIES})

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(SDL3_DIR "${EXTERNAL_DIR}/SDL3-3.2.18/cmake")
    find_package(SDL3 CONFIG REQUIRED)

    set(SDL_INCLUDE_DIRS ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/include)
    set(SDL_LIBRARIES ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/bin/SDL3.dll)
    message(STATUS "🪟 SDL3 경로 사용: ${SDL_INCLUDE_DIRS}")
     message(STATUS "SDL_LIBRARIES : ${SDL_LIBRARIES}")

    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_SDL3=1)

    set(SDL3_DLL ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/bin/SDL3.dll)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL3_DLL}"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
    message(STATUS "🪟 SDL3.dll을 ${CMAKE_BINARY_DIR}/bin 으로 복사하도록 설정했습니다.")

    else()
    message(FATAL_ERROR "⚠️ Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()


# ----------------------------------------
# 🔍 include 디렉토리
# ----------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/common

    ${CMAKE_SOURCE_DIR}/navsys/dstar_lite

    ${CMAKE_SOURCE_DIR}/balix/numal
    ${CMAKE_SOURCE_DIR}/balix/trajectory

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
    ${EXTERNAL_DIR}/glad/include
    ${SDL_INCLUDE_DIRS}
)

# ----------------------------------------
# 🔗 라이브러리 링크
# ----------------------------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/balix
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDL_LIBRARIES}
        opengl32
        balix
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDL_LIBRARIES}
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
set(GPU_SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/clock.comp
    ${CMAKE_CURRENT_SOURCE_DIR}/shader.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/shader.frag
)

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

install(FILES ${GPU_SHADER_FILES}
    DESTINATION glsl
)

# SDL DLL (Windows에서만 설치)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    install(FILES
        ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/bin/SDL3.dll
        DESTINATION bin
    )
    install(FILES
        ${EXTERNAL_DIR}/SDL3-3.2.18/LICENSE.txt
        DESTINATION share/sdl
    )
endif()
