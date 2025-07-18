cmake_minimum_required(VERSION 3.20)

project(gpu_comp_tester LANGUAGES C CXX)

# ----------------------------------------
# 📁 외부 디렉토리 정의
# ----------------------------------------
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "EXTERNAL_DIR = ${EXTERNAL_DIR}")


# ✅ C++17 이상 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # gnu++17이 아닌 순수 c++17 사용

# ----------------------------------------
# 🧭 플랫폼 확인 및 SDL2 / SDL3 자동 분기
# ----------------------------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(SDL2 REQUIRED)
    set(SDL_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS})
    set(SDL_LIBRARIES ${SDL2_LIBRARIES})
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    find_package(SDL3 CONFIG REQUIRED)
    set(SDL_INCLUDE_DIRS ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/include)
    set(SDL_LIBRARIES ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/bin/SDL3.dll)
    message(STATUS "🪟 SDL3 경로 사용: ${SDL_INCLUDE_DIRS}")
     message(STATUS "SDL_LIBRARIES : ${SDL_LIBRARIES}")

    # ✅ SDL3용 매크로 정의
    add_compile_definitions(USE_SDL3=1)

    # 정적 링크
    # 이유는 우분투에서 빌드해서 실행할때는 이미 모든 so가 이미 준비되어있다
    # 우분투 자체에 이미 설치되어있다
    # 하지만 win11용으로 빌드할때는 libstdc++.dll이 필요하다 따로 관리할수 없으니
    # 통합해서 다른 dll이 필요없게 만들어야 한다.
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")    
else()
    message(FATAL_ERROR "⚠️ Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()


# ----------------------------------------
# 🛠️ 공통 설정
# ----------------------------------------
enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

# ----------------------------------------
# 🔍 include 디렉토리
# ----------------------------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
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
    ${CMAKE_BINARY_DIR}/numal
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDL_LIBRARIES}
        opengl32
        numal
        m
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${SDL_LIBRARIES}
        GL
        numal
        m
    )
endif()

# ----------------------------------------
# 🧾 컴파일 define
# ----------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    BYUL_STATIC
    # SHADER_DIR="${SHADER_DIR}"
)

# ----------------------------------------
# 📁 GLSL 셰이더 파일
# ----------------------------------------
set(GPU_SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/clock.comp
    ${CMAKE_CURRENT_SOURCE_DIR}/shader.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/shader.frag
)

# ----------------------------------------
# 📦 설치 설정
# ----------------------------------------
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)

install(FILES ${GPU_SHADER_FILES}
    DESTINATION ${SHADER_DIR}
)

# SDL DLL (Windows에서만 설치)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    install(FILES
        ${EXTERNAL_DIR}/SDL3-3.2.18/x86_64-w64-mingw32/bin/SDL3.dll
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
    install(FILES
        ${EXTERNAL_DIR}/SDL3-3.2.18/LICENSE.txt
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/sdl
    )
endif()
