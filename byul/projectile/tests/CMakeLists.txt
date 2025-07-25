cmake_minimum_required(VERSION 3.20)

project(test_projectile)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# 소스 파일 설정
set(SOURCES
    main.cpp
    test_module_projectile_common.cpp    
    test_module_propulsion.cpp    
    test_module_guidance.cpp    
    test_module_projectile_predict.cpp    
)

# ------------------- 테스트 실행 파일 -------------------
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

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

# 외부 include 경로 설정
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../external)
message(STATUS "${PROJECT_NAME} : EXTERNAL_DIR = ${EXTERNAL_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/include/byul
    ${EXTERNAL_DIR}
)

# ------------------- 정적 라이브러리 연결 -------------------
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_BINARY_DIR}/projectile
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    projectile
    entity
    m
)

# coord는 STATIC 라이브러리이므로 필요 시 심볼 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE BYUL_STATIC)

# ------------------- CTest -------------------
enable_testing()
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

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