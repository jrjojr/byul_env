cmake_minimum_required(VERSION 3.20)
project(byul_sdk_consumer LANGUAGES C)

if(NOT BYUL_SDK_ROOT)
    message(FATAL_ERROR "Set BYUL_SDK_ROOT to a clean BYUL install prefix")
endif()

find_path(BYUL_INCLUDE_DIR
    NAMES byul.h
    PATHS "${BYUL_SDK_ROOT}/include/byul"
    NO_DEFAULT_PATH
    REQUIRED
)
find_library(BYUL_IMPORT_LIBRARY
    NAMES byul
    PATHS "${BYUL_SDK_ROOT}/lib/byul"
    NO_DEFAULT_PATH
    REQUIRED
)

add_executable(byul_sdk_consumer main.c)
target_include_directories(byul_sdk_consumer PRIVATE "${BYUL_INCLUDE_DIR}")
target_link_libraries(byul_sdk_consumer PRIVATE "${BYUL_IMPORT_LIBRARY}")
target_compile_options(byul_sdk_consumer PRIVATE
    $<$<C_COMPILER_ID:MSVC>:/utf-8>
)

if(WIN32)
    find_file(BYUL_RUNTIME_LIBRARY
        NAMES byul.dll libbyul.dll
        PATHS "${BYUL_SDK_ROOT}/bin"
        NO_DEFAULT_PATH
        REQUIRED
    )
    add_custom_command(TARGET byul_sdk_consumer POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${BYUL_RUNTIME_LIBRARY}"
            "$<TARGET_FILE_DIR:byul_sdk_consumer>"
    )
endif()
