cmake_minimum_required(VERSION 3.20)
project(byul_sdk_consumer LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    PATHS
        "${BYUL_SDK_ROOT}/lib/byul"
        "${BYUL_SDK_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)

add_executable(byul_sdk_consumer main.c)
add_executable(byul_sdk_consumer_cpp main.cpp)
add_executable(byul_navgrid_header_consumer navgrid_header_main.c)
add_executable(byul_navgrid_header_consumer_cpp navgrid_header_main.cpp)
set(BYUL_CONSUMER_TARGETS
    byul_sdk_consumer
    byul_sdk_consumer_cpp
    byul_navgrid_header_consumer
    byul_navgrid_header_consumer_cpp
)

if(BYUL_ABI1_COMPAT_ROOT)
    find_path(BYUL_ABI1_COMPAT_INCLUDE_DIR
        NAMES navgrid_abi1.h
        PATHS "${BYUL_ABI1_COMPAT_ROOT}/include/byul/compat/abi1"
        NO_DEFAULT_PATH
        REQUIRED
    )
    add_executable(byul_sdk_abi1_consumer abi1_main.c)
    add_executable(byul_sdk_abi1_consumer_cpp abi1_main.cpp)
    list(APPEND BYUL_CONSUMER_TARGETS
        byul_sdk_abi1_consumer
        byul_sdk_abi1_consumer_cpp
    )
    target_include_directories(byul_sdk_abi1_consumer
        PRIVATE "${BYUL_ABI1_COMPAT_INCLUDE_DIR}")
    target_include_directories(byul_sdk_abi1_consumer_cpp
        PRIVATE "${BYUL_ABI1_COMPAT_INCLUDE_DIR}")
endif()

foreach(consumer IN LISTS BYUL_CONSUMER_TARGETS)
    target_include_directories(${consumer} PRIVATE "${BYUL_INCLUDE_DIR}")
    target_link_libraries(${consumer} PRIVATE "${BYUL_IMPORT_LIBRARY}")
    target_compile_options(${consumer} PRIVATE
        $<$<COMPILE_LANG_AND_ID:C,MSVC>:/utf-8>
        $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/utf-8>
    )
    if(UNIX AND NOT APPLE)
        set_target_properties(${consumer} PROPERTIES
            BUILD_RPATH "${BYUL_SDK_ROOT}/lib"
        )
    endif()
endforeach()

if(WIN32)
    find_file(BYUL_RUNTIME_LIBRARY
        NAMES byul.dll libbyul.dll
        PATHS "${BYUL_SDK_ROOT}/bin"
        NO_DEFAULT_PATH
        REQUIRED
    )
    foreach(consumer IN LISTS BYUL_CONSUMER_TARGETS)
        add_custom_command(TARGET ${consumer} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${BYUL_RUNTIME_LIBRARY}"
                "$<TARGET_FILE_DIR:${consumer}>"
        )
    endforeach()
endif()
