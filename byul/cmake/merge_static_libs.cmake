# --------------------------------------------------------------
# merge_static_libraries(OUTPUT_LIB lib1 lib2 lib3 ...)
#
# OUTPUT_LIB: 최종 통합할 STATIC 라이브러리 이름
# lib1, lib2, ...: 병합할 기존 STATIC 라이브러리 타겟들
#
# 최종적으로 OUTPUT_LIB.a 안에 모든 오브젝트 파일을 포함시킴
# --------------------------------------------------------------
function(merge_static_libraries OUTPUT_LIB)
    set(libs ${ARGN})
    set(MERGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/merge_${OUTPUT_LIB})
    file(MAKE_DIRECTORY ${MERGE_DIR})

    set(merge_commands "")
    list(APPEND merge_commands
        COMMAND ${CMAKE_COMMAND} -E echo "Merging libraries into ${OUTPUT_LIB}.a ..."
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${MERGE_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${MERGE_DIR}
        COMMAND ${CMAKE_COMMAND} -E chdir ${MERGE_DIR} ${CMAKE_AR} x $<TARGET_FILE:${OUTPUT_LIB}>
    )

    foreach(lib ${libs})
        list(APPEND merge_commands
            COMMAND ${CMAKE_COMMAND} -E chdir ${MERGE_DIR} ${CMAKE_AR} x $<TARGET_FILE:${lib}>
        )
    endforeach()

    # GLOB으로 .o 파일 리스트 수집
    file(GLOB OBJ_FILES "${MERGE_DIR}/*.o")

    # ar rcs 실행
    list(APPEND merge_commands
        COMMAND ${CMAKE_COMMAND} -E echo "Merging ${OBJ_FILES}"
        COMMAND ${CMAKE_AR} rcs $<TARGET_FILE:${OUTPUT_LIB}> ${OBJ_FILES}
    )

    add_custom_command(TARGET ${OUTPUT_LIB} POST_BUILD ${merge_commands})
endfunction()
