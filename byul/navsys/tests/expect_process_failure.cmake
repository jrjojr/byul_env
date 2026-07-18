if(NOT DEFINED TEST_EXECUTABLE
    OR NOT DEFINED TEST_ARGUMENT
    OR NOT DEFINED EXPECTED_EXIT_CODE
    OR NOT DEFINED EXPECTED_PATTERN)
    message(FATAL_ERROR "expected-failure fixture arguments are incomplete")
endif()

execute_process(
    COMMAND "${TEST_EXECUTABLE}" "${TEST_ARGUMENT}"
    RESULT_VARIABLE actual_exit_code
    OUTPUT_VARIABLE actual_stdout
    ERROR_VARIABLE actual_stderr
)

if(NOT "${actual_exit_code}" STREQUAL "${EXPECTED_EXIT_CODE}")
    message(FATAL_ERROR
        "expected exit ${EXPECTED_EXIT_CODE}, got ${actual_exit_code}\n"
        "stdout:\n${actual_stdout}\n"
        "stderr:\n${actual_stderr}"
    )
endif()

set(actual_output "${actual_stdout}${actual_stderr}")
if(NOT actual_output MATCHES "${EXPECTED_PATTERN}")
    message(FATAL_ERROR
        "expected diagnostic was absent: ${EXPECTED_PATTERN}\n"
        "output:\n${actual_output}"
    )
endif()

message(STATUS
    "expected failure detected: exit=${actual_exit_code}, "
    "pattern=${EXPECTED_PATTERN}"
)
