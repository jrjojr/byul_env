#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "byul.h"
#include "navsys_status.h"

static_assert(NAVSYS_STATUS_OK == 0, "NAVSYS_STATUS_OK ABI");
static_assert(NAVSYS_STATUS_INVALID_ARGUMENT == -1, "NAVSYS_STATUS_INVALID_ARGUMENT ABI");
static_assert(NAVSYS_STATUS_OUT_OF_MEMORY == -2, "NAVSYS_STATUS_OUT_OF_MEMORY ABI");
static_assert(NAVSYS_STATUS_UNSUPPORTED == -3, "NAVSYS_STATUS_UNSUPPORTED ABI");
static_assert(NAVSYS_STATUS_CALLBACK_FAILED == -4, "NAVSYS_STATUS_CALLBACK_FAILED ABI");
static_assert(NAVSYS_STATUS_CORRUPT_STATE == -5, "NAVSYS_STATUS_CORRUPT_STATE ABI");
static_assert(NAVSYS_STATUS_NOT_FOUND == -6, "NAVSYS_STATUS_NOT_FOUND ABI");
static_assert(NAVSYS_STATUS_INVALIDATED == -7, "NAVSYS_STATUS_INVALIDATED ABI");
static_assert(NAVSYS_STATUS_NO_PATH == -8, "NAVSYS_STATUS_NO_PATH ABI");
static_assert(NAVSYS_STATUS_CANCELLED == -9, "NAVSYS_STATUS_CANCELLED ABI");
static_assert(NAVSYS_STATUS_LIMIT_REACHED == -10, "NAVSYS_STATUS_LIMIT_REACHED ABI");
static_assert(NAVSYS_STATUS_INCOMPLETE == -11, "NAVSYS_STATUS_INCOMPLETE ABI");
static_assert(NAVSYS_STATUS_IN_PROGRESS == -12, "NAVSYS_STATUS_IN_PROGRESS ABI");

int main(void) {
    const char* version = byul_version_string();
    if (version == NULL || strcmp(version, BYUL_VERSION_STRING) != 0) {
        fprintf(stderr, "unexpected BYUL version\n");
        return 1;
    }
    if (coord_sizeof() != sizeof(coord_t)
        || coord_alignof() != _Alignof(coord_t)
        || coord_offsetof_x() != offsetof(coord_t, x)
        || coord_offsetof_y() != offsetof(coord_t, y)) {
        fprintf(stderr, "unexpected coord_t ABI layout\n");
        return 2;
    }
    byul_print_version();
    return 0;
}
