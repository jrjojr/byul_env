#include <stdio.h>
#include "byul.h"

const char* byul_version_string() {
    return BYUL_VERSION_STRING;
}

void byul_print_version() {
    printf("byul version %s (%d)\n", BYUL_VERSION_STRING, BYUL_VERSION);
}
