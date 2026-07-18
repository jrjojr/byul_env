#include <stdio.h>
#include <string.h>

#include "byul.h"

int main(void) {
    const char* version = byul_version_string();
    if (version == NULL || strcmp(version, BYUL_VERSION_STRING) != 0) {
        fprintf(stderr, "unexpected BYUL version\n");
        return 1;
    }
    byul_print_version();
    return 0;
}
