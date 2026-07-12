/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* base_converter 
*
***************************************************************************/

#include "base_converter.h"

#include <stdio.h>

LIBAPI void base_converter_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "base_converter", 
        BASE_CONVERTER_VERSION_MAJOR,
        BASE_CONVERTER_VERSION_MINOR,
        BASE_CONVERTER_VERSION_PATCH,
        BASE_CONVERTER_VERSION_TWEAK);
}

LIBAPI const char* base_converter_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        BASE_CONVERTER_VERSION_MAJOR,
        BASE_CONVERTER_VERSION_MINOR,
        BASE_CONVERTER_VERSION_PATCH,
        BASE_CONVERTER_VERSION_TWEAK
        );
    return buf;
}

int dec_to_bin(int tDec){
    return 0;
}