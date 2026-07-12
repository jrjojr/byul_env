/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* base_converter 
*
***************************************************************************/

#ifndef BASE_CONVERTER_H
#define BASE_CONVERTER_H

#include "base_converter/base_converter_config.h"

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void base_converter_print_version();
LIBAPI const char* base_converter_version(char* buf);

int dec_to_bin(int tDec);

#ifdef __cplusplus
}
#endif

#endif // BASE_CONVERTER_H
