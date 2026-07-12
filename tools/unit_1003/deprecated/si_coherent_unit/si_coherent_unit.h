/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_coherent_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#ifndef SI_COHERENT_UNIT_H
#define SI_COHERENT_UNIT_H

#include "unit/unit_config.h"
#include "si_basic_unit.h"

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_si_coherent_unit_version(char* buf);
LIBAPI const char* get_si_coherent_unit_version(char* buf);

#ifdef __cplusplus
}
#endif

#endif // SI_COHERENT_UNIT_H
