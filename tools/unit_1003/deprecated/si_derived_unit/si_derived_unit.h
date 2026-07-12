/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_derived_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#ifndef SI_DERIVED_UNIT_H
#define SI_DERIVED_UNIT_H

#include "unit/unit_config.h"
#include "si_basic_unit.h"

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_si_derived_unit_version(char* buf);
LIBAPI const char* get_si_derived_unit_version(char* buf);

LIBAPI const char* get_si_derived_unit_symbol(const SiBasicUnit* tSiBasicUnit);

LIBAPI const char* get_si_derived_unit_name(const SiBasicUnit* tSiBasicUnit);

LIBAPI const char* get_si_derived_unit_quantity_name(const SiBasicUnit* tSiBasicUnit);

#ifdef __cplusplus
}
#endif

#endif // SI_DERIVED_UNIT_H
