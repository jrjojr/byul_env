/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* basic_si_unit
*
* basic_si_unit는 
* si_prefix와 si_unit를 포함한다.
***************************************************************************/

#ifndef BASIC_SI_UNIT_H
#define BASIC_SI_UNIT_H

#include "basic_si_unit/basic_si_unit_config.h"
#include "si_unit.h"
#include "si_prefix.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_basic_si_unit{
    SiUnit mSiUnit;
    SiUnitPrefix mSiUnitPrefix;
}basic_si_unit_t;

typedef basic_si_unit_t BasicSiUnit;
typedef BasicSiUnit* BasicSiUnitPtr;

LIBAPI void print_basic_si_unit_version(char* buf);
LIBAPI const char* basic_si_unit_version(char* buf);

LIBAPI int init_basic_si_unit(BasicSiUnit* tUnit);
LIBAPI int release_basic_si_unit(BasicSiUnit* tUnit);

LIBAPI int assign_basic_si_unit(BasicSiUnit* tUnit, const BasicSiUnit* tOther);
LIBAPI int set_basic_si_prefix(BasicSiUnit* tUnit, SiUnitPrefix tPrefix);
LIBAPI int set_basic_si_unit_si_unit(BasicSiUnit* tUnit, const SiUnit* tSiUnit);

const SiUnitPrefix LIBAPI get_basic_si_prefix(const BasicSiUnit* tUnit);
const SiUnit LIBAPI get_basic_si_unit_si_unit(const BasicSiUnit* tUnit);

#ifdef __cplusplus
}
#endif

#endif // BASIC_SI_UNIT_H
