/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* basic_si_unit
*
***************************************************************************/

#include "basic_si_unit.h"

#include <stdio.h>

LIBAPI void print_basic_si_unit_version(char* buf){
    printf("%s version : %d.%d.%d.%d\n", "basic_si_unit", 
        BASIC_SI_UNIT_VERSION_MAJOR,
        BASIC_SI_UNIT_VERSION_MINOR,
        BASIC_SI_UNIT_VERSION_PATCH,
        BASIC_SI_UNIT_VERSION_TWEAK);
}

LIBAPI const char* basic_si_unit_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        BASIC_SI_UNIT_VERSION_MAJOR,
        BASIC_SI_UNIT_VERSION_MINOR,
        BASIC_SI_UNIT_VERSION_PATCH,
        BASIC_SI_UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_basic_si_unit(BasicSiUnit* tUnit){
    init_si_unit(&tUnit->mSiUnit);
    init_si_prefix(&tUnit->mSiUnitPrefix);
    return 0;
}

LIBAPI int release_basic_si_unit(BasicSiUnit* tUnit){
    // 주로 해야 할 일은 메모리 해제인데,
    // 아직 필요없다.
    return 0;
}

LIBAPI int assign_basic_si_unit(BasicSiUnit* tUnit, const BasicSiUnit* tOther){
    assign_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);
    set_basic_si_prefix(tUnit, tOther->mSiUnitPrefix);
    return 0;
}

LIBAPI int set_basic_si_prefix(BasicSiUnit* tUnit, SiUnitPrefix tPrefix){
    tUnit->mSiUnitPrefix = tPrefix;
    return 0;
}

LIBAPI int set_basic_si_unit_si_unit(BasicSiUnit* tUnit, const SiUnit* tSiUnit){
    assign_si_unit(&tUnit->mSiUnit, tSiUnit);
    return 0;
}

const SiUnitPrefix LIBAPI get_basic_si_prefix(const BasicSiUnit* tUnit){
    return tUnit->mSiUnitPrefix;
}

const SiUnit LIBAPI get_basic_si_unit_si_unit(const BasicSiUnit* tUnit){
    return tUnit->mSiUnit;
}
