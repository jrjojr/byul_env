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

#include "si_coherent_unit.h"

#include <stdio.h>

     // mTime;     mLength;     mMass;     mElectricCurrent;     mThermodynamicTemperature;     mAmountOfSubstance;     mLuminousIntensity;
     


LIBAPI void print_si_coherent_unit_version(char* buf){
    printf("%s version : %d.%d.%d.%d\n", "si_coherent_unit", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* get_si_coherent_unit_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}
