/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* non_si_unit
*
***************************************************************************/

#include "non_si_unit.h"

#include <stdio.h>

// min	    time	1 min = 60 s
const Unit Minute           = Angle;

// h	            1 h = 60 min = 3600 s
const Unit Hour             = Angle;

// d	            1 d = 24 h = 1440 min = 86400 s
const Unit Day              = Angle;

// au	    length	1 au = 149597870700 m
const Unit AstronomicalUnit = Angle;

// °	    plane angle and phase angle	    1° = (π/180) rad
const Unit Degree           = Angle;

// ′	            1′ = (1/60)° = (π/10800) rad
const Unit Arcminute        = Angle;

// ″	            1″ = (1/60)′ = (1/3600)° = (π/648000) rad
const Unit Arcsecond        = Angle;

// ha	    area	1 ha = 1 hm2 = 10000 m2
const Unit Hectare          = Area;

// L	    volume	1 L = 1 dm3 = 1000 cm3 = 0.001 m^3
const Unit Litre            = Volume;

// t	    mass    1 t = 1 Mg = 1000 kg
const Unit Tonne            = Mass;

// Da	            1 Da = 1.66053906892(52)×10^-27 kg
const Unit Dalton           = Mass;

// eV	    energy  1 eV = 1.602176634×10^-19 J
const Unit ElectronVolt     = Energy;

// logarithmic ratio quantity
// 로그비 양
// 음압을 표현할 때 자주 사용해서,
// 압력으로 표현하는 것이 좋겠다.
const Unit Neper            = Pressure;
const Unit Bel              = Pressure;
const Unit DeciBel          = Pressure;

LIBAPI void non_si_unit_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "non_si_unit", 
        NON_SI_UNIT_VERSION_MAJOR,
        NON_SI_UNIT_VERSION_MINOR,
        NON_SI_UNIT_VERSION_PATCH,
        NON_SI_UNIT_VERSION_TWEAK);
}

LIBAPI const char* non_si_unit_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        NON_SI_UNIT_VERSION_MAJOR,
        NON_SI_UNIT_VERSION_MINOR,
        NON_SI_UNIT_VERSION_PATCH,
        NON_SI_UNIT_VERSION_TWEAK
        );
    return buf;
}

