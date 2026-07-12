/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit_prefix 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#include "si_unit_prefix.h"

#include <stdio.h>

LIBAPI void print_si_unit_prefix_version(){
    printf("%s version : %d.%d.%d.%d\n", "si_unit_prefix", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* si_unit_prefix_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix){
    tSiUnitPrefix->mPolicy = SI_UNIT_PREFIX_OP_POLICY_NORMAL;
    tSiUnitPrefix->mPrefix = SI_UNIT_PREFIX_ZERO;
    return 0;
}

LIBAPI int release_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix){
    //할 일이 없다.
    // 메모리 할당을 하면, 할 일이 메모리 해제이다.
    return 0;
}

LIBAPI int assign_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefix* tOther){
        tSiUnitPrefix->mPolicy = tOther->mPolicy;
        tSiUnitPrefix->mPrefix = tOther->mPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_op_policy(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefixOpPolicy tOpPolicy){
        tSiUnitPrefix->mPolicy = tOpPolicy;
        return 0;
    }

const SiUnitPrefixOpPolicy LIBAPI get_si_unit_prefix_op_policy(
    const SiUnitPrefix* tSiUnitPrefix){
        return tSiUnitPrefix->mPolicy;
    }

LIBAPI int set_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefixEnum tEnum){
        tSiUnitPrefix->mPrefix = tEnum;
        return 0;
    }

const SiUnitPrefixEnum LIBAPI get_si_unit_prefix(
    const SiUnitPrefix* tSiUnitPrefix){
        return tSiUnitPrefix->mPrefix;
    }

// 접두어 를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther){

    int aDistantFromZeroSelf = abs(tSiUnitPrefix->mPrefix - 0);
    int aDistantFromZeroOther = abs(tOther->mPrefix - 0);

    // 접두어 덧셈은 원하는 접두어를 선택해서 self에 할당하는 것이다.
    switch(tSiUnitPrefix->mPolicy){
    case SI_UNIT_PREFIX_OP_POLICY_NORMAL:
    // self의 접두어를 사용한다.
    return UNIT_OP_RESULT_SUCCESS;

    case SI_UNIT_PREFIX_OP_POLICY_HIGH:
    // 높은 접두어를 사용한다.
    if ( tSiUnitPrefix->mPrefix < tOther->mPrefix){
        tSiUnitPrefix->mPrefix  = tOther->mPrefix;
    }
    return UNIT_OP_RESULT_SUCCESS;

    case SI_UNIT_PREFIX_OP_POLICY_LOW:
    // 낮은 접두어를 사용한다.
    if ( tSiUnitPrefix->mPrefix > tOther->mPrefix){
        tSiUnitPrefix->mPrefix  = tOther->mPrefix;
    }
    return UNIT_OP_RESULT_SUCCESS;

    case SI_UNIT_PREFIX_OP_POLICY_ZERO_NEAREST:
    // 0에 가까운 접두어를 사용한다.
    if ( aDistantFromZeroSelf > aDistantFromZeroOther){
        tSiUnitPrefix->mPrefix  = tOther->mPrefix;
    }    
    return UNIT_OP_RESULT_SUCCESS;

    case SI_UNIT_PREFIX_OP_POLICY_ZERO_FARTHEST:
    // 0에서 먼 접두어를 사용한다.
    if ( aDistantFromZeroSelf < aDistantFromZeroOther){
        tSiUnitPrefix->mPrefix  = tOther->mPrefix;
    }        
    return UNIT_OP_RESULT_SUCCESS;

    }
    // 정책이 없다. self가 초기화가 안되어 있다.    
    return UNIT_OP_RESULT_FAIL;
}

// 접두어 를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther){
    // 접두어 뺄셈은 덧셈과 같다.
    return add_si_unit_prefix(tSiUnitPrefix, tOther);
}

// 접두어 를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther){
    // 접두어 곱셈은 덧셈과 같다.
    return add_si_unit_prefix(tSiUnitPrefix, tOther);
}

// 접두어 를 나눗셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int div_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther){
    // 접두어 나눗셈은 덧셈과 같다.
    return add_si_unit_prefix(tSiUnitPrefix, tOther);        
    }

LIBAPI bool is_si_unit_prefix_equal(
    const SiUnitPrefix* tSelf, const SiUnitPrefix* tOther){
        if (tSelf->mPrefix == tOther->mPrefix)
        {
            return true;
        }
        return false;
    }    

LIBAPI const char* get_si_unit_prefix_name(const SiUnitPrefix* tSelf){
    switch(tSelf->mPrefix){
        case SI_UNIT_PREFIX_QUETTA  : 
            // quetta   = 30,
            return "quetta";
        case SI_UNIT_PREFIX_RONNA   : 
            // ronna    = 27,
            return "ronna";
        case SI_UNIT_PREFIX_YOTTA   : 
            // yotta    = 24,
            return "yotta";
        case SI_UNIT_PREFIX_ZETTA   : 
            // zetta    = 21,
            return "zetta";
        case SI_UNIT_PREFIX_EXA     : 
            // exa      = 18,
            return "exa";
        case SI_UNIT_PREFIX_PETA    : 
            // peta     = 15,
            return "peta";
        case SI_UNIT_PREFIX_TERA    : 
            // tera     = 12,
            return "tera";
        case SI_UNIT_PREFIX_GIGA    : 
            // giga     = 9,
            return "giga";
        case SI_UNIT_PREFIX_MEGA    : 
            // mega     = 6,
            return "mega";
        case SI_UNIT_PREFIX_KILO    : 
            // kilo     = 3,
            return "kilo";
        case SI_UNIT_PREFIX_HECTO   : 
            // hecto    = 2,
            return "hecto";
        case SI_UNIT_PREFIX_DECA    : 
            // deca     = 1,
            return "deca";
        case SI_UNIT_PREFIX_ZERO    : 
            // zero     = 0,
            return "zero";
        case SI_UNIT_PREFIX_DECI    : 
            // deci     = -1,
            return "deci";
        case SI_UNIT_PREFIX_CENTI   : 
            // centi    = -2,
            return "centi";
        case SI_UNIT_PREFIX_MILLI   : 
            // milli    = -3,
            return "milli";
        case SI_UNIT_PREFIX_MICRO   : 
            // micro    = -6,
            return "micro";
        case SI_UNIT_PREFIX_NANO    : 
            // nano     = -9,
            return "nano";
        case SI_UNIT_PREFIX_PICO    : 
            // pico     = -12,
            return "pico";
        case SI_UNIT_PREFIX_FEMTO   : 
            // femto    = -15,
            return "femto";
        case SI_UNIT_PREFIX_ATTO    : 
            // atto     = -18,
            return "atto";
        case SI_UNIT_PREFIX_ZEPTO   : 
            // zepto    = -21,
            return "zepto";
        case SI_UNIT_PREFIX_YOCTO   : 
            // yocto    = -24,
            return "yocto";
        case SI_UNIT_PREFIX_RONTO   : 
            // ronto    = -27,
            return "ronto";
        case SI_UNIT_PREFIX_QUECTO  : 
            // quecto   = -30
            return "quecto";
    }
}    

LIBAPI int init_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList){
    init_si_unit_prefix(&tSiUnitPrefixList->mLengthSiUnitPrefix);
    init_si_unit_prefix(&tSiUnitPrefixList->mMassSiUnitPrefix);
    init_si_unit_prefix(&tSiUnitPrefixList->mTimeSiUnitPrefix);
    init_si_unit_prefix(&tSiUnitPrefixList->mAngleSiUnitPrefix);

    init_si_unit_prefix(&tSiUnitPrefixList->mElectricCurrentSiUnitPrefix);
    init_si_unit_prefix(
        &tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix);
    
    init_si_unit_prefix(&tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix);
    init_si_unit_prefix(&tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix);
    return 0;
}

LIBAPI int release_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList){
    release_si_unit_prefix(&tSiUnitPrefixList->mLengthSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mMassSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mTimeSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mAngleSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mElectricCurrentSiUnitPrefix);
    release_si_unit_prefix(
        &tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix);
    release_si_unit_prefix(&tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix);
    return 0;    
}

LIBAPI int assign_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefixList* tOther){
    assign_si_unit_prefix(
        &tSiUnitPrefixList->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix);
    assign_si_unit_prefix(
        &tSiUnitPrefixList->mMassSiUnitPrefix, &tOther->mMassSiUnitPrefix);
    assign_si_unit_prefix(
        &tSiUnitPrefixList->mTimeSiUnitPrefix, &tOther->mTimeSiUnitPrefix);
    assign_si_unit_prefix(
        &tSiUnitPrefixList->mAngleSiUnitPrefix, &tOther->mAngleSiUnitPrefix);

    assign_si_unit_prefix(&tSiUnitPrefixList->mElectricCurrentSiUnitPrefix, 
        &tOther->mElectricCurrentSiUnitPrefix);
    assign_si_unit_prefix(
        &tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix, 
        &tOther->mThermodynamicTemperatureSiUnitPrefix);
    assign_si_unit_prefix(&tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix, 
        &tOther->mAmountOfSubstanceSiUnitPrefix);
    assign_si_unit_prefix(&tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix, 
        &tOther->mLuminousIntensitySiUnitPrefix);

    return 0;
}

LIBAPI int set_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList, 
    SiUnitPrefixEnum tLengthSiUnitPrefix,
    SiUnitPrefixEnum tMassSiUnitPrefix,
    SiUnitPrefixEnum tTimeSiUnitPrefix,
    SiUnitPrefixEnum tAngleSiUnitPrefix,
    SiUnitPrefixEnum tElectricCurrentSiUnitPrefix,
    SiUnitPrefixEnum tThermodynamicTemperatureSiUnitPrefix,
    SiUnitPrefixEnum tAmountOfSubstanceSiUnitPrefix,
    SiUnitPrefixEnum tLuminousIntensitySiUnitPrefix
    ){

    set_si_unit_prefix(
        &tSiUnitPrefixList->mLengthSiUnitPrefix, tLengthSiUnitPrefix);
    set_si_unit_prefix(
        &tSiUnitPrefixList->mMassSiUnitPrefix, tMassSiUnitPrefix);
    set_si_unit_prefix(
        &tSiUnitPrefixList->mTimeSiUnitPrefix, tTimeSiUnitPrefix);
    set_si_unit_prefix(
        &tSiUnitPrefixList->mAngleSiUnitPrefix, tAngleSiUnitPrefix);

    set_si_unit_prefix(&tSiUnitPrefixList->mElectricCurrentSiUnitPrefix, 
        tElectricCurrentSiUnitPrefix);
    set_si_unit_prefix(
        &tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix, 
        tThermodynamicTemperatureSiUnitPrefix);
    set_si_unit_prefix(&tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix, 
        tAmountOfSubstanceSiUnitPrefix);
    set_si_unit_prefix(&tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix, 
        tLuminousIntensitySiUnitPrefix);

        return 0;
    }


LIBAPI int set_si_unit_prefix_list_length(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mLengthSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_mass(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mMassSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_time(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mTimeSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_angle(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mAngleSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_electric_current(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mElectricCurrentSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_thermodynamic_temperature(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_amount_of_substance(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix = tPrefix;
        return 0;
    }

LIBAPI int set_si_unit_prefix_list_luminous_intensity(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix){
        tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix = tPrefix;
        return 0;
    }

const SiUnitPrefixList* LIBAPI get_si_unit_prefix_list(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_length(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mLengthSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_mass(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mMassSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_time(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mTimeSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_angle(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mAngleSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_electric_current(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mElectricCurrentSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_thermodynamic_temperature(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mThermodynamicTemperatureSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_amount_of_substance(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mAmountOfSubstanceSiUnitPrefix;
    }

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_luminous_intensity(
    const SiUnitPrefixList* tSiUnitPrefixList){
        return tSiUnitPrefixList->mLuminousIntensitySiUnitPrefix;
    }

// 접두어 리스트를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther){
        add_si_unit_prefix(
            &tSiUnitPrefix->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix);
        add_si_unit_prefix(&tSiUnitPrefix->mMassSiUnitPrefix, 
            &tOther->mMassSiUnitPrefix);
        add_si_unit_prefix(&tSiUnitPrefix->mTimeSiUnitPrefix, 
            &tOther->mTimeSiUnitPrefix);
        add_si_unit_prefix(&tSiUnitPrefix->mAngleSiUnitPrefix, 
            &tOther->mAngleSiUnitPrefix);
        add_si_unit_prefix(&tSiUnitPrefix->mElectricCurrentSiUnitPrefix, 
            &tOther->mElectricCurrentSiUnitPrefix);
        add_si_unit_prefix(
            &tSiUnitPrefix->mThermodynamicTemperatureSiUnitPrefix, 
            &tOther->mThermodynamicTemperatureSiUnitPrefix);
        add_si_unit_prefix(
            &tSiUnitPrefix->mAmountOfSubstanceSiUnitPrefix, 
            &tOther->mAmountOfSubstanceSiUnitPrefix);
        add_si_unit_prefix(
            &tSiUnitPrefix->mLuminousIntensitySiUnitPrefix, 
            &tOther->mLuminousIntensitySiUnitPrefix);

        return UNIT_OP_RESULT_SUCCESS;        
    }

// 접두어 리스트를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther){
        sub_si_unit_prefix(
            &tSiUnitPrefix->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix);
        sub_si_unit_prefix(&tSiUnitPrefix->mMassSiUnitPrefix, 
            &tOther->mMassSiUnitPrefix);
        sub_si_unit_prefix(&tSiUnitPrefix->mTimeSiUnitPrefix, 
            &tOther->mTimeSiUnitPrefix);
        sub_si_unit_prefix(&tSiUnitPrefix->mAngleSiUnitPrefix, 
            &tOther->mAngleSiUnitPrefix);
        sub_si_unit_prefix(&tSiUnitPrefix->mElectricCurrentSiUnitPrefix, 
            &tOther->mElectricCurrentSiUnitPrefix);
        sub_si_unit_prefix(
            &tSiUnitPrefix->mThermodynamicTemperatureSiUnitPrefix, 
            &tOther->mThermodynamicTemperatureSiUnitPrefix);
        sub_si_unit_prefix(
            &tSiUnitPrefix->mAmountOfSubstanceSiUnitPrefix, 
            &tOther->mAmountOfSubstanceSiUnitPrefix);
        sub_si_unit_prefix(
            &tSiUnitPrefix->mLuminousIntensitySiUnitPrefix, 
            &tOther->mLuminousIntensitySiUnitPrefix);

        return UNIT_OP_RESULT_SUCCESS;                
    }

// 접두어 리스트를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther){
        mul_si_unit_prefix(
            &tSiUnitPrefix->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix);
        mul_si_unit_prefix(&tSiUnitPrefix->mMassSiUnitPrefix, 
            &tOther->mMassSiUnitPrefix);
        mul_si_unit_prefix(&tSiUnitPrefix->mTimeSiUnitPrefix, 
            &tOther->mTimeSiUnitPrefix);
        mul_si_unit_prefix(&tSiUnitPrefix->mAngleSiUnitPrefix, 
            &tOther->mAngleSiUnitPrefix);
        mul_si_unit_prefix(&tSiUnitPrefix->mElectricCurrentSiUnitPrefix, 
            &tOther->mElectricCurrentSiUnitPrefix);
        mul_si_unit_prefix(
            &tSiUnitPrefix->mThermodynamicTemperatureSiUnitPrefix, 
            &tOther->mThermodynamicTemperatureSiUnitPrefix);
        mul_si_unit_prefix(
            &tSiUnitPrefix->mAmountOfSubstanceSiUnitPrefix, 
            &tOther->mAmountOfSubstanceSiUnitPrefix);
        mul_si_unit_prefix(
            &tSiUnitPrefix->mLuminousIntensitySiUnitPrefix, 
            &tOther->mLuminousIntensitySiUnitPrefix);

        return UNIT_OP_RESULT_SUCCESS;                
    }

// 접두어 리스트를 나눗셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int div_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther){
        div_si_unit_prefix(
            &tSiUnitPrefix->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix);
        div_si_unit_prefix(&tSiUnitPrefix->mMassSiUnitPrefix, 
            &tOther->mMassSiUnitPrefix);
        div_si_unit_prefix(&tSiUnitPrefix->mTimeSiUnitPrefix, 
            &tOther->mTimeSiUnitPrefix);
        div_si_unit_prefix(&tSiUnitPrefix->mAngleSiUnitPrefix, 
            &tOther->mAngleSiUnitPrefix);
        div_si_unit_prefix(&tSiUnitPrefix->mElectricCurrentSiUnitPrefix, 
            &tOther->mElectricCurrentSiUnitPrefix);
        div_si_unit_prefix(
            &tSiUnitPrefix->mThermodynamicTemperatureSiUnitPrefix, 
            &tOther->mThermodynamicTemperatureSiUnitPrefix);
        div_si_unit_prefix(
            &tSiUnitPrefix->mAmountOfSubstanceSiUnitPrefix, 
            &tOther->mAmountOfSubstanceSiUnitPrefix);
        div_si_unit_prefix(
            &tSiUnitPrefix->mLuminousIntensitySiUnitPrefix, 
            &tOther->mLuminousIntensitySiUnitPrefix);

        return UNIT_OP_RESULT_SUCCESS;                
    }

LIBAPI int si_unit_prefix_list(const SiUnitPrefixList* tSiUnitPrefix,
    SiUnitPrefixEnum* retLengthSiUnitPrefix,
    SiUnitPrefixEnum* retMassSiUnitPrefix,
    SiUnitPrefixEnum* retTimeSiUnitPrefix,
    SiUnitPrefixEnum* retAngleSiUnitPrefix,
    SiUnitPrefixEnum* retElectricCurrentSiUnitPrefix,
    SiUnitPrefixEnum* retThermodynamicTemperatureSiUnitPrefix,
    SiUnitPrefixEnum* retAmountOfSubstanceSiUnitPrefix,
    SiUnitPrefixEnum* retLuminousIntensitySiUnitPrefix
){
    *retLengthSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mLengthSiUnitPrefix);
    *retMassSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mMassSiUnitPrefix);
    *retTimeSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mTimeSiUnitPrefix);
    *retAngleSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mAngleSiUnitPrefix);

    *retElectricCurrentSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mElectricCurrentSiUnitPrefix);
    *retThermodynamicTemperatureSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mThermodynamicTemperatureSiUnitPrefix);
    *retAmountOfSubstanceSiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mAmountOfSubstanceSiUnitPrefix);
    *retLuminousIntensitySiUnitPrefix = get_si_unit_prefix(
        &tSiUnitPrefix->mLuminousIntensitySiUnitPrefix);
    return 0;
}    

LIBAPI bool is_si_unit_prefix_list_equal(
    const SiUnitPrefixList* tSelf, const SiUnitPrefixList* tOther){
    if( is_si_unit_prefix_equal(
            &tSelf->mLengthSiUnitPrefix, &tOther->mLengthSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mMassSiUnitPrefix, &tOther->mMassSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mTimeSiUnitPrefix, &tOther->mTimeSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mAngleSiUnitPrefix, &tOther->mAngleSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mElectricCurrentSiUnitPrefix, 
            &tOther->mElectricCurrentSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mThermodynamicTemperatureSiUnitPrefix, 
            &tOther->mThermodynamicTemperatureSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mAmountOfSubstanceSiUnitPrefix, 
            &tOther->mAmountOfSubstanceSiUnitPrefix) &&
        is_si_unit_prefix_equal(
            &tSelf->mLuminousIntensitySiUnitPrefix, 
            &tOther->mLuminousIntensitySiUnitPrefix) ){
                return true;
            }

    return false;

    }