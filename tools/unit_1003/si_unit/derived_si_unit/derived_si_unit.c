/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* derived_si_unit
*
***************************************************************************/

#include "derived_si_unit.h"

#include <stdio.h>
#include <stdlib.h>

LIBAPI void print_derived_si_unit_version(char* buf){
    printf("%s version : %d.%d.%d.%d\n", "derived_si_unit", 
        DERIVED_SI_UNIT_VERSION_MAJOR,
        DERIVED_SI_UNIT_VERSION_MINOR,
        DERIVED_SI_UNIT_VERSION_PATCH,
        DERIVED_SI_UNIT_VERSION_TWEAK);
}

LIBAPI const char* derived_si_unit_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        DERIVED_SI_UNIT_VERSION_MAJOR,
        DERIVED_SI_UNIT_VERSION_MINOR,
        DERIVED_SI_UNIT_VERSION_PATCH,
        DERIVED_SI_UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_derived_si_unit(DerivedSiUnit* tUnit){
    set_si_unit_bits(&tUnit->mSiUnit, 0);
    init_si_prefix_list(&tUnit->mSiUnitPrefixList);
    return 0;
}

LIBAPI int release_derived_si_unit(DerivedSiUnit* tUnit){
    // 메모리 할당을 했으면, 메모리 해제한다.
    return 0;
}

LIBAPI int assign_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther){
    
    if (tUnit==NULL || tOther==NULL){
        printf("error, tUnit or tOther is not init.\n");
        return -1;
    }
    assign_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);
    assign_si_prefix_list(&tUnit->mSiUnitPrefixList,
        &tOther->mSiUnitPrefixList);

    return 0;
}

LIBAPI int assign_derived_si_unit_si_unit(DerivedSiUnit* tUnit, 
    const SiUnit* tOther){
    init_derived_si_unit(tUnit);
    assign_si_unit(&tUnit->mSiUnit, tOther);
        return 0;
    }

LIBAPI int assign_derived_si_unit_unit(DerivedSiUnit* tUnit, 
    const Unit* tOther){
    SiUnit aSiUnit;
    init_derived_si_unit(tUnit);

    aSiUnit = get_derived_si_unit_si_unit(tUnit);
    assign_unit(&tUnit->mSiUnit.mSiUnit_t.mUnit, tOther);
        return 0;
    }

LIBAPI int set_derived_si_prefix_list(DerivedSiUnit* tUnit, 
    const SiUnitPrefixList* tPrefixList){
        assign_si_prefix_list(&tUnit->mSiUnitPrefixList, tPrefixList);
    return 0;
}

LIBAPI int set_derived_si_unit_si_unit(DerivedSiUnit* tUnit, const SiUnit* tSiUnit){
    assign_si_unit(&tUnit->mSiUnit, tSiUnit);
    return 0;
}

const SiUnitPrefixList LIBAPI get_derived_si_prefix_list(const DerivedSiUnit* tUnit){
    return tUnit->mSiUnitPrefixList;
}

const SiUnit LIBAPI get_derived_si_unit_si_unit(const DerivedSiUnit* tUnit){
    return tUnit->mSiUnit;
}

LIBAPI int add_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther){
        add_si_prefix_list(&tUnit->mSiUnitPrefixList,
            &tOther->mSiUnitPrefixList);
        add_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);
        return UNIT_OP_RESULT_SUCCESS;
}

LIBAPI int sub_derived_si_unit(
    DerivedSiUnit* tUnit, const DerivedSiUnit* tOther){

        sub_si_prefix_list(&tUnit->mSiUnitPrefixList,
            &tOther->mSiUnitPrefixList);
        sub_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);
        return UNIT_OP_RESULT_SUCCESS;
    }

LIBAPI int mul_derived_si_unit(
    DerivedSiUnit* tUnit, const DerivedSiUnit* tOther){

        mul_si_prefix_list(&tUnit->mSiUnitPrefixList,
            &tOther->mSiUnitPrefixList);
        mul_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);
        return UNIT_OP_RESULT_SUCCESS;        
    }

LIBAPI int div_derived_si_unit(
    DerivedSiUnit* tUnit, const DerivedSiUnit* tOther){

        div_si_prefix_list(&tUnit->mSiUnitPrefixList,
            &tOther->mSiUnitPrefixList);
        div_si_unit(&tUnit->mSiUnit, &tOther->mSiUnit);        
        return UNIT_OP_RESULT_SUCCESS;        
    }

LIBAPI bool is_derived_si_unit_equal(
    DerivedSiUnit* tSelf, DerivedSiUnit* tOther){
        return is_si_unit_equal(&tSelf->mSiUnit, &tOther->mSiUnit) && 
            is_si_prefix_list_equal(&tSelf->mSiUnitPrefixList, 
            &tOther->mSiUnitPrefixList);
    }    