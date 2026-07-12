/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* quantity
*
***************************************************************************/

#include "quantity.h"

#include <stdio.h>
#include <stdlib.h> // malloc, realloc, free
#include "math.h"

LIBAPI void quantity_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "quantity", 
        QUANTITY_VERSION_MAJOR,
        QUANTITY_VERSION_MINOR,
        QUANTITY_VERSION_PATCH,
        QUANTITY_VERSION_TWEAK);
}

LIBAPI const char* quantity_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        QUANTITY_VERSION_MAJOR,
        QUANTITY_VERSION_MINOR,
        QUANTITY_VERSION_PATCH,
        QUANTITY_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_quantity(Quantity* tQuantity){
    init_dec_real(&tQuantity->mDecReal);
    init_units(&tQuantity->mUnits);
    return 0;
}

LIBAPI int release_quantity(Quantity* tQuantity){
    // 메모리 할당을 했으면, 해제를 한다.
    release_dec_real(&tQuantity->mDecReal);
    release_units(&tQuantity->mUnits);
    return 0;
}

LIBAPI int assign_quantity(Quantity* tQuantity, const Quantity* tOther){
        assign_dec_real(&tQuantity->mDecReal, &tOther->mDecReal);
        assign_units(&tQuantity->mUnits, &tOther->mUnits);
        return 0;
}

LIBAPI int set_quantity_dec_real(Quantity* tQuantity, const DecReal* tDecReal){
    UnitPrefixEnum aPrefix;
    // &tQuantity->mDecReal = *tDecReal;
    assign_dec_real(&tQuantity->mDecReal, tDecReal);
    update_quantity(tQuantity);

    return SUCCESS;
}

LIBAPI int set_quantity_units(Quantity* tQuantity, const Units* tUnits){
    assign_units(&tQuantity->mUnits, tUnits);
    update_quantity(tQuantity);
    return SUCCESS;
}

LIBAPI const DecReal quantity_dec_real(const Quantity* tQuantity){
    return tQuantity->mDecReal;
}

const Units LIBAPI quantity_units(const Quantity* tQuantity){
    return tQuantity->mUnits;
}

LIBAPI int add_quantity(Quantity* tQuantity, const Quantity* tOther){
    /*
    * real을 연산한다.
    * unit를 연산한다.
    */
   if( units_equal(&tQuantity->mUnits, &tOther->mUnits)){
        add_dec_real(&tQuantity->mDecReal, &tOther->mDecReal);
        
        add_units(&tQuantity->mUnits, &tOther->mUnits);

        update_quantity(tQuantity);

        return SUCCESS;
   }
   else{
        // quantity의 단위가 다르다.
        // 덧셈이 불가능이다.
        return QUANTITY_UNIT_NOT_EQUAL;
   }
}

LIBAPI int sub_quantity(Quantity* tQuantity, const Quantity* tOther){
    if( unit_is_equal(&tQuantity->mUnits.mUnit, &tOther->mUnits.mUnit)){
        sub_dec_real(&tQuantity->mDecReal, &tOther->mDecReal);

        sub_unit_prefix_enum(&tQuantity->mUnits.mPrefixList0, 
            tOther->mUnits.mPrefixList0, tQuantity->mUnits.mSnapPolicy);

        sub_unit_prefix_enum(&tQuantity->mUnits.mPrefixList1, 
            tOther->mUnits.mPrefixList1, tQuantity->mUnits.mSnapPolicy);

        sub_unit_prefix_enum(&tQuantity->mUnits.mPrefixList2, 
            tOther->mUnits.mPrefixList2, tQuantity->mUnits.mSnapPolicy);

        sub_unit_prefix_enum(&tQuantity->mUnits.mPrefixList3, 
            tOther->mUnits.mPrefixList3, tQuantity->mUnits.mSnapPolicy);    

        update_quantity(tQuantity);

        return SUCCESS;
    }
    else{
        // quantity의 단위가 다르다.
        // 뺄셈이 불가능이다.
        return QUANTITY_UNIT_NOT_EQUAL;
    }
}

LIBAPI int mul_quantity(Quantity* tQuantity, const Quantity* tOther){
    mul_dec_real(&tQuantity->mDecReal, &tOther->mDecReal);
    mul_unit(&tQuantity->mUnits.mUnit, &tOther->mUnits.mUnit);
    mul_unit_prefix_enum(&tQuantity->mUnits.mPrefixList0, 
        tOther->mUnits.mPrefixList0, tQuantity->mUnits.mSnapPolicy);

    mul_unit_prefix_enum(&tQuantity->mUnits.mPrefixList1, 
        tOther->mUnits.mPrefixList1, tQuantity->mUnits.mSnapPolicy);

    mul_unit_prefix_enum(&tQuantity->mUnits.mPrefixList2, 
        tOther->mUnits.mPrefixList2, tQuantity->mUnits.mSnapPolicy);

    mul_unit_prefix_enum(&tQuantity->mUnits.mPrefixList3, 
        tOther->mUnits.mPrefixList3, tQuantity->mUnits.mSnapPolicy);    

    update_quantity(tQuantity);        

    return SUCCESS;
}

LIBAPI int div_quantity(Quantity* tQuantity, const Quantity* tOther){
    div_dec_real(&tQuantity->mDecReal, &tOther->mDecReal);
    div_unit(&tQuantity->mUnits.mUnit, &tOther->mUnits.mUnit);

    div_unit_prefix_enum(&tQuantity->mUnits.mPrefixList0, 
        tOther->mUnits.mPrefixList0, tQuantity->mUnits.mSnapPolicy);

    div_unit_prefix_enum(&tQuantity->mUnits.mPrefixList1, 
        tOther->mUnits.mPrefixList1, tQuantity->mUnits.mSnapPolicy);

    div_unit_prefix_enum(&tQuantity->mUnits.mPrefixList2, 
        tOther->mUnits.mPrefixList2, tQuantity->mUnits.mSnapPolicy);

    div_unit_prefix_enum(&tQuantity->mUnits.mPrefixList3, 
        tOther->mUnits.mPrefixList3, tQuantity->mUnits.mSnapPolicy);        

    update_quantity(tQuantity);
    return SUCCESS;
}

LIBAPI int pow_quantity(Quantity* tQuantity, const REAL tValue){
    DecReal *aPow;
    aPow = create_dec_real_from(tValue);

    pow_dec_real(&tQuantity->mDecReal, aPow);

    delete_dec_real(aPow);

    update_quantity(tQuantity);    
    return SUCCESS;
}

Quantity* LIBAPI create_quantity(const DecReal* tDecReal, const Units* tUnits){

    Quantity* r;
    r = create_quantity_default();
    assign_dec_real(&r->mDecReal, tDecReal);
    assign_units(&r->mUnits, tUnits);

    return r;
    }

Quantity* LIBAPI create_quantity_default(){
    Quantity* r;
    r = (Quantity*)malloc(sizeof(Quantity));
    init_quantity(r);
    return r;
}

LIBAPI int delete_quantity(Quantity* tSelf){
    if(tSelf != NULL){
        release_quantity(tSelf);
        free(tSelf);
    }
    return 0;
}

LIBAPI int get_quantity_dec_real(const Quantity* tQuantity, DecReal* retDecReal){
    *retDecReal = tQuantity->mDecReal;
    return 0;
}

LIBAPI int get_quantity_units(const Quantity* tQuantity, Units* retUnits){
    *retUnits = tQuantity->mUnits;
    return 0;
}

LIBAPI bool quantity_equal(const Quantity* tQuantity, const Quantity* tOther,
    const REAL tTol){

    return dec_real_equal(&tQuantity->mDecReal, &tOther->mDecReal, tTol) &&
            units_equal(&tQuantity->mUnits, &tOther->mUnits);
}

LIBAPI bool quantity_equal_rel(const Quantity* tQuantity, 
    const Quantity* tOther, const REAL tTol){

    return dec_real_equal_rel(&tQuantity->mDecReal, &tOther->mDecReal, tTol) &&
            units_equal(&tQuantity->mUnits, &tOther->mUnits);
}

const Quantity LIBAPI quantity_add(const Quantity* tQuantity, 
    const Quantity* tOther){

    Quantity r;
    assign_quantity(&r, tQuantity);
    if (add_quantity(&r, tOther) != SUCCESS){
        // 덧셈은 단위가 다르면 연산 불가능이다.
        // 셀프를 리턴한다.
        return *tQuantity;
    }
    return r;
}

const Quantity LIBAPI quantity_sub(const Quantity* tQuantity, 
    const Quantity* tOther){

    Quantity r;
    assign_quantity(&r, tQuantity);
    if (add_quantity(&r, tOther) != SUCCESS){
        // 뺄셈은 단위가 다르면 연산 불가능이다.
        // 셀프를 리턴한다.
        return *tQuantity;
    }
    return r;
}

const Quantity LIBAPI quantity_mul(const Quantity* tQuantity, 
    const Quantity* tOther){

    Quantity r;
    assign_quantity(&r, tQuantity);
    mul_quantity(&r, tOther);
    return r;
}

const Quantity LIBAPI quantity_div(const Quantity* tQuantity, 
    const Quantity* tOther){

    Quantity r;
    assign_quantity(&r, tQuantity);
    div_quantity(&r, tOther);
    return r;
}


const Quantity LIBAPI quantity_pow(const Quantity* tQuantity, 
    const REAL tValue){

    Quantity r;
    assign_quantity(&r, tQuantity);
    pow_quantity(&r, tValue);
    return r;
}

LIBAPI int update_quantity(Quantity* tSelf){
    // 숫자의 지수와 접두어를 같게 만든다.

    if(dec_real_exp(&tSelf->mDecReal) == units_prefix_total(&tSelf->mUnits)){
        // 숫자의 지수와 접두어가 같다.
        // 종료한다.
        return SUCCESS;
    }
    
    // quantity의 지수는 모든 유닛의 지수를 연산한 후에 결과를 숫자에 대입한다.
    DEC aExp;
    aExp = unit_prefix_enum_snap_from(tSelf->mDecReal.mExp, 
        tSelf->mUnits.mSnapPolicy);

    set_units_prefix_total(&tSelf->mUnits, aExp);
    fix_dec_real_exp_trunc(&tSelf->mDecReal, aExp);

    // 접두어의 숫자만큼 지수를 빼야한다.
    set_dec_real_exp(&tSelf->mDecReal, tSelf->mDecReal.mExp-aExp);

    return SUCCESS;
}

LIBAPI const REAL quantity_value(const Quantity* tSelf){
    REAL aReal;
    DEC aExp;
    aExp = units_prefix_total(&tSelf->mUnits);
    aReal = dec_real_to(&tSelf->mDecReal);
    aReal *= pow(10, aExp);
    return aReal;
}

LIBAPI const REAL quantity_dec_real_value(const Quantity* tSelf){
    return dec_real_to(&tSelf->mDecReal);
}

LIBAPI int set_quantity_value(Quantity* tSelf, const REAL tValue){
    DecReal aDecReal;
    REAL aValue = tValue;
    init_dec_real(&aDecReal);
    
    aValue /= pow(10, units_prefix_total(&tSelf->mUnits));
    set_dec_real_from(&aDecReal, aValue);

    return SUCCESS;
}

LIBAPI int set_quantity_dec_real_value(Quantity* tSelf, 
    const REAL tValue){

    set_dec_real_from(&tSelf->mDecReal, tValue);
    return SUCCESS;
    }

LIBAPI int get_quantity_value(const Quantity* tSelf, REAL* retValue){
    *retValue = quantity_value(tSelf);
    return SUCCESS;
}

LIBAPI int get_quantity_dec_real_value(const Quantity* tSelf, REAL* retValue){
    *retValue = dec_real_to(&tSelf->mDecReal);
    return SUCCESS;
}

LIBAPI int get_quantity_to_ustr(const Quantity* tSelf, Ustring* retStr){
    // quantity의 값을 문자열로 얻는다.
    Ustring* aBufStr = NULL;
    aBufStr = create_ustring_default();
    get_dec_real_str(&tSelf->mDecReal, retStr);
    get_units_symbol(&tSelf->mUnits, aBufStr);
    append_ustring_from(retStr, " ");
    append_ustring(retStr, aBufStr);
    delete_ustring(aBufStr);
    return SUCCESS;
}

LIBAPI const char* quantity_to(const Quantity* tSelf, char* buf){
    // quantity의 값을 문자열로 얻는다.
    // buf는 문자열이 자리잡을 저장소이다.
    // 저장소가 있어야 문자열을 반환 받는다.
    // 저장소에는 미리 메모리가 할당되어 있어야 한다.
    Ustring* aStr = NULL;
    aStr = create_ustring_default();
    get_quantity_to_ustr(tSelf, aStr);

    strcpy(buf, ustring(aStr));
    delete_ustring(aStr);
    return buf;
}
