/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* units 
*
***************************************************************************/

#include "units.h"

#include <stdio.h>
#include <stdlib.h> 

LIBAPI void print_units_version(){
    printf("%s version : %d.%d.%d.%d\n", "units", 
        UNITS_VERSION_MAJOR,
        UNITS_VERSION_MINOR,
        UNITS_VERSION_PATCH,
        UNITS_VERSION_TWEAK);
}

LIBAPI const char* units_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNITS_VERSION_MAJOR,
        UNITS_VERSION_MINOR,
        UNITS_VERSION_PATCH,
        UNITS_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_units(Units* tSelf){
    init_unit(&tSelf->mUnit);
    // tSelf->mPrefixList = (UnitPrefixEnum*)malloc(sizeof(UnitPrefixEnum)*4);
    // tSelf->mPrefixList[0] = UNIT_PREFIX_ZERO;
    // tSelf->mPrefixList[1] = UNIT_PREFIX_ZERO;
    // tSelf->mPrefixList[2] = UNIT_PREFIX_ZERO;
    // tSelf->mPrefixList[3] = UNIT_PREFIX_ZERO;
    tSelf->mPrefixList0 = UNIT_PREFIX_ZERO;
    tSelf->mPrefixList1 = UNIT_PREFIX_ZERO;
    tSelf->mPrefixList2 = UNIT_PREFIX_ZERO;
    tSelf->mPrefixList3 = UNIT_PREFIX_ZERO;
    tSelf->mSnapPolicy = UNIT_PREFIX_SNAP_POLICY_NORMAL;
    return SUCCESS;
}

LIBAPI int release_units(Units* tSelf){
    // if(tSelf->mPrefixList != NULL){
        // free(tSelf->mPrefixList);
    // }
    return SUCCESS;
}

LIBAPI int set_units(Units* tSelf, const Unit* tUnit,
    const UnitPrefixEnum tLengthPrefix,
    const UnitPrefixEnum tMassPrefix,
    const UnitPrefixEnum tTimePrefix,
    const UnitPrefixEnum tAnglePrefix,
    const UnitPrefixSnapPolicy tSnapPolicy){

    assign_unit(&tSelf->mUnit, tUnit);
    tSelf->mPrefixList0 = tLengthPrefix;
    tSelf->mPrefixList1 = tMassPrefix;
    tSelf->mPrefixList2 = tTimePrefix;
    tSelf->mPrefixList3 = tAnglePrefix;
    tSelf->mSnapPolicy = tSnapPolicy;
    return SUCCESS;
}

LIBAPI int set_units_unit(Units* tSelf, const Unit* tUnit){
    assign_unit(&tSelf->mUnit, tUnit);
    return SUCCESS;
}

LIBAPI int set_units_prefix_length(Units* tSelf, 
    const UnitPrefixEnum tPrefix){

    tSelf->mPrefixList0 = tPrefix;
    return SUCCESS;
}

LIBAPI int set_units_prefix_mass(Units* tSelf, 
    const UnitPrefixEnum tPrefix){

    tSelf->mPrefixList1 = tPrefix;
    return SUCCESS;
}

LIBAPI int set_units_prefix_time(Units* tSelf, 
    const UnitPrefixEnum tPrefix){

    tSelf->mPrefixList2 = tPrefix;
    return SUCCESS;    
}

LIBAPI int set_units_prefix_angle(Units* tSelf, 
    const UnitPrefixEnum tPrefix){

    tSelf->mPrefixList3 = tPrefix;
    return SUCCESS;
}

LIBAPI int set_units_prefix_snap_policy(Units* tSelf, 
    const UnitPrefixSnapPolicy tSnapPolicy){

    tSelf->mSnapPolicy = tSnapPolicy;
    return SUCCESS;
}

LIBAPI int get_units_unit(const Units* tSelf, Unit* retUnit){
    *retUnit = tSelf->mUnit;
    return SUCCESS;
}

LIBAPI int get_units_prefix_length(const Units* tSelf, 
    UnitPrefixEnum* retPrefix){

    *retPrefix = tSelf->mPrefixList0;
    return SUCCESS;
}

LIBAPI int get_units_prefix_mass(const Units* tSelf, 
    UnitPrefixEnum* retPrefix){

    *retPrefix = tSelf->mPrefixList1;
    return SUCCESS;
}

LIBAPI int get_units_prefix_time(const Units* tSelf, 
    UnitPrefixEnum* retPrefix){

    *retPrefix = tSelf->mPrefixList2;
    return SUCCESS;        
}

LIBAPI int get_units_prefix_angle(const Units* tSelf, 
    UnitPrefixEnum* retPrefix){

    *retPrefix = tSelf->mPrefixList3;
    return SUCCESS;
}

LIBAPI int get_units_prefix_snap_policy(const Units* tSelf, 
    UnitPrefixSnapPolicy* retSnapPolicy){

    *retSnapPolicy = tSelf->mSnapPolicy;
    return SUCCESS;
}

const Unit LIBAPI units_unit(const Units* tSelf){
    return tSelf->mUnit;
}

const UnitPrefixSnapPolicy LIBAPI units_prefix_snap_policy(
    const Units* tSelf){

    return tSelf->mSnapPolicy;
}

LIBAPI const UnitPrefixEnum units_prefix_length(const Units* tSelf){
    return tSelf->mPrefixList0;
}

LIBAPI const UnitPrefixEnum units_prefix_mass(const Units* tSelf){
    return tSelf->mPrefixList1;
}

LIBAPI const UnitPrefixEnum units_prefix_time(const Units* tSelf){
    return tSelf->mPrefixList2;
}

LIBAPI const UnitPrefixEnum units_prefix_angle(const Units* tSelf){
    return tSelf->mPrefixList3;
}

Units* LIBAPI create_units(const Unit* tUnit, 
    const UnitPrefixEnum tLengthPrefix,
    const UnitPrefixEnum tMassPrefix,
    const UnitPrefixEnum tTimePrefix,
    const UnitPrefixEnum tAnglePrefix,
    const UnitPrefixSnapPolicy tSnapPolicy){

    Units* r = NULL;
    r = (Units*)malloc(sizeof(Units));
    init_units(r);
    assign_unit(&r->mUnit, tUnit);
    r->mPrefixList0 = tLengthPrefix;
    r->mPrefixList1 = tMassPrefix;
    r->mPrefixList2 = tTimePrefix;
    r->mPrefixList3 = tAnglePrefix;
    r->mSnapPolicy = tSnapPolicy;
    return r;
}

Units* LIBAPI create_units_default(){
    Units* r = NULL;
    r = (Units*)malloc(sizeof(Units));
    init_units(r);
    return r;
}

LIBAPI int delete_units(Units* tSelf){
    release_units(tSelf);
    if(tSelf != NULL){
        free(tSelf);
    }
    return SUCCESS;
}

LIBAPI int assign_units(Units* tSelf, const Units* tOther){
    assign_unit(&tSelf->mUnit, &tOther->mUnit);
    tSelf->mPrefixList0 = tOther->mPrefixList0;
    tSelf->mPrefixList1 = tOther->mPrefixList1;
    tSelf->mPrefixList2 = tOther->mPrefixList2;
    tSelf->mPrefixList3 = tOther->mPrefixList3;
    tSelf->mSnapPolicy = tOther->mSnapPolicy;
    return SUCCESS;
}

LIBAPI bool units_equal(const Units* tSelf, const Units* tOther){
    return unit_is_equal(&tSelf->mUnit, &tSelf->mUnit) &&
        tSelf->mPrefixList0 == tOther->mPrefixList0 &&
        tSelf->mPrefixList1 == tOther->mPrefixList1 &&
        tSelf->mPrefixList2 == tOther->mPrefixList2 &&
        tSelf->mPrefixList3 == tOther->mPrefixList3 &&
        tSelf->mSnapPolicy == tOther->mSnapPolicy;
}

LIBAPI int set_units_prefix_total(Units* tSelf, const UnitPrefixEnum tPrefix){
    // 하나의 접두어를 찾기위해서는 모든 유닛을 덧셈하면 된다.
    // 이유는 유닛은 모든 단위의 지수를 연산하는 것과 같다.
    // 유닛들은 곱셈한 것을 표현하는 것이다.
    // 예를 들어, 시간 매 길이는 거리를 표현한다. m/s
    // 시간 제곱 매 길이는 가속도를 표현한다. m/s^2
    // 접두어는 지수를 표현하는 방버이다.
    // 지수의 곱셈은 지수끼리 덧셈이다.
    // aTotalExp = length + mass + time + angle;
    // 하나의 접두어는 aTotalExp이다. 
    // aTotalExp 를 3이라고 하면
    // 3 = length + mass + time + angle;
    // 현재의 총 접두어를 알아낸다.
    // 현재의 총 접두어와 새로 설정할 접두어의 차이를 연산한다.
    // 차이만큼 현재 접두어에 덧셈한다.
    // 덧셈을 의해서는 우선순위가 있다.
    // 예를 들어, 현재의 총 접두어는 3이다. 새로 적용할 접두어는 6이다.
    // 3의 차이가 있다. 3을 현재의 접두어에 덧셈해야 한다.
    // 방법은 여러가지가 있다.
    // 첫번째 유닛에 모두 3을 더하기 : 
    //  UNIT_PREFIX_TOTAL_POLICY_ONE
    // 모든 유닛에 3을 분배해서 모든 유닛에 더하기
    //  UNIT_PREFIX_TOTAL_POLICY_ALL
    //  단위의 분자에만 더하기
    //  UNIT_PREFIX_TOTAL_POLICY_NUMERATOR
    //  단위의 분모에만 더하기
    //  UNIT_PREFIX_TOTAL_POLICY_DENOMINATOR
    // 일단 동작을 해야하니까 첫번째 유닛에 모두 더하기 정책을 사용한다.
    // 첫번째 양수의 유닛을 찾아야 한다.
    int aCurTotal = units_prefix_total(tSelf);
    int aDiffTotal = tPrefix - aCurTotal;
    int aFirstPositiveUnit = 0;
    for(int i=0; i<unit_count(&tSelf->mUnit); i++){
        if (unit_dim_at(&tSelf->mUnit, i) > 0){
            aFirstPositiveUnit = i;
            break;
        }
    }

    aCurTotal = units_prefix_at(tSelf, aFirstPositiveUnit) + aDiffTotal;
    aCurTotal = unit_prefix_enum_snap_from(aCurTotal, tSelf->mSnapPolicy);
    set_units_prefix_at(tSelf, aFirstPositiveUnit, aCurTotal);
    
    return SUCCESS;
}

LIBAPI int get_units_prefix_total(const Units* tSelf, UnitPrefixEnum* retPrefix){
    // 하나의 접두어를 찾기위해서는 모든 유닛을 덧셈하면 된다.
    // 이유는 유닛은 모든 단위의 지수를 연산하는 것과 같다.
    // 유닛들은 곱셈한 것을 표현하는 것이다.
    // 예를 들어, 시간 매 길이는 거리를 표현한다. m/s
    // 시간 제곱 매 길이는 가속도를 표현한다. m/s^2
    // 접두어는 지수를 표현하는 방버이다.
    // 지수의 곱셈은 지수끼리 덧셈이다.
    // aTotalExp = length + mass + time + angle;
    // 하나의 접두어는 aTotalExp이다.     
    *retPrefix = tSelf->mPrefixList0 + 
        tSelf->mPrefixList1 + 
        tSelf->mPrefixList2 + 
        tSelf->mPrefixList3;

    *retPrefix = unit_prefix_enum_snap_from(*retPrefix, tSelf->mSnapPolicy);
    return SUCCESS;
}

LIBAPI const UnitPrefixEnum units_prefix_total(const Units* tSelf){
    UnitPrefixEnum r;
    get_units_prefix_total(tSelf, &r);
    return r;
}

LIBAPI int set_units_prefix_at(Units* tSelf, int tIndex, 
    const UnitPrefixEnum tPrefix){
    
    int count = unit_count(&tSelf->mUnit);
    if(tIndex > count){
        tIndex = count - 1;
    }
    else if(tIndex < 0){
        tIndex = 0;
    }

    // tSelf->mPrefixList[tIndex] = tPrefix;
    switch(tIndex){
        case 0:
        tSelf->mPrefixList0 = tPrefix;
        break;
        case 1:
        tSelf->mPrefixList1 = tPrefix;
        break;
        case 2:
        tSelf->mPrefixList2 = tPrefix;
        break;
        case 3:
        tSelf->mPrefixList3 = tPrefix;
        break;
    }

    return SUCCESS;
}

LIBAPI int get_units_prefix_at(const Units* tSelf, int tIndex, 
    UnitPrefixEnum* retPrefix){

    int count = unit_count(&tSelf->mUnit);
    if(tIndex > count){
        tIndex = count - 1;
    }
    else if(tIndex < 0){
        tIndex = 0;
    }

    // *retPrefix = tSelf->mPrefixList[tIndex];
    switch(tIndex){
        case 0:
*retPrefix = tSelf->mPrefixList0;
        break;
        case 1:
*retPrefix = tSelf->mPrefixList1;
        break;
        case 2:
*retPrefix = tSelf->mPrefixList2;
        break;
        case 3:
*retPrefix = tSelf->mPrefixList3;
        break;
    }    

    return SUCCESS;
}

LIBAPI const UnitPrefixEnum units_prefix_at(const Units* tSelf, int tIndex){
    UnitPrefixEnum r;

    get_units_prefix_at(tSelf, tIndex, &r);

    return r;
}

LIBAPI const char* units_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_length_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_length_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_mass_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_mass_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_time_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_time_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_angle_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_angle_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_abs_length_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_abs_length_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_abs_mass_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_abs_mass_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_abs_time_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_abs_time_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI const char* units_abs_angle_symbol(const Units* tSelf, char* buf){
    Ustring* aUstr = NULL;
    aUstr = create_ustring_default();

    get_units_abs_angle_symbol(tSelf, aUstr);

    strcpy(buf, ustring(aUstr));
    delete_ustring(aUstr);    
    return buf;
}

LIBAPI int get_units_symbol(const Units* tSelf, Ustring* retStr){
    // kg*mm^2/s^2
    // m k s rad <-
    // rad s kg mm
    // angle , time, mass, length의 순서로 값을 조사한다.
    // 값이 음수이면 length, mass, time, angle의 순서로 값을 조사한다.
    // rad*s^2*kg*mm^2/mm^2*kg*s^3*rad
    Ustring* aStr;
    Ustring* aTempStr;

    aStr = create_ustring_default();
    aTempStr = create_ustring_default();

    if(unit_angle(&tSelf->mUnit) > 0){
        // 양수의 단위를 추가한다.
        get_units_angle_symbol(tSelf, aTempStr);
        append_ustring(aStr, aTempStr);        
    }
    if(unit_time(&tSelf->mUnit) > 0){
        if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
            // 단위에 먼저 추가된 단위가 있으면
            // 단위의 연결을 위해 "*"를 추가한다.
            append_ustring_from(aStr, "*");
        }
        // 양수의 단위를 추가한다.
        get_units_time_symbol(tSelf, aTempStr);
        append_ustring(aStr, aTempStr);
    }
    if(unit_mass(&tSelf->mUnit) > 0){
        if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
            // 단위에 먼저 추가된 단위가 있으면
            // 단위의 연결을 위해 "*"를 추가한다.
            append_ustring_from(aStr, "*");
        }        
        // 양수의 단위를 추가한다.
        get_units_mass_symbol(tSelf, aTempStr);
        append_ustring(aStr, aTempStr);
    }
    if(unit_length(&tSelf->mUnit) > 0){
        if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
            // 단위에 먼저 추가된 단위가 있으면
            // 단위의 연결을 위해 "*"를 추가한다.
            append_ustring_from(aStr, "*");
        }        
        // 양수의 단위를 추가한다.
        get_units_length_symbol(tSelf, aTempStr);
        append_ustring(aStr, aTempStr);
    }    

    // 양의 단위는 전부 추가가 되었다.
    // 이제는 음의 단위를 추가한다.
    // 음의 단위는 추가된 양의 단위가 있으면
    // 구분을 위해 "/"를 추가한다.
    // 추가된 양의 단위가 없으면
    // 지수에 '-'를 추가한다.
    // 기본적으로 단위는 -가 추가된 지수를 출력한다.
    // 하지만, 양의 단위가 하나라도 있으면 "/"를 추가한뒤에 '-'를 제거한
    // 단위를 추가해야 한다.
    if (ustring_size(aStr) > USTRING_EMPTY_SIZE){
        // 양의 단위가 먼저 추가되어 있다.
        Units* aTempUnits = NULL;
        short aTempUnitValue;
        size_t aSizePositiveUnitsSymbol;
        aSizePositiveUnitsSymbol = ustring_size(aStr);

        aTempUnits = create_units_default();
        assign_units(aTempUnits, tSelf);

        if(unit_length(&tSelf->mUnit) < 0){
            // 양의 단위기호가 있으므로
            // 구분기호 "/"를 추가한 뒤에
            // 지수에서 '-'를 제거한 절대값 단위 기호를 추가한다.
            append_ustring_from(aStr, "/");            
            get_units_abs_length_symbol(aTempUnits, aTempStr);
            append_ustring(aStr, aTempStr);
        }                    
        if(unit_mass(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > aSizePositiveUnitsSymbol){
                // 단위에 먼저 추가된 음의 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }
            else{
                // 음의 단위의 처음이면
                // 구분기호 "/"를 추가한 뒤에
                append_ustring_from(aStr, "/");
            }
            // 양의 단위기호가 있으므로
            // 지수에서 '-'를 제거한 절대값 단위 기호를 추가한다.            
            get_units_abs_mass_symbol(aTempUnits, aTempStr);
            append_ustring(aStr, aTempStr);
        }                            
        if(unit_time(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > aSizePositiveUnitsSymbol){
                // 단위에 먼저 추가된 음의 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }
            else{
                // 음의 단위의 처음이면
                // 구분기호 "/"를 추가한 뒤에
                append_ustring_from(aStr, "/");
            }
            // 양의 단위기호가 있으므로
            // 지수에서 '-'를 제거한 절대값 단위 기호를 추가한다.            
            get_units_abs_time_symbol(aTempUnits, aTempStr);
            append_ustring(aStr, aTempStr);            
        }                                    
        if(unit_angle(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > aSizePositiveUnitsSymbol){
                // 단위에 먼저 추가된 음의 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }
            else{
                // 음의 단위의 처음이면
                // 구분기호 "/"를 추가한 뒤에
                append_ustring_from(aStr, "/");
            }
            // 양의 단위기호가 있으므로
            // 지수에서 '-'를 제거한 절대값 단위 기호를 추가한다.            
            // append_ustring(aStr, &units_abs_angle_symbol(aTempUnits));
            get_units_abs_angle_symbol(aTempUnits, aTempStr);
            append_ustring(aStr, aTempStr);                        
        }
        delete_units(aTempUnits);
    }
    else{
        // 먼저 추가된 양의 단위 기호가 없다.
        // 지수에 '-'가 추가된 기호를 바로 추가한다.
        if(unit_length(&tSelf->mUnit) < 0){
            // 음수의 단위를 추가한다.
            // append_ustring(aStr, &units_length_symbol(tSelf));
            get_units_length_symbol(tSelf, aTempStr);
            append_ustring(aStr, aTempStr);                        
        }            
        if(unit_mass(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
                // 단위에 먼저 추가된 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }        
            // 음수의 단위를 추가한다.
            // append_ustring(aStr, &units_mass_symbol(tSelf));
            get_units_mass_symbol(tSelf, aTempStr);
            append_ustring(aStr, aTempStr);            
        }        
        if(unit_time(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
                // 단위에 먼저 추가된 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }        
            // 음수의 단위를 추가한다.
            // append_ustring(aStr, &units_time_symbol(tSelf));
            get_units_time_symbol(tSelf, aTempStr);
            append_ustring(aStr, aTempStr);            
        }
        if(unit_angle(&tSelf->mUnit) < 0){
            if(ustring_size(aStr) > USTRING_EMPTY_SIZE){
                // 단위에 먼저 추가된 단위가 있으면
                // 단위의 연결을 위해 "*"를 추가한다.
                append_ustring_from(aStr, "*");
            }        
            // 음수의 단위를 추가한다.
            // append_ustring(aStr, &units_angle_symbol(tSelf));
            get_units_angle_symbol(tSelf, aTempStr);
            append_ustring(aStr, aTempStr);            
        }
    }

    assign_ustring(retStr, aStr);
    delete_ustring(aTempStr);
    delete_ustring(aStr);
    return 0;    
}

LIBAPI int get_units_length_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_length(&tSelf->mUnit) == 0){
        // 단위의 값이 없으므로 빈 문자열을 반환한다.
        set_ustring(retStr, "");
        return 0;
    }
    if(units_prefix_length(tSelf) == UNIT_PREFIX_ZERO){
        // 접두어가 없어서 원래의 단위 기호를 반환한다.
        set_ustring(retStr, unit_length_symbol(&tSelf->mUnit));
        return 0;
    }
    Ustring *aStr = NULL;
    aStr = create_ustring_default();
    append_ustring_from(aStr, unit_prefix_symbol(tSelf->mPrefixList0));
    append_ustring_from(aStr, unit_length_symbol(&tSelf->mUnit));

    assign_ustring(retStr, aStr);
    delete_ustring(aStr);
    return 0;
}

LIBAPI int get_units_mass_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_mass(&tSelf->mUnit) == 0){
        // 단위의 값이 없으므로 빈 문자열을 반환한다.
        set_ustring(retStr, "");
        return 0;
    }
    if(units_prefix_mass(tSelf) == UNIT_PREFIX_ZERO){
        // 접두어가 없어서 원래의 단위 기호를 반환한다.
        set_ustring(retStr, unit_mass_symbol(&tSelf->mUnit));
        return 0;
    }
    Ustring *aStr = NULL;
    aStr = create_ustring_default();
    append_ustring_from(aStr, unit_prefix_symbol(tSelf->mPrefixList1));
    append_ustring_from(aStr, unit_mass_symbol(&tSelf->mUnit));

    assign_ustring(retStr, aStr);
    delete_ustring(aStr);
    return 0;
}

LIBAPI int get_units_time_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_time(&tSelf->mUnit) == 0){
        // 단위의 값이 없으므로 빈 문자열을 반환한다.
        set_ustring(retStr, "");
        return 0;
    }
    if(units_prefix_time(tSelf) == UNIT_PREFIX_ZERO){
        // 접두어가 없어서 원래의 단위 기호를 반환한다.
        set_ustring(retStr, unit_time_symbol(&tSelf->mUnit));
        return 0;
    }
    Ustring *aStr = NULL;
    aStr = create_ustring_default();
    append_ustring_from(aStr, unit_prefix_symbol(tSelf->mPrefixList2));
    append_ustring_from(aStr, unit_time_symbol(&tSelf->mUnit));

    assign_ustring(retStr, aStr);
    delete_ustring(aStr);
    return 0;
}

LIBAPI int get_units_angle_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_angle(&tSelf->mUnit) == 0){
        // 단위의 값이 없으므로 빈 문자열을 반환한다.
        set_ustring(retStr, "");
        return 0;
    }
    if(units_prefix_angle(tSelf) == UNIT_PREFIX_ZERO){
        // 접두어가 없어서 원래의 단위 기호를 반환한다.
        set_ustring(retStr, unit_angle_symbol(&tSelf->mUnit));
        return 0;
    }
    Ustring *aStr = NULL;
    aStr = create_ustring_default();
    append_ustring_from(aStr, unit_prefix_symbol(tSelf->mPrefixList3));
    append_ustring_from(aStr, unit_angle_symbol(&tSelf->mUnit));

    assign_ustring(retStr, aStr);
    delete_ustring(aStr);
    return 0;
}

LIBAPI int get_units_abs_length_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_length(&tSelf->mUnit) < 0){
        // 단위들의 값이 음수이다.
        // 절대값으로 변환해야 한다.

        // 임시 단위들을 생성한다.
        Units* aTempUnits = NULL;
        aTempUnits = create_units_default();
        assign_units(aTempUnits, tSelf);
        set_unit_length(&aTempUnits->mUnit, 
            unit_length(&aTempUnits->mUnit)*-1);

        Ustring *aStr = NULL;
        aStr = create_ustring_default();

        append_ustring_from(aStr, 
            unit_prefix_symbol(aTempUnits->mPrefixList0));
            
        append_ustring_from(aStr, unit_length_symbol(&aTempUnits->mUnit));

        delete_units(aTempUnits);

        assign_ustring(retStr, aStr);
        delete_ustring(aStr);
        return 0;
    }
    get_units_length_symbol(tSelf, retStr);
    return 0;
}

LIBAPI int get_units_abs_mass_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_mass(&tSelf->mUnit) < 0){
        // 단위들의 값이 음수이다.
        // 절대값으로 변환해야 한다.

        // 임시 단위들을 생성한다.
        Units* aTempUnits = NULL;
        aTempUnits = create_units_default();
        assign_units(aTempUnits, tSelf);
        set_unit_mass(&aTempUnits->mUnit, 
            unit_mass(&aTempUnits->mUnit)*-1);

        Ustring *aStr = NULL;
        aStr = create_ustring_default();

        append_ustring_from(aStr, 
            unit_prefix_symbol(aTempUnits->mPrefixList1));
            
        append_ustring_from(aStr, unit_mass_symbol(&aTempUnits->mUnit));

        delete_units(aTempUnits);

        assign_ustring(retStr, aStr);
        delete_ustring(aStr);
        return 0;
    }
    get_units_mass_symbol(tSelf, retStr);
    return 0;
}

LIBAPI int get_units_abs_time_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_time(&tSelf->mUnit) < 0){
        // 단위들의 값이 음수이다.
        // 절대값으로 변환해야 한다.

        // 임시 단위들을 생성한다.
        Units* aTempUnits = NULL;
        aTempUnits = create_units_default();
        assign_units(aTempUnits, tSelf);
        set_unit_time(&aTempUnits->mUnit, 
            unit_time(&aTempUnits->mUnit)*-1);

        Ustring *aStr = NULL;
        aStr = create_ustring_default();

        append_ustring_from(aStr, 
            unit_prefix_symbol(aTempUnits->mPrefixList2));
            
        append_ustring_from(aStr, unit_time_symbol(&aTempUnits->mUnit));

        delete_units(aTempUnits);

        assign_ustring(retStr, aStr);
        delete_ustring(aStr);
        return 0;
    }
    get_units_time_symbol(tSelf, retStr);
    return 0;
}

LIBAPI int get_units_abs_angle_symbol(const Units* tSelf, Ustring* retStr){
    if(unit_angle(&tSelf->mUnit) < 0){
        // 단위들의 값이 음수이다.
        // 절대값으로 변환해야 한다.

        // 임시 단위들을 생성한다.
        Units* aTempUnits = NULL;
        aTempUnits = create_units_default();
        assign_units(aTempUnits, tSelf);
        set_unit_angle(&aTempUnits->mUnit, 
            unit_angle(&aTempUnits->mUnit)*-1);

        Ustring *aStr = NULL;
        aStr = create_ustring_default();

        append_ustring_from(aStr, 
            unit_prefix_symbol(aTempUnits->mPrefixList3));
            
        append_ustring_from(aStr, unit_angle_symbol(&aTempUnits->mUnit));

        delete_units(aTempUnits);

        assign_ustring(retStr, aStr);
        delete_ustring(aStr);
        return 0;
    }
    get_units_angle_symbol(tSelf, retStr);
    return 0;
}

LIBAPI int add_units(Units* tSelf, const Units* tOther){
    if( unit_is_equal(&tSelf->mUnit, &tOther->mUnit)){
        add_unit_prefix_enum(&tSelf->mPrefixList0, 
            tOther->mPrefixList0, tSelf->mSnapPolicy);

        add_unit_prefix_enum(&tSelf->mPrefixList1, 
            tOther->mPrefixList1, tSelf->mSnapPolicy);

        add_unit_prefix_enum(&tSelf->mPrefixList2, 
            tOther->mPrefixList2, tSelf->mSnapPolicy);

        add_unit_prefix_enum(&tSelf->mPrefixList3, 
            tOther->mPrefixList3, tSelf->mSnapPolicy);

        add_unit(&tSelf->mUnit, &tOther->mUnit);
        return SUCCESS;
    }
    else{
        // 덧셈이 불가능이다.
        // return UNITS_IMPOSSIBLE_ADD;
        return FAIL;
    }
}

LIBAPI int sub_units(Units* tSelf, const Units* tOther);
LIBAPI int mul_units(Units* tSelf, const Units* tOther);
LIBAPI int div_units(Units* tSelf, const Units* tOther);

LIBAPI const Units units_add(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_sub(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_mul(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_div(const Units* tSelf, const Units* tOther);