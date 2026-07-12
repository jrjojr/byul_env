/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit_prefix 
*
***************************************************************************/

#include "unit_prefix.h"

#include <stdio.h>
#include <stdlib.h> // malloc realloc free

LIBAPI void print_unit_prefix_version(){
    printf("%s version : %d.%d.%d.%d\n", "unit_prefix", 
        UNIT_PREFIX_VERSION_MAJOR,
        UNIT_PREFIX_VERSION_MINOR,
        UNIT_PREFIX_VERSION_PATCH,
        UNIT_PREFIX_VERSION_TWEAK);
}

LIBAPI const char* unit_prefix_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_PREFIX_VERSION_MAJOR,
        UNIT_PREFIX_VERSION_MINOR,
        UNIT_PREFIX_VERSION_PATCH,
        UNIT_PREFIX_VERSION_TWEAK
        );
    return buf;
}

LIBAPI const UnitPrefixTable create_unit_prefix_table(){
    UnitPrefixTable aTable;
    aTable = (UnitPrefixTable)malloc(sizeof(int)*UNIT_PREFIX_TOTAL_COUNT);
    aTable[0] =     UNIT_PREFIX_QUETTA; //   =  30, // quetta   = 30,
    aTable[1] =     UNIT_PREFIX_RONNA;  //    =  27, // ronna    = 27,
    aTable[2] =     UNIT_PREFIX_YOTTA;  //    =  24, // yotta    = 24,
    aTable[3] =     UNIT_PREFIX_ZETTA;  //    =  21, // zetta    = 21,
    aTable[4] =     UNIT_PREFIX_EXA;    //      =  18, // exa      = 18,
    aTable[5] =     UNIT_PREFIX_PETA;   //     =  15, // peta     = 15,
    aTable[6] =     UNIT_PREFIX_TERA;   //     =  12, // tera     = 12,
    aTable[7] =     UNIT_PREFIX_GIGA;   //     =   9, // giga     = 9,
    aTable[8] =     UNIT_PREFIX_MEGA;   //     =   6, // mega     = 6,
    aTable[9] =     UNIT_PREFIX_KILO;   //     =   3, // kilo     = 3,
    aTable[10] =     UNIT_PREFIX_HECTO;  //    =   2, // hecto    = 2,
    aTable[11] =     UNIT_PREFIX_DECA;   //     =   1, // deca     = 1,
    aTable[12] =     UNIT_PREFIX_ZERO;   //     =   0, // zero     = 0,
    aTable[13] =     UNIT_PREFIX_DECI;   //     =  -1, // deci     = -1,
    aTable[14] =     UNIT_PREFIX_CENTI;  //    =  -2, // centi    = -2,
    aTable[15] =     UNIT_PREFIX_MILLI;  //    =  -3, // milli    = -3,
    aTable[16] =     UNIT_PREFIX_MICRO;  //    =  -6, // micro    = -6,
    aTable[17] =     UNIT_PREFIX_NANO;   //     =  -9, // nano     = -9,
    aTable[18] =     UNIT_PREFIX_PICO;   //     = -12, // pico     = -12,
    aTable[19] =     UNIT_PREFIX_FEMTO;  //    = -15, // femto    = -15,
    aTable[20] =     UNIT_PREFIX_ATTO;   //     = -18, // atto     = -18,
    aTable[21] =     UNIT_PREFIX_ZEPTO;  //    = -21, // zepto    = -21,
    aTable[22] =     UNIT_PREFIX_YOCTO;  //    = -24, // yocto    = -24,
    aTable[23] =     UNIT_PREFIX_RONTO;  //    = -27, // ronto    = -27,
    aTable[24] =     UNIT_PREFIX_QUECTO; //   = -30  // quecto   = -30
    return aTable; // 0;
}

LIBAPI int delete_table(UnitPrefixTable tTable){
    if (tTable != NULL){
        free(tTable);
        return 0;
    }
    return -1;
}

LIBAPI const UnitPrefixEnum unit_prefix_enum_snap_from(const int tExp,
    const UnitPrefixSnapPolicy tPolicy){

// 정책에 따라 tExp에서 가까운 접두어 하나를 선택한다.
    bool aSign;
    int aExp;
    int aRemainder;

    if (tExp == 0){
        return UNIT_PREFIX_ZERO;
    }
    else if (tExp < 0){
        if ( tExp < UNIT_PREFIX_QUECTO){
            // 가장 작은 접두어보다 작다.
            return UNIT_PREFIX_QUECTO;
        }

        if (tExp > -3){
            // 0, -1, -2
            // 0은 아니다. 
            // 먼저 검사했다.
            // 지수가 바로 접두어가 된다.
            return tExp;
        }
        aSign = true;
    }
    else{
        if (tExp > UNIT_PREFIX_QUETTA){
            // 가장 큰 접두어보다 크다.
            return UNIT_PREFIX_QUETTA;
        }

        if (tExp < 3){
            // 0, 1, 2
            // 0은 아니다. 
            // 먼저 검사했다.
            // 지수가 바로 접두어가 된다.
            return tExp;            
        }
        aSign = false;
    }

    aExp = tExp;
    // tExp는  +- 3 ~ 30 사이에 있다.
    // aExp가 3의 배수에 해당되는 지 확인한다.
    // 3으로 나눴을 때 나머지가 0이면 3의 배수이다.
    aRemainder = tExp % 3;
    if (aRemainder == 0){
        // 그러면 바로 tExp를 반환한다.
        return tExp;
    }
    else{
        // 나머지는 1 또는 2이다.
        aExp /= 3;

        switch(tPolicy){
        case UNIT_PREFIX_SNAP_POLICY_NORMAL:
        // 스냅에서 NORMAL은 의미가 없다.
        break;

        case UNIT_PREFIX_SNAP_POLICY_ZERO_NEAREST:
        // 부호를 확인해서 양수이면
        // 작은 값을 선택한다.
        // 음수이면,
        // 큰 값을 선택한다.
        // 예를 들어 +- 4는 3과 6사이에 있는데 작은 값은 3이다.
        //      -4는 -3과 -6 사이에 있는데 큰 값은 -3이다.
        return aExp * 3;

        case UNIT_PREFIX_SNAP_POLICY_ZERO_FARTHEST:
        // 부호를 확인해서 양수이면
        // 큰 값을 선택한다.
        // 음수이면,
        // 작은 값을 선택한다.
        return (aExp+1) * 3;

        case UNIT_PREFIX_SNAP_POLICY_LOW:
        // 부호를 확인해서 작은 값을 선택한다.
        // 예를 들어 4는 3과 6사이에 있는데 작은 값은 3이다.
        // -4는 -3과 -6사이에 있는데 작은 값은 -6이다.
        if (aSign){
            // 부호가 음수이다.
            return (aExp+1) * 3;
        }
        return aExp * 3;

        case UNIT_PREFIX_SNAP_POLICY_HIGH:
        if (aSign){
            // 부호가 음수이다.
            return aExp * 3;
        }
        return (aExp+1) * 3;
        }        
    }
    // 기본적으로 0에 가까운 값을 선택한다
    return aExp * 3;    
}

LIBAPI const int unit_prefix_enum_snap_to(const UnitPrefixEnum tPrefix){
// 접두어에서 가까운 정수를 스냅하는 것은 정책이 필요없다.
// 접두어에서 바로 정수가 추출 된다.
    return tPrefix;
}

// 접두어 를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy){

    int aDistantFromZeroSelf = abs(*retSelf - 0);
    int aDistantFromZeroOther = abs(tOther - 0);

    // 접두어 덧셈은 원하는 접두어를 선택해서 self에 할당하는 것이다.
    switch(tPolicy){
    case UNIT_PREFIX_SNAP_POLICY_NORMAL:
    // self의 접두어를 사용한다.
    return SUCCESS;

    case UNIT_PREFIX_SNAP_POLICY_HIGH:
    // 높은 접두어를 사용한다.
    if ( *retSelf < tOther){
        *retSelf  = tOther;
    }
    return SUCCESS;

    case UNIT_PREFIX_SNAP_POLICY_LOW:
    // 낮은 접두어를 사용한다.
    if ( *retSelf > tOther){
        *retSelf  = tOther;
    }
    return SUCCESS;

    case UNIT_PREFIX_SNAP_POLICY_ZERO_NEAREST:
    // 0에 가까운 접두어를 사용한다.
    if ( aDistantFromZeroSelf > aDistantFromZeroOther){
        *retSelf  = tOther;
    }    
    return SUCCESS;

    case UNIT_PREFIX_SNAP_POLICY_ZERO_FARTHEST:
    // 0에서 먼 접두어를 사용한다.
    if ( aDistantFromZeroSelf < aDistantFromZeroOther){
        *retSelf  = tOther;
    }        
    return SUCCESS;

    }
    // 정책이 없다. self가 초기화가 안되어 있다.    
    return FAIL;
}

// 접두어 를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy){
    
    // 접두어 뺄셈은 add와 같다.
    return add_unit_prefix_enum(retSelf, tOther, tPolicy);
}

// 접두어 를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy){
    
    // 접두어끼리 덧셈한다.
    // 덧셈하는 데 정책은 없어도 된다.
    *retSelf += tOther;
    if( *retSelf > UNIT_PREFIX_QUETTA){
        *retSelf = UNIT_PREFIX_QUETTA;
    }
    return SUCCESS;    
}

LIBAPI int div_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy){
    
    // 접두어끼리 뺄셈한다.
    // 뺄셈하는 데 정책은 없어도 된다.
    *retSelf -= tOther;
    if(*retSelf < UNIT_PREFIX_QUECTO){
        *retSelf = UNIT_PREFIX_QUECTO;
    }

    return SUCCESS;    
}

LIBAPI int pow_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy){
    // 접두어끼리 곱셈 한다.
    // 곱셈하는 데 정책은 없어도 된다.
    *retSelf *= tOther;
    if( *retSelf > UNIT_PREFIX_QUETTA){
        *retSelf = UNIT_PREFIX_QUETTA;
    }
    if(*retSelf < UNIT_PREFIX_QUECTO){
        *retSelf = UNIT_PREFIX_QUECTO;
    }
    return SUCCESS;    
}

LIBAPI const char* unit_prefix_name(UnitPrefixEnum tEnum){
    switch(tEnum){
        case UNIT_PREFIX_QUETTA  : 
            // quetta   = 30,
            return "quetta";
        case UNIT_PREFIX_RONNA   : 
            // ronna    = 27,
            return "ronna";
        case UNIT_PREFIX_YOTTA   : 
            // yotta    = 24,
            return "yotta";
        case UNIT_PREFIX_ZETTA   : 
            // zetta    = 21,
            return "zetta";
        case UNIT_PREFIX_EXA     : 
            // exa      = 18,
            return "exa";
        case UNIT_PREFIX_PETA    : 
            // peta     = 15,
            return "peta";
        case UNIT_PREFIX_TERA    : 
            // tera     = 12,
            return "tera";
        case UNIT_PREFIX_GIGA    : 
            // giga     = 9,
            return "giga";
        case UNIT_PREFIX_MEGA    : 
            // mega     = 6,
            return "mega";
        case UNIT_PREFIX_KILO    : 
            // kilo     = 3,
            return "kilo";
        case UNIT_PREFIX_HECTO   : 
            // hecto    = 2,
            return "hecto";
        case UNIT_PREFIX_DECA    : 
            // deca     = 1,
            return "deca";
        case UNIT_PREFIX_ZERO    : 
            // zero     = 0,
            return "zero";
        case UNIT_PREFIX_DECI    : 
            // deci     = -1,
            return "deci";
        case UNIT_PREFIX_CENTI   : 
            // centi    = -2,
            return "centi";
        case UNIT_PREFIX_MILLI   : 
            // milli    = -3,
            return "milli";
        case UNIT_PREFIX_MICRO   : 
            // micro    = -6,
            return "micro";
        case UNIT_PREFIX_NANO    : 
            // nano     = -9,
            return "nano";
        case UNIT_PREFIX_PICO    : 
            // pico     = -12,
            return "pico";
        case UNIT_PREFIX_FEMTO   : 
            // femto    = -15,
            return "femto";
        case UNIT_PREFIX_ATTO    : 
            // atto     = -18,
            return "atto";
        case UNIT_PREFIX_ZEPTO   : 
            // zepto    = -21,
            return "zepto";
        case UNIT_PREFIX_YOCTO   : 
            // yocto    = -24,
            return "yocto";
        case UNIT_PREFIX_RONTO   : 
            // ronto    = -27,
            return "ronto";
        case UNIT_PREFIX_QUECTO  : 
            // quecto   = -30
            return "quecto";
    }
    return "";
}    

LIBAPI const char* unit_prefix_symbol(UnitPrefixEnum tEnum){
    switch(tEnum){
        case UNIT_PREFIX_QUETTA  : 
            // quetta   = 30,
            return "Q";
        case UNIT_PREFIX_RONNA   : 
            // ronna    = 27,
            return "R";
        case UNIT_PREFIX_YOTTA   : 
            // yotta    = 24,
            return "Y";
        case UNIT_PREFIX_ZETTA   : 
            // zetta    = 21,
            return "Z";
        case UNIT_PREFIX_EXA     : 
            // exa      = 18,
            return "E";
        case UNIT_PREFIX_PETA    : 
            // peta     = 15,
            return "P";
        case UNIT_PREFIX_TERA    : 
            // tera     = 12,
            return "T";
        case UNIT_PREFIX_GIGA    : 
            // giga     = 9,
            return "G";
        case UNIT_PREFIX_MEGA    : 
            // mega     = 6,
            return "M";
        case UNIT_PREFIX_KILO    : 
            // kilo     = 3,
            return "k";
        case UNIT_PREFIX_HECTO   : 
            // hecto    = 2,
            return "h";
        case UNIT_PREFIX_DECA    : 
            // deca     = 1,
            return "da";
        case UNIT_PREFIX_ZERO    : 
            // zero     = 0,
            return "";
        case UNIT_PREFIX_DECI    : 
            // deci     = -1,
            return "d";
        case UNIT_PREFIX_CENTI   : 
            // centi    = -2,
            return "c";
        case UNIT_PREFIX_MILLI   : 
            // milli    = -3,
            return "m";
        case UNIT_PREFIX_MICRO   : 
            // micro    = -6,
            return "u";
        case UNIT_PREFIX_NANO    : 
            // nano     = -9,
            return "n";
        case UNIT_PREFIX_PICO    : 
            // pico     = -12,
            return "p";
        case UNIT_PREFIX_FEMTO   : 
            // femto    = -15,
            return "f";
        case UNIT_PREFIX_ATTO    : 
            // atto     = -18,
            return "a";
        case UNIT_PREFIX_ZEPTO   : 
            // zepto    = -21,
            return "z";
        case UNIT_PREFIX_YOCTO   : 
            // yocto    = -24,
            return "y";
        case UNIT_PREFIX_RONTO   : 
            // ronto    = -27,
            return "r";
        case UNIT_PREFIX_QUECTO  : 
            // quecto   = -30
            return "q";
    }
    return "";    
}

