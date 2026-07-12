/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit_prefix
*
* SI 접두어
* 
* 10n	    접두어              기호	배수
* 
* 10^30	    퀘타 (quetta)	    Q	    백양            
* 10^27	    론나 (ronna)	    R	    천자            
* 10^24	    요타 (yotta)	    Y	    일자            
* 10^21	    제타 (zetta)	    Z	    십해            
* 10^18	    엑사 (exa)	        E	    백경            
* 10^15	    페타 (peta)	        P	    천조            
* 10^12	    테라 (tera)	        T	    일조            
* 10^9	    기가 (giga)	        G	    십억            
* 10^6	    메가 (mega)	        M	    백만            
* 10^3	    킬로 (kilo)	        k	    천	            
* 10^2	    헥토 (hecto)	    h	    백	            
* 10^1	    데카 (deca)	        da	    십	            
* 10^0			                        일	            
* 10^-1	    데시 (deci)	        d	    십분의 일       
* 10^-2	    센티 (centi)	    c	    백분의 일       
* 10^-3	    밀리 (milli)	    m	    천분의 일       
* 10^-6	    마이크로 (micro)	u	    백만분의 일     
* 10^-9	    나노 (nano)	        n	    십억분의 일     
* 10^-12	피코 (pico)	        p	    일조분의 일 
* 10^-15	펨토 (femto)	    f	    천조분의 일 
* 10^-18	아토 (atto)	        a	    백경분의 일 
* 10^-21	젭토 (zepto)	    z	    십해분의 일 
* 10^-24	욕토 (yocto)	    y	    일자분의 일 
* 10^-27	론토 (ronto)	    r	    천자분의 일 
* 10^-30	퀙토 (quecto)	    q	    백양분의 일 
*
***************************************************************************/

#ifndef UNIT_PREFIX_H
#define UNIT_PREFIX_H

#include "unit_prefix/unit_prefix_config.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_unit_prefix_version();
LIBAPI const char* unit_prefix_version();

/* mm um를 op(add, sub, mul, div)할 때 접두어를 선택하는 규칙이다.
* 일반 : 앞의 접두어를 사용한다. 접두어를 변경하지 않는다는 뜻이다.
* 높은 값 : mm
* 낮은 값 : um
* 1 kilo m * 1 mega m = 1 kilo m * 1000 kilo m = 1000 kilo m^2
* 1 mega m * 1 kilo m = 0.001 mega m * 1 mega m = 0.001 mega m^2
*/
typedef enum e_unit_prefix_snap_policy{
    UNIT_PREFIX_SNAP_POLICY_NORMAL,
    UNIT_PREFIX_SNAP_POLICY_ZERO_NEAREST,
    UNIT_PREFIX_SNAP_POLICY_ZERO_FARTHEST,
    UNIT_PREFIX_SNAP_POLICY_LOW,
    UNIT_PREFIX_SNAP_POLICY_HIGH
}unit_prefix_snap_policy_m;
typedef unit_prefix_snap_policy_m UnitPrefixSnapPolicy;

#define UNIT_PREFIX_TOTAL_COUNT     25

typedef enum e_unit_prefix{
    UNIT_PREFIX_QUETTA   =  30, // quetta   = 30,
    UNIT_PREFIX_RONNA    =  27, // ronna    = 27,

    UNIT_PREFIX_YOTTA    =  24, // yotta    = 24,
    UNIT_PREFIX_ZETTA    =  21, // zetta    = 21,
    UNIT_PREFIX_EXA      =  18, // exa      = 18,
    UNIT_PREFIX_PETA     =  15, // peta     = 15,
    UNIT_PREFIX_TERA     =  12, // tera     = 12,
    
    UNIT_PREFIX_GIGA     =   9, // giga     = 9,
    UNIT_PREFIX_MEGA     =   6, // mega     = 6,
    UNIT_PREFIX_KILO     =   3, // kilo     = 3,
    UNIT_PREFIX_HECTO    =   2, // hecto    = 2,
    UNIT_PREFIX_DECA     =   1, // deca     = 1,
    
    UNIT_PREFIX_ZERO     =   0, // zero     = 0,
    
    UNIT_PREFIX_DECI     =  -1, // deci     = -1,
    UNIT_PREFIX_CENTI    =  -2, // centi    = -2,
    UNIT_PREFIX_MILLI    =  -3, // milli    = -3,
    UNIT_PREFIX_MICRO    =  -6, // micro    = -6,
    UNIT_PREFIX_NANO     =  -9, // nano     = -9,

    UNIT_PREFIX_PICO     = -12, // pico     = -12,
    UNIT_PREFIX_FEMTO    = -15, // femto    = -15,
    UNIT_PREFIX_ATTO     = -18, // atto     = -18,
    UNIT_PREFIX_ZEPTO    = -21, // zepto    = -21,
    UNIT_PREFIX_YOCTO    = -24, // yocto    = -24,

    UNIT_PREFIX_RONTO    = -27, // ronto    = -27,
    UNIT_PREFIX_QUECTO   = -30  // quecto   = -30
}unit_prefix_m;
typedef unit_prefix_m UnitPrefixEnum;
typedef UnitPrefixEnum* UnitPrefixTable;

// extern char[UNIT_PREFIX_TOTAL_COUNT] UnitPrefixTable;
// 모든 단위 접두어가 하나의 테이블에 있다.
LIBAPI const UnitPrefixTable create_unit_prefix_table();
LIBAPI int delete_unit_prefix_table(UnitPrefixTable tTable);

// 정책에 따라 tExp에서 가까운 접두어 하나를 선택한다.
LIBAPI const UnitPrefixEnum unit_prefix_enum_snap_from(const int tExp,
    const UnitPrefixSnapPolicy tPolicy);

// 접두어에서 가까운 정수를 스냅하는 것은 정책이 필요없다.
// 접두어에서 바로 정수가 추출 된다.
LIBAPI const int unit_prefix_enum_snap_to(const UnitPrefixEnum tPrefix);    

LIBAPI const char* unit_prefix_name(UnitPrefixEnum tEnum);
LIBAPI const char* unit_prefix_symbol(UnitPrefixEnum tEnum);

// 접두어 를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy);

// 접두어 를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy);

// 접두어 를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy);

// 접두어 를 나눗셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int div_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy);

// 접두어 를 지수연산한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int pow_unit_prefix_enum(UnitPrefixEnum* retSelf, 
    const UnitPrefixEnum tOther, const UnitPrefixSnapPolicy tPolicy);    

#ifdef __cplusplus
}
#endif

#endif // UNIT_PREFIX_H
