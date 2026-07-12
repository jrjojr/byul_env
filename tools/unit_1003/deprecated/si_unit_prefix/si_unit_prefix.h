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

#ifndef SI_UNIT_PREFIX_H
#define SI_UNIT_PREFIX_H

#include "unit/unit_config.h"
#include "unit.h"

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_si_unit_prefix_version();
LIBAPI const char* si_unit_prefix_version();

/* mm um를 op(add, sub, mul, div)할 때 접두어를 선택하는 규칙이다.
* 일반 : 앞의 접두어를 사용한다. 접두어를 변경하지 않는다는 뜻이다.
* 높은 값 : mm
* 낮은 값 : um
* 1 kilo m * 1 mega m = 1 kilo m * 1000 kilo m = 1000 kilo m^2
* 1 mega m * 1 kilo m = 0.001 mega m * 1 mega m = 0.001 mega m^2
*/
typedef enum e_si_unit_prefix_op_policy{
    SI_UNIT_PREFIX_OP_POLICY_NORMAL,
    SI_UNIT_PREFIX_OP_POLICY_HIGH,
    SI_UNIT_PREFIX_OP_POLICY_LOW,
    SI_UNIT_PREFIX_OP_POLICY_ZERO_NEAREST,
    SI_UNIT_PREFIX_OP_POLICY_ZERO_FARTHEST
}si_unit_prefix_op_policy_m;
typedef si_unit_prefix_op_policy_m SiUnitPrefixOpPolicy;

typedef enum e_si_unit_prefix{
    SI_UNIT_PREFIX_QUETTA   =  30, // quetta   = 30,
    SI_UNIT_PREFIX_RONNA    =  27, // ronna    = 27,
    SI_UNIT_PREFIX_YOTTA    =  24, // yotta    = 24,
    SI_UNIT_PREFIX_ZETTA    =  21, // zetta    = 21,
    SI_UNIT_PREFIX_EXA      =  18, // exa      = 18,
    SI_UNIT_PREFIX_PETA     =  15, // peta     = 15,
    SI_UNIT_PREFIX_TERA     =  12, // tera     = 12,
    SI_UNIT_PREFIX_GIGA     =   9, // giga     = 9,
    SI_UNIT_PREFIX_MEGA     =   6, // mega     = 6,
    SI_UNIT_PREFIX_KILO     =   3, // kilo     = 3,
    SI_UNIT_PREFIX_HECTO    =   2, // hecto    = 2,
    SI_UNIT_PREFIX_DECA     =   1, // deca     = 1,
    SI_UNIT_PREFIX_ZERO     =   0, // zero     = 0,
    SI_UNIT_PREFIX_DECI     =  -1, // deci     = -1,
    SI_UNIT_PREFIX_CENTI    =  -2, // centi    = -2,
    SI_UNIT_PREFIX_MILLI    =  -3, // milli    = -3,
    SI_UNIT_PREFIX_MICRO    =  -6, // micro    = -6,
    SI_UNIT_PREFIX_NANO     =  -9, // nano     = -9,
    SI_UNIT_PREFIX_PICO     = -12, // pico     = -12,
    SI_UNIT_PREFIX_FEMTO    = -15, // femto    = -15,
    SI_UNIT_PREFIX_ATTO     = -18, // atto     = -18,
    SI_UNIT_PREFIX_ZEPTO    = -21, // zepto    = -21,
    SI_UNIT_PREFIX_YOCTO    = -24, // yocto    = -24,
    SI_UNIT_PREFIX_RONTO    = -27, // ronto    = -27,
    SI_UNIT_PREFIX_QUECTO   = -30  // quecto   = -30
}si_unit_prefix_m;
typedef si_unit_prefix_m SiUnitPrefixEnum;

typedef struct s_si_unit_prefix{
    SiUnitPrefixEnum mPrefix;
    SiUnitPrefixOpPolicy mPolicy;
}si_unit_prefix_t;
typedef si_unit_prefix_t SiUnitPrefix;

LIBAPI int init_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix);
LIBAPI int release_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix);

LIBAPI int assign_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefix* tOther);

LIBAPI int set_si_unit_prefix_op_policy(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefixOpPolicy tOpPolicy);

const SiUnitPrefixOpPolicy LIBAPI get_si_unit_prefix_op_policy(
    const SiUnitPrefix* tSiUnitPrefix);

LIBAPI int set_si_unit_prefix(SiUnitPrefix* tSiUnitPrefix, 
    const SiUnitPrefixEnum tEnum);

const SiUnitPrefixEnum LIBAPI get_si_unit_prefix(
    const SiUnitPrefix* tSiUnitPrefix);

// 접두어 를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther);

// 접두어 를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther);

// 접두어 를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther);

// 접두어 를 나눗셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int div_si_unit_prefix(
    SiUnitPrefix* tSiUnitPrefix, const SiUnitPrefix* tOther);    

LIBAPI bool is_si_unit_prefix_equal(
    const SiUnitPrefix* tSelf, const SiUnitPrefix* tOther);

LIBAPI const char* get_si_unit_prefix_name(const SiUnitPrefix* tSelf);    

typedef struct s_si_unit_prefix_list{
    SiUnitPrefix mLengthSiUnitPrefix;
    SiUnitPrefix mMassSiUnitPrefix;
    SiUnitPrefix mTimeSiUnitPrefix;
    SiUnitPrefix mAngleSiUnitPrefix;
    SiUnitPrefix mElectricCurrentSiUnitPrefix;
    SiUnitPrefix mThermodynamicTemperatureSiUnitPrefix;
    SiUnitPrefix mAmountOfSubstanceSiUnitPrefix;
    SiUnitPrefix mLuminousIntensitySiUnitPrefix;
}si_unit_prefix_list_t;
typedef si_unit_prefix_list_t SiUnitPrefixList;

LIBAPI int init_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList);
LIBAPI int release_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList);

LIBAPI int assign_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefixList* tOther);

LIBAPI int set_si_unit_prefix_list(SiUnitPrefixList* tSiUnitPrefixList, 
    SiUnitPrefixEnum tLengthSiUnitPrefix,
    SiUnitPrefixEnum tMassSiUnitPrefix,
    SiUnitPrefixEnum tTimeSiUnitPrefix,
    SiUnitPrefixEnum tAngleSiUnitPrefix,
    SiUnitPrefixEnum tElectricCurrentSiUnitPrefix,
    SiUnitPrefixEnum tThermodynamicTemperatureSiUnitPrefix,
    SiUnitPrefixEnum tAmountOfSubstanceSiUnitPrefix,
    SiUnitPrefixEnum tLuminousIntensitySiUnitPrefix
    );

LIBAPI int set_si_unit_prefix_list_length(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_mass(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_time(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_angle(SiUnitPrefixList* tSiUnitPrefixList, 
    const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_electric_current(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix);    

LIBAPI int set_si_unit_prefix_list_thermodynamic_temperature(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_amount_of_substance(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix);

LIBAPI int set_si_unit_prefix_list_luminous_intensity(
    SiUnitPrefixList* tSiUnitPrefixList, const SiUnitPrefix tPrefix);    

const SiUnitPrefixList* LIBAPI get_si_unit_prefix_list(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_length(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_mass(
    const SiUnitPrefixList* tSiUnitPrefixList);    

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_time(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_angle(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_electric_current(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_thermodynamic_temperature(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_amount_of_substance(
    const SiUnitPrefixList* tSiUnitPrefixList);

const SiUnitPrefix LIBAPI get_si_unit_prefix_list_luminous_intensity(
    const SiUnitPrefixList* tSiUnitPrefixList);

// 접두어 리스트를 덧셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int add_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther);

// 접두어 리스트를 뺄셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int sub_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther);

// 접두어 리스트를 곱셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int mul_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther);

// 접두어 리스트를 나눗셈한다.
// 정책에 따라 prefix를 할당한다.
LIBAPI int div_si_unit_prefix_list(
    SiUnitPrefixList* tSiUnitPrefix, const SiUnitPrefixList* tOther);    

LIBAPI int si_unit_prefix_list(const SiUnitPrefixList* tSiUnitPrefix,
    SiUnitPrefixEnum* retLengthSiUnitPrefix,
    SiUnitPrefixEnum* retMassSiUnitPrefix,
    SiUnitPrefixEnum* retTimeSiUnitPrefix,
    SiUnitPrefixEnum* retAngleSiUnitPrefix,
    SiUnitPrefixEnum* retElectricCurrentSiUnitPrefix,
    SiUnitPrefixEnum* retThermodynamicTemperatureSiUnitPrefix,
    SiUnitPrefixEnum* retAmountOfSubstanceSiUnitPrefix,
    SiUnitPrefixEnum* retLuminousIntensitySiUnitPrefix
);

LIBAPI bool is_si_unit_prefix_list_equal(
    const SiUnitPrefixList* tSelf, const SiUnitPrefixList* tOther);

#ifdef __cplusplus
}
#endif

#endif // SI_UNIT_PREFIX_H
