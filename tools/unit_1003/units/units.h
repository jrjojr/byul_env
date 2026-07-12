/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* units 
*
* 단위의 요소
* 접두어
* 단위표식
* 
***************************************************************************/

#ifndef UNITS_H
#define UNITS_H

#include "units/units_config.h"

#include "unit.h"
#include "unit_prefix.h"
#include "ustring.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_units_version();
LIBAPI const char* units_version();

typedef struct s_units{
    Unit mUnit;
    UnitPrefixSnapPolicy mSnapPolicy;
    UnitPrefixEnum mPrefixList0;
    UnitPrefixEnum mPrefixList1;
    UnitPrefixEnum mPrefixList2;
    UnitPrefixEnum mPrefixList3;
}units_t;

typedef units_t Units;
typedef Units* UnitsPtr;

LIBAPI int init_units(Units* tSelf);
LIBAPI int release_units(Units* tSelf);

LIBAPI int set_units(Units* tSelf, const Unit* tUnit,
    const UnitPrefixEnum tLengthPrefix,
    const UnitPrefixEnum tMassPrefix,
    const UnitPrefixEnum tTimePrefix,
    const UnitPrefixEnum tAnglePrefix,
    const UnitPrefixSnapPolicy tSnapPolicy);

LIBAPI int set_units_unit(Units* tSelf, const Unit* tUnit);

// 모든 접두어를 연산해서 하나의 접두어를 찾는다.
// 하나의 접두어는 unit과 연관된 숫자에 적용할 것이다.
LIBAPI int set_units_prefix_total(Units* tSelf, const UnitPrefixEnum tPrefix);

LIBAPI int get_units_prefix_total(const Units* tSelf, UnitPrefixEnum* retPrefix);
LIBAPI const UnitPrefixEnum units_prefix_total(const Units* tSelf);

LIBAPI int set_units_prefix_length(Units* tSelf, 
    const UnitPrefixEnum tPrefix);

LIBAPI int set_units_prefix_mass(Units* tSelf, 
    const UnitPrefixEnum tPrefix);

LIBAPI int set_units_prefix_time(Units* tSelf, 
    const UnitPrefixEnum tPrefix);

LIBAPI int set_units_prefix_angle(Units* tSelf, 
    const UnitPrefixEnum tPrefix);

LIBAPI int set_units_prefix_snap_policy(Units* tSelf, 
    const UnitPrefixSnapPolicy tSnapPolicy);

LIBAPI int get_units(const Units* tSelf, Unit* retUnit,
    UnitPrefixEnum* retLengthPrefix, 
    UnitPrefixEnum* retMassPrefix, 
    UnitPrefixEnum* retTimePrefix, 
    UnitPrefixEnum* retAnglePrefix, 
    UnitPrefixSnapPolicy* retSnapPolicy);

LIBAPI int get_units_unit(const Units* tSelf, Unit* retUnit);

LIBAPI int get_units_prefix_length(const Units* tSelf, 
    UnitPrefixEnum* retPrefix);

LIBAPI int get_units_prefix_mass(const Units* tSelf, 
    UnitPrefixEnum* retPrefix);

LIBAPI int get_units_prefix_time(const Units* tSelf, 
    UnitPrefixEnum* retPrefix);

LIBAPI int get_units_prefix_angle(const Units* tSelf, 
    UnitPrefixEnum* retPrefix);

LIBAPI int get_units_prefix_snap_policy(const Units* tSelf, 
    UnitPrefixSnapPolicy* retSnapPolicy);

const Unit LIBAPI units_unit(const Units* tSelf);

const UnitPrefixSnapPolicy LIBAPI units_prefix_snap_policy(
    const Units* tSelf);    

LIBAPI const UnitPrefixEnum units_prefix_length(const Units* tSelf);

LIBAPI const UnitPrefixEnum units_prefix_mass(const Units* tSelf);

LIBAPI const UnitPrefixEnum units_prefix_time(const Units* tSelf);

LIBAPI const UnitPrefixEnum units_prefix_angle(const Units* tSelf);    

Units* LIBAPI create_units(const Unit* tUnit, 
    const UnitPrefixEnum tLengthPrefix,
    const UnitPrefixEnum tMassPrefix,
    const UnitPrefixEnum tTimePrefix,
    const UnitPrefixEnum tAnglePrefix,
    const UnitPrefixSnapPolicy tSnapPolicy);

Units* LIBAPI create_units_default();

LIBAPI int delete_units(Units* tSelf);

LIBAPI int assign_units(Units* tSelf, const Units* tOther);

LIBAPI bool units_equal(const Units* tSelf, const Units* tOther);

LIBAPI int set_units_prefix_at(Units* tSelf, int tIndex, 
    const UnitPrefixEnum tPrefix);

LIBAPI int get_units_prefix_at(const Units* tSelf, int tIndex, 
    UnitPrefixEnum* retPrefix);

LIBAPI const UnitPrefixEnum units_prefix_at(const Units* tSelf, int tIndex);

LIBAPI const char* units_symbol(const Units* tSelf, char* buf);

LIBAPI const char* units_length_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_mass_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_time_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_angle_symbol(const Units* tSelf, char* buf);

LIBAPI const char* units_abs_length_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_abs_mass_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_abs_time_symbol(const Units* tSelf, char* buf);
LIBAPI const char* units_abs_angle_symbol(const Units* tSelf, char* buf);

LIBAPI int get_units_symbol(const Units* tSelf, Ustring* retStr);

LIBAPI int get_units_length_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_mass_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_time_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_angle_symbol(const Units* tSelf, Ustring* retStr);

LIBAPI int get_units_abs_length_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_abs_mass_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_abs_time_symbol(const Units* tSelf, Ustring* retStr);
LIBAPI int get_units_abs_angle_symbol(const Units* tSelf, Ustring* retStr);

LIBAPI int add_units(Units* tSelf, const Units* tOther);
LIBAPI int sub_units(Units* tSelf, const Units* tOther);
LIBAPI int mul_units(Units* tSelf, const Units* tOther);
LIBAPI int div_units(Units* tSelf, const Units* tOther);

LIBAPI const Units units_add(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_sub(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_mul(const Units* tSelf, const Units* tOther);
LIBAPI const Units units_div(const Units* tSelf, const Units* tOther);

#ifdef __cplusplus
}
#endif

#endif // UNITS_H
