/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* derived_si_unit
*
* derived_si_unit는 derived_si_unit 여러개를 연산한다.
* 연산을 하고 난뒤 결과를 저장한다.
* prefix를 연산하고 규칙에 맞게 각각의 DerivedSiUnit 에 적용한다.
* SiUnit에 연산 결과를 적용한다.
* SiUnit는 하나만 사용하고, prefix는 리스트로 있어야 겠다.
***************************************************************************/

#ifndef DERIVED_SI_UNIT_H
#define DERIVED_SI_UNIT_H

#include "derived_si_unit/derived_si_unit_config.h"
#include "si_unit.h"
#include "si_prefix.h"

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_derived_si_unit_version(char* buf);
LIBAPI const char* derived_si_unit_version(char* buf);

// typedef enum e_si_prefix_prefer_type{
//     PREFIX_NORMAL,
//     PREFIX_SMALL_LETTERS,
//     PREFIX_ENG_SMALL,
//     PREFIX_ENG_LARGE,
//     PREFIX_LARGE_LETTERS    
// }si_prefix_prefer_type_m;
// typedef si_prefix_prefer_type_m SiUnitPrefixPreferType;

typedef struct s_derived_si_unit{
    SiUnit mSiUnit;
    SiUnitPrefixList mSiUnitPrefixList;
}derived_si_unit_t;
typedef derived_si_unit_t DerivedSiUnit;
typedef DerivedSiUnit* DerivedSiUnitPtr;

LIBAPI int init_derived_si_unit(DerivedSiUnit* tUnit);
LIBAPI int release_derived_si_unit(DerivedSiUnit* tUnit);

LIBAPI int assign_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther);

LIBAPI int assign_derived_si_unit_si_unit(DerivedSiUnit* tUnit, 
    const SiUnit* tOther);    

LIBAPI int assign_derived_si_unit_unit(DerivedSiUnit* tUnit, 
    const Unit* tOther);

LIBAPI int set_derived_si_prefix_list(DerivedSiUnit* tUnit, 
    const SiUnitPrefixList* tPrefixList);

LIBAPI int set_derived_si_unit_si_unit(DerivedSiUnit* tUnit, 
    const SiUnit* tSiUnit);

const SiUnitPrefixList LIBAPI get_derived_si_prefix_list(
    const DerivedSiUnit* tUnit);

const SiUnit LIBAPI get_derived_si_unit_si_unit(const DerivedSiUnit* tUnit);

LIBAPI int add_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther);

LIBAPI int sub_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther);

LIBAPI int mul_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther);

LIBAPI int div_derived_si_unit(DerivedSiUnit* tUnit, 
    const DerivedSiUnit* tOther);

LIBAPI bool is_derived_si_unit_equal(
    DerivedSiUnit* tSelf, DerivedSiUnit* tOther);

#ifdef __cplusplus
}
#endif

#endif // DERIVED_SI_UNIT_H
