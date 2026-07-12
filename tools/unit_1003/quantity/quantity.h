/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* quantity 
*
* quantity = num * 10^unit_prefix unit
*
* quantity의 요소
* 숫자
* 단위
*
***************************************************************************/

#ifndef QUANTITY_H
#define QUANTITY_H

#include "quantity/quantity_config.h"
#include "units.h"
#include "dec_real.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QUANTITY_UNIT_NOT_EQUAL -10

LIBAPI void quantity_print_version();
LIBAPI const char* quantity_version(char* buf);

typedef struct s_quantity{
    DecReal mDecReal;
    Units mUnits;
}quantity_t;
typedef quantity_t Quantity;

LIBAPI int init_quantity(Quantity* tQuantity);
LIBAPI int release_quantity(Quantity* tQuantity);

LIBAPI int set_quantity_dec_real(Quantity* tQuantity, const DecReal* tDecReal);

LIBAPI int set_quantity_units(Quantity* tQuantity, const Units* tUnits);

LIBAPI const DecReal quantity_dec_real(const Quantity* tQuantity);

const Units LIBAPI quantity_units(const Quantity* tQuantity);

LIBAPI int assign_quantity(Quantity* tQuantity, const Quantity* tOther);
LIBAPI int add_quantity(Quantity* tQuantity, const Quantity* tOther);
LIBAPI int sub_quantity(Quantity* tQuantity, const Quantity* tOther);
LIBAPI int mul_quantity(Quantity* tQuantity, const Quantity* tOther);
LIBAPI int div_quantity(Quantity* tQuantity, const Quantity* tOther);

LIBAPI int pow_quantity(Quantity* tQuantity, const REAL tValue);

Quantity* LIBAPI create_quantity(const DecReal* tDecReal, const Units* tUnits);

Quantity* LIBAPI create_quantity_default();

LIBAPI int delete_quantity(Quantity* tSelf);

LIBAPI int get_quantity_dec_real(const Quantity* tQuantity, 
    DecReal* retDecReal);

LIBAPI int get_quantity_units(const Quantity* tQuantity, Units* retUnits);

LIBAPI bool quantity_equal(const Quantity* tQuantity, 
    const Quantity* tOther, const REAL tTol);

LIBAPI bool quantity_equal_rel(const Quantity* tQuantity, 
    const Quantity* tOther, const REAL tTol);


const Quantity LIBAPI quantity_add(const Quantity* tQuantity, 
    const Quantity* tOther);

const Quantity LIBAPI quantity_sub(const Quantity* tQuantity, 
    const Quantity* tOther);

const Quantity LIBAPI quantity_mul(const Quantity* tQuantity, 
    const Quantity* tOther);

const Quantity LIBAPI quantity_div(const Quantity* tQuantity, 
    const Quantity* tOther);

const Quantity LIBAPI quantity_pow(const Quantity* tQuantity, 
    const REAL tValue);

// 접두어와 숫자의 지수를 일치시킨다.
LIBAPI int update_quantity(Quantity* tSelf);

// 접두어를 숫자의 지수에 포함해서 실수의 값을 얻어낸다.
// quantity의 실수의 값은 DecReal * 10^접두어 이다.
LIBAPI const REAL quantity_value(const Quantity* tSelf);

// DecReal의 값만 얻어낸다.
LIBAPI const REAL quantity_dec_real_value(const Quantity* tSelf);

// 접두어를 숫자의 지수에 포함해서 실수의 값을 대입한다.
// quantity의 실수의 값은 DecReal * 10^접두어 이다.
LIBAPI int set_quantity_value(Quantity* tSelf, const REAL tValue);

// DecReal의 값만 대입한다.
LIBAPI int set_quantity_dec_real_value(Quantity* tSelf, 
    const REAL tValue);

// 접두어를 숫자의 지수에 포함해서 실수의 값을 얻어낸다.
// quantity의 실수의 값은 DecReal * 10^접두어 이다.
LIBAPI int get_quantity_value(const Quantity* tSelf, REAL* retValue);

// DecReal의 값만 얻어낸다.
LIBAPI int get_quantity_dec_real_value(const Quantity* tSelf, REAL* retValue);

// quantity의 값을 문자열로 얻는다.
LIBAPI int get_quantity_to_ustr(const Quantity* tSelf, Ustring* retStr);

// quantity의 값을 문자열로 얻는다.
// buf는 문자열이 자리잡을 저장소이다.
// 저장소가 있어야 문자열을 반환 받는다.
// 저장소에는 미리 메모리가 할당되어 있어야 한다.
LIBAPI const char* quantity_to(const Quantity* tSelf, char* buf);

#ifdef __cplusplus
}
#endif

#endif // QUANTITY_H
