/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec_real 1.0.2.2
*
* 십진수의 실수는 decIntPart.decFracPart * 10^decExp이다
* 실수부는 decIntPart.decFracPart -> decIntPart + 0.decFracPart
* 실수부는 decIntPart.decFracPart -> decIntPart * 1.decFracPart
* 실수부는 1234567890123456789.01234567890123456789
* 지수부는 10^decExp
***************************************************************************/

#ifndef DEC_REAL_H
#define DEC_REAL_H

#include "unit/unit_config.h"
#include "real.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REAL_OVERFLOW           -11
#define REAL_UNDERFLOW          -12

#define REAL_SIG_DIGITS              9  
#define REAL_MAX                999999999
#define DEC_REAL_MIN                -999999999
#define DEC_REAL_EXP_MAX                 99
#define DEC_REAL_EXP_MIN                 -99

#define DBL_DEC                 int64_t

#define DEC_REAL_UNDERFLOW    110
#define DEC_REAL_OVERFLOW     111

LIBAPI void dec_real_print_version();
LIBAPI const char* dec_real_version(char* buf);

// typedef struct s_dec_frac{
//     DEC mFrac;
//     DEC mExtraFrac;
//     char mZeroCount;
// }dec_frac_t;
// typedef dec_frac_t DecFrac;
// typedef DecFrac* DecFracPtr;

// LIBAPI int init_dec_frac(DecFrac* tSelf);
// LIBAPI int release_dec_frac(DecFrac* tSelf);

// // tFrac 범위 0 ~ REAL_MAX
// // tZeroCount 범위 -REAL_SIG_DIGITS-1 ~ REAL_SIG_DIGITS-1
// LIBAPI int set_dec_frac(DecFrac* tSelf, DEC tFrac, char tZeroCount);

// // tFrac 범위 0 ~ REAL_MAX
// LIBAPI int set_dec_frac_frac(DecFrac* tSelf, DEC tFrac);

// // tZeroCount 범위 -REAL_SIG_DIGITS-1 ~ REAL_SIG_DIGITS-1
// LIBAPI int set_dec_frac_zero_count(DecFrac* tSelf, char tZeroCount);

// LIBAPI int get_dec_frac(DecFrac* tSelf, DEC* retFrac, char* retZeroCount);

// LIBAPI int get_dec_frac_frac(DecFrac* tSelf, DEC* retFrac);

// LIBAPI int get_dec_frac_zero_count(DecFrac* tSelf, char* retZeroCount);

// LIBAPI int assign_dec_frac(DecFrac* tSelf, const DecFrac* tOther);
// LIBAPI int add_dec_frac(DecFrac* tSelf, const DecFrac* tOther);
// LIBAPI int sub_dec_frac(DecFrac* tSelf, const DecFrac* tOther);
// LIBAPI int mul_dec_frac(DecFrac* tSelf, const DecFrac* tOther);
// LIBAPI int div_dec_to(DecFrac* tSelf, const DecFrac* tOther);
// LIBAPI int pow_dec_frac(DecFrac* tSelf, const DecFrac* tOther);

typedef struct s_dec_real{
    DEC mIntPart;
    DEC mFracPart;
    char mExp;
    RealTruncPolicy mPolicy;
}dec_real_t;
typedef dec_real_t DecReal;
typedef DecReal* DecRealPtr;

LIBAPI int init_dec_real(DecReal* tSelf);
LIBAPI int release_dec_real(DecReal* tSelf);

LIBAPI int set_dec_real(DecReal* tSelf, DEC tIntPart, DEC tFracPart, 
    char tExp, RealTruncPolicy tPolicy);
LIBAPI int set_dec_real_int_part(DecReal* tSelf, DEC tDec);
LIBAPI int set_dec_real_frac_part(DecReal* tSelf, DEC tDec);
LIBAPI int set_dec_real_trunc_policy(DecReal* tSelf, 
    RealTruncPolicy tPolicy);

// dec_real_exp 범위는 REAL_EXPONENT_DEC_MIN ~ REAL_EXPONENT_DEC_MAX
LIBAPI int set_dec_real_exp(DecReal* tSelf, char tChar);

LIBAPI int get_dec_real(const DecReal* tSelf, 
    DEC* retIntPart, DEC* retFracPart, char* retExp, 
    RealTruncPolicy* retPolicy);

LIBAPI int get_dec_real_int_part(const DecReal* tSelf, DEC* retDec);
LIBAPI int get_dec_real_frac_part(const DecReal* tSelf, DEC* retDec);
LIBAPI int get_dec_real_exp(const DecReal* tSelf, char* retChar);
LIBAPI int get_dec_real_trunc_policy(DecReal* tSelf, 
    RealTruncPolicy* retPolicy);

LIBAPI const DEC dec_real_int_part(const DecReal* tSelf);
LIBAPI const DEC dec_real_frac_part(const DecReal* tSelf);
const char dec_real_exp(const DecReal* tSelf);
const RealTruncPolicy LIBAPI dec_real_trunc_policy(DecReal* tSelf);

LIBAPI const bool dec_real_is_zero(const DecReal* tSelf);

LIBAPI int assign_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int add_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int sub_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int mul_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int div_dec_real(DecReal* tSelf, const DecReal* tOther);

// 고정된 지수부만큼 실수부의 값을 얻어낸다.
// 현재 지수부의 범위와 새로 고정 시킬 지수부의 범위를 확인 후에,
// 연산이 불가능하면, 실패를 반환한다.
// tDecExpFix의 범위는 
// REAL_EXPONENT_DEC_MIN ~ REAL_EXPONENT_DEC_MAX
LIBAPI int fix_dec_real_exp(const DecReal* tSelf, DEC tDecExpFix, 
    DEC* retIntPart, DEC* retFracPart);

LIBAPI int convert_dec_real_to(const DecReal* tSelf, REAL* retReal);
LIBAPI int convert_dec_real_from(DecReal* tSelf, const REAL tReal);

LIBAPI const REAL dec_real_to(const DecReal* tSelf);
LIBAPI const DecReal dec_real_from(const REAL tReal);

LIBAPI int div_dec_to(DEC tSelf, const DEC tOther, 
    DEC* retIntPart, DEC* retFracPart);

DEC LIBAPI dec_real_dec_part(const DecReal* tSelf);

#ifdef __cplusplus
}
#endif

#endif // DEC_REAL_H
