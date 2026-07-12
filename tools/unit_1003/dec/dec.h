/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec 
*
***************************************************************************/

#ifndef DEC_H
#define DEC_H

#include "dec/dec_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

LIBAPI void print_dec_version();
LIBAPI const char* dec_version(char* buf);

#define DEC         long
#define UDEC        unsigned long

#define DBL_DEC         long long
#define DBL_UDEC        unsigned long long

#define DEC_MAX         999999999
#define DEC_MIN         -999999999
#define DEC_DIGITS      9

#define DEC_UNDERFLOW       110
#define DEC_OVERFLOW        111
#define DEC_DIV_ZERO        112

#define DBL_DEC_LOW(tDblDec) (tDblDec & 0x00000000ffffffff)
#define DBL_DEC_HIGH(tDblDec) ( (tDblDec>>8) & 0x00000000ffffffff )
#define DBL_DEC_FROM(tLowDec, tHighDec) \
    ( ( (tHighDec<<8)&0xffffffff00000000 | tLowDec&0x00000000ffffffff ) )

typedef enum e_dec_min_trunc_policy{
DEC_MIN_TRUNC_POLICY_FLOOR,
DEC_MIN_TRUNC_POLICY_CEILING,
DEC_MIN_TRUNC_POLICY_ROUND
}dec_min_trunc_policy_m;
typedef dec_min_trunc_policy_m DecMinTruncPolicy;

// 정수의 자리수를 확인한다.
// DBL_DEC을 매개 변수로 받는다
// DBL_DEC이 곱셈이나 기타 등등에서 사용 된다.
// 범위는 0 ~ REAL_SIG_DIGITS
LIBAPI const DEC dec_digits(const DBL_DEC tDec);

// DEC의 유효 자리수를 확인한다.
// DBL_DEC을 매개 변수로 받는다
// DBL_DEC이 곱셈이나 기타 등등에서 사용 된다.
// 001234000의 유효자리수는 1234의 숫자를 세면 4이다.
// 범위는 0 ~ REAL_SIG_DIGITS
LIBAPI const DEC dec_sig_digits(const DBL_DEC tDec);

// DEC의 유효숫자를 확인한다.
// DBL_DEC을 매개 변수로 받는다
// DBL_DEC이 곱셈이나 기타 등등에서 사용 된다.
// 001234000의 유효숫자는 1234이다.
LIBAPI const DEC dec_sig(const DBL_DEC tDec);

// self와 other의 나눗셈을 한다.
LIBAPI int dec_div_to(const DEC tSelf, const DEC tOther, 
    const DecMinTruncPolicy tPolicy,
    DEC* retIntPart, DEC* retFracPart);

// shift 값이 양수이면 오른쪽으로 이동, 음수이면, 왼쪽으로 이동
LIBAPI const DEC dec_shift(const DEC tDec, const DEC tShift);

typedef struct s_dbl_dec{
    DEC mLow;
    DEC mHigh;
}dbl_dec_t;
typedef dbl_dec_t DblDec;
typedef DblDec* DblDecPtr;

LIBAPI int init_dbl_dec(DblDecPtr tSelf);
LIBAPI int release_dbl_dec(DblDecPtr tSelf);

LIBAPI int set_dbl_dec(DblDecPtr tSelf, const DEC tLow, const DEC tHigh);
LIBAPI int set_dbl_dec_low(DblDecPtr tSelf, const DEC tLow);
LIBAPI int set_dbl_dec_high(DblDecPtr tSelf, const DEC tHigh);

LIBAPI int get_dbl_dec(const DblDecPtr tSelf, DEC* retLow, DEC* retHigh);
LIBAPI int get_dbl_dec_low(const DblDecPtr tSelf, DEC* retLow);
LIBAPI int get_dbl_dec_high(const DblDecPtr tSelf, DEC* retHigh);

LIBAPI const DEC dbl_dec_low(const DblDecPtr tSelf);
LIBAPI const DEC dbl_dec_high(const DblDecPtr tSelf);

LIBAPI const DblDecPtr create_dbl_dec(const DEC tLow, const DEC tHigh);
LIBAPI const DblDecPtr create_dbl_dec_default(const DEC tLow, const DEC tHigh);
LIBAPI int delete_dbl_dec(DblDecPtr tSelf);

LIBAPI int assign_dbl_dec(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI int add_dbl_dec(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI int sub_dbl_dec(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI int mul_dbl_dec(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI int div_dbl_dec(DblDecPtr tSelf, const DblDecPtr tOther, 
    const DecMinTruncPolicy tPolicy);

LIBAPI int pow_dbl_dec(DblDecPtr tSelf, const DEC tExp);

// shift 값이 양수이면 오른쪽으로 이동, 음수이면, 왼쪽으로 이동
LIBAPI int shift_dbl_dec(DblDecPtr tSelf, const DEC tShift,
    const DecMinTruncPolicy tPolicy);

LIBAPI const DblDec dbl_dec_add(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const DblDec dbl_dec_sub(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const DblDec dbl_dec_mul(DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const DblDec dbl_dec_div(DblDecPtr tSelf, const DblDecPtr tOther,
    const DecMinTruncPolicy tPolicy);

LIBAPI const DblDec dbl_dec_pow(DblDecPtr tSelf, const DEC tExp, 
    const DecMinTruncPolicy tPolicy);

// shift 값이 양수이면 오른쪽으로 이동, 음수이면, 왼쪽으로 이동
LIBAPI const DblDec dbl_dec_shift(DblDecPtr tSelf, const DEC tShift, 
    const DecMinTruncPolicy tPolicy);

LIBAPI const bool dbl_dec_equal(const DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const bool dbl_dec_gt(const DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const bool dbl_dec_ge(const DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const bool dbl_dec_lt(const DblDecPtr tSelf, const DblDecPtr tOther);
LIBAPI const bool dbl_dec_le(const DblDecPtr tSelf, const DblDecPtr tOther);

#ifdef __cplusplus
}
#endif

#endif // DEC_H
