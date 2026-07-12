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

#include "dec_real/dec_real_config.h"
#include "real.h"

#include "ustring.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEC_REAL_DIGITS              9  
#define DEC_REAL_MAX                999999999
#define DEC_REAL_MIN                -999999999
#define DEC_REAL_EXP_MAX                 99
#define DEC_REAL_EXP_MIN                 -99

#define DEC_REAL_UNDERFLOW      110
#define DEC_REAL_OVERFLOW       111
#define DEC_REAL_DEC_ZERO       112

#define DEC_REAL_EXP_ZERO       1110
#define DEC_REAL_EXP_OVER       1111
#define DEC_REAL_EXP_UNDER      1112

#define DEC_REAL_EXP_OVER_RANGE       11111
#define DEC_REAL_EXP_UNDER_RANGE      11112

#define DEC_REAL_FIX_EXP_OVER_RANGE       111111
#define DEC_REAL_FIX_EXP_UNDER_RANGE      111112


LIBAPI void dec_real_print_version();
LIBAPI const char* dec_real_version(char* buf);

typedef struct s_dec_real{
    bool mSign;
    UDEC mFracPart;
    UDEC mIntPart;
    DEC mExp;
    DecMinTruncPolicy mPolicy;
}dec_real_t;
typedef dec_real_t DecReal;
typedef DecReal* DecRealPtr;

extern const DecReal DecRealInf;

LIBAPI DecReal* create_dec_real_default();

LIBAPI DecReal* create_dec_real(const bool tSign, 
    const UDEC tFracPart, const UDEC tIntPart, const DEC tExpPart, 
    const DecMinTruncPolicy tPolicy);

LIBAPI DecReal* create_dec_real_from(REAL tValue);

LIBAPI int delete_dec_real(DecReal* tSelf);

LIBAPI int init_dec_real(DecReal* tSelf);
LIBAPI int release_dec_real(DecReal* tSelf);

LIBAPI int set_dec_real(DecReal* tSelf, bool tSign, 
    UDEC tFracPart, UDEC tIntPart, DEC tExp,
    const DecMinTruncPolicy tPolicy);

LIBAPI int set_dec_real_sign(DecReal* tSelf, bool tSign);
LIBAPI int set_dec_real_frac(DecReal* tSelf, UDEC tDec);
LIBAPI int set_dec_real_int(DecReal* tSelf, UDEC tDec);

// dec_real_exp 범위는 REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
LIBAPI int set_dec_real_exp(DecReal* tSelf, DEC tDec);

LIBAPI int set_dec_real_policy(DecReal* tSelf, DecMinTruncPolicy tPolicy);

LIBAPI int get_dec_real(const DecReal* tSelf, bool* retSign,
    UDEC* retFracPart, UDEC* retIntPart, DEC* retExp,
    DecMinTruncPolicy* retPolicy);

LIBAPI int get_dec_real_sign(const DecReal* tSelf, bool* retSign);
LIBAPI int get_dec_real_frac(const DecReal* tSelf, UDEC* retDec);
LIBAPI int get_dec_real_int(const DecReal* tSelf, UDEC* retDec);
LIBAPI int get_dec_real_exp(const DecReal* tSelf, DEC* retDec);
LIBAPI int get_dec_real_policy(const DecReal* tSelf, 
    DecMinTruncPolicy* retPolicy);

LIBAPI const bool dec_real_sign(const DecReal* tSelf);
LIBAPI const UDEC dec_real_frac(const DecReal* tSelf);
LIBAPI const UDEC dec_real_int(const DecReal* tSelf);
const DEC dec_real_exp(const DecReal* tSelf);
const DecMinTruncPolicy dec_real_policy(const DecReal* tSelf);

LIBAPI const bool dec_real_is_zero(const DecReal* tSelf);

LIBAPI int assign_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int add_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int sub_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int mul_dec_real(DecReal* tSelf, const DecReal* tOther);
LIBAPI int div_dec_real(DecReal* tSelf, const DecReal* tOther);

LIBAPI int pow_dec_real(DecReal* tSelf, const DecReal* tOther);

// 고정된 지수부만큼 정수부와 소수부의 값을 얻어낸다.
// 원래의 값과 얻어진 값이 지수만 다르고 같은 값이다.
// 실제적으로 적용 될 범위는
// 정수부의 남은 자리수 ~ 소수부의 남은 자리수
// 만약, 정수부와 소수부 모두 0이면
// return DEC_REAL_DEC_ZERO
// 만약, 실제 적용 범위를 tDecExpFix가 넘어가면
// return DEC_REAL_EXP_OVER_RANGE
// 만약, 실제 적용 범위보다 tDecExpFix가 작으면
// return DEC_REAL_EXP_UNDER_RANGE
// tDecExpFix의 범위는 
// REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
// 만약 tDecExpFix이 실제 범위 밖에 있으면
// tDecExpFix에 범위 내의 값을 반환한다.
// tDecExpFix이 OVER 이면 실제 범위의 최대값
// tDecExpFix이 UNDER 이면 실제 범위의 최소값
LIBAPI int fix_dec_real_exp(DecReal* tSelf, const DEC tDecExpFix);

// 고정된 지수부만큼 정수부와 소수부의 값을 얻어낸다.
// dec_real_fix_exp_ret와 같다.
// 하지만 정책에 따라 최소값을 자르기한다.
// 왼쪽으로 이동할 때는 dec_real_fix_exp_ret와 같다.
// 오른쪽으로 이동할 때는 최소값을 자른다.
// 자르다 보면 결국에는 0이 된다. 
// 오른쪽으로 이동하면 지수는 올라가는데
// 유효숫자가 0이 되면 지수가 아무리 높아도 0이다.
LIBAPI int fix_dec_real_exp_trunc(DecReal* tSelf, const DEC tDecExpFix);

// 고정된 지수부만큼 정수부와 소수부의 값을 얻어낸다.
// 원래의 값과 얻어진 값이 지수만 다르고 같은 값이다.
// 실제적으로 적용 될 범위는
// 정수부의 남은 자리수 ~ 소수부의 남은 자리수
// 만약, 정수부와 소수부 모두 0이면
// return DEC_REAL_DEC_ZERO
// 만약, 실제 적용 범위를 tDecExpFix가 넘어가면
// return DEC_REAL_EXP_OVER_RANGE
// 만약, 실제 적용 범위보다 tDecExpFix가 작으면
// return DEC_REAL_EXP_UNDER_RANGE
// tDecExpFix의 범위는 
// REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
// 만약 tDecExpFix이 실제 범위 밖에 있으면
// tDecExpFix에 범위 내의 값을 반환한다.
// tDecExpFix이 OVER 이면 실제 범위의 최대값
// tDecExpFix이 UNDER 이면 실제 범위의 최소값
LIBAPI int dec_real_fix_exp_ret(const DecReal* tSelf, DEC *tDecExpFix, 
    bool* retSign, UDEC* retFracPart, UDEC* retIntPart);

// 고정된 지수부만큼 정수부와 소수부의 값을 얻어낸다.
// dec_real_fix_exp_ret와 같다.
// 하지만 정책에 따라 최소값을 자르기한다.
// 왼쪽으로 이동할 때는 dec_real_fix_exp_ret와 같다.
// 오른쪽으로 이동할 때는 최소값을 자른다.
// 자르다 보면 결국에는 0이 된다. 
// 오른쪽으로 이동하면 지수는 올라가는데
// 유효숫자가 0이 되면 지수가 아무리 높아도 0이다.
LIBAPI int dec_real_fix_exp_trunc_ret(const DecReal* tSelf, DEC *tDecExpFix, 
    bool* retSign, UDEC* retFracPart, UDEC* retIntPart);

LIBAPI int convert_dec_real_to(const DecReal* tSelf, REAL* retReal);

LIBAPI const REAL dec_real_to(const DecReal* tSelf);

// DecReal의 소수부만 0.###########의 형태로 부동소수점으로 얻어낸다.
LIBAPI int convert_dec_real_frac_to(const DecReal* tSelf, REAL* retReal);

// DecReal의 소수부만 0.###########의 형태로 부동소수점으로 반환한다.
LIBAPI const REAL dec_real_frac_to(const DecReal* tSelf);

// shift를 하게 되면 유효 숫자들이 이동할 수 있는 자리수를 알야야 할 때가 있다.
// 시프트를 한다는 것은 지수가 이동한다는 의미이다.
// 지수의 숫자가 변하지 않고 정수부와 소수부의 유효 숫자만 이동하는 범위이다.
// 시프트를 할 때에 범위 안의 숫자를 입력하면 지수는 변하지 않는다.
// 단지, 정수부와 소수부만 자리수가 이동한다.
// 정수부의 남은 자리수(Min) ~ 소수부의 남은 자리수(Max)
LIBAPI int dec_real_shift_range_ret(const DecReal* tSelf, 
    DEC* retMin, DEC* retMax);

// 유효 숫자들이 소수점을 기준으로 왼쪽이나 오른쪽으로 지수가 이동한다.
// 유효 숫자들이 남은 자리수가 있으면 이동한다.
// 남은 자리수가 없으면 지수가 내려가거나 올라간다.
// 최종 지수 = 원래 지수 + 이동할 값 - 유효숫자가 이동한 거리
// 왼쪽으로 시프트하면 지수가 점점 작아지면서 가장 작아지면 0에 가까워진다.
// return DEC_REAL_EXP_UNDER;
// 정수부와 소수부의 유효숫자는 유지된다.
// 오른쪽으로 이동시에는  지수가 점점 커진다. 가장 커지면 맥스가 된다.
// return DEC_REAL_EXP_OVER;
// 숫자가 양수이면 오른쪽으로 이동한다.
// 숫자가 음수이면 왼쪽으로 이동한다.
// 범위는 DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
LIBAPI int shift_dec_real(DecReal* tSelf, const DEC tDec);

// shift_dec_real 와 같다.
// 하지만 오른쪽으로 이동할 때에 정책에 따라 자른다.
// 유효 숫자가 자르다 보면 결국에는 0이 된다.
// 유효 숫자가 0이 되면 지수가 높아도 0이다.
// return DEC_REAL_DEC_ZERO;
LIBAPI int shift_dec_real_trunc(DecReal* tSelf, const DEC tDec);

// 절대 공차 fabs(a-b) <= epsilon
// 공차를 0.000001이라고 입력해야 하는데
// 1도 가능하고 10, 100, 1000도 가능하네
LIBAPI bool dec_real_equal(const DecReal* tSelf, const DecReal* tOther,
    const REAL tTol);

// 상대 공차 fabs(a-b)/길이(a,b) <= epsilon
// 상대 공차 fabs(a-b) <= epsilon*max(a,b)
// 상대 공차는 비율의 차이가 공차보다 작으면 같다는 것이다.
// 비율의 차이로 인식하므로 
// 공차 비율이 1/100이면 1mm 와 1.01mm 는 같다.
// 공차 비율 1/100이면 1m와 1.01미터는 같다.
// 최대 공차 비율은 1.0이다.
// 공차 비율 1.0의 의미는 self와 other중에 큰값이 기준이 되서,
// epsilon이 큰값이라는 의미이다.
// -큰값 <= (a-b)/길이(a,b) <= 큰값 이라는 의미이다.
LIBAPI bool dec_real_equal_rel(const DecReal* tSelf, const DecReal* tOther, 
    const REAL tTol);

// self가 other보다 크냐?
LIBAPI bool dec_real_gt(const DecReal* tSelf, const DecReal* tOther);

// self가 other보다 크거나 같냐?
LIBAPI bool dec_real_ge(const DecReal* tSelf, const DecReal* tOther);

// self가 other보다 작냐?
LIBAPI bool dec_real_lt(const DecReal* tSelf, const DecReal* tOther);

// self가 other보다 작거나 같냐?
LIBAPI bool dec_real_le(const DecReal* tSelf, const DecReal* tOther);

LIBAPI int set_dec_real_from(DecReal* tSelf, const REAL tReal);
LIBAPI int set_dec_real_from_real(DecReal* tSelf, const Real* tReal);

LIBAPI const DecReal dec_real_add(const DecReal* tSelf, const DecReal* tOther);
LIBAPI const DecReal dec_real_sub(const DecReal* tSelf, const DecReal* tOther);
LIBAPI const DecReal dec_real_mul(const DecReal* tSelf, const DecReal* tOther);
LIBAPI const DecReal dec_real_div(const DecReal* tSelf, const DecReal* tOther);
LIBAPI const DecReal dec_real_pow(const DecReal* tSelf, const DecReal* tOther);

LIBAPI const DecReal dec_real_shift(const DecReal* tSelf, const DEC tDec);
LIBAPI const DecReal dec_real_shift_trunc(const DecReal* tSelf, const DEC tDec);

LIBAPI const DecReal dec_real_abs(const DecReal* tSelf);
LIBAPI int abs_dec_real(DecReal* tSelf);

LIBAPI const DecReal dec_real_max(const DecReal* tSelf, const DecReal* tOther);
LIBAPI int max_dec_real(DecReal* tSelf, const DecReal* tOther);

LIBAPI int convert_dec_real_to_real(const DecReal* tSelf, Real* retReal);
LIBAPI const REAL dec_real_to_real(const DecReal* tSelf);

// 유효 숫자들이 소수점을 기준으로 오른쪽으로 지수가 이동한다.
// 유효 숫자들이 남은 자리수가 있으면 이동한다.
// 남은 자리수가 없으면 지수가 올라간다.
// 최종 지수 = 원래 지수 + 이동할 값 - 유효숫자가 이동한 거리
// 정수부와 소수부의 유효숫자는 유지된다.
// 오른쪽으로 이동시에는  지수가 점점 커진다. 가장 커지면 맥스가 된다.
// return DEC_REAL_EXP_MAX;
// 범위는 0 ~ DEC_REAL_EXP_MAX
LIBAPI int shift_dec_real_to_right(DecReal* tSelf, const UDEC tDec);

// shift_dec_real_to_right와 같다.
// 하지만 최소값을 정책에 따라 자른다.
// 자르다 보면 결국에는 0이 된다.
// return DEC_REAL_DEC_ZERO;
// 유효 숫자가 0이 된면 지수가 높아도 0이다.
LIBAPI int shift_dec_real_to_right_trunc(DecReal* tSelf, const UDEC tDec);

// 유효 숫자들이 소수점을 기준으로 왼쪽으로 이동한다.
// 유효 숫자들이 남은 자리수가 있으면 이동한다.
// 남은 자리수가 없으면 지수가 내려간다.
// 최종 지수 = 원래 지수 + 이동할 값 - 유효숫자가 이동한 거리
// 왼쪽으로 시프트하면 지수가 점점 작아지면서 가장 작아지면 0에 가까워진다.
// return DEC_REAL_EXP_ZERO;
// 정수부와 소수부의 유효숫자는 유지된다.
// 범위는 0 ~ DEC_REAL_EXP_MAX
LIBAPI int shift_dec_real_to_left(DecReal* tSelf, const UDEC tDec);

// 자리수를 확인한다.
// 정수부의 자리수와 소수부의 자리수가 합쳐진 값이다.
LIBAPI const UDEC dec_real_digits(const DecReal* tSelf);

// 유효 숫자를 확인한다.
// 1230.0456의 dec_real 유효 숫자는
// 12300456 이다.
// 유효 숫자는 정수부의 유효 숫자와 소수부의 유효숫자가 합쳐진다.
// 유효 자리수가 DEC의 범위를 넘어선다.
// 그래서 DBL_DEC을 반환한다.
LIBAPI const DBL_DEC dec_real_sig(const DecReal* tSelf);

// 정수부의 자리수를 확인한다.
LIBAPI const UDEC dec_real_int_digits(const DecReal* tSelf);

// 소수부의 자리수를 확인한다.
// 소수부의 자리수는 정수부의 자리수와 다르다.
// 001234000의 예로 보면,
// 정수부의 자리수는 1234000 이다. 총 7자리
// 소수부의 자리수는 001234 이다. 총 6자리
// 범위는 0 ~ REAL_SIG_DIGITS
LIBAPI const UDEC dec_real_frac_digits(const DecReal* tSelf);

// 정수부의 유효 숫자를 확인한다.
// 정수부 1230의 유효 숫자는 123이다.
// 정수부의 유효숫자는 소수부와 합쳐지면
// 1230의 유효 숫자는 1230이다.
// 하지만, 아래 함수는 정수부 단독으로 유효숫자를 찾는거라서,
// 123이 결과이다.
LIBAPI const UDEC dec_real_int_sig(const DecReal* tSelf);

// 소수부의 유효 숫자를 확인한다.
// 소수부 1230의 유효 숫자는 123이다.
// 소수부 0123의 유효 숫자는 123이다.
// 소수부의 유효 숫자는 정수부와 합쳐지면
// 0123의 유효 숫자는 0123이다.
// 하지만, 아래 함수는 소수부 단독으로 유효숫자를 찾는거라서,
// 123이 결과이다.
LIBAPI const UDEC dec_real_frac_sig(const DecReal* tSelf);

LIBAPI const DecReal dec_real_from_real(const Real* tReal);
LIBAPI const DecReal dec_real_from(const REAL tReal);

LIBAPI int get_dec_real_str(const DecReal* tSelf, Ustring* retStr);
LIBAPI const char* dec_real_str(const DecReal* tSelf, char* buf);

LIBAPI int get_dec_real_frac_str(const DecReal* tSelf, Ustring* retStr);
LIBAPI const char* dec_real_frac_str(const DecReal* tSelf, char* buf);

#ifdef __cplusplus
}
#endif

#endif // DEC_REAL_H
