/***************************************************************************8901
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* real
*
* real은 두가지 종류가 있다.
* float real    : 정수 7    자리, 지수 +-38
*   - 1.2e38 ~ 3.4e38 : 약 1.17549435E-38 ~ 3.40277175E38
*   - 가수(mantissa) 23비트, 지수(exponent) 8비트
*   - 지수 : 2^8 비트
*   - 가수 : 01000001110100000000000000000000
*       + 1 + 010000011101 = 1010000011101
*       + 1*1/2 + 0*1/4 + 1*1/8 + 0*1/16 + 0*1/32 ...
*   - 지수 : 2의 거듭제곱
*   - 절대값 118.625 를 이진법으로 변환해준다. -> 1110110.101(2)
*   - 소수점을 이동시켜 정수부가 한자리가 되도록 변환해준다. 
        (그러면 지수 6이 만들어진다) ->
*       + 1110110.101 → 1.110110101 x 2^6
*        - 가수부에 입력한다.
*   - 지수에 바이어스 127을 더하고, 지수부 비트에 넣는다.
*       + 32bit IEEE 754 형식에는 bias라는 고정값(127)이 존재한다.
*       + 이 bias라는 값을 왜 쓰냐면, 
*       + 지수가 음수가 될 수도 있는 케이스가 있기 때문이다. 
*       + (2^10 or 2^-10)
*       + 예를 들면 0.000101(2)이라는 이진수가 있다고 가정하자
*       + 이를 1.xxxx... * 2^n 형식으로 표현하기 위해선, 
*       + 오른쪽으로 소수점을 밀면 1.01 * 2^-4 가 된다. 
*       + 이 -4 음수 지수를 8자리 비트로 표현하기 위해, 
*       + (10진수 기준으로) 0~127 구간은 음수, 
*       + 128~255 구간은 양수를 표현하도록 만든 것이다.
*       + 그래서 계산된 지수에 127 bias 값을 더하여, 
*       + 127보다 작으면 음수, 
*       + 127보다 크면 양수로 구분할 수 있는 것이다.
*       + 6 + 127 
*                   = 133
*                   = 100000101(2)
*   - 소수점을 정확히 계산하는 방법
*       + 25.35
*           - 100을 곱해서 2535로 만든뒤 계산하고
*           - 다시 100으롤 나눠서 결과를 얻어낸다.
* DBL_REAL real   : 정수 15   자리, 지수
*   - 1.7 × 10^-308에서 1.7 × 10^308
*   - 가수 52 비트, 지수 11비트
* 
* (-1)^s * (1.m) * 2^(e-127)
* s : sign
* m : mantissa
* e : exponent
* 
* s 1.. .exp . 2.. .... .... mantissa. ...0
* 0 000 0000 0 000 0000 0000 0000 0000 0000        0.0   모두 0000 0000    0
* 0 011 1111 0 100 0000 0000 0000 0000 0000        0.9   지수 0111 1110  126 가수 0.9해당되는 이진수르 연산해야 한다.
* 0 011 1111 1 100 0000 0000 0000 0000 0000        1.0   지수 0111 1111  127
* 0 100 0000 0 100 0000 0000 0000 0000 0000        2.0   지수 1000 0000  128
*
// 십진번의 지수값을 입력하면 내부적으로 필요한 지수값으로 변환된다.
// 범위는 2^-REAL_EXP_BIAS ~ 2^-REAL_EXP_BIAS + REAL_EXP_BIAS+1
// 위의 2진수 범위를 십진수 범위로 바꿔야 한다. 그러면,
//              float에서 2^-127 ~ ( 2^-127 + 255 ) 
// 5.8774717541114375398436826861112e-39 ~ 
//      3.4028236692093846346337460743177e+38
//
// 또는 
//              DBL_REAL형에서 2^-1023 ~ ( 2^-1023 + 2048 )
// 1.1125369292536006915451163586662e-308 ~
//      1.797693134862315907729305190789e+308
// 이다. 범위 안으로 지수값을 입력해야한다.
// 정리하면, float에서 -39 ~ 38
//          DBL_REAL에서 -308 ~ 308
// 가수부의 내용도 변해야 할 것인데, 그건 아직 모르겠다. 2024-06-03 12:01:47
// 추가적으로, 가수부가 소수점으로 인식해서, 그걸 정수형으로 바꾸러면,
// 2^REAL_EXP_OFFSET을 추가해야 가수부 소수점이 아니라, 정수형으로 바뀐다.
// float에서 EXPONENT_OFFSET  -> 23
// DBL_REAL에서 EXPONENT_OFFSET -> 52
// float에서    2^-127  + 23 ~ ( 2^-127 + 255 ) + 23 
//      2^-104 ~ 2^151 : 
// DBL_REAL형에서 2^-1023 + 52 ~ ( 2^-1023 + 2048 ) + 52 
*
***************************************************************************/

#ifndef REAL_H
#define REAL_H

#include "real/real_config.h"
#include <float.h>
#include <stdbool.h>

#include "dec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REAL_OVERFLOW           -11
#define REAL_UNDERFLOW          -12

#ifndef USE_DBL_PREC

// float real
#define REAL_SIGN_BITS      1
#define REAL_EXP_BITS  8
#define REAL_MANTISSA_BITS  FLT_MANT_DIG-1
#define REAL_BITS           32

#define REAL_EXP_BIAS  127

#define REAL_MANTISSA_DEC_DIGITS 7
#define REAL_MANTISSA_DEC_MAX   8388607
#define REAL_MANTISSA_DEC_MIN   0000000

// 반올림을 기준으로 한다. 최소자리수가 5와 같거나 크면 올림한다.
#define REAL_MANTISSA_TRUNC_ROUND_FACTOR    0x00000008

#define REAL float
#define DBL_REAL double
#define MANTISSA   unsigned long // 32비트

#define REAL_EXP_MAX       FLT_MAX_EXP
#define REAL_EXP_MIN       FLT_MIN_EXP

#define REAL_EXP_DEC_MAX   FLT_MAX_10_EXP
#define REAL_EXP_DEC_MIN   FLT_MIN_10_EXP
#define REAL_SIG_DIGITS         FLT_DIG
#define REAL_DIGITS             FLT_DECIMAL_DIG
#define REAL_EXP_OFFSET_FOR_REAL_DIGITS    29

#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN

#else
// DBL_REAL real
#define REAL_SIGN_BITS      1
#define REAL_EXP_BITS  11
#define REAL_MANTISSA_BITS  DBL_MANT_DIG-1
#define REAL_BITS           64

#define REAL_EXP_BIAS  1023

#define REAL_MANTISSA_DEC_DIGITS 16
#define REAL_MANTISSA_DEC_MAX   4503599627370495
#define REAL_MANTISSA_DEC_MIN   0000000000000000
#define REAL_MANTISSA_TRUNC_ROUND_FACTOR    0x0000000000000008

#define REAL DBL_REAL
#define DBL_REAL long double
#define MANTISSA   unsigned long long // 64비트 

#define REAL_EXP_MAX       DBL_MAX_EXP
#define REAL_EXP_MIN       DBL_MIN_EXP

#define REAL_EXP_DEC_MAX   DBL_MAX_10_EXP
#define REAL_EXP_DEC_MIN   DBL_MIN_10_EXP
#define REAL_SIG_DIGITS         DBL_DIG
#define REAL_DIGITS             DBL_DECIMAL_DIG
#define REAL_EXP_OFFSET_FOR_REAL_DIGITS    56

#define REAL_MAX DBL_MAX
#define REAL_MIN DLB_MIN

#endif  // USE_DBL_PREC

LIBAPI void real_print_version();
LIBAPI const char* real_version();

typedef struct s_real{
    unsigned mMantissa:REAL_MANTISSA_BITS;
    unsigned mExponent:REAL_EXP_BITS;
    unsigned mSign:REAL_SIGN_BITS;
}real_t;

typedef union u_real{
    REAL mReal;
    real_t mReal_t;
}real_n;

typedef real_n Real;

// 무한대를 상수로 ...
extern const Real Inf;

LIBAPI Real* create_real(const REAL tReal);
LIBAPI Real* create_real_default();
LIBAPI int delete_real(Real* tReal);

LIBAPI int init_real(Real* tReal);
LIBAPI int release_real(Real* tReal);

LIBAPI int set_real(Real* tReal, REAL tValue);

// set_real_sign범위는 0 ~ 1
LIBAPI int set_real_sign(Real* tReal, bool tSign);

// real_exponent 범위는 -REAL_EXP_BIAS ~ REAL_EXP_BIAS + 1
LIBAPI int set_real_exponent(Real* tReal, DEC tExp);

// real_mantissa 범위는 0 ~ REAL_MANTISSA_DEC_MAX
LIBAPI int set_real_mantissa(Real* tReal, MANTISSA tMant);

LIBAPI bool real_is_zero(const Real* tReal);

LIBAPI REAL real(const Real* tReal);
LIBAPI bool real_sign(const Real* tReal);
DEC real_exponent(const Real* tReal);
// MANTISSA real_mantissa(const Real* tReal);
inline MANTISSA real_mantissa(const Real* tReal){
    return tReal->mReal_t.mMantissa;
}

LIBAPI int assign_real(Real* tReal, const Real* tOther);
LIBAPI int add_real(Real* tReal, const Real* tOther);
LIBAPI int sub_real(Real* tReal, const Real* tOther);
LIBAPI int mul_real(Real* tReal, const Real* tOther);
LIBAPI int div_real(Real* tReal, const Real* tOther);
LIBAPI int pow_real(Real* tReal, const Real* tOther);

LIBAPI Real real_add(const Real* tReal, const Real* tOther);
LIBAPI Real real_sub(const Real* tReal, const Real* tOther);
LIBAPI Real real_mul(const Real* tReal, const Real* tOther);
LIBAPI Real real_div(const Real* tReal, const Real* tOther);
LIBAPI Real real_pow(const Real* tReal, const Real* tOther);

// 실수를 정수 형태의 소수부, 정수부, 지수부로 분리한다.
// 소수부와 정수부의 자리수는 실수의 자리수와 같다.
// REAL_DIGITS_MAX
// 지수부의 범위는 REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
LIBAPI int split_real(const Real* tReal, bool* retSign,
    DEC* retFracPart, DEC* retIntPart, DEC* retExpPart);

// 정수 형태의 소수부, 정수부, 지수부를 합쳐서 실수를 만든다.
// 정수 형태의 소수부, 정수부, 지수부는 split_real을 통해 구할 수 있다.
// 소수부의 형태는 120.034 의 경우 034각 소수부가 아니라, 20034가 소수부가 된다.
// 1은 정수부이다.
// 소수부는 0이 아닌 숫자로 시작한다.
// 소수부와 정수부의 자리수는 실수의 자리수와 같다.
// REAL_DIGITS_MAX
// 지수부의 범위는 REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
LIBAPI int merge_real(Real* tReal, const bool tSign,
    const DEC tFracPart, const DEC tIntPart, const DEC tExpPart);

// 고정된 지수부만큼 실수부의 값을 얻어낸다.
// 현재 지수부의 범위와 새로 고정 시킬 지수부의 범위를 확인 후에,
// 연산이 불가능하면, 실패를 반환한다.
// tDecExpFix의 범위는 
// REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
// fix_real_exp , get_fix_real_exp
LIBAPI int fix_real_exp(const Real* tReal, DEC tDecExpFix, 
    REAL* tRetIntPart, REAL* tRetFracPart);

// 실수를 설정한 정수만큼 시프트한다.
// 정수가 양수이면, 오른쪽으로 시프트한다.
// 123.456이 0.123456이 된다는 의미이다.
// 음수이면, 왼쪽으로 시프트한다.
// 123.456이 123456이 된다는 의미이다.
// 지수가 올라가거나, 내려간다.
// 지수는 최소 지수까지 내려가고 더 내려가면 언더플로우를 반환한다.
// 언더 플로우이면 0과 가깝다는 의미이다.
// 최고 지수까지 올라가고 더 올라가면 오버플로우를 반환한다.
// 오버플로우이면, 무한대와 가깝다는 의미이다.
// 2024-06-18 11:00:27
// 유효자리수는 그대로이고 지수만 옮긴다는 건데, 이진수이다.
// 십진수 123.456을 옮기려니까 값이 많이 달라딘다.
// 123.456을 정수로 바꾸고 지수를 옮긴뒤에 다시 이진수로 바꾸면 값이 비슷하려나?
// 내가 원하느 시프트는 10진수의 지수를 이동하느 거지 
// 이진수의 지수를 이동하는게 아니다.
LIBAPI int shift_real(const Real* tReal, DEC tDec, Real* retReal);

// 절대 공차 fabs(a-b) <= epsilon
LIBAPI bool real_equal(const Real* tSelf, const Real* tOther, const REAL tTol);

// 상대 공차 fabs(a-b)/길이(a,b) <= epsilon
// 상대 공차 fabs(a-b) <= epsilon*max(a,b)
// 상대 공차는 비율의 차이가 공차보다 작으면 같다는 것이다.
// 비율의 차이로 인식하므로 
// 공차 비율이 1/100이면 1mm 와 1.01mm 는 같다.
// 공차 비율 1/100이면 1m와 1.01미터는 같다.
LIBAPI bool real_equal_rel(const Real* tSelf, const Real* tOther, 
    const REAL tTol);

// ulp(Unit in the last place)공차 
// float를 int 로 인식시켜서 
// int(a)-int(b) <= 8
// 기준 값은 8이다.
// (일반적으로 상대 공차가 0.000001일 때 8이고, 0 ~ 원하는 거리를 정할 수 있다.)
// ulp를 내가 생각해보니까 만티사의 거리를 비교하는 것같다.
// 지수가 다르면 무조건 다르지만, mant(a)-mant(b) <= 8면 같다고 인식시킨다.
// 상대 공차와 비슷한데 속도가 아주 미미하게 빠르다.
// 약간의 개선이 더 필요하다. 2024-06-17 16:30:37
// 지수 차이가 1이고 만티사의 크기가 많이 다를 때 
// 예를 들어 1과 만티사 맥스의 차이
// 공차 안에 들어 갈수도 있겠다는 생각이 든다.
LIBAPI bool real_equal_ulp(const Real* tSelf, const Real* tOther, 
    const DEC tTol);

// 절대 공차 fabs(a-b) <= epsilon
LIBAPI bool value_equal(const REAL tSelf, const REAL tOther, const REAL tTol);

// 상대 공차 fabs(a-b)/길이(a,b) <= epsilon
// 상대 공차 fabs(a-b) <= epsilon*max(a,b)
// 상대 공차는 비율의 차이가 공차보다 작으면 같다는 것이다.
// 비율의 차이로 인식하므로 
// 공차 비율이 1/100이면 1mm 와 1.01mm 는 같다.
// 공차 비율 1/100이면 1m와 1.01미터는 같다.
LIBAPI const bool value_equal_rel(const REAL tSelf, const REAL tOther, 
    const REAL tTol);

// ulp(Unit in the last place)공차 
// float를 int 로 인식시켜서 
// int(a)-int(b) <= 8
// 기준 값은 8이다.
// (일반적으로 상대 공차가 0.000001일 때 8이고, 0 ~ 원하는 거리를 정할 수 있다.)
// ulp를 내가 생각해보니까 만티사의 거리를 비교하는 것같다.
// 지수가 다르면 무조건 다르지만, mant(a)-mant(b) <= 8면 같다고 인식시킨다.
// 상대 공차와 비슷한데 속도가 아주 미미하게 빠르다.
// 약간의 개선이 더 필요하다. 2024-06-17 16:30:37
// 지수 차이가 1이고 만티사의 크기가 많이 다를 때 
// 예를 들어 1과 만티사 맥스의 차이
// 공차 안에 들어 갈수도 있겠다는 생각이 든다.
LIBAPI bool value_equal_ulp(const REAL tSelf, const REAL tOther, 
    const DEC tTol);

#ifdef __cplusplus
}
#endif

#endif // REAL_H
