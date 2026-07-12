/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec_real 1.0.2.2
*
***************************************************************************/

#include "dec_real.h"

#include <stdio.h>
#include <stdlib.h> // malloc, free
#include <stdint.h>
#include <math.h>
#include <string.h>

LIBAPI void dec_real_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "dec_real", 
        DEC_REAL_VERSION_MAJOR,
        DEC_REAL_VERSION_MINOR,
        DEC_REAL_VERSION_PATCH,
        DEC_REAL_VERSION_TWEAK);
}

LIBAPI const char* dec_real_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        DEC_REAL_VERSION_MAJOR,
        DEC_REAL_VERSION_MINOR,
        DEC_REAL_VERSION_PATCH,
        DEC_REAL_VERSION_TWEAK
        );
    return buf;
}

const DecReal DecRealInf = {false, 0xefffff, 0xefffff, 0xefff,
    DEC_MIN_TRUNC_POLICY_FLOOR};

LIBAPI int init_dec_real(DecReal* tSelf){
    tSelf->mSign = false;
    tSelf->mFracPart = 0;
    tSelf->mIntPart = 0;

    tSelf->mExp = 0;
    tSelf->mPolicy = DEC_MIN_TRUNC_POLICY_FLOOR;
    return 0;
}

LIBAPI int release_dec_real(DecReal* tSelf){
    // 메모리가 할당되었으면, 해제한다.
    return 0;
}

LIBAPI int set_dec_real(DecReal* tSelf, bool tSign,
    UDEC tFracPart, UDEC tIntPart, DEC tExp, const DecMinTruncPolicy tPolicy){

    tSelf->mSign = tSign;
    tSelf->mFracPart = tFracPart;
    tSelf->mIntPart = tIntPart;
    
    tSelf->mExp = tExp;
    tSelf->mPolicy = tPolicy;
    return 0;
}
LIBAPI int set_dec_real_sign(DecReal* tSelf, bool tSign){
    tSelf->mSign = tSign;
    return 0;
}

LIBAPI int set_dec_real_int(DecReal* tSelf, UDEC tDec){
    tSelf->mIntPart = tDec;
    return 0;
}

LIBAPI int set_dec_real_frac(DecReal* tSelf, UDEC tDec){
    tSelf->mFracPart = tDec;
    return 0;
}

// dec_real_exp 범위는 DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
LIBAPI int set_dec_real_exp(DecReal* tSelf, DEC tDec){
    if (tDec > DEC_REAL_EXP_MAX){
        tDec = DEC_REAL_EXP_MAX;
    }
    if (tDec < DEC_REAL_EXP_MIN){
        tDec = DEC_REAL_EXP_MIN;
    }
    tSelf->mExp = tDec;
    return 0;
}

LIBAPI int set_dec_real_policy(DecReal* tSelf, DecMinTruncPolicy tPolicy){
    tSelf->mPolicy = tPolicy;
    return 0;
}

LIBAPI int get_dec_real(const DecReal* tSelf, bool* retSign,
    UDEC* retFracPart, UDEC* retIntPart, DEC* retExp,
    DecMinTruncPolicy* retPolicy){

    *retSign = tSelf->mSign;
    *retFracPart = tSelf->mFracPart;    
    *retIntPart = tSelf->mIntPart;

    *retExp = tSelf->mExp;
    *retPolicy = tSelf->mPolicy;
    return 0;
    }

LIBAPI int get_dec_real_sign(const DecReal* tSelf, bool* retSign){
    
    *retSign = tSelf->mSign;
    return 0;
}

LIBAPI int get_dec_real_int(const DecReal* tSelf, UDEC* retDec){
    
    *retDec = tSelf->mIntPart;
    return 0;
}

LIBAPI int get_dec_real_frac(const DecReal* tSelf, UDEC* retDec){
    
    *retDec = tSelf->mFracPart;
    return 0;
}

LIBAPI int get_dec_real_exp(const DecReal* tSelf, DEC* retDEC){
    *retDEC = tSelf->mExp;
    return 0;    
}

LIBAPI int get_dec_real_policy(const DecReal* tSelf, 
    DecMinTruncPolicy* retPolicy){

        *retPolicy = tSelf->mPolicy;
        return 0;
    }

LIBAPI const bool dec_real_sign(const DecReal* tSelf){
    return tSelf->mSign;
}

LIBAPI const UDEC dec_real_frac(const DecReal* tSelf){
    return tSelf->mFracPart;
}

LIBAPI const UDEC dec_real_int(const DecReal* tSelf){
    return tSelf->mIntPart;
}

const DEC dec_real_exp(const DecReal* tSelf){
    return tSelf->mExp;
}

const DecMinTruncPolicy dec_real_policy(const DecReal* tSelf){
    return tSelf->mPolicy;
}

LIBAPI const bool dec_real_is_zero(const DecReal* tSelf){
    return tSelf->mIntPart==0 && tSelf->mExp==0;
}

LIBAPI int assign_dec_real(DecReal* tSelf, const DecReal* tOther){
    
    tSelf->mSign = tOther->mSign;

    tSelf->mFracPart = tOther->mFracPart;    
    tSelf->mIntPart = tOther->mIntPart;
    
    tSelf->mExp = tOther->mExp;
    tSelf->mPolicy = tOther->mPolicy;
    return 0;
}

LIBAPI int add_dec_real(DecReal* tSelf, const DecReal* tOther){
    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    aSelfValue += aOtherValue;

    set_dec_real_from(tSelf, aSelfValue);

    return SUCCESS;
}

LIBAPI int sub_dec_real(DecReal* tSelf, const DecReal* tOther){
    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    aSelfValue -= aOtherValue;

    set_dec_real_from(tSelf, aSelfValue);

    return SUCCESS;
}
LIBAPI int mul_dec_real(DecReal* tSelf, const DecReal* tOther){
    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    aSelfValue *= aOtherValue;

    set_dec_real_from(tSelf, aSelfValue);

    return SUCCESS;
}

LIBAPI int div_dec_real(DecReal* tSelf, const DecReal* tOther){
    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    if(aOtherValue == 0.0)
    {
        set_dec_real_from_real(tSelf, &Inf);
    }
    else{
        aSelfValue /= aOtherValue;
        set_dec_real_from(tSelf, aSelfValue);
    }
    return SUCCESS;
}

LIBAPI int pow_dec_real(DecReal* tSelf, const DecReal* tOther){
    REAL aSelf;
    REAL aOther;

    aSelf = dec_real_to(tSelf);
    aOther = dec_real_to(tOther);

    aSelf = pow(aSelf, aOther);
    set_dec_real_from(tSelf, aSelf);
    return SUCCESS;
}

LIBAPI int fix_dec_real_exp(DecReal* tSelf, const DEC tDecExpFix){
    DEC aDiffExp;
    DEC retMin;
    DEC retMax;
    
    if (tSelf->mIntPart==0 && tSelf->mFracPart==0){
        // 정수부와 소수부가 0이면 지수 고정시키는 의미가 없다.
        return DEC_REAL_DEC_ZERO;
    }

    // 고정할 지수와 현재 지수의 차이를 확인한다.
    aDiffExp = tDecExpFix - tSelf->mExp;
    if(aDiffExp == 0){
        // 고정할 지수와 현재 지수가 같으면 종료한다.
        return DEC_REAL_DEC_ZERO;
    }

    dec_real_shift_range_ret(tSelf, &retMin, &retMax);
    // 지수픽스가 지수 이동 범위 보다 크면
    // return DEC_REAL_FIX_EXP_OVER_RANGE
    if( aDiffExp > retMax ){
        return DEC_REAL_FIX_EXP_OVER_RANGE;
    }

    // 작으면
    // return DEC_REAL_FIX_EXP_UNDER_RANGE
    if (aDiffExp < retMin ){
        return DEC_REAL_FIX_EXP_UNDER_RANGE;
    }

    // 지수 차이 만큼 시프트한다.
    // DEC_REAL_EXP_UNDER 지수가 0이됨
    // DEC_REAL_EXP_OVER 무한대가 됨.
    shift_dec_real(tSelf, aDiffExp);

    // 원래의 시프트는 지수 이동 범위 내에 유효 숫자들이 이동하면
    // 지수는 안 변한다.
    // 하지만, 고정지수는 원래의 값과 이동한 값이 같아야 한다.
    // 지수는 이동한 방향으로 덧셈한다.
    set_dec_real_exp(tSelf, tSelf->mExp + aDiffExp);

    return SUCCESS;
}

LIBAPI int fix_dec_real_exp_trunc(DecReal* tSelf, const DEC tDecExpFix){
    DEC aDiffExp;
    DEC retMin;
    DEC retMax;
    
    if (tSelf->mIntPart==0 && tSelf->mFracPart==0){
        // 정수부와 소수부가 0이면 지수 고정시키는 의미가 없다.
        return DEC_REAL_DEC_ZERO;
    }

    // 고정할 지수와 현재 지수의 차이를 확인한다.
    aDiffExp = tDecExpFix - tSelf->mExp;
    if(aDiffExp == 0){
        // 고정할 지수와 현재 지수가 같으면 종료한다.
        return DEC_REAL_DEC_ZERO;
    }

    dec_real_shift_range_ret(tSelf, &retMin, &retMax);
    
    // 작으면
    // return DEC_REAL_FIX_EXP_UNDER_RANGE
    // 왼쪽으로 시프트하는데 왼쪽은 최대값이라 자르기가 안된다.
    // 자르기는 오른쪽으로 이동 할 때만 적용된다.
    if (aDiffExp < retMin ){
        return DEC_REAL_FIX_EXP_UNDER_RANGE;
    }

    // 지수 차이 만큼 시프트한다.
    // DEC_REAL_EXP_OVER 무한대가 됨.
    // 자르기는 오른쪽으로 이동 할 때만 적용된다.
    shift_dec_real_trunc(tSelf, aDiffExp);

    // 원래의 시프트는 지수 이동 범위 내에 유효 숫자들이 이동하면
    // 지수는 안 변한다.
    // 하지만, 고정지수는 원래의 값과 이동한 값이 같아야 한다.
    // 지수는 이동한 방향으로 덧셈한다.
    set_dec_real_exp(tSelf, tSelf->mExp + aDiffExp);

    return SUCCESS;
}

LIBAPI int dec_real_fix_exp_ret(const DecReal* tSelf, DEC *tDecExpFix, 
    bool* retSign, UDEC* retFracPart, UDEC* retIntPart){
    
    DEC aDiffExp;
    DecReal* aSelf = NULL;
    DEC aFixResult = SUCCESS;
    DEC retMin;
    DEC retMax;
    
    aSelf = create_dec_real_default();
    assign_dec_real(aSelf, tSelf);

    if (tSelf->mIntPart==0 && tSelf->mFracPart==0){
        // 정수부와 소수부가 0이면 지수 고정시키는 의미가 없다.
        *retIntPart = 0;
        *retFracPart = 0;
        *retSign = tSelf->mSign;
    delete_dec_real(aSelf);
        return DEC_REAL_DEC_ZERO;
    }

    // 고정할 지수와 현재 지수의 차이를 확인한다.
    aDiffExp = *tDecExpFix - tSelf->mExp;
    if(aDiffExp == 0){
        *retIntPart = tSelf->mIntPart;
        *retFracPart = tSelf->mFracPart;
        *retSign = tSelf->mSign;
            delete_dec_real(aSelf);
        return aFixResult;
    }

    dec_real_shift_range_ret(tSelf, &retMin, &retMax);
    // 지수픽스가 지수 이동 범위 보다 크면
    // return DEC_REAL_FIX_EXP_OVER_RANGE
    if( aDiffExp > retMax ){
        aDiffExp = retMax;
        aFixResult = DEC_REAL_FIX_EXP_OVER_RANGE;
    }

    // 작으면
    // return DEC_REAL_FIX_EXP_UNDER_RANGE
    if (aDiffExp < retMin ){
        aDiffExp = retMin;
        aFixResult = DEC_REAL_FIX_EXP_UNDER_RANGE;
    }
    // 범위 밖이라고 반환은 하지만
    // 연산은 범위 내의 값으로 완료한다.


    // 지수 차이 만큼 시프트한다.
    shift_dec_real(aSelf, aDiffExp);

    // 원래의 시프트는 지수 이동 범위 내에 유효 숫자들이 이동하면
    // 지수는 안 변한다.
    // 하지만, 고정지수는 원래의 값과 이동한 값이 같아야 한다.
    // 지수는 이동한 방향의 반대방향으로 덧셈한다.
    // 하지만, 현재 함수는 dec_real을 반환하는 것이 아니다.
    // 정수부와 소수부만 반환하므로, 아래 작업은 필요 없다.
    // aSelf->mExp -= aDiffExp;

    *retSign = dec_real_sign(aSelf);
    *retFracPart = dec_real_frac(aSelf);
    *retIntPart = dec_real_int(aSelf);
    *tDecExpFix = aDiffExp + tSelf->mExp;

    delete_dec_real(aSelf);
    return aFixResult;
}

LIBAPI int dec_real_fix_exp_trunc_ret(const DecReal* tSelf,
    DEC *tDecExpFix, 
    bool* retSign, UDEC* retFracPart, UDEC* retIntPart){

    DEC aDiffExp;
    DecReal* aSelf = NULL;
    DEC aFixResult = SUCCESS;
    DEC retMin;
    DEC retMax;
    
    aSelf = create_dec_real_default();
    assign_dec_real(aSelf, tSelf);

    if (tSelf->mIntPart==0 && tSelf->mFracPart==0){
        // 정수부와 소수부가 0이면 지수 고정시키는 의미가 없다.
        *retIntPart = 0;
        *retFracPart = 0;
        *retSign = tSelf->mSign;
    delete_dec_real(aSelf);
        return DEC_REAL_DEC_ZERO;
    }

    // 고정할 지수와 현재 지수의 차이를 확인한다.
    aDiffExp = *tDecExpFix - tSelf->mExp;
    if(aDiffExp == 0){
        *retIntPart = tSelf->mIntPart;
        *retFracPart = tSelf->mFracPart;
        *retSign = tSelf->mSign;
            delete_dec_real(aSelf);
        return aFixResult;
    }

    // 지수 차이 만큼 시프트한다.
    shift_dec_real_trunc(aSelf, aDiffExp);

    // 원래의 시프트는 지수 이동 범위 내에 유효 숫자들이 이동하면
    // 지수는 안 변한다.
    // 하지만, 고정지수는 원래의 값과 이동한 값이 같아야 한다.
    // 지수는 이동한 방향의 반대방향으로 덧셈한다.
    // 하지만, 현재 함수는 dec_real을 반환하는 것이 아니다.
    // 정수부와 소수부만 반환하므로, 아래 작업은 필요 없다.
    // aSelf->mExp -= aDiffExp;

    *retSign = dec_real_sign(aSelf);
    *retFracPart = dec_real_frac(aSelf);
    *retIntPart = dec_real_int(aSelf);
    *tDecExpFix = aDiffExp + tSelf->mExp;

    delete_dec_real(aSelf);
    return aFixResult;
}

LIBAPI int convert_dec_real_to(const DecReal* tSelf, REAL* retReal){
    
    UDEC aFrac = tSelf->mFracPart;
    UDEC aInt = tSelf->mIntPart;
    DEC aExp = tSelf->mExp;

    DEC aShift;
    
    Real* r = NULL;
    r = create_real(0.0);

    // real의 소수부에 dec_real의 소수부를 적용하려면 변환이 필요하다.
    // 소수부의 456은 dec_real에서는 456 000 000을 의미한다.
    // 소수부의 456789 는 dec_real에서는 456 789 000 을 의미한다.
    // dec_real의 000 456 000 은 소수부에서는 ###.000 456 이 된다.
    // dec_real의 456 000 000 은 소수부에서는 ###.456 이 된다.
    // dec_real이 000 456 000 일 경우에 소수부로 가면 ###.000456이 되는 것이 아니라,
    // 정수부의 숫자를 가져와서 ##.#000456이 되야 한다.
    // 이게 안되면 000을 정수부로 이동 시키고 ###000.456을 만들어야 한다.
    // 지수가 이동한 만큼 덧셈하거나 뺄셈한다.
    // 그걸 여기서 구현해야 한다.
    // 그렇게 만들어놓고 merge_real을 해야 한다.

    // 소수부가 0으로 시작하는 지 확인한다.
    // 방법은 
    // 소수부의 자리수를 확인한다.
    UDEC aDigitsFrac = dec_real_frac_digits(tSelf);

    // 소수부의 유효 숫자를 확인한다.
    UDEC aSigFrac = dec_real_frac_sig(tSelf);

    // 소수부의 유효 숫자의 자리수가 소수부의 자리수보다 작으면 0으로 시작한다.
    // 소수부의 자리수 - 유효 숫자는 0 부터 시작한다. 음수가 나올 수가 없다.
    UDEC aFrontZeroCountFrac = aDigitsFrac - dec_digits(aSigFrac);

    // 만약 유효 숫자와 자리수가 같으면
    if ( aFrontZeroCountFrac == 0){
        // FracPart를 merge_real 할 수 있게 변환한다.
        aFrac /= (UDEC)pow(10, DEC_REAL_DIGITS - aDigitsFrac);
    }
    else{
        // 정수부와 연동해서 0으로 시작하는 부분을 없앤다.
        // 지수부도 연동되어서 같이 이동한다.
        DecReal aSelf;
        assign_dec_real(&aSelf, tSelf);            

        // 정수부의 자리수를 확인한다.
        UDEC aIntDigits = dec_real_int_digits(tSelf);
        UDEC aIntRemainDigits = DEC_REAL_DIGITS - aIntDigits;

        // 정수부의 남은 자리수가 aFrontZeroCountFrac 보다 크면
        if ( aIntRemainDigits >= aFrontZeroCountFrac){
            // 왼쪽으로 시프트한다. aFrontZeroCountFrac 만큼...
            shift_dec_real(&aSelf, -aFrontZeroCountFrac);

            // 이동한 만큼 지수에서 뺀다.
            set_dec_real_exp(&aSelf, aSelf.mExp-aFrontZeroCountFrac);
        }
        else{
            // 소수부 시작부의 0이 제거될 만큼 정수부에 남은 자리수가 없다.
            // 오른쪽으로 이동한다.
            // 정수부의 유효숫자 1개가 소수부의 시작부에 이동할 만큼...
            // 이동할 만큼이 어느정도 냐면
            // aShift = 정수부의 자리수 - 유효 자리수 +1;
            aShift = aIntDigits - dec_sig_digits(aSelf.mIntPart) + 1;
            shift_dec_real(&aSelf, aShift);
            
            // 이동한 만큼 지수에서 뺀다.
            set_dec_real_exp(&aSelf, aSelf.mExp-aShift);
        }

        aFrac = aSigFrac;
        aInt = aSelf.mIntPart;
        aExp = aSelf.mExp;
    }

    merge_real(r, tSelf->mSign, aFrac, aInt, aExp);
    *retReal = real(r);
    delete_real(r);
    return SUCCESS;        
}

LIBAPI const REAL dec_real_to(const DecReal* tSelf){
    REAL r;
    convert_dec_real_to(tSelf, &r);
    return r;
}

LIBAPI int convert_dec_real_frac_to(const DecReal* tSelf, REAL* retReal){
// DecReal의 소수부만 0.###########의 형태로 부동소수점으로 얻어낸다.
    UDEC aDigits;
    UDEC aSigDigits;
    // aDigits = dec_real_frac_digits(tSelf);
    // aSigDigits = dec_real_frac_sig(tSelf);
    aDigits = dec_digits(tSelf->mFracPart);
    *retReal = tSelf->mFracPart * pow(10,-DEC_REAL_DIGITS);
    return 0;
}

LIBAPI const REAL dec_real_frac_to(const DecReal* tSelf){
// DecReal의 소수부만 0.###########의 형태로 부동소수점으로 반환한다.
    REAL r;
    convert_dec_real_frac_to(tSelf, &r);
    return r;
}

LIBAPI bool dec_real_equal(const DecReal* tSelf, const DecReal* tOther,
    const REAL tTol){

// 절대 공차 fabs(a-b) <= epsilon
// 공차를 0.000001이라고 입력해야 하는데
// 1도 가능하고 10, 100, 1000도 가능하네
DecReal a;
DecReal b;
DecReal t;

bool aResult;

init_dec_real(&a);
init_dec_real(&b);
init_dec_real(&t);

assign_dec_real(&a, tSelf);
assign_dec_real(&b, tOther);
set_dec_real_from(&t, tTol);

sub_dec_real(&a, &b);
abs_dec_real(&a);

aResult = dec_real_le(&a, &t);

release_dec_real(&a);
release_dec_real(&b);
release_dec_real(&t);

return aResult;
}

LIBAPI bool dec_real_equal_rel(const DecReal* tSelf, const DecReal* tOther, 
    const REAL tTol){

// 상대 공차 fabs(a-b)/길이(a,b) <= epsilon
// 상대 공차 fabs(a-b) <= epsilon*max(a,b)
// 상대 공차는 비율의 차이가 공차보다 작으면 같다는 것이다.
// 비율의 차이로 인식하므로 
// 공차 비율이 1/100이면 1mm 와 1.01mm 는 같다.
// 공차 비율 1/100이면 1m와 1.01미터는 같다.        
DecReal a;
DecReal b;
DecReal t;
DecReal m;
DecReal r;

bool aResult;

init_dec_real(&a);
init_dec_real(&b);
init_dec_real(&t);
init_dec_real(&r);

assign_dec_real(&a, tSelf);
assign_dec_real(&b, tOther);
set_dec_real_from(&t, tTol);

// 상대 공차 fabs(a-b) <= epsilon*max(a,b)

// (a-b)
r = dec_real_sub(&a, &b);

// fabs(a-b)
abs_dec_real(&r);

abs_dec_real(&a);
abs_dec_real(&b);
// epsilon*max(a,b)
m = dec_real_max(&a, &b);
mul_dec_real(&t, &m);

aResult = dec_real_le(&r, &t);

release_dec_real(&a);
release_dec_real(&b);
release_dec_real(&t);
release_dec_real(&r);

return aResult;
    }

LIBAPI bool dec_real_gt(const DecReal* tSelf, const DecReal* tOther){
    // self가 other보다 크냐?

    // 부호가 같은지 다른지 확인한다.
    // 부호가 다르면 양수가 크다.
    if (tSelf->mSign != tOther->mSign){
        if(tSelf->mSign){
            // 셀프가 음수이면 작다.
            return false;
        }
        else{
            return true;
        }
    }

    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    return aSelfValue > aOtherValue;
}

LIBAPI bool dec_real_ge(const DecReal* tSelf, const DecReal* tOther){
    // self가 other보다 크거나 같냐?    

    // 부호가 같은지 다른지 확인한다.
    // 부호가 다르면 양수가 크다.
    if (tSelf->mSign != tOther->mSign){
        if(tSelf->mSign){
            // 셀프가 음수이면, 작다.
            return false;
        }
        else{
            return true;
        }
    }

    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    return aSelfValue >= aOtherValue;
}

LIBAPI bool dec_real_lt(const DecReal* tSelf, const DecReal* tOther){
    // self가 other보다 작냐?

    // 부호가 같은지 다른지 확인한다.
    // 부호가 다르면 음수가 작다.
    if (tSelf->mSign != tOther->mSign){
        if(tSelf->mSign){
            // 셀프가 음수이면 작다.
            return true;
        }
        else{
            return false;
        }
    }

    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    return aSelfValue < aOtherValue;
}

LIBAPI bool dec_real_le(const DecReal* tSelf, const DecReal* tOther){
    // self가 other보다 작거나 같냐?    

    // 부호가 같은지 다른지 확인한다.
    // 부호가 다르면 음수가 작다.
    if (tSelf->mSign != tOther->mSign){
        if(tSelf->mSign){
            // 셀프가 음수이면 작다.
            return true;
        }
        else{
            return false;
        }
    }

    REAL aSelfValue;
    REAL aOtherValue;

    aSelfValue = dec_real_to(tSelf);
    aOtherValue = dec_real_to(tOther);

    return aSelfValue <= aOtherValue;
}

// 가감승제를 하다보면 오버플로우나 언더플로우 또는 0으로 나누기등등의 상황이 나온다.
// 오버플로우와 언더플로우가 나와도 그냥 무시하고 연산한다.
// 0으로 나누기가 나오면 무한대라고 하자.
// 무한대는 전부 max로 설정한다.
LIBAPI const DecReal dec_real_add(const DecReal* tSelf, const DecReal* tOther){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    add_dec_real(&aResult, tOther);
    return aResult;
}

LIBAPI const DecReal dec_real_sub(const DecReal* tSelf, const DecReal* tOther){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    sub_dec_real(&aResult, tOther);
    return aResult;
}

LIBAPI const DecReal dec_real_mul(const DecReal* tSelf, const DecReal* tOther){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    mul_dec_real(&aResult, tOther);
    return aResult;
}

LIBAPI const DecReal dec_real_div(const DecReal* tSelf, const DecReal* tOther){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    div_dec_real(&aResult, tOther);
    return aResult;
}

LIBAPI const DecReal dec_real_pow(const DecReal* tSelf, const DecReal* tOther){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    pow_dec_real(&aResult, tOther);
    return aResult;    
}

LIBAPI const DecReal dec_real_shift(const DecReal* tSelf, const DEC tDec){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    shift_dec_real(&aResult, tDec);
    return aResult;
}

LIBAPI const DecReal dec_real_shift_trunc(const DecReal* tSelf, const DEC tDec){
    DecReal aResult;
    assign_dec_real(&aResult, tSelf);
    shift_dec_real_trunc(&aResult, tDec);
    return aResult;    
}

LIBAPI int abs_dec_real(DecReal* tSelf){
    // 부호가 따로 되어 있어서 무조건 양수이다.
    if(tSelf->mSign){
        tSelf->mSign = false;
    }
    return SUCCESS;
}

LIBAPI const DecReal dec_real_abs(const DecReal* tSelf){
    DecReal r;
    assign_dec_real(&r, tSelf);
    abs_dec_real(&r);
    return r;
}

LIBAPI const DecReal dec_real_max(const DecReal* tSelf, const DecReal* tOther){
    if(dec_real_ge(tSelf,tOther)){
        return *tSelf;
    }
    return *tOther;
}

LIBAPI int max_dec_real(DecReal* tSelf, const DecReal* tOther){
    if(dec_real_lt(tSelf,tOther)){
        assign_dec_real(tSelf, tOther);
    }
    return SUCCESS;
}

LIBAPI int shift_dec_real(DecReal* tSelf, const DEC tDec){
    // 왼쪽이나 오른쪽으로 전체 자리수가 이동한다.
    // 오른쪽 끝에 유효자리수 까지 도착하면, 지수를 더한다.
    // 계속 이동하면 결국에는 지수가 최고 지수가 된다.
    // 지수가 최고 지수보다 크면
    // 오버플로우
    // 원래의 유효자리수 값을 유지한다.
    // 왼쪽으로 이동 시에는 정수의 최대 자리의 숫자가 벽에 도착하면,
    // 지수를 뺄셈한다.
    // 결국에는 지수가 최소 지수가 된다.
    // 지수가 최소지수보다 작으면
    // 언더플로우
    // 원래 0이었으면 그냥 0을 반환한다.
    // tDec 숫자가 양수이면 오른쪽으로 이동한다.
    // tDec 숫자가 음수이면 왼쪽으로 이동한다.

    // 지수 차이가 +이면 오른쪽으로 시프트한다.
    // 123.456 -> 0.123456 이 된다는 의미이다.

    if(tDec == 0){
        // 0이면 아무것도 안하고 종료한다.
        return SUCCESS;
    }

    if (tDec > 0){
        return shift_dec_real_to_right(tSelf, tDec);
    }

    // 지수 차이가 -이면 왼쪽으로 시프트한다.
    return shift_dec_real_to_left(tSelf, abs(tDec));
}

LIBAPI int shift_dec_real_trunc(DecReal* tSelf, const DEC tDec){
    if(tDec == 0){
        // 0이면 아무것도 안하고 종료한다.
        return SUCCESS;
    }

    if (tDec > 0){
        return shift_dec_real_to_right_trunc(tSelf, tDec);
    }

    // 지수 차이가 -이면 왼쪽으로 시프트한다.
    return shift_dec_real_to_left(tSelf, abs(tDec));
}

LIBAPI int set_dec_real_from(DecReal* tSelf, const REAL tReal){

    Real aReal_t;
    init_real(&aReal_t);
    set_real(&aReal_t, tReal);
    return set_dec_real_from_real(tSelf, &aReal_t);
}

LIBAPI int set_dec_real_from_real(DecReal* tSelf, const Real* tReal){
    DecReal aDecReal;
    
    bool retSign;
    UDEC retFracPart;
    UDEC retIntPart;
    UDEC aRemainDigits;

    DEC retExpPart;    

    split_real(tReal, &retSign, &retFracPart, &retIntPart, &retExpPart);

    init_dec_real(&aDecReal);
    
    // // dec_real의 소수부에 숫자를 설정하려면,
    // // 최고 자리수에서 소수부자리수를 빼고 남은 자리수를 
    // // 분리된소수부 * 10^남은지수로 곱셈을 해야 한다.
    // aRemainDigits = DEC_REAL_DIGITS - dec_digits(retFracPart);
    // retFracPart *= (UDEC)pow(10, aRemainDigits);

    // 2024-06-25 11:53:21
    // 위의 방법은 소수부가 정수부로 이동할 시에 자리수 오류가 생긴다.
    // 120.0456 이 120.456이 된다.
    // value: 7.00014045e+28 , dec_real_member_var: 714.45000000 * 10^26

    // 12003456e10 이 1, 2003456, 10으로 분리되었을 것이다.
    // 정수부, 지수부, 부호는 그대로 dec_real 에 대입한다.
    // 소수부는
    // 2003456이 dec_real에서는 200 345 600이 된다.
    // 소수부에 최고 지수 - 현재 소수부 자리수를 10의거듭제곱한다.
    retFracPart *= (UDEC)pow(10, DEC_REAL_DIGITS - dec_digits(retFracPart) );
    

    set_dec_real_sign(&aDecReal, retSign);
    set_dec_real_frac(&aDecReal, retFracPart);
    set_dec_real_int(&aDecReal, retIntPart);
    set_dec_real_exp(&aDecReal, retExpPart);

    // DEC aDiffDigits = dec_real_int_digits(&aDecReal) - 
    //     dec_real_frac_digits(&aDecReal);
    // shift_dec_real(&aDecReal, aDiffDigits/2);
    assign_dec_real(tSelf, &aDecReal);
    
    return SUCCESS;
}

LIBAPI int shift_dec_real_to_right(DecReal* tSelf, const UDEC tDec){
    // 1230.0456 -> 0.012 300 456 이 된다.

    if(dec_real_is_zero(tSelf)){
        return SUCCESS;
    }

    UDEC aPoppedIntPart;

    DEC aPowShift;
    UDEC aRemainDigits;
    UDEC aShift = 0;
    UDEC aRemainDec;    
    
    UDEC aPoppedFracPart;

    UDEC aFracPartDigits;

    DEC expPart = tSelf->mExp;

    UDEC bufFracPart;
    UDEC bufIntPart;

    // 소수부의 자리수를 소수부 형태로 확인한다.
    aFracPartDigits = dec_real_frac_digits(tSelf);

    // 소수부의 남은 자리수는 
    aRemainDigits = DEC_REAL_DIGITS - aFracPartDigits;

    // tDec이 남은 자리수보다 크면
    if (tDec > aRemainDigits){
        // 우선 남은 자리수만큼 시프트한다.
        aShift = aRemainDigits;

        // 이동할 지수에서 시프트 한만큼은 뺄셈한다.
        aRemainDec = tDec - aShift;

    }
    // tDec이 남은 자리수보다 작으면
    else{
        // tDec 만큼 소수부와 정수부와 지수부를 이동한다.
        aShift = tDec;
        aRemainDec = 0;
    }

    if ( aShift == 0){
        aPowShift = 1;
    }
    else{
        aPowShift = (DEC)pow(10,aShift);
    }

    bufIntPart = tSelf->mIntPart / aPowShift;
    bufFracPart = tSelf->mFracPart / aPowShift;

    aPoppedIntPart = 
        tSelf->mIntPart -  bufIntPart * aPowShift;

    if (aPoppedIntPart != 0){
        bufFracPart += (aPoppedIntPart * 
            (UDEC)pow(10,DEC_REAL_DIGITS-aShift) );

    }

    set_dec_real_frac(tSelf, bufFracPart);
    set_dec_real_int(tSelf, bufIntPart);

    // 총 이동할 지수에서 시프트 한 만큼 빼고 지수에 더한다.
    expPart = tSelf->mExp + aRemainDec;

    // 시프트 한 지수가 지수 최대값보다 크면,
        if (expPart > DEC_REAL_EXP_MAX){
        // 시프트할 지수가 너무 크다.
        // // 유효 숫자는 그대로 놔둔다.
        // // 지수를 변경해야 하는데,
        // // 지수맥스 또는 오버플로우의 의미대로 라면 지수맥스가 적당한 값이다.
        // set_dec_real_exp(tSelf, DEC_REAL_EXP_MAX);
        // 지수가 크면 무한대가 되야 한다. 2024-07-15 11:46:10
        set_dec_real_from_real(tSelf, &Inf);            

        return DEC_REAL_EXP_OVER;

    }

    set_dec_real_exp(tSelf, expPart);
    return SUCCESS;    
}

LIBAPI int shift_dec_real_to_right_trunc(DecReal* tSelf, const UDEC tDec){

    // 1230.0456 -> 0.012 300 456 이 된다.
    // 오른쪽으로 이동시에는 
    // 오른쪽으로 이동하면서 최소자리수가 잘린다.
    // 정책에 따라 올림한다.
    // 정수부와 소수부가 모두 0이 된다.
    // 지수부는 유지한다.
    // return DEC_REAL_DEC_ZERO;    

    
        // // TRUNC_POLICY에 따라 소수자리의 1번자리에 
        // // 반올림, 올림, 내림결정해서 덧셈한다.
        // switch(tSelf->mPolicy){
        //     case DEC_MIN_TRUNC_POLICY_FLOOR:
        //         // bufFracPart += 0;
        //     break;
        //     case DEC_MIN_TRUNC_POLICY_CEILING:
        //         bufFracPart += 1;
        //     break;
        //     case DEC_MIN_TRUNC_POLICY_ROUND:
        //     aPoppedFracPart = ((tSelf->mFracPart - bufFracPart*aPowShift)*10) /
        //         aPowShift;
        //     if (aPoppedFracPart >= 5){
        //         bufFracPart += 1;
        //     }
        //     break;
        // }


    return shift_dec_real_to_right(tSelf, tDec);        
}

LIBAPI int shift_dec_real_to_left(DecReal* tSelf, const UDEC tDec){
    // 123.456 -> 123456.0 이 된다는 의미이다.
    // 먼저, 지수 차이가 현재정수부의남은자리수 보다 큰지 확인한다.
    // 남은자리수 = REAL_DIGITS - 정수자리수
    // 남은자리수보다 지수 차이가 크면 오버플로우이다.

    UDEC aDec = tDec;
    UDEC aRemainDec;
    UDEC aPowShift;
    UDEC aShift = 0;
    
    UDEC aPoppedFracPart;
    UDEC aPowRemainIntDigits;

    UDEC aRemainIntDigits;
    UDEC aIntDigits;
    UDEC aFracDigits;

    UDEC intPart;
    DBL_UDEC fracPart;

    DEC expPart;    

    aIntDigits = dec_digits(tSelf->mIntPart);
    aRemainIntDigits = DEC_REAL_DIGITS - aIntDigits;

    // tDec 이 남은 자리수보다 크면
    if ( tDec > aRemainIntDigits){
        // 소수부와 정수부를 남은 자리수 만큼 이동한다.
        aShift = aRemainIntDigits;

        // 그러고도 더 이동해야 한다.
        // 하지만 유효숫자는 건들 수가 없다.
        // 지수만 계속 작아지다가 결국에는 0이 된다.
        aRemainDec = tDec - aShift;
    }
    else{

        // tDec이 남은 자리수보다 작으면
        // 소수부와 정수부를 tDec 만큼 이동한다.
        aShift = tDec;
        aRemainDec = 0;
    }

    if ( aShift == 0){
        aPowShift = 1;
    }
    else{
        aPowShift = (UDEC)pow(10,aShift);
    }

    intPart = tSelf->mIntPart;
    if(tSelf->mIntPart != 0){
        intPart *= aPowShift;
    }
    
    // 2024-06-27 08:34:31
    // 소수부를 왼쪽으로 이동한다.
    fracPart = tSelf->mFracPart;
    fracPart *= aPowShift;

    // 소수부의 자리수가 DEC_REAL_DIGITS 보다 크면 
    aFracDigits = dec_digits(fracPart);
    if (aFracDigits > DEC_REAL_DIGITS){
        // 최고자리수를 자른다.
        UDEC aPowDigits = (UDEC)pow(10, DEC_REAL_DIGITS);
        aPoppedFracPart = fracPart / aPowDigits;
        // 자른 만큼 정수부에 더한다.
        fracPart -= (aPoppedFracPart * aPowDigits);
        intPart += aPoppedFracPart;
    }

    set_dec_real_int(tSelf, intPart);
    set_dec_real_frac(tSelf, fracPart);

    // 지수부를 변경한다.
    // 먼저 시프트한 지수는 더하면 안된다.
    expPart = tSelf->mExp - aRemainDec;

    if (expPart < DEC_REAL_EXP_MIN){
        // 유효 자리수는 유지해야겠다.
        // 왼쪽으로 시프트하면 지수가 점점 작아지면서 가장 작아지면 0에 가까워진다.
        set_dec_real_exp(tSelf, 0);        
        return DEC_REAL_EXP_UNDER;
    }    
    set_dec_real_exp(tSelf, expPart);
    return SUCCESS;
}

LIBAPI const UDEC dec_real_digits(const DecReal* tSelf){
    UDEC aIntPartCount;
    UDEC aFracPartCount;

    aIntPartCount = dec_real_int_digits(tSelf); 
    aFracPartCount = dec_real_frac_digits(tSelf);

    return aIntPartCount + aFracPartCount;
}

LIBAPI const DBL_DEC dec_real_sig(const DecReal* tSelf){
    return dec_real_int_sig(tSelf) * 
        (DEC)pow(10, dec_real_frac_digits(tSelf) ) +
        dec_real_frac_sig(tSelf);
}

LIBAPI const UDEC dec_real_int_digits(const DecReal* tSelf){
    return dec_digits(tSelf->mIntPart);
}

LIBAPI const UDEC dec_real_int_sig(const DecReal* tSelf){
    return dec_sig(tSelf->mIntPart);
}

LIBAPI const UDEC dec_real_frac_sig(const DecReal* tSelf){
    UDEC aFrac;

    // 001234000의 유효 숫자는 1234이다.
    // 정수부의 유효 숫자와 같다.
    aFrac = dec_sig(tSelf->mFracPart);
    return aFrac;
}

LIBAPI const UDEC dec_real_frac_digits(const DecReal* tSelf){
    // 소수의 자리수를 확인한다.
    // 범위는 0 ~ REAL_DIGITS
    // 유효자리수는 001234000에서 앞,뒤의 0들을 빼고
    // 1234 가 원래의 유효자리수이다.
    // 하지만, dec_real 소수부의 유효자리수는 뒤의 0들을 빼고
    // 최고자리수는 9이고 
    // 현재 자리수는 7이고 
    // 유효자리수는 4이다.
    // dec_real의 소수의 유효자리수는 
    // 남은 자리수 = 최고자리수 - 현재 자리수 
    // 소수의 유효자리수 = 유효 자리수 + 남은 자리수
    // 그래서 소수의 유효 자리수는 6 이다.
    // 001234 가 유효자리수이다.
    // 자리수 끝부분에 0이 계속 반복되는 구간이 있으면 
    // 그만큼 자리수에서 제거해야 한다.

    if (tSelf->mFracPart == 0){
        // 소수부의 자리수는 0이다.
        return 0;
    }
    // 현재 자리수 = dec_digits(tSelf->mFracPart);
    UDEC aFracPartDigits = dec_digits(tSelf->mFracPart);

    // 유효 자리수 = dec_sig_digits(tSelf->mFracPart);
    UDEC aFracPartSigDigits = dec_sig_digits(tSelf->mFracPart);    

    // 남은 자리수 = 최고 자리수 - 현재 자리수
    UDEC aFracPartRemainDigits = DEC_REAL_DIGITS - aFracPartDigits;

    // dec_real의 소수부의 자리수 = 남은 자리수 + 유효자리수
    return aFracPartRemainDigits + aFracPartSigDigits;
}

LIBAPI int convert_dec_real_to_real(const DecReal* tSelf, Real* retReal){
    REAL aReal;
    aReal = (tSelf->mIntPart+dec_real_frac_to(tSelf)) * pow(10, tSelf->mExp);
    if (tSelf->mSign){
        // 부호가 음수이면
        aReal *= -1.0;
    }
    set_real(retReal, aReal);
    return SUCCESS;
}

LIBAPI const REAL dec_real_to_real(const DecReal* tSelf){
    REAL r;
    convert_dec_real_to_real(tSelf, &r);
    return r;
}

LIBAPI const DecReal dec_real_from_real(const Real* tReal){
    DecReal aDecReal;

    init_dec_real(&aDecReal);
    set_dec_real_from_real(&aDecReal, tReal);

    return aDecReal;
}

LIBAPI const DecReal dec_real_from(const REAL tReal){
    DecReal aDecReal;

    init_dec_real(&aDecReal);
    set_dec_real_from(&aDecReal, tReal);

    return aDecReal;
}

LIBAPI DecReal* create_dec_real_default(){
    DecReal* r = NULL;
    r = (DecReal*)malloc(sizeof(DecReal));
    if (r == NULL){
        return NULL;
    }
    init_dec_real(r);

    return r;
}

LIBAPI DecReal* create_dec_real(const bool tSign, 
    const UDEC tFracPart, const UDEC tIntPart, const DEC tExpPart, 
    const DecMinTruncPolicy tPolicy){

    DecReal* r = NULL;
    r = (DecReal*)malloc(sizeof(DecReal));
    if (r == NULL){
        return NULL;
    }
    init_dec_real(r);
    set_dec_real(r, tSign, tFracPart, tIntPart, tExpPart, tPolicy);

    return r;
}

LIBAPI DecReal* create_dec_real_from(REAL tValue){
    DecReal* r = NULL;
    r = create_dec_real_default();
    set_dec_real_from(r, tValue);
    return r;
}
    
LIBAPI int delete_dec_real(DecReal* tSelf){
    if (tSelf != NULL){
        free(tSelf);
        tSelf = NULL;
    }
    return 0;
}

LIBAPI int dec_real_shift_range_ret(const DecReal* tSelf, 
    DEC* retMin, DEC* retMax){

    DEC aRemainIntDigits;
    DEC aRemainFracDigits;

    aRemainIntDigits = DEC_REAL_DIGITS - dec_real_int_digits(tSelf);
    aRemainFracDigits = DEC_REAL_DIGITS - dec_real_frac_digits(tSelf);

    *retMin = -aRemainIntDigits;
    *retMax = aRemainFracDigits;
    return 0;
}

LIBAPI int get_dec_real_str(const DecReal* tSelf, Ustring* retStr){
    char buf[64];

    set_ustring(retStr, "");

    if(tSelf->mIntPart == 0){
        append_ustring_from(retStr, "0");
    }
    else{
        append_ustring_from(retStr,(tSelf->mSign?"-":""));
        // memset(buf,0, 64);
        sprintf(buf, "%d", tSelf->mIntPart);
        append_ustring_from(retStr, buf);        
    }
    
    if(tSelf->mFracPart != 0){
        // memset(buf,0, 64);
        append_ustring_from(retStr, ".");
        sprintf(buf, "%s", dec_real_frac_str(tSelf, buf));
        append_ustring_from(retStr, buf);
    }
    if(tSelf->mExp != 0){
        // memset(buf,0, 64);
        sprintf(buf, "e%d", tSelf->mExp);
        append_ustring_from(retStr, buf);
    }
    return SUCCESS;
}

LIBAPI const char* dec_real_str(const DecReal* tSelf, char* buf){
    Ustring* aStr = NULL;
    aStr = create_ustring_default();
    get_dec_real_str(tSelf, aStr);
    strcpy(buf, ustring(aStr));
    delete_ustring(aStr);

    return buf;
}

LIBAPI int get_dec_real_frac_str(const DecReal* tSelf, Ustring* retStr){
    // 소수부의 숫자를 문자열로 출력하려면 '.'다음에 12.00456일 때
    // 00456을 출력해야 한다.
    // 유효숫자는 456이다.
    // 하지만 00을 시작할 때 추가해야한다.
    // DEC_REAL_DIGITS - 소수부의 자리수를 정수부의 자리수 구하기로 구한다.
    // 차이만큼 0을 추가해야한다.
    // 001234567일 경우에 자리수를 정수부처럼 구하면 7이다. 123456이니까...
    // 차이는 2이다. 0을 2번 추가한다.
    // 그 후에 유효숫자를 문자열로 변환하고 추가한다.
    // 반환한다.
    char buf[64];
    DEC aDiffDigits;
    DEC aSig;

    set_ustring(retStr, "");
    if( tSelf->mFracPart == 0){
        aDiffDigits = 0;
        aSig = 0;
    }
    else{
        aDiffDigits = DEC_REAL_DIGITS - dec_digits(tSelf->mFracPart);
        aSig = dec_real_frac_sig(tSelf);
        
        for(int i=0; i<aDiffDigits; i++){
            append_ustring_from(retStr, "0");
        }        
        sprintf(buf, "%d", aSig);
        append_ustring_from(retStr, buf);
    }

    return SUCCESS;
}

LIBAPI const char* dec_real_frac_str(const DecReal* tSelf, char* buf){
    Ustring* aStr = NULL;
    aStr = create_ustring_default();

    get_dec_real_frac_str(tSelf, aStr);
    strcpy(buf, ustring(aStr));
    delete_ustring(aStr);
    return buf;
}
