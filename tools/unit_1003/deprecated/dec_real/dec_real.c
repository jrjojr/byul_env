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

// 정수의 자리수를 확인한다.
// 범위는 1 ~ REAL_SIG_DIGITS
const char dec_digits_count(DBL_DEC tDec);

// 일반적인 범위는 DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
// 하지만, 실수의 정수부와 소수부에 약간의 조정을 거쳐서 범위 안에 들수 있다.
const bool is_exp_in_limit(const char tExp, 
    const DEC tDecIntPart, const DEC tDecFracPart);

// 소수부를 연산하기 위해서는 정규화를 해야한다.
// 123의 형태를 123000000의 형태로 변환해야 한다.
// 현재 자리수를 REAL_SIG_DIGITS에서 뺄셈을 하고 나머지 만큼 시프트를 한다.
int normalize_frac_part(const DEC tFracPart, const char tZeroCount, 
    DEC* retFracPart);

// 정규화된 소수부를 원래의 형태로 변환한다.
int denormalize_frac_part(const DEC tFracPart, 
    DEC* retFracPart, char* retZeroCount);

DEC dec_shift_right(DEC tDec, DEC tShiftCount, DEC* aRetPoppedDec);    
DEC dec_shift_left(DEC tDec, DEC tShiftCount, DEC* aRetPoppedDec);    

bool is_dec_in_limit(DEC tDec);

DEC shift_dec_dbl(DBL_DEC tDecDbl, char tZeroCount);

// 십진수의 정수부와 소수부를 합친 고정 소수점을 
// 점을 기준으로 숫자들을 오른쪽으로 이동한다.
// 123.456 -> 0.123456
int shift_right_num(DEC tIntPart, DEC tFracPart, DEC tShiftCount,
    DEC* tRetIntPart, DEC* tRetFracPart);

// 십진수의 정수부와 소수부를 합친 고정 소수점을 
// 점을 기준으로 숫자들을 왼쪽으로 이동한다.
// 0.123456 -> 123.456
int shift_left_num(DEC tIntPart, DEC tFracPart, DEC tShiftCount,
    DEC* tRetIntPart, DEC* tRetFracPart);


LIBAPI void dec_real_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "dec_real", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* dec_real_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_dec_real(DecReal* tSelf){
    tSelf->mIntPart = 0;
    tSelf->mFracPart = 0;
    tSelf->mExp = 0;
    tSelf->mPolicy = REAL_TRUNC_POLICY_ROUND;
    return 0;
}

LIBAPI int release_dec_real(DecReal* tSelf){
    // 메모리가 할당되었으면, 해제한다.
    return 0;
}

LIBAPI int set_dec_real(DecReal* tSelf, 
    DEC tIntPart, DEC tFracPart, char tExp, RealTruncPolicy tPolicy){

    tSelf->mIntPart = tIntPart;
    tSelf->mFracPart = tFracPart;
    tSelf->mExp = tExp;
    tSelf->mPolicy = tPolicy;
    return 0;
}

LIBAPI int set_dec_real_int_part(DecReal* tSelf, DEC tDec){
    tSelf->mIntPart = tDec;
    return 0;
}

LIBAPI int set_dec_real_frac_part(DecReal* tSelf, DEC tDec){
    tSelf->mFracPart = tDec;
    return 0;
}

// dec_real_exp 범위는 DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
LIBAPI int set_dec_real_exp(DecReal* tSelf, char tChar){
    if (tChar > DEC_REAL_EXP_MAX){
        tChar = DEC_REAL_EXP_MAX;
    }
    if (tChar < DEC_REAL_EXP_MIN){
        tChar = DEC_REAL_EXP_MIN;
    }
    tSelf->mExp = tChar;
    return 0;
}

LIBAPI int set_dec_real_trunc_policy(DecReal* tSelf, 
    RealTruncPolicy tPolicy){
        tSelf->mPolicy = tPolicy;
        return 0;
    }

LIBAPI int get_dec_real(const DecReal* tSelf, 
    DEC* retIntPart, DEC* retFracPart, char* retExp, 
    RealTruncPolicy* retPolicy){
    
    *retIntPart = tSelf->mIntPart;
    *retFracPart = tSelf->mFracPart;
    *retExp = tSelf->mExp;
    *retPolicy = tSelf->mPolicy;
    return 0;
    }

LIBAPI int get_dec_real_int_part(const DecReal* tSelf, DEC* retDec){
    
    *retDec = tSelf->mIntPart;
    return 0;
}

LIBAPI int get_dec_real_frac_part(const DecReal* tSelf, DEC* retDec){
    
    *retDec = tSelf->mFracPart;
    return 0;
}

LIBAPI int get_dec_real_exp(const DecReal* tSelf, char* retChar){
    *retChar = tSelf->mExp;
    return 0;    
}

LIBAPI int get_dec_real_trunc_policy(DecReal* tSelf, 
    RealTruncPolicy* retPolicy){
        *retPolicy = tSelf->mPolicy;
        return 0;
    }

LIBAPI const DEC dec_real_int_part(const DecReal* tSelf){
    return tSelf->mIntPart;
}

LIBAPI const DEC dec_real_frac_part(const DecReal* tSelf){
    return tSelf->mIntPart;
}

const char dec_real_exp(const DecReal* tSelf){
    return tSelf->mExp;
}

const RealTruncPolicy LIBAPI dec_real_trunc_policy(DecReal* tSelf){
    return tSelf->mPolicy;
}

LIBAPI const bool dec_real_is_zero(const DecReal* tSelf){
    return tSelf->mIntPart==0 && tSelf->mExp==0;
}

LIBAPI int assign_dec_real(DecReal* tSelf, const DecReal* tOther){
    tSelf->mIntPart = tOther->mIntPart;
    tSelf->mFracPart = tOther->mFracPart;
    tSelf->mExp = tOther->mExp;
    return 0;
}

LIBAPI int add_dec_real(DecReal* tSelf, const DecReal* tOther){
    DecReal aSelf;
    DecReal aOther;
    DecReal aResult;
    int aResultFixDecRealExp;    

    DEC aSelfFixedExpIntPart;
    DEC aSelfFixedExpFracPart;

    DEC aOtherFixedExpIntPart;
    DEC aOtherFixedExpFracPart;    

    init_dec_real(&aResult);

    if(tSelf->mExp == tOther->mExp){
        assign_dec_real(&aSelf, tSelf);
        assign_dec_real(&aOther, tOther);
    }
    else {
        // 작은쪽을 큰쪽의 지수에 고정시킨다.
        // 작은쪽이 더 작아진다는 의미이다.
        // 가장 작아지면 0에 가까워진다.
        // 가장 커지면 무한대가 된다.
        if (tSelf->mExp < tOther->mExp){
            aResultFixDecRealExp = fix_dec_real_exp(tSelf, 
                tOther->mExp, 
                &aSelfFixedExpIntPart, &aSelfFixedExpFracPart);

            if (aResultFixDecRealExp == DEC_REAL_UNDERFLOW){
                set_dec_real(&aSelf, 0, 0, 0, REAL_TRUNC_POLICY_ROUND);
            }
            else if(aResultFixDecRealExp == DEC_REAL_OVERFLOW){
                return REAL_OVERFLOW;
            }
            else{
                set_dec_real(&aSelf, 
                    aSelfFixedExpIntPart, aSelfFixedExpFracPart, tOther->mExp,
                        REAL_TRUNC_POLICY_ROUND);
            }
            assign_dec_real(&aOther, tOther);
        }

        else{
            aResultFixDecRealExp = fix_dec_real_exp(tOther, 
                tSelf->mExp, &aOtherFixedExpIntPart, &aOtherFixedExpFracPart);

            if (aResultFixDecRealExp == DEC_REAL_UNDERFLOW){
                set_dec_real(&aOther, 0, 0, 0, REAL_TRUNC_POLICY_ROUND);
            }
            else if(aResultFixDecRealExp == DEC_REAL_OVERFLOW){
                return REAL_OVERFLOW;
            }
            else{
                set_dec_real(&aOther, 
                    aOtherFixedExpIntPart, aOtherFixedExpFracPart, tSelf->mExp,
                        REAL_TRUNC_POLICY_ROUND);
            }
            assign_dec_real(&aSelf, tSelf);
        }
    }
    aResult.mFracPart = aSelf.mFracPart + aOther.mFracPart;
    if (aResult.mFracPart > REAL_MAX){
        aResult.mIntPart += 1;
    }
    aResult.mIntPart =  aSelf.mIntPart + aOther.mIntPart;
    // 더한 결과가 REAL_MAX를 넘어가면 지수부에 1을 더한다.
    if (aResult.mIntPart > REAL_MAX){
        aResult.mExp += 1;
        if ( aResult.mExp > DEC_REAL_EXP_MAX){
            // 만약 지수부가 DEC_REAL_EXP_MAX를 넘어가면
            return REAL_OVERFLOW;
        }

        // 정수부에 최고 자리수의 REAL_MAX+1을 뺀 나머지를 할당한다.
        aResult.mIntPart -= REAL_MAX+1;
    }
    assign_dec_real(tSelf, &aResult);
    return SUCCESS;
}

LIBAPI int sub_dec_real(DecReal* tSelf, const DecReal* tOther){
    DecReal aSelf;
    DecReal aOther;
    int aResultFixDecRealExp;    

    DEC aSelfFixedExpIntPart;
    DEC aSelfFixedExpFracPart;

    DEC aOtherFixedExpIntPart;
    DEC aOtherFixedExpFracPart;    

    if(tSelf->mExp == tOther->mExp){
        assign_dec_real(&aSelf, tSelf);
        assign_dec_real(&aOther, tOther);
    }
    else {
        // 작은쪽을 큰쪽의 지수에 고정시킨다.
        // 작은쪽이 더 작아진다는 의미이다.
        // 가장 작아지면 0에 가까워진다.
        // 가장 커지면 무한대가 된다.
        if (tSelf->mExp < tOther->mExp){
            aResultFixDecRealExp = fix_dec_real_exp(tSelf, 
                tOther->mExp, 
                &aSelfFixedExpIntPart, &aSelfFixedExpFracPart);

            if (aResultFixDecRealExp == DEC_REAL_UNDERFLOW){
                set_dec_real(&aSelf, 0, 0, 0, REAL_TRUNC_POLICY_ROUND);
            }
            else if(aResultFixDecRealExp == DEC_REAL_OVERFLOW){
                return REAL_OVERFLOW;
            }            
            else{
                set_dec_real(&aSelf, 
                    aSelfFixedExpIntPart, aSelfFixedExpFracPart, tOther->mExp,
                        REAL_TRUNC_POLICY_ROUND);
            }
            assign_dec_real(&aOther, tOther);
        }

        else{
            aResultFixDecRealExp = fix_dec_real_exp(tOther, 
                tSelf->mExp, &aOtherFixedExpIntPart, &aOtherFixedExpFracPart);

            if (aResultFixDecRealExp == DEC_REAL_UNDERFLOW){
                set_dec_real(&aOther, 0, 0, 0, REAL_TRUNC_POLICY_ROUND);
            }
            else if(aResultFixDecRealExp == DEC_REAL_OVERFLOW){
                return REAL_OVERFLOW;
            }            
            else{
                set_dec_real(&aOther, 
                    aOtherFixedExpIntPart, aOtherFixedExpFracPart, tSelf->mExp, 
                        REAL_TRUNC_POLICY_ROUND);
            }
            assign_dec_real(&aSelf, tSelf);
        }
    }

    aSelf.mFracPart -= aOther.mFracPart;
    if(aSelf.mFracPart < 0){
        aSelf.mIntPart -= 1;
        aSelf.mFracPart += REAL_MAX+1;
    }
    aSelf.mIntPart -= aOther.mIntPart;
    // 최소값보다 작으면, 
    if (aSelf.mIntPart < DEC_REAL_MIN){
        // 지수부에 1을 뺄셈한다. 
        aSelf.mExp -= 1;
        if (aSelf.mExp < DEC_REAL_EXP_MIN){
            return REAL_UNDERFLOW;
        }

        // 정수부에 최소값+1을 더하고 결과를 할당한다.
        aSelf.mIntPart += DEC_REAL_MIN+1;
    }

    assign_dec_real(tSelf, &aSelf);
    return SUCCESS;
}
LIBAPI int mul_dec_real(DecReal* tSelf, const DecReal* tOther){
    DecReal aSelf;
    DecReal aOther;
    int aResultFixDecRealExp;    

    DEC aSelfFixedExpIntPart;
    DEC aSelfFixedExpFracPart;

    DEC aOtherFixedExpIntPart;
    DEC aOtherFixedExpFracPart;    

    DBL_DEC aMultResult;

    assign_dec_real(&aSelf, tSelf);
    assign_dec_real(&aOther, tOther);

    // 곱셈을 위해서는 DBL_DEC 타입을 사용해야 한다.
    // 지수부는 지수부끼리 덧셈을 한다.
    aSelf.mExp += aOther.mExp;
    if (aSelf.mExp > DEC_REAL_EXP_MAX){
        return REAL_OVERFLOW;
    }

    // 정수부를 곱셈한다.
    aMultResult = aSelf.mIntPart * aOther.mIntPart;
    if (aMultResult > REAL_MAX){
        // 결과와 맥스의 차이만큼 시프트한다.
        DEC aDiffExp = dec_digits_count(aMultResult) - REAL_SIG_DIGITS;
        aSelf.mExp += aDiffExp;
        if (aSelf.mExp > DEC_REAL_EXP_MAX){
            return REAL_OVERFLOW;
        }
        DEC aResult = aMultResult/(DEC)pow(10, aDiffExp);
        aSelf.mIntPart = aResult;
    }
    else{
        aSelf.mIntPart = (DEC)aMultResult;
    }

    // 소수부를 곱셈한다.
    aMultResult = aSelf.mFracPart * aSelf.mFracPart;
    if (aMultResult > REAL_MAX){
        // 결과와 맥스의 차이만큼 시프트한다.
        DEC aDiffExp = dec_digits_count(aMultResult) - REAL_SIG_DIGITS;
        DEC aResult = aMultResult/(DEC)pow(10, aDiffExp);
        aSelf.mFracPart = aResult;        
    }
    else{
        aSelf.mFracPart = (DEC)aMultResult;
    }
    return SUCCESS;
}

LIBAPI int div_dec_real(DecReal* tSelf, const DecReal* tOther){
    DecReal aSelf;
    DecReal aOther;

    DEC aDivRemainderResult;

    int aResultFixDecRealExp;    

    DEC aSelfFixedExpIntPart;
    DEC aSelfFixedExpFracPart;

    DEC aOtherFixedExpIntPart;
    DEC aOtherFixedExpFracPart;    

    assign_dec_real(&aSelf, tSelf);
    assign_dec_real(&aOther, tOther);

    // 나눗셈한다.
    // 지수부를 뺄셈한다.
    aSelf.mExp -= aOther.mExp;
    if(aSelf.mExp < DEC_REAL_EXP_MIN){
        return REAL_UNDERFLOW;
    }

    DEC retIntPart;
    DEC retFracPart;
    // 전체 정수를 나눗셈한다.
    div_dec_to(tSelf->mIntPart, tSelf->mIntPart, &retIntPart, &retFracPart);

    aSelf.mIntPart = retIntPart;
    aSelf.mFracPart = retFracPart;

    return SUCCESS;
}

// 고정된 지수부만큼 실수부의 값을 얻어낸다.
// 현재 지수부의 범위와 새로 고정 시킬 지수부의 범위를 확인 후에,
// 연산이 불가능하면, 실패를 반환한다.
// tDecExpFix의 범위는 
// DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
// 범위가 DEC_REAL_EXP_MIN 보다 작으면 DEC_REAL_UNDERFLOW 반환
// 성공하면 SUCCESS 반환
// 범위가 DEC_REAL_EXP_MAX 보다 크면 DEC_REAL_EXP_FIX_RESULT_NEAREST_INF 반환
LIBAPI int fix_dec_real_exp(const DecReal* tSelf, DEC tDecExpFix, 
    DEC* retIntPart, DEC* retFracPart){
    
    DecReal aSelf;
    DEC aDiffExp;
    DEC aTotalDigitsCount;
    DEC aPoppedIntPart;
    DEC aPoppedFracPart;
    DEC aOrgFracPart;
    DEC aSelfFracPart;
    DEC count=0;
    DEC aPowDiffExp;
    DEC aRemainIntDigitsCount;

    assign_dec_real(&aSelf, tSelf);
    // 고정할 지수와 현재 지수의 차이를 확인한다.
    aDiffExp = tDecExpFix - aSelf.mExp;
    aTotalDigitsCount = dec_digits_count(aSelf.mIntPart) + 
        dec_digits_count(aSelf.mFracPart);    

    // 지수 차이가 +이면 오른쪽으로 시프트한다.
    // 123.456 -> 0.123456 이 된다는 의미이다.
    if (aDiffExp >= 0){
        if (aDiffExp > aTotalDigitsCount){
            return DEC_REAL_UNDERFLOW;
        }        
        aPowDiffExp = (DEC)pow(10,aDiffExp);
        *retIntPart = aSelf.mIntPart/aPowDiffExp;
        aPoppedIntPart = 
            aSelf.mIntPart - (*retIntPart)*aPowDiffExp;
        
        *retFracPart = aSelf.mFracPart/aPowDiffExp;
        // TRUNC_POLICY에 따라 소수자리의 1번자리에 
        // 반올림, 올림, 내림결정해서 덧셈한다.
        switch(tSelf->mPolicy){
            case REAL_TRUNC_POLICY_ROUND:
            aPoppedFracPart = aSelf.mFracPart - (*retFracPart)*aPowDiffExp;
            aPoppedFracPart /= (DEC)pow(10,aDiffExp-1);
            if (aPoppedFracPart >= 5){
                *retFracPart += 1;
            }
            break;
            case REAL_TRUNC_POLICY_CEILING:
            *retFracPart += 1;
            break;
            case REAL_TRUNC_POLICY_FLOOR:
            // *retFracPart += 0;
            break;
        }
        *retFracPart += (aPoppedIntPart*aPowDiffExp);
        return SUCCESS;
    }

    // 지수 차이가 -이면 왼쪽으로 시프트한다.
    // 123.456 -> 123456.0 이 된다는 의미이다.
    // 정수부의 남은 자리수를 확인한다.
    // 남은자리수 = REAL_SIG_DIGITS - 정수자리수
    // 남은자리수보다 지수 차이가 크면 오버플로우이다.
    aRemainIntDigitsCount = 
        REAL_SIG_DIGITS - dec_digits_count(aSelf.mIntPart);

    aDiffExp = abs(aDiffExp);
    aPowDiffExp = (DEC)pow(10,aDiffExp);
    if (aDiffExp > aRemainIntDigitsCount){
        return DEC_REAL_OVERFLOW;
    }    
    aPoppedFracPart = (aSelf.mFracPart/aPowDiffExp) * aPowDiffExp;

    aSelfFracPart = (aSelf.mFracPart - aPoppedFracPart) * aPowDiffExp;
    *retFracPart = aSelfFracPart;

    *retIntPart *= aPowDiffExp;
    *retIntPart += (aPoppedFracPart*aPowDiffExp);

    return SUCCESS;
}

LIBAPI int convert_dec_real_to(const DecReal* tSelf, REAL* retReal){
    Real aReal;
    bool aSign;
    unsigned long aMantissa;
    long aExp;

    if (tSelf->mIntPart < 0){
        aSign = true;
    }
    else{
        aSign = false;
    }

    aExp = (long)log2(tSelf->mExp);
    aMantissa = dec_real_dec_part(tSelf);

    set_real_sign(&aReal, aSign);
    set_real_mantissa(&aReal, aMantissa);
    set_real_exponent(&aReal, aExp);

    *retReal = real(&aReal);
    return 0;
}

LIBAPI int convert_dec_real_from(DecReal* tSelf, const REAL tReal){
    Real aReal;
    bool aSign;
    DEC aExp;
    unsigned long aMantissa;
    set_real(&aReal, tReal);
    aSign = real_sign(&aReal);
    aExp = real_exponent(&aReal);
    aMantissa = real_mantissa(&aReal);

    set_dec_real_exp(tSelf, aExp*log10(2));
    if (aSign == true){
        set_dec_real_int_part(tSelf, -aMantissa);
    }
    else{
        set_dec_real_int_part(tSelf, aMantissa);
    }
    set_dec_real_frac_part(tSelf, 0);
    set_dec_real_trunc_policy(tSelf, REAL_TRUNC_POLICY_ROUND);
    return 0;

}

LIBAPI const REAL dec_real_to(const DecReal* tSelf){
    REAL aReal;
    convert_dec_real_to(tSelf,&aReal);
    return aReal;
}

LIBAPI const DecReal dec_real_from(const REAL tReal){
    DecReal aDecReal;
    Real aReal;
    bool aSign;
    unsigned long aMant;
    DEC aExp;
    init_real(&aReal);
    set_real(&aReal, tReal);
    aSign = real_sign(&aReal);
    aExp = real_exponent(&aReal);
    aMant = real_mantissa(&aReal);

    init_dec_real(&aDecReal);
    if (aSign == true){
        set_dec_real_int_part(&aDecReal, -aMant);
    }
    else{
        set_dec_real_int_part(&aDecReal, aMant);
    }

    set_dec_real_exp(&aDecReal, log10(2)*aExp);

    return aDecReal;

}

const bool is_exp_in_limit(const char tExp, 
    const DEC tDecIntPart, const DEC tDecFracPart){
    // 일반적인 범위는 DEC_REAL_EXP_MIN ~ DEC_REAL_EXP_MAX
    char aRemainDigitsCount;
    char aCurrentDigitsCount;
    if (tExp == 0){
        return true;
    }
    else if(tExp > DEC_REAL_EXP_MAX){
        // 현재 지수는 지수 범위의 최대값을 넘었다.
        // 정수부 파트의 자리수를 올려서, 최대값 이내로 변환 가능하다.
        // 현재 정수부의 자리수를 확인한다.
        // 현재 자리수는 최소 1이다.
        // 현재 정수부의 자리수에서 남은 자리수를 확인한다.
        // 남은 자리수 = REAL_SIG_DIGITS - 현재 자리수
        aCurrentDigitsCount = dec_digits_count(tDecIntPart);
        aRemainDigitsCount = REAL_SIG_DIGITS - aCurrentDigitsCount;
        // if (aExp - 남은 자리수 > REAL_SIG_DIGITS)
        if (tExp - aRemainDigitsCount > DEC_REAL_EXP_MAX){
            // 범위 바깥이다. 거짓을 반환한다.
            return false;
        }
    }
    else if(tExp < DEC_REAL_EXP_MIN){
        // 현재 지수는 지수 범위의 최소값보다 작다.
        // 소수부 파트의 자리수를 내려서, 최소값이내로 변환 가능한다.
        // 현재 소수부의 자리수를 확인한다.
        // 소수부는 정수부와 반대로 REAL_SIG_DIGITS이어야 남은 자리수가 많은거다.
        // 현재 소수부의 자리수를 확인한다.
        // 현재 자리수는 최소 1이다.
        // 남은 자리수 = 현재 자리수
        aRemainDigitsCount = dec_digits_count(tDecFracPart);
        // 남은 자리수의 최소값은 1이다. 
        // if (aExp + 남은 자리수 < DEC_DIGITS_MIN)
        if (tExp + aRemainDigitsCount < DEC_REAL_EXP_MIN){
            // 범위 바깥이다. 거짓을 반환한다.
            return false;
        }
    }
    return true;
}

const char dec_digits_count(DBL_DEC tDec){
    int count = 0;
    do {
        tDec /= 10;
        ++count;
    } while (tDec != 0);
    return count;
}

// LIBAPI int init_dec_frac(DecFrac* tSelf){
//     tSelf->mFrac = 0;
//     tSelf->mZeroCount = 0;
//     return 0;
// }

// LIBAPI int release_dec_frac(DecFrac* tSelf){
//     // 메모리 할당했으면 해제한다.
//     return 0;
// }

// // tFrac 범위 0 ~ REAL_MAX
// // tZeroCount 범위 -REAL_SIG_DIGITS-1 ~ REAL_SIG_DIGITS-1
// LIBAPI int set_dec_frac(DecFrac* tSelf, DEC tFrac, char tZeroCount){
//     return set_dec_frac_frac(tSelf, tFrac) &&
//         set_dec_frac_zero_count(tSelf, tZeroCount);
// }

// // tFrac 범위 0 ~ REAL_MAX
// LIBAPI int set_dec_frac_frac(DecFrac* tSelf, DEC tFrac){
//     if (tFrac < 0){
//         tFrac = 0;
//     }
//     else if ( tFrac > REAL_MAX){
//         tFrac = REAL_MAX;
//     }
//     tSelf->mFrac = tFrac;
//     return 0;
// }

// // tZeroCount 범위 -REAL_SIG_DIGITS-1 ~ REAL_SIG_DIGITS-1
// LIBAPI int set_dec_frac_zero_count(DecFrac* tSelf, char tZeroCount){
//     if( tZeroCount < -REAL_SIG_DIGITS-1){
//         tZeroCount = -REAL_SIG_DIGITS-1;
//     }
//     else if(tZeroCount > REAL_SIG_DIGITS-1){
//         tZeroCount = REAL_SIG_DIGITS-1;
//     }
//     tSelf->mZeroCount = tZeroCount;
//     return 0;
// }

// LIBAPI int get_dec_frac(DecFrac* tSelf, DEC* retFrac, char* retZeroCount){
//     *retFrac = tSelf->mFrac;
//     *retZeroCount = tSelf->mZeroCount;
//     return 0;
// }

// LIBAPI int get_dec_frac_frac(DecFrac* tSelf, DEC* retFrac){
//     *retFrac = tSelf->mFrac;
//     return 0;
// }

// LIBAPI int get_dec_frac_zero_count(DecFrac* tSelf, char* retZeroCount){
//     *retZeroCount = tSelf->mZeroCount;
//     return 0;    
// }

// LIBAPI int assign_dec_frac(DecFrac* tSelf, const DecFrac* tOther){
//     tSelf->mFrac = tOther->mFrac;
//     tSelf->mZeroCount = tOther->mZeroCount;
//     return 0;
// }

// LIBAPI int add_dec_frac(DecFrac* tSelf, const DecFrac* tOther){
//     // 정규화를 한다.
//     // frac과 zeroCount를 합해야 한다.
//     DEC aSelf = normalize_dec_frac(tSelf);
//     DEC aOther = normalize_dec_frac(tOther);
//     char aZeroCount;
//     aSelf += aOther;
//     aZeroCount = REAL_SIG_DIGITS - dec_digits_count(aSelf);
//     set_dec_frac(tSelf, aSelf, aZeroCount);

//     return 0;
// }

// LIBAPI int sub_dec_frac(DecFrac* tSelf, const DecFrac* tOther){
//     // 정규화를 한다.
//     // frac과 zeroCount를 합해야 한다.
//     DEC aSelf = normalize_dec_frac(tSelf);
//     DEC aOther = normalize_dec_frac(tOther);
//     char aZeroCount;
//     aSelf -= aOther;
//     aZeroCount = REAL_SIG_DIGITS - dec_digits_count(aSelf);
//     set_dec_frac(tSelf, aSelf, aZeroCount);

//     return 0;
// }

// LIBAPI int mul_dec_frac(DecFrac* tSelf, const DecFrac* tOther){
//     // 정규화를 한다.
//     // frac과 zeroCount를 합해야 한다.
//     DEC aSelf = normalize_dec_frac(tSelf);
//     DEC aOther = normalize_dec_frac(tOther);
//     DBL_DEC aResult;
//     char aZeroCount;
//     aResult = aSelf * aOther;
//     aResult = shift_dec_frac(aResult,
//         dec_digits_count(aResult)-REAL_SIG_DIGITS);
        
//     aSelf = aResult;
//     aZeroCount = dec_digits_count(aSelf) + dec_digits_count(aOther);

//     set_dec_frac(tSelf, aSelf, aZeroCount);

//     return 0;    
// }

// LIBAPI int div_dec_to(DecFrac* tSelf, const DecFrac* tOther){
//     // 정규화를 한다.
//     // frac과 zeroCount를 합해야 한다.
//     DEC aSelf = normalize_dec_frac(tSelf);
//     DEC aOther = normalize_dec_frac(tOther);
//     DBL_DEC aResult;
//     char aZeroCount;
//     aResult = aSelf * 10/aOther;
//     aZeroCount = dec_digits_count(aSelf) - dec_digits_count(aOther);
//     set_dec_frac(tSelf, aResult, aZeroCount);

//     return 0;        lld
// }

// LIBAPI int pow_dec_frac(DecFrac* tSelf, const DecFrac* tOther);

LIBAPI int div_dec_to(DEC tSelf, const DEC tOther, 
    DEC* retIntPart, DEC* retFracPart){

    // DBL_DEC aFracPart = 0;
    DEC a = abs(tSelf);
    DEC b = abs(tOther);
    DEC q = 0;
    DEC r = 0;
    DEC aExp = 0;
    DEC count = 0;
    *retIntPart = 0;
    *retFracPart = 0;    

    if(a==b){
        if (tSelf* tOther < 0){
            *retIntPart = -1;
            *retFracPart = 0;            
        }
        else{
            *retIntPart = 1;
            *retFracPart = 0;
        }
        return SUCCESS;
    }

    // 루프 전에 정수값을 먼저 추출한다.
    if(a >= b){
        q = a / b;
        *retIntPart = q;
        r = a % b;
        if (r ==0){
            *retFracPart = 0;
            return SUCCESS;
        }
        a = r;
    }
    else{
        *retIntPart = 0;
        a *= 10;
        aExp++;
    }

    // 이제 정수부는 없고 소수부만 있다.
    // 루프를 돈다.
    while (count < REAL_SIG_DIGITS){
        if(a >= b){
            q = a / b;
            // aFracPart += q * (DBL_DEC)pow(10,REAL_SIG_DIGITS-aExp);
            *retFracPart += (q * (DEC)pow(10,REAL_SIG_DIGITS-aExp));
            r = a % b;
            if (r ==0){
                break;
            }            
            a = r;
            count++;            
        }
        else{
            a *= 10;
            aExp++;
        }
    }

    if (tSelf* tOther < 0){
        // *retFracPart = -aFracPart;
        *retFracPart = -*retFracPart;
        *retIntPart = -*retIntPart;
    }
    // else{
    //     *retFracPart = aFracPart;
    // }
    return SUCCESS;
}

DEC dec_shift_right(DEC tDec, DEC tShiftCount, DEC* aRetPoppedDec){
    // 정수를 시프트 시키려면 문자열로 변환하고,
    // 문자열을 다시 정수로 변환해서 값을 얻어야 한다.
    // 숫자들을 오른쪽으로 이동한다.
    // 1234567  -> (000)1234 -> 1234
    DEC aDec;
    char aStrDec[REAL_SIG_DIGITS+2] ="";
    char aStrPoppedDec[REAL_SIG_DIGITS+2] = "";
    char aStrRetDec[REAL_SIG_DIGITS+2] = "";
    sprintf(aStrDec, "%d", tDec);
    DEC aLenDigits;
    char* end;
    aLenDigits = strlen(aStrDec);

    if (aLenDigits > tShiftCount){
        strncpy_s(aStrRetDec, aLenDigits-tShiftCount+1, 
            aStrDec, aLenDigits-tShiftCount);
        strncpy_s(aStrPoppedDec, tShiftCount+1, aStrDec+tShiftCount, tShiftCount);
        aDec = strtol(aStrRetDec,NULL, 10);
        *aRetPoppedDec= strtol(aStrPoppedDec, NULL, 10);
    }
    else{
        aDec = 0;
        *aRetPoppedDec= tDec;
    }

    return aDec;
}

DEC dec_shift_left(DEC tDec, DEC tShiftCount, DEC* aRetPoppedDec){
    // 정수를 시프트 시키려면 문자열로 변환하고,
    // 문자열을 다시 정수로 변환해서 값을 얻어야 한다.
    // 숫자들을 왼쪽으로 이동한다.
    // 1234567 -> 4567(000) -> 4567
    DEC aDec;
    char aStrDec[REAL_SIG_DIGITS+2] ="";
    char aStrPoppedDec[REAL_SIG_DIGITS+2] = "";
    char aStrRetDec[REAL_SIG_DIGITS+2] = "";
    sprintf(aStrDec, "%d", tDec);
    DEC aLenDigits;
    char* end;
    aLenDigits = strlen(aStrDec);
    if (aLenDigits > tShiftCount){
        strncpy_s(aStrRetDec, aLenDigits-tShiftCount+1, 
            aStrDec+tShiftCount, aLenDigits-tShiftCount);
        strncpy_s(aStrPoppedDec, tShiftCount+1, 
            aStrDec, tShiftCount);
        aDec = strtol(aStrRetDec,NULL, 10) * pow(10, tShiftCount);
        *aRetPoppedDec= strtol(aStrPoppedDec, NULL, 10);
    }
    else{
        aDec = 0;
        *aRetPoppedDec= tDec;        
    }
    return aDec;
}

bool is_dec_in_limit(DEC tDec){
    // 정수의 자리수를 확인하려면 문자열로 변환하고,
    // 문자열의 길이를 확인한다.

    char aStrDec[REAL_SIG_DIGITS+2] ="";
    sprintf(aStrDec, "%d", tDec);
    DEC aLenDigits;
    aLenDigits = strlen(aStrDec);

    if (aLenDigits < REAL_SIG_DIGITS){
        return true;
    }
    return false;
}

int shift_right_num(DEC tIntPart, DEC tFracPart, DEC tShiftCount,
    DEC* tRetIntPart, DEC* tRetFracPart){
    // 실제로 구현하려니까 머리가 더 잘 돌아갔으면 좋겠다는 생각이 든다.

    // 십진수의 정수부와 소수부를 합친 고정 소수점을 
    // 점을 기준으로 숫자들을 오른쪽으로 이동한다.
    // 123.4567 -> 0.1234567

    // 시프트 카운트는 십진수의 한계 안에 위치해야 한다.
    if (!is_dec_in_limit(tShiftCount)){
        return -1;
    }

    // tIntPart의 자리수를 확인한다.
    DEC aIntPartDigitsCount = dec_digits_count(tIntPart);

    // tFracPart의 자리수를 확인한다.
    DEC aFracPartDigitCount = dec_digits_count(tFracPart);

    // tFracPart의 자리수와 시프트 카운트를 더했을때,
    // 십진수의 한계 안에 있는지 확인한다.
    if (!is_dec_in_limit(tShiftCount + aFracPartDigitCount)){
        return -2;
    }

    DEC aOrgFracPart = tFracPart;
    DEC aRetPoppedDec;
    DEC aRetFracPartPoppedDec;
    *tRetIntPart = dec_shift_right(tIntPart, tShiftCount, &aRetPoppedDec);
    *tRetFracPart = dec_shift_right(tFracPart, tShiftCount, 
        &aRetFracPartPoppedDec);
    *tRetFracPart = 
        aRetPoppedDec*pow(10,REAL_DIGITS-tShiftCount) + *tRetFracPart;
    return 0;
}

int shift_left_num(DEC tIntPart, DEC tFracPart, DEC tShiftCount,
    DEC* tRetIntPart, DEC* tRetFracPart){
    // 실제로 구현하려니까 머리가 더 잘 돌아갔으면 좋겠다는 생각이 든다.        

    // 십진수의 정수부와 소수부를 합친 고정 소수점을 
    // 점을 기준으로 숫자들을 왼쪽으로 이동한다.
    // 0.123456 -> 123.456
    // 시프트 카운트는 십진수의 한계 안에 위치해야 한다.
    if (!is_dec_in_limit(tShiftCount)){
        return -1;
    }

    // tIntPart의 자리수를 확인한다.
    DEC aIntPartDigitsCount = dec_digits_count(tIntPart);

    // tFracPart의 자리수를 확인한다.
    DEC aFracPartDigitsCount = dec_digits_count(tFracPart);

    // tIntPart의 자리수와 시프트 카운트를 더했을때,
    // 십진수의 한계 안에 있는지 확인한다.
    if (!is_dec_in_limit(tShiftCount + aIntPartDigitsCount)){
        return -2;
    }

    DEC aOrgFracPart = tFracPart;
    DEC aOrgIntPart = tIntPart;
    DEC aRetPoppedDec;
    DEC aRetFracPartPoppedDec;    
    // while (tShiftCount != 0){
    //     tIntPart = tIntPart * 10 + tFracPart>>(aFracPartDigitsCount-1);
       
    //     tFracPart = (aOrgFracPart<<1)/10;
    //     tShiftCount--;
    // }    
    tIntPart = dec_shift_left(tIntPart, tShiftCount, &aRetPoppedDec);
    tFracPart = dec_shift_left(tFracPart, tShiftCount, &aRetFracPartPoppedDec);
    *tRetIntPart += tIntPart*pow(10, tShiftCount) + aRetFracPartPoppedDec;
    *tRetFracPart = tFracPart;
    return 0;
}

// 소수부를 연산하기 위해서는 정규화를 해야한다.
// 123의 형태를 123000000의 형태로 변환해야 한다.
// 현재 자리수를 REAL_SIG_DIGITS에서 뺄셈을 하고 나머지 만큼 시프트를 한다.
int normalize_frac_part(const DEC tFracPart, const char tZeroCount, 
    DEC* retFracPart){
        return 0;
    }

// 정규화된 소수부를 원래의 형태로 변환한다.
int denormalize_frac_part(const DEC tFracPart, 
    DEC* retFracPart, char* retZeroCount){
        return 0;
    }

DEC shift_dec_dbl(DBL_DEC tDecDbl, char tZeroCount){
    return 0;
}    

DEC LIBAPI dec_real_dec_part(const DecReal* tSelf){
    DEC aDigitsCountIntPart;
    DEC aDigitsCountFracPart;
    DEC aResult;
    DEC aRemainDigitsCount;
    DEC aPowRemainDigitsCount;
    DEC aPoppedFracPart;

    aDigitsCountIntPart = dec_digits_count(tSelf->mIntPart);
    if( aDigitsCountIntPart == REAL_SIG_DIGITS){
        return tSelf->mIntPart;
    }

    if (tSelf->mIntPart==0 && tSelf->mFracPart==0){
        return 0;
    }

    aDigitsCountFracPart = dec_digits_count(tSelf->mFracPart);
    aRemainDigitsCount = REAL_SIG_DIGITS - aDigitsCountIntPart;

    aPowRemainDigitsCount = (DEC)pow(10, aRemainDigitsCount);
    aResult = tSelf->mIntPart * aPowRemainDigitsCount;
    aPoppedFracPart = (tSelf->mFracPart/aPowRemainDigitsCount);
    aResult += aPoppedFracPart;

    return aResult;
}