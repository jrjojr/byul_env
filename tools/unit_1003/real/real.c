/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* real
*
***************************************************************************/

#include "real.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

const Real Inf = {INFINITY};

// 십진수의 가수부를 확인한다.
const DBL_REAL real_dec(const Real* tSelf);

// 십진수의 지수부를 확인한다.
const DBL_REAL real_dec_exp(const Real* tSelf);

// MANTISSA를 십진수의 소수로 변환하는 함수
// 결과는 1.000 000 000 ~ 1.999 999 999
// 부동 소수점의 한계에 따라 최소 자리수는 약간의 차이가 있다.
// 1.999 999 999가 1.999 999 8## 이 될 수도 있다.
DBL_REAL convert_mantissa_to_real(MANTISSA tMant);

bool get_mantissa_bit(MANTISSA tMant, int tIndex);

// 십진수의 정수부와 소수부를 합친 고정 소수점을 
// 점을 기준으로 숫자들을 오른쪽으로 이동한다.
// 123.456 -> 0.123456
int shift_real_right(REAL tIntPart, REAL tFracPart, DEC tShiftCount,
    REAL* tRetIntPart, REAL* tRetFracPart);

// 십진수의 정수부와 소수부를 합친 고정 소수점을 
// 점을 기준으로 숫자들을 왼쪽으로 이동한다.
// 0.123456 -> 123.456
int shift_real_left(REAL tIntPart, REAL tFracPart, DEC tShiftCount,
    REAL* tRetIntPart, REAL* tRetFracPart);

bool is_real_dec_exp_in_limit(DEC tDecExp);
bool is_real_exp_in_limit(DEC tBinExp);

LIBAPI void real_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "real", 
        REAL_VERSION_MAJOR,
        REAL_VERSION_MINOR,
        REAL_VERSION_PATCH,
        REAL_VERSION_TWEAK);
}

LIBAPI const char* real_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        REAL_VERSION_MAJOR,
        REAL_VERSION_MINOR,
        REAL_VERSION_PATCH,
        REAL_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_real(Real* tReal){
    tReal->mReal = 0.0;
    return 0;
}

LIBAPI int release_real(Real* tReal){
    // 메모리 할당했으면, 해제한다.
    return 0;
}

LIBAPI int assign_real(Real* tReal, const Real* tOther){
    tReal->mReal = tOther->mReal;
    return 0;
}

LIBAPI int set_real(Real* tReal, REAL tValue){
    tReal->mReal = tValue;
    return 0;
}

LIBAPI int set_real_sign(Real* tReal, bool tSign){
    tReal->mReal_t.mSign = tSign;
    return 0;
}

// -127에서 128까지 입력할 수 있다.
// REAL_EXP_MIN 에서 REAL_EXP_MAX까지 입력할 수 있다.
// 나중의 확장을 위해 char가 아니라, 롱으로 설정함
LIBAPI int set_real_exponent(Real* tReal, DEC tExp){
    /*
    * 0     2^0     -REAL_EXP_BIAS
    * 1     2^1     -126
    * 2     2^2     -125
    * 3     2^3     
    * ...
    * 127   2^REAL_EXP_BIAS        0   
    * 128   2^-(REAL_EXP_BIAS+1)   1
    * 129   2^-2                        2
    * 130   2^-3                        3
    * ..
    * 255   2^-1    REAL_EXP_BIAS+1
    * 
    * 1.2^23 = 2^24 * 2^-127 인데,
    * 내가 원하는 것은 
    * 2^24 * 2^-(127+x)
    */
    tReal->mReal_t.mExponent = tExp + REAL_EXP_BIAS; 
    return 0;
}

LIBAPI int set_real_mantissa(Real* tReal, MANTISSA tMant){
    /*
    * 31  , 30...23,  22...      0,             비트 번호
    * 부호, 지수    , 101010110101,  .1
    * 부호, 지수    ,         가수,  가상의 1   
    *               2^-23 + 2^-22 + 2^-21 + ... + 2^-2 + 2^-1 + 2^-0
    * mantissa 000 ... 000(가수23비트)가 전부 이면, 1.0000이라는 뜻이다.
    * 1.은 항상 1.으로 시작하니 , 생략했다.
    *   2^-1 + 2^-2 + 2^-3 + ... + 2^-21 + 2^-22 + 2^-23
    * 1.0100101 ... 0101(가수23비트)가 진짜 소수점자리이다.
    * float 32비트 의 범위는 +-(1 + (0.0 ~ 0.8388608) ) * 
    *   ( 2^-REAL_EXP_BIAS ~ 2^(REAL_EXP_BIAS+1) ) 이다.
    */

    if (tMant > REAL_MANTISSA_DEC_MAX){
        tMant = REAL_MANTISSA_DEC_MAX;
    }
    else if(tMant > REAL_MANTISSA_DEC_MIN){
        tMant = REAL_MANTISSA_DEC_MIN;
    }
    tReal->mReal_t.mMantissa = tMant ;
    return 0; 
}

LIBAPI REAL real(const Real* tReal){
    //     Real aReal;
    //     assign_real(&aReal, tReal);

    // if (real_mantissa(tReal) > REAL_MANTISSA_DEC_MAX-8){
    //     set_real_mantissa(&aReal, 0);
    //     set_real_exponent(&aReal, real_exponent(&aReal)+1);
    //     return aReal.mReal;
    // }
    // else if (real_mantissa(tReal) < 8){
    //     set_real_mantissa(&aReal, 0);
    //     // set_real_exponent(&aReal, real_exponent(&aReal)+1);
    //     return aReal.mReal;
    // }
    return tReal->mReal;

    // return tReal->mReal + REAL_MANTISSA_TRUNC_OFFSET;
}

LIBAPI bool real_sign(const Real* tReal){
    if (tReal->mReal_t.mSign){
        return true;
    }
    return false;
}

long real_exponent(const Real* tReal){
    return tReal->mReal_t.mExponent - REAL_EXP_BIAS;
}

// MANTISSA real_mantissa(const Real* tReal){
//     return tReal->mReal_t.mMantissa;
// }

LIBAPI int add_real(Real* tReal, const Real* tOther){
    tReal->mReal += tOther->mReal;
    return 0;
}

LIBAPI int sub_real(Real* tReal, const Real* tOther){
    tReal->mReal -= tOther->mReal;
    return 0;
}

LIBAPI int mul_real(Real* tReal, const Real* tOther){
    tReal->mReal *= tOther->mReal;
    return 0;
}

LIBAPI int div_real(Real* tReal, const Real* tOther){
    if( tOther->mReal == 0.0){
        assign_real(tReal, &Inf);
        return FAIL;
    }
    tReal->mReal /= tOther->mReal;
    return 0;
}

LIBAPI int pow_real(Real* tReal, const Real* tOther){
    tReal->mReal = pow(tReal->mReal, tOther->mReal);
    return 0;
}

LIBAPI Real real_add(const Real* tReal, const Real* tOther){
    Real aResult;

    assign_real(&aResult, tReal);
    add_real(&aResult, tOther);

    return aResult;
}

LIBAPI Real real_sub(const Real* tReal, const Real* tOther){
    Real aResult;

    assign_real(&aResult, tReal);
    sub_real(&aResult, tOther);

    return aResult;
}

LIBAPI Real real_mul(const Real* tReal, const Real* tOther){
    Real aResult;

    assign_real(&aResult, tReal);
    mul_real(&aResult, tOther);

    return aResult;
}

LIBAPI Real real_div(const Real* tReal, const Real* tOther){
    Real aResult;

    assign_real(&aResult, tReal);
    div_real(&aResult, tOther);

    return aResult;
}

LIBAPI Real real_pow(const Real* tReal, const Real* tOther){
    Real aResult;

    assign_real(&aResult, tReal);
    pow_real(&aResult, tOther);

    return aResult;
}

LIBAPI int  fix_real_exp(const Real* tReal, DEC tDecExpFix, 
    REAL* tRetIntPart, REAL* tRetFracPart){
        // tDecExpFix의 범위를 확인한25 .
        // real의 정규화된 실수부는 1.##############이다.
        // #의 개수는 REAL_DIGITS이다.
        // tDecExpFix의 범위는
        // REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
        // 여기에 정규화된 실수부를 정수로 변환 했을때 필요한 거리
        // REAL_DIGITS이다 더해야 한다.
        // 그러면 실제 범위는 
        // REAL_EXP_DEC_MIN + REAL_DIGITS이다 ~ 
        //      REAL_EXP_DEC_MAX + REAL_DIGITS이다
        if (!is_real_dec_exp_in_limit(tDecExpFix)){
            DEBUG_PRINT("오류! : 설정한 지수 %d가 셀프의 지수 범위 안에 없다.\n",
                tDecExpFix);
            return -1;
        }

        // 현재의 real의 지수부를 확인한다.
        // REAL aDecExp = real_dec_exp(tReal);
        DEC aBinExp = real_exponent(tReal);

        // tDecExpFix와 현재 real의 지수부의 거리를 알아낸다.
        // tDecExpFix를 이진수의 지수로 변환한다.
        DBL_REAL aFixBinExp = tDecExpFix * log2(10);

        DBL_REAL aDistance = aFixBinExp - aBinExp;

        // 거리가 Real의 범위보다 크면 실패한다.
        // if (!is_real_dec_exp_in_limit(aDistance)){
        if (!is_real_exp_in_limit(aDistance)){
            DEBUG_PRINT(
                "오류! : 설정한 지수 %d와 셀프의 지수의 거리가 범위 안에 없다.\n",
                    tDecExpFix);
            return -2;
        }

        // 거리가 Real의 범위보다 작으면
        // 현재 real의 지수부와 거리를 더한다.
        aDistance = aDistance + aBinExp;
        // 더한 거리가 Real의 범위보다 크면 실패한다.
        // if (!is_real_dec_exp_in_limit(aDistance)){
        if (!is_real_exp_in_limit(aDistance)){
            DEBUG_PRINT(
                "오류! : 실제 이동할 지수 %lf가 셀프의 지수 범위 안에 없다.\n",
                    aDistance);
            return -3;
        }

        // 지수부의 거리만큼 현재 실수부의 위치를 조정한다.
        // 1.############이 0.00000000000000000000# 이거나 ##########.#이거나
        // 조정한 실수부를 정수 부분과 소수 부분에 돌려준다.
        DBL_REAL aRealIntPart;
        DBL_REAL aRealFracPart;
        REAL aRetRealIntPart;
        REAL aRetRealFracPart;        
        REAL aResult;
        DEC aDecDistance;
        DBL_REAL aRealDecExpDistance;

        aRealDecExpDistance = aDistance * log10(2);
        aDecDistance = round(aRealDecExpDistance);
        aRealFracPart = modf(real(tReal), &aRealIntPart);
        if (aDistance <= 0){
            aDecDistance = abs(aDecDistance);
            shift_real_left(aRealIntPart, aRealFracPart, aDecDistance,
                &aRetRealIntPart, &aRetRealFracPart);
        }
        else{
            shift_real_right(aRealIntPart, aRealFracPart, aDecDistance,
                &aRetRealIntPart, &aRetRealFracPart);
        }
        *tRetIntPart = aRetRealIntPart;
        *tRetFracPart = aRetRealFracPart;
        return 0;
    }

bool is_real_dec_exp_in_limit(DEC tDecExp){
    // tDecExpFix의 범위를 확인한다.
    // real의 정규화된 실수부는 1.##############이다.
    // #의 개수는 REAL_DIGITS이다.
    // tDecExpFix의 범위는
    // REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
    // 여기에 정규화된 실수부를 정수로 변환 했을때 필요한 거리
    // REAL_DIGITS이다 더해야 한다.
    // 그러면 실제 범위는 
    // REAL_EXP_DEC_MIN + REAL_DIGITS이다 ~ 
    //      REAL_EXP_DEC_MAX + REAL_DIGITS이다
    if (tDecExp > REAL_EXP_DEC_MAX + REAL_DIGITS){
        return false;
    }
    else if (tDecExp < REAL_EXP_DEC_MIN + REAL_DIGITS){
        return false;
    }
    return true;
}

// 십진수의 정수부와 소수부를 합친 소수점을 
// 점을 기준으로 숫자들을 오른쪽으로 이동한다.
// 123.456 -> 0.123456
int shift_real_right(REAL tIntPart, REAL tFracPart, DEC tShiftCount,
    REAL* tRetIntPart, REAL* tRetFracPart){
        REAL aIntPart;
        REAL aFracPart;
        REAL aPoppedFromIntPart;
        DBL_REAL aPowOffset;

        aPowOffset = pow(10, -tShiftCount);
        aIntPart = floor(tIntPart * aPowOffset);
        aPoppedFromIntPart = tIntPart - (aIntPart/aPowOffset);

        aFracPart = (aPoppedFromIntPart + tFracPart)*aPowOffset;

        *tRetIntPart = aIntPart;
        *tRetFracPart = aFracPart;

        return 0;
    }

// 십진수의 정수부와 소수부를 합친 소수점을 
// 점을 기준으로 숫자들을 왼쪽으로 이동한다.
// 0.123456 -> 123.456
int shift_real_left(REAL tIntPart, REAL tFracPart, DEC tShiftCount,
    REAL* tRetIntPart, REAL* tRetFracPart){
        REAL aIntPart;
        REAL aFracPart;
        DBL_REAL aPoppedFromFracPart;
        DBL_REAL aPowOffset;
        REAL aReal;

        aPowOffset = pow(10, tShiftCount);
        aFracPart = tFracPart * aPowOffset;
        aFracPart = modf(aFracPart, &aPoppedFromFracPart);
        aIntPart = tIntPart * aPowOffset + aPoppedFromFracPart;

        *tRetIntPart = aIntPart;
        *tRetFracPart = aFracPart;        
        return 0;
    }

bool is_real_exp_in_limit(DEC tBinExp){
    // tDecExpFix의 범위를 확인한다.
    // real의 정규화된 실수부는 1.##############이다.
    // #의 개수는 REAL_DIGITS이다.
    // tDecExpFix의 범위는
    // REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX
    // 여기에 정규화된 실수부를 정수로 변환 했을때 필요한 거리
    // REAL_DIGITS이다 더해야 한다.
    // 그러면 실제 범위는 
    // REAL_EXP_DEC_MIN + REAL_DIGITS이다 ~ 
    //      REAL_EXP_DEC_MAX + REAL_DIGITS이다
    // 이진수 지수의 범위는
    // REAL_EXP_MIN + REAL_EXP_OFFSET_FOR_REAL_DIGITS ~ 
    //      REAL_EXP_MAX + REAL_EXP_OFFSET_FOR_REAL_DIGITS 이다.
    if (tBinExp > REAL_EXP_MAX + REAL_EXP_OFFSET_FOR_REAL_DIGITS){
        return false;
    }
    else if (tBinExp < REAL_EXP_MIN + 
        REAL_EXP_OFFSET_FOR_REAL_DIGITS){
        return false;
    }
    return true;    
}    

DBL_REAL convert_mantissa_to_real(MANTISSA tMant){
    // 가수부를 십진수로 변환한다.
    // 형태는 0.###########이다.
    // 모든 연산이 완료된 후에 1.0을 더해서
    // 1.################의 형태로 변환한다.
    // 종료한다.
    // 1.aMant to 십진수
    // 1. 은 십진수도 1.0이다.
    // 1.00000000000000011
    // aMant 3 = 1.0000 0000 0000 0000 0000 011 이라는 의미이다. 
    // 11 * 2^-23 
    // 1/2  + 1/4   + 1/8   + 2^-4 + 2^-5 + ... 2^-24
    // 2^-1 + 2^-2  + 2^-3  + 2^-4 + 2^-5 + ... 2^-24

    // 만티사가 0이어도 기본적으로 1이 숨어있다.
    // 그래서 1 * 2^-1 이 기본값이다.
    // REAL aResult = 0.5;
    
    // for (int i=1; i < REAL_MANTISSA_BITS+1; i++){
    //     if(get_mantissa_bit(tMant,i)){
    //         aResult += pow(2, -(i+1));
    //     }
    // }
    // // return 1.0 + aResult;
    // return aResult;

    DBL_REAL aResult = 0.0;    
    for (int i=0; i < REAL_MANTISSA_BITS; i++){
        if(get_mantissa_bit(tMant,i)){
            aResult += pow(2, -(i+1));
        }
    }
    return 1.0 + aResult;
}

bool get_mantissa_bit(MANTISSA tMant, int tIndex){
    MANTISSA aMask = 0;
    return tMant & (1<<(22-tIndex));
}

LIBAPI bool real_equal(const Real* tSelf, const Real* tOther, const REAL tTol){

    REAL a,b;

    a = real(tSelf);
    b = real(tOther);
    return fabs(a-b)<=tTol;

    }

// 상대 공차
LIBAPI bool real_equal_rel(const Real* tSelf, const Real* tOther, 
    const REAL tTol){

    REAL a,b, m;

    a = fabs(real(tSelf));
    b = fabs(real(tOther));

    m = max(a, b);
    return fabs(a-b)<=(tTol*m);

    }

// ulp(Unit in the last place)공차 
LIBAPI bool real_equal_ulp(const Real* tSelf, const Real* tOther, 
    const DEC tTol){

    DEC a,b;
    a = real_mantissa(tSelf);
    b = real_mantissa(tOther);
    return abs(a-b)<=tTol;

    }

LIBAPI Real* create_real(const REAL tReal){
    Real* aReal = NULL;
    aReal = (Real*)malloc(sizeof(Real));
    if (aReal == NULL)
    {
        return NULL;
    }
    init_real(aReal);
    set_real(aReal, tReal);
    return aReal;
}

LIBAPI Real* create_real_default(){
    Real* aReal = NULL;
    aReal = (Real*)malloc(sizeof(Real));
    if (aReal == NULL)
    {
        return NULL;
    }
    init_real(aReal);
    return aReal;
}

LIBAPI int delete_real(Real* tReal){
    if ( tReal != NULL){
        free(tReal);
    }
    return 0;
}

LIBAPI int shift_real(const Real* tReal, DEC tDec, Real* retReal){
    DBL_REAL aPowOffset;
    DBL_REAL aBinPowOffset;

    aPowOffset = pow(10,tDec);
    aBinPowOffset = tDec*log2(10) + real_exponent(tReal);

    if (aBinPowOffset > REAL_EXP_MAX)
    {
        return REAL_OVERFLOW;
    }
    else if (aBinPowOffset < REAL_EXP_MIN){
        return REAL_UNDERFLOW;
    }

    set_real(retReal, aPowOffset * real(tReal));
    return SUCCESS;
}

LIBAPI int split_real(const Real* tReal, bool* retSign,
    DEC* retFracPart, DEC* retIntPart, DEC* retExpPart){

    if(real(tReal) == 0.0){
        *retSign = false;
        *retIntPart = 0;

        *retExpPart = 0;        

        return SUCCESS;
    }
    
    bool aSign = real_sign(tReal);

    // aDec은 1.###############의 형태로 정수부의 자리수가 1이다.
    DBL_REAL aDec = real_dec(tReal);
    DBL_REAL aDecExp = real_dec_exp(tReal);

    DBL_REAL aDecExpIntPart;
    // 지수에서 소수점만 떼어내면
    DBL_REAL aDecExpFracPart = modf(aDecExp, &aDecExpIntPart);
    // 지수는 정수형태로 바뀌고,
    // 실수는 정확한 유효 숫자를 알아 낼 수 있다.
    DBL_REAL aSigReal = aDec * pow(10,aDecExpFracPart);

        // 2024-06-24 13:51:04 
        // 위의 방법은 6자리의 유효숫자공차를 넘어서 차이가 생긴다.
        // 다섯자리 유효 숫자에서 차이가 생긴다.
        // 다른 방법을 찾아야 한다.
        // SigReal과 aDecExp를 더 정확하게 알아낼 방법이 필요하다.
        // 이진수로 연산해야 가장 정확하다.
        // 연산이 완료되면 그 후에 이진수를 십진수로 변환한다.
        // 2024-06-24 19:35:29
        // REAL을 DBL_REAL로 바꿨지만 여전히 오차가 발생한다.
        // 6자리가 같아야 하는데, 
        // 중간 중간 5자리만 같은게 나온다.
        // 2024-06-25 00:00:23
        // split가 문제가 아니라  merge가 문제인 거 같다.

      // DEC aBinExp = real_exponent(tReal);

    // aSigReal = aDec;

    // REAL orgValue = aSigReal * pow(2,aBinExp);
    /******************************************************************* */

    DBL_REAL aSigDecIntPart;
    DBL_REAL aSigDecFracPart = modf(aSigReal, &aSigDecIntPart);

    // 소수부의 소수점을 제거하고 정수형태로 변환 하려면, 
    // 총 자리수에서 정수부의 자리수를 뺄셈하고 
    // 남은 자리수를 pow한다.
    // 소수부의 자리수 = REAL_DIGITS - 정수부의 자리수
    // retFrac = 소수부 * 10^소수부의자리수

    UDEC aIntPartDigitsCount;
    aIntPartDigitsCount = dec_digits((UDEC)aSigDecIntPart);
    UDEC aSigDecFracPartDigitsCount = REAL_DIGITS - aIntPartDigitsCount;

    *retSign = aSign;
    *retIntPart = (UDEC)aSigDecIntPart;

    *retExpPart = (UDEC)aDecExpIntPart;

    // 0.001을 1로 만들려면 10^3을 해야한다.
    // 하지만 소수부에서 지수를 뺄셈해서 0.001을 1로 만들었으면
    // 정수부에도 3만큼 왼쪽으로 이동한다.
    // 지수부에서도 3을 뺄셈한다.
    // 1.001^3을 1000, 1, 10^3-3을 해야 제대로 된 값이다.
    //  1001 * 10^3-3 
    // 0.1이면 아무 문제가 없다.
    // 0.01부터 문제가 생긴다.
    // 0.09부터 0.000000009까지를 해결해야 한다.
    // -값은 생각하지 않아도 된다.
    // 0.000 ... 001 부터 0.09까지만 생각하면 딘다.
    if ( aSigDecFracPart >= 0.1){
        *retFracPart = 
            (UDEC)(aSigDecFracPart * pow(10, aSigDecFracPartDigitsCount));
    }
    else{
        // 현재 소수부가 0.1보다 작다.
        // 0.09 부터는 소수부를 정수로 만들기 위해 
        // 추가적으로 지수를 더 뺄셈해야 한다.
        // 0.09 를 정수로 만들려면 1을 지수에서 빼야 한다.
        // 지수에서 1을 뺀만큼 정수부도 이동해야 한다.
        // 지수에 얼마를 더할지는 루프를 돌려서 카운트해서 알아낸다.
        if ( aSigDecFracPart == 0.0){
           *retFracPart = 0;
        }
        else{
            int count = 0;
            for(int i=0; i < aSigDecFracPartDigitsCount; i++){
                if(aSigDecFracPart < 0.1){
                    aSigDecFracPart *= 10.0;
                    count++;
                }
            }
            *retFracPart = 
                (UDEC)(aSigDecFracPart * pow(10, 
                    aSigDecFracPartDigitsCount - count));
            *retIntPart *= (UDEC)pow(10, count);
            *retExpPart -= count;
        }
    }

    return SUCCESS;
    }

LIBAPI int merge_real(Real* tReal, const bool tSign,
    const DEC tFracPart, const DEC tIntPart, const DEC tExpPart){

// 정수 형태의 소수부, 정수부, 지수부를 합쳐서 실수를 만든다.
// 정수 형태의 소수부, 정수부, 지수부는 split_real을 통해 구할 수 있다.
// 소수부의 형태는 120.034 의 경우 034각 소수부가 아니라, 20034가 소수부가 된다.
// 1은 정수부이다.
// 소수부는 0이 아닌 숫자로 시작한다.
// 소수부와 정수부의 자리수는 실수의 자리수와 같다.
// REAL_DIGITS_MAX
// 만약, 소수부와 정수부의 자리수가 한계를 넘으면,
// 소수부의 최소 자리수가 잘린다.
// 지수부의 범위는 REAL_EXP_DEC_MIN ~ REAL_EXP_DEC_MAX

    // 우선 자르기가 필요없는 상황에 사용 할 코드를 작성한다.
    UDEC aIntPartDigits;
    UDEC aFracPartDigits;

    aIntPartDigits = dec_digits(tIntPart);
    aFracPartDigits = dec_digits(tFracPart);

    if (aIntPartDigits + aFracPartDigits > REAL_DIGITS){
        // 소수부의 유효 숫자에서 자르기를 해야 한다.
        // 그래야 부동소수점의 범위에 적당한 값이 들어간다.
        // 하지만 자르기는 부동소수점 연산 중에 자동 으로 진행 된다.
        // 나중에 추가해야 겠다.

    }
    
    // 위의 코드에서 자르기가 필요한 코드는 작성될 것이다.
    // 이제는 자르기가 없어도 되는 코드만 작성하면 된다.

        DBL_REAL aReal;
        DEC aExp;

        // 지수에서 소수부의 자리수를 빼는 이유는
        // 1.23456 이 123456이 되었다.
        // 이거 원래대로 하려면 소수부의 자리수 만큼 빼야한다.
        aExp = tExpPart - aFracPartDigits;

    // 먼저 정수의 형태로 정수부와 소수부를 합쳐야 겠다.
    UDEC aTotalSig;
    
        aTotalSig = tIntPart*pow(10, aFracPartDigits) + tFracPart;

        aReal = (DBL_REAL)(aTotalSig * pow(10, aExp));

        if (tSign){
            // 음수이면
            aReal *= -1;
        }

        // if (aReal > REAL_MAX){
        //     aReal = REAL_MAX;
        // }
        // else if(aReal < REAL_MIN){
        //     aReal = REAL_MIN;
        // }


        return set_real(tReal, aReal);        
}  

const DBL_REAL real_dec(const Real* tSelf){
    MANTISSA aMant;
    aMant = real_mantissa(tSelf);
    return convert_mantissa_to_real(aMant);
}

const DBL_REAL real_dec_exp(const Real* tSelf){
    return real_exponent(tSelf) * log10(2);
}

LIBAPI bool value_equal(const REAL tSelf, const REAL tOther, const REAL tTol){
    return fabs(tSelf-tOther)<=tTol;
}

LIBAPI const bool value_equal_rel(const REAL tSelf, const REAL tOther, 
    const REAL tTol){

    REAL m;
    bool r;

    m = max(fabs(tSelf), fabs(tOther));
    r = (fabs(tSelf-tOther)<=(tTol*m));
    return r;
    }
