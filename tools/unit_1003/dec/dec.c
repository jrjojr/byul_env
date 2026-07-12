/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec 
*
***************************************************************************/

#include "dec.h"

#include <stdio.h>

LIBAPI void print_dec_version(){
    printf("%s version : %d.%d.%d.%d\n", "dec", 
        DEC_VERSION_MAJOR,
        DEC_VERSION_MINOR,
        DEC_VERSION_PATCH,
        DEC_VERSION_TWEAK);
}

LIBAPI const char* dec_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        DEC_VERSION_MAJOR,
        DEC_VERSION_MINOR,
        DEC_VERSION_PATCH,
        DEC_VERSION_TWEAK
        );
    return buf;
}

LIBAPI const DEC dec_sig_digits(const DBL_DEC tDec){
    int count;
    DEC aRemainder;
    DEC aTotalCount;
    DBL_DEC aDec;

    // 총 자리수에서 0이 최소 자리에 연속으로 있으면
    // 10으로 나누면서 0을 제거한다. 제거하면서 카운트한다.
    // 나머지가 생기면 0이 다 제거되었다는 의미이다.
    // 총 자리수에서 카운트를 뺄셈하면 유효 자리수이다.
    aTotalCount = dec_digits(tDec);

    if(aTotalCount == 0){
        return 0;
    }
    else{
        count = 0;
        aDec = tDec;
        do{
            aRemainder = aDec%10;
            if(aRemainder != 0){
                break;
            }
            aDec /= 10;
            count++;
        }while(count < aTotalCount);
        
        aTotalCount -= count;
    }
    return aTotalCount;
}

LIBAPI const DEC dec_sig(const DBL_DEC tDec){
    int count;
    DEC aRemainder;
    DEC aTotalCount;
    DBL_DEC aDec;

    // 총 자리수에서 0이 최소 자리에 연속으로 있으면
    // 10으로 나누면서 0을 제거한다. 제거하면서 카운트한다.
    // 나머지가 생기면 0이 다 제거되었다는 의미이다.
    // 총 자리수에서 카운트를 뺄셈하면 유효 자리수이다.
    aTotalCount = dec_digits(tDec);

    if(aTotalCount == 0){
        return 0;
    }
    count = 0;
    aDec = tDec;
    do{
        aRemainder = aDec%10;
        if(aRemainder != 0){
            break;
        }
        aDec /= 10;
        count++;
    }while(count < aTotalCount);

    return aDec;
}

LIBAPI const DEC dec_digits(const DBL_DEC tDec){
    int count = 0;
    DBL_DEC aDec = tDec;

    if (aDec == 0){
        return 0;
    }    
    do {
        aDec /= 10;
        ++count;
    } while (aDec != 0);
    return count;
}

LIBAPI int dec_div_to(const DEC tSelf, const DEC tOther, 
    const DecMinTruncPolicy tPolicy,
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

    if(tSelf==0){
        *retIntPart = 0;
        *retFracPart = 0;
        return SUCCESS;            
    }

    if(tOther==0){
        *retIntPart = DEC_MAX;
        *retFracPart = DEC_MAX;
        return FAIL;
    }

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
            if (tSelf * tOther < 0){
                *retIntPart = 0 - (*retIntPart);
            }            
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
    while (count < DEC_DIGITS){
        if(a >= b){
            q = a / b;
            // aFracPart += q * (DBL_DEC)pow(10,REAL_SIG_DIGITS-aExp);
            *retFracPart += (q * (DEC)pow(10,DEC_DIGITS-aExp));
            r = a % b;
            if (r ==0){
                if (tSelf * tOther < 0){
                    // *retFracPart = -aFracPart;
                    *retFracPart = 0 - (*retFracPart);
                    *retIntPart = 0 - (*retIntPart);
                }
                return SUCCESS;
            }            
            a = r;
            count++;            
        }
        else{
            a *= 10;
            aExp++;
        }
    }

    // 나머지가 0이 안나온다. 순환소수라서 아무리 연산해도 0이 안나온다.
    // 자르기 정책에 따라 소수부의 마지막 자리수를 올리거나. 내린다.
    switch (tPolicy){
        case DEC_MIN_TRUNC_POLICY_ROUND:
        if (r >= 5){
            *retFracPart += 1;
        }
        break;
        case DEC_MIN_TRUNC_POLICY_CEILING:
        *retFracPart +=  1;

        break;
        case DEC_MIN_TRUNC_POLICY_FLOOR:
        // *retFracPart +=  0;
        break;
    }

    if (tSelf * tOther < 0){
        // *retFracPart = -aFracPart;
        *retFracPart = 0 - (*retFracPart);
        *retIntPart = 0 - (*retIntPart);
    }
    return SUCCESS;
}