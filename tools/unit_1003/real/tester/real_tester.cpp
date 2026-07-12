/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* real 1.0.2.2
*
***************************************************************************/

#define BOOST_TEST_MODULE real tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <iomanip>

#include "real.h"
#include "real_tester.hpp"

#include "math.h"

#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <vector>

namespace ut = boost::unit_test;
namespace bdata = boost::unit_test::data;
namespace tt = boost::test_tools;

BOOST_AUTO_TEST_SUITE(real_tester_suite,
    * ut::fixture<Stopwatch>(std::string("real_tester_suite")))

    Real aSelf;
    Real aOther;    

    Real aOrgSelf;

    bool aSign = true;
    float aFloat = 123.564;
        long aExp = 2;
    long aMant = 7;
    long aDec = 18;

BOOST_AUTO_TEST_CASE(init_real_tester){
    init_real(&aSelf);
    BOOST_TEST(real(&aSelf) == 0.0);
    BOOST_TEST(real_sign(&aSelf) == false);
    BOOST_TEST(real_mantissa(&aSelf) == 0);
    BOOST_TEST(real_exponent(&aSelf) == 0);    

    init_real(&aOther);
    set_real(&aOther, aFloat);
    BOOST_TEST(real(&aOther) == aFloat);
    BOOST_TEST(real_sign(&aSelf) == false);
    BOOST_TEST(real_mantissa(&aSelf) == 0);
    BOOST_TEST(real_exponent(&aSelf) == 0);    

    assign_real(&aSelf, &aOther);
    BOOST_TEST(real(&aSelf) == real(&aOther));
    BOOST_TEST(real_sign(&aSelf) == real_sign(&aOther));
    BOOST_TEST(real_mantissa(&aSelf) == real_mantissa(&aOther));
    BOOST_TEST(real_exponent(&aSelf) == real_exponent(&aOther));    

    set_real_sign(&aSelf, aSign);
    BOOST_TEST(real(&aSelf) == -aFloat);

    set_real_mantissa(&aSelf, aMant);
    BOOST_TEST(real_mantissa(&aSelf) == aMant);

    set_real_exponent(&aSelf, aExp);
    BOOST_TEST(real_exponent(&aSelf) == aExp);
}

BOOST_DATA_TEST_CASE(set_get_real_tester,
     bdata::xrange(-15, 15, 1),self){
    set_real(&aSelf, self);
    BOOST_TEST(real(&aSelf) == self*1.0);
    BOOST_TEST(real_sign(&aSelf) == false);
    BOOST_TEST(real_mantissa(&aSelf) == 0);
    BOOST_TEST(real_exponent(&aSelf) == 0);    

}

BOOST_DATA_TEST_CASE(op_real_tester,
    bdata::xrange(-15, 15, 1)*bdata::xrange(-15, 15, 1),
    self, other){
set_real(&aOther, other);

set_real(&aSelf, self);
    add_real(&aSelf, &aOther);
    BOOST_TEST(real(&aSelf) == self + other, tt::tolerance(0.000001));

set_real(&aSelf, self);
    sub_real(&aSelf, &aOther);
    BOOST_TEST(real(&aSelf) == self - other, tt::tolerance(0.000001));

set_real(&aSelf, self);
    mul_real(&aSelf, &aOther);
    BOOST_TEST(real(&aSelf) == self * other, tt::tolerance(0.000001));

set_real(&aSelf, self);
    div_real(&aSelf, &aOther);
    BOOST_TEST(real(&aSelf) == (REAL)self / other, tt::tolerance(0.000001));    

    set_real(&aSelf, self);
    pow_real(&aSelf, &aOther);    
    BOOST_TEST(real(&aSelf) == pow(self, other) , tt::tolerance(0.000001));
}
    
    Real aResult;
BOOST_DATA_TEST_CASE(ret_op_real_tester,
    bdata::xrange(-15, 15, 1)*bdata::xrange(-15, 15, 1),
    self, other){

set_real(&aOther, other);

set_real(&aSelf, self);

    aResult = real_add(&aSelf, &aOther);
    BOOST_TEST(real(&aResult) == self + other, tt::tolerance(0.000001));

    aResult = real_sub(&aSelf, &aOther);
    BOOST_TEST(real(&aResult) == self - other, tt::tolerance(0.000001));

    aResult = real_mul(&aSelf, &aOther);
    BOOST_TEST(real(&aResult) == self * other, tt::tolerance(0.000001));

    aResult = real_div(&aSelf, &aOther);
    BOOST_TEST(real(&aResult) == (REAL)self / other, tt::tolerance(0.000001));

    aResult = real_pow(&aSelf, &aOther);    
    BOOST_TEST(real(&aResult) == pow(self, other), tt::tolerance(0.000001) );

}

REAL aTol = 0.000001;
REAL aOffsetFactor = aTol/10;
int aRepeatCount = 10;
int kFactor = 15;
DEC aUlpTol =8;

REAL aOffset;

BOOST_AUTO_TEST_CASE(abs_tol_real_tester){
    
    // 실제 코드 시작

    // 실제 코드 시작
    // float aInf = 0xefffffff;
    // float aInf = (float)0xffffffff;
    // float aInf = FLT_MAX;
    // float aInf =INFINITY;

    REAL aOffset;

    Real* a=NULL;
    Real* b=NULL;

    bool aComp;

    a = create_real(1.0);
    b = create_real(1.0);
    assign_real(b, a);

    std::cout << "절대 공차 시작 \n";
    int aProgressStatusLimitColumn = 50;
    int count=0;
    int aProgressCount=0;
    int aCompleteCount=0;
    int iFactor = abs(REAL_EXP_MIN)+REAL_EXP_MAX;
    int jFactor = REAL_MANTISSA_DEC_MAX;
    int jOffset = ((DEC)pow(2,31)-1) / aRepeatCount;
    int aTotalCount = iFactor * jFactor/jOffset * kFactor;
    int aProgressFactor = aTotalCount/(aProgressStatusLimitColumn*100);
    int aCompleteFactor = aTotalCount/100;    
    for (int i=REAL_EXP_MIN; i < REAL_EXP_MAX; i++){
        // k는 만티사의 값이다. 0 에서 REAL_MANTISSA_DEC_MAX까지이다.
        for(int j=0; j<REAL_MANTISSA_DEC_MAX+1; j += jOffset){

        // offset factor에 곱하는 값이다.
        // 옵셋 인자는 공차의 1/10이다. 그래서 10이면, 공차와 같다는 의미이다.
            for(int k=0; k<kFactor; k++){

        aOffset = k*aOffsetFactor;
        set_real(a, (j/REAL_MANTISSA_DEC_MAX + 1.0) * pow(2,i));
        set_real(b, real(a) + aOffset);
        BOOST_TEST(real_equal(a, b, aTol), 
            "i:" << i << ", j:" << j << ", k:" << k << 
            ", a:"<<real(a) << ", b:" << real(b) <<
            ", aOffset:" << aOffset << ", aTol:" << aTol);
        // if(!real_equal(a, b, aTol)){
        //     printf("i:%d, j:%d, k:%d, a:%f, b:%f, offset:%f, tol:%f\n", i, j, k,
        //         real(a), real(b), aOffset, aTol
        //     );
        // }

        // // 진행상태는 총합/5000
        // if (aProgressCount > aProgressFactor){
        //     printf("#");
        //     aProgressCount=0;
        // }

        // // 완료율은 총합/100
        // if ( count > aCompleteFactor){
        //     count =0;
        //     aCompleteCount++;
        //     printf(" %d %% completes\n", aCompleteCount);
        // }

        // count++;
        // aProgressCount++;

            }
        }
    }    
    std::cout << "절대 공차 종료 \n\n";

    delete_real(a);
    delete_real(b);

    // 실제 코드 종료
}


BOOST_AUTO_TEST_CASE(rel_tol_real_tester){
    std::cout << "rel_tol_real_tester starts.\n";
    
    // 실제 코드 시작

    // 실제 코드 시작
    // float aInf = 0xefffffff;
    // float aInf = (float)0xffffffff;
    // float aInf = FLT_MAX;
    // float aInf =INFINITY;

    REAL aOffset;

    Real* a=NULL;
    Real* b=NULL;

    bool aComp;

    a = create_real(1.0);
    b = create_real(1.0);
    assign_real(b, a);

    std::cout << "상대 공차 시작 \n";
    int aProgressStatusLimitColumn = 50;
    int count=0;
    int aProgressCount=0;
    int aCompleteCount=0;
    int iFactor = abs(REAL_EXP_MIN)+REAL_EXP_MAX;
    int jFactor = REAL_MANTISSA_DEC_MAX;
    int jOffset = ((DEC)pow(2,31)-1) / aRepeatCount;
    int aTotalCount = iFactor * jFactor/jOffset * kFactor;
    int aProgressFactor = aTotalCount/(aProgressStatusLimitColumn*100);
    int aCompleteFactor = aTotalCount/100;    
    for (int i=REAL_EXP_MIN; i < REAL_EXP_MAX; i++){
        // k는 만티사의 값이다. 0 에서 REAL_MANTISSA_DEC_MAX까지이다.
        for(int j=0; j<REAL_MANTISSA_DEC_MAX+1; j += jOffset){

        // offset factor에 곱하는 값이다.
        // 옵셋 인자는 공차의 1/10이다. 그래서 10이면, 공차와 같다는 의미이다.
            for(int k=0; k<kFactor; k++){

        aOffset = k*aOffsetFactor;
        set_real(a, (j/REAL_MANTISSA_DEC_MAX + 1.0) * pow(2,i));
        set_real(b, real(a) + real(a)*aOffset);
        aComp = real_equal_rel(a, b, aTol);
        BOOST_TEST(aComp, 
            "i:" << i << ", j:" << j << ", k:" << k << 
            ", a:"<<real(a) << ", b:" << real(b) <<
            ", aOffset:" << aOffset << ", aTol:" << aTol);

            }
        }
    }    
    std::cout << "상대 공차 종료 \n\n";

    delete_real(a);
    delete_real(b);

    // 실제 코드 종료

}

BOOST_AUTO_TEST_CASE(ulp_tol_real_tester){
    
    // 실제 코드 시작

    // 실제 코드 시작
    // float aInf = 0xefffffff;
    // float aInf = (float)0xffffffff;
    // float aInf = FLT_MAX;
    // float aInf =INFINITY;

    REAL aOffset;

    Real* a=NULL;
    Real* b=NULL;

    bool aComp;

    a = create_real(1.0);
    b = create_real(1.0);
    assign_real(b, a);

    std::cout << "ULP 공차 시작 \n";
    int aProgressStatusLimitColumn = 50;
    int count=0;
    int aProgressCount=0;
    int aCompleteCount=0;
    int iFactor = abs(REAL_EXP_MIN)+REAL_EXP_MAX;
    int jFactor = REAL_MANTISSA_DEC_MAX;
    int jOffset = ((DEC)pow(2,31)-1) / aRepeatCount;
    int aTotalCount = iFactor * jFactor/jOffset * kFactor;
    int aProgressFactor = aTotalCount/(aProgressStatusLimitColumn*100);
    int aCompleteFactor = aTotalCount/100;    
    for (int i=REAL_EXP_MIN; i < REAL_EXP_MAX; i++){
        // k는 만티사의 값이다. 0 에서 REAL_MANTISSA_DEC_MAX까지이다.
        for(int j=0; j<REAL_MANTISSA_DEC_MAX+1; j += jOffset){

        // offset factor에 곱하는 값이다.
        // 옵셋 인자는 공차의 1/10이다. 그래서 10이면, 공차와 같다는 의미이다.
            for(int k=0; k<kFactor; k++){

        aOffset = k*aOffsetFactor;
        set_real(a, (j/REAL_MANTISSA_DEC_MAX + 1.0) * pow(2,i));
        set_real(b, real(a) + real(a)*aOffset);
        bool aComp = real_equal_ulp(a, b, aUlpTol);
        BOOST_TEST(aComp, 
            "i:" << i << ", j:" << j << ", k:" << k << 
            ", a:"<<real(a) << ", b:" << real(b) <<
            ", aOffset:" << aOffset << ", aUlpTol:" << aUlpTol);

            }
        }
    }    
    std::cout << "ULP 공차 종료 \n\n";

    delete_real(a);
    delete_real(b);

    // 실제 코드 종료
}

BOOST_AUTO_TEST_CASE(shift_real_tester){
    // 실제 코드 시작

    REAL aOffset;

    Real* a=NULL;
    Real* b=NULL;
    Real* c=NULL;

    bool aComp;

    int aDec;
    int aResult;

    a = create_real(123456);
    b = create_real(1.0);
    c = create_real(1.0);

    std::cout << "shift real 시작 \n";
    for (int i=-99; i < 99; i++){
        aResult = shift_real(a, i, b);
        set_real(c, real(a)*pow(10,i));
        if(aResult == REAL_OVERFLOW){
            printf("error!, overflow occur at i(%d)\n", i);
            continue;
        }
        if(aResult == REAL_UNDERFLOW){
            printf("error!, underflow occur at i(%d)\n", i);
            continue;
        }
        BOOST_TEST(real_equal_rel(b, c, aTol), 
            "shift(a,i) != a*10^i)" <<
            ", i = " << i << 
            ", a = "<<real(a) << 
            ", b = " << real(b) <<
            ", a*10^i = " << real(c)
            );
    }

    std::cout << "shift real 종료 \n\n";

    delete_real(a);
    delete_real(b);
    delete_real(c);

    // 실제 코드 종료
}

BOOST_AUTO_TEST_CASE(fix_real_exp_tester){
    // 실제 코드 시작

    REAL aOffset;

    Real* a=NULL;
    Real* b=NULL;
    Real* c=NULL;
    Real* buf=NULL;

    bool aComp;

    REAL retIntPart;
    REAL retFracPart;

    a = create_real(123456);
    b = create_real(0.0);
    c = create_real(0.0);
    buf = create_real(0.0);

    std::cout << "fix real exp 시작 \n";
    for (int i=-50; i < 50; i++){
        fix_real_exp(a, i, &retIntPart, &retFracPart);
        set_real(b, retIntPart);
        set_real(c, retFracPart);

        set_real(buf, real(a)-real(b));
        BOOST_TEST(real_equal_rel(buf, c, aTol), 
            "i = " << i <<
            ", a = " << real(a) <<
            ", intPart = " << real(b) <<
            ", fracPart = " << real(c) <<
            ", a - intPart = " << real(buf)
            );
        set_real(buf, real(a)-real(c));
        BOOST_TEST(real_equal_rel(buf, b, aTol), 
            "i = " << i <<
            ", a = " << real(a) <<
            ", intPart = " << real(b) <<
            ", fracPart = " << real(c) <<
            ", a - fracPart = " << real(buf)
            );
    }

    std::cout << "fix real exp 종료 \n\n";

    delete_real(a);
    delete_real(b);
    delete_real(c);
    delete_real(buf);

    // 실제 코드 종료
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_CASE(real_tester_split,
    * ut::fixture<Stopwatch>(std::string("real_tester_split"))
    * ut::fixture<Logger>(std::string("real_tester_split"))
){
    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndDec(111, 999);
    boost::random::uniform_real_distribution<> rndReal(-0.001,1.0);
    boost::random::uniform_int_distribution<> rndSign(0, 1);
    boost::random::uniform_int_distribution<> rndExp(-36, 37);
    boost::random::uniform_int_distribution<> rndInt(-9, 9);

    std::vector<int> aErrorExp = {3,6,7};
    Real* aSelf = NULL;
    Real* aOther = NULL;

    std::vector<int> aSignVector = {-1, 1};
    DEC aFrac;
    DEC aInt;
    DEC aExp;
    DEC aSign;

    bool retSign;
    DEC retFrac;
    DEC retInt;
    DEC retExp;

    REAL aReal;
    REAL aTol = 0.000001;

    aSelf = create_real(0.0);
    aOther = create_real(0.0);
    for(int i=0; i<3; i++){
        // BOOST_TEST(i*it == 0);
            // aFrac  = rndDec(rng);
            // aInt = rndDec(rng);
            // aExp = rndExp(rng);
            // aSign = aSignVector[rndSign(rng)];

        aReal = rndReal(rng);
        // aReal = 1 * pow(10, rndInt(rng));
        // aReal = 1 * pow(10, aErrorExp[i]);
        
        set_real(aSelf, aReal);
        split_real(aSelf, &retSign, &retFrac, &retInt, &retExp);
        BOOST_TEST(aReal != real(aSelf),
        // BOOST_TEST(real_equal_rel(aSelf, aOther, aTol),
        std::setw(10) << "org Value:" << 
        std::setw(15) << aReal <<
        std::setw(15) << ", split value:" << 
        std::setw(5) << retInt << "." << 
        std::setw(10) << retFrac << 
        " * 10^" << retExp <<
        ", self:" << real(aSelf)
        );
    }

    for(int i=0; i<1000; i++){
        // BOOST_TEST(i*it == 0);
            // aFrac  = rndDec(rng);
            // aInt = rndDec(rng);
            // aExp = rndExp(rng);
            // aSign = aSignVector[rndSign(rng)];

        aReal = rndReal(rng);
        
        set_real(aSelf, aReal);
        split_real(aSelf, &retSign, &retFrac, &retInt, &retExp);
        BOOST_TEST(aReal != real(aSelf),
        // BOOST_TEST(real_equal_rel(aSelf, aOther, aTol),
        std::setw(10) << "org Value:" << 
        std::setw(15) << aReal <<
        std::setw(15) << ", split value:" << 
        std::setw(5) << retInt << "." << 
        std::setw(10) << retFrac << 
        " * 10^" << retExp <<
        ", self:" << real(aSelf)
        );
    }

    delete_real(aOther);
    delete_real(aSelf);
}

BOOST_AUTO_TEST_CASE(real_tester_merge,
    * ut::fixture<Stopwatch>(std::string("real_tester_merge"))
    * ut::fixture<Logger>(std::string("real_tester_merge"))
){
    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndDec(111, 999);
    boost::random::uniform_real_distribution<> rndReal(111.0,999.0);
    boost::random::uniform_int_distribution<> rndSign(0, 1);
    boost::random::uniform_int_distribution<> rndExp(-30, 30);

    Real* aSelf = NULL;
    Real* aOther = NULL;

    std::vector<int> aSignVector = {-1, 1};
    DEC aFrac;
    DEC aInt;
    DEC aExp;
    DEC aSign;

    bool retSign;
    DEC retFrac;
    DEC retInt;
    DEC retExp;

    REAL aReal;
    REAL aTol = 0.000001;

    aSelf = create_real(0.0);
    aOther = create_real(0.0);
    for(int i=0; i<999; i++){
        // BOOST_TEST(i*it == 0);
            aFrac  = rndDec(rng);
            aInt = rndDec(rng);
            aExp = rndExp(rng);
            aSign = rndSign(rng);

        aReal = aSignVector[rndSign(rng)] * rndReal(rng);
        
        // set_real(aSelf, aReal);
        // split_real(aSelf, &retSign, &retFrac, &retInt, &retExp);
        merge_real(aSelf, aSign, aFrac, aInt, aExp);
        aReal = real(aSelf);
        BOOST_TEST(aReal != real(aSelf),
        // BOOST_TEST(real_equal_rel(aSelf, aOther, aTol),
        std::setw(15) << ", merge value:" << 
        std::setw(2) << (aSign?"-":" ") << 
        std::setw(5) << aInt << "." << 
        std::setw(10) << aFrac << 
        " * 10^" << aExp <<
        std::setw(10) << "to Value:" << 
        std::setw(15) << aReal
        // ", self:" << real(aSelf) <<
        // ", other:" << real(aOther) <<
        // ", tol:" << aTol
        );
    }

    delete_real(aOther);
    delete_real(aSelf);
}

BOOST_AUTO_TEST_CASE(real_tester_split_merge,
    * ut::fixture<Stopwatch>(std::string("real_tester_split_merge"))
    * ut::fixture<Logger>(std::string("real_tester_split_merge"))
){
    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndDec(111, 999);
    boost::random::uniform_real_distribution<> rndReal(1.0e-30, 1.0e30);
    boost::random::uniform_int_distribution<> rndSign(0, 1);
    boost::random::uniform_int_distribution<> rndExp(-30, 30);

    Real* aSelf = NULL;
    Real* aOther = NULL;

    std::vector<int> aSignVector = {-1, 1};
    DEC aFrac;
    DEC aInt;
    DEC aExp;
    DEC aSign;

    bool retSign;
    DEC retFrac;
    DEC retInt;
    DEC retExp;

    REAL aReal;
    REAL aTol = 0.000001;

    aSelf = create_real(0.0);
    aOther = create_real(0.0);
    for(int i=0; i<999; i++){
        // BOOST_TEST(i*it == 0);
            aFrac  = rndDec(rng);
            aInt = rndDec(rng);
            aExp = rndExp(rng);
            aSign = rndSign(rng);

        aReal = aSignVector[rndSign(rng)] * rndReal(rng);
        
        set_real(aSelf, aReal);
        split_real(aSelf, &retSign, &retFrac, &retInt, &retExp);
        merge_real(aOther, retSign, retFrac, retInt, retExp);

        BOOST_TEST(aReal != real(aOther),
        // BOOST_TEST(real_equal_rel(aSelf, aOther, aTol),
        std::setw(10) << "org Value:" << 
        std::setw(15) << aReal <<
        std::setw(15) << ", split value:" << 
        std::setw(2) << (retSign?"-":" ") << 
        std::setw(5) << retInt << "." << 
        std::setw(10) << retFrac << 
        " * 10^" << retExp <<
        ", other:" << real(aOther)
        );
    }

    delete_real(aOther);
    delete_real(aSelf);
}