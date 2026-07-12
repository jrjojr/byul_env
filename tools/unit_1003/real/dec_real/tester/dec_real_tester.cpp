/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec_real 1.0.2.2
*
***************************************************************************/

#define BOOST_TEST_MODULE dec_real tester

#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <iostream>
#include <iomanip> // setw
#include <string>
#include <memory>
#include <vector>

#include "dec_real.h"
#include "dec_real_tester.hpp"

#include <boost/test/data/test_case.hpp>
#include <boost/test/data/monomorphic.hpp>

namespace ut = boost::unit_test;
namespace bdata = boost::unit_test::data;
namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(dec_real_tester_get_set,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_get_set"))    ){

    DecReal aSelf;
    DecReal aOther;

    DEC retExpPart;
    UDEC retIntPart;
    UDEC retFracPart;
    bool retSign;
    DecMinTruncPolicy retPolicy;

    init_dec_real(&aSelf);
    BOOST_TEST(aSelf.mPolicy == DEC_MIN_TRUNC_POLICY_ROUND);
    BOOST_TEST(aSelf.mFracPart == 0);
    BOOST_TEST(aSelf.mIntPart == 0);
    BOOST_TEST(aSelf.mExp == 0);

    init_dec_real(&aOther);
    BOOST_TEST(dec_real_is_zero(&aOther));
    set_dec_real(&aOther, false, 123, 456, 78, DEC_MIN_TRUNC_POLICY_FLOOR);
    assign_dec_real(&aSelf, &aOther);
    BOOST_TEST(aSelf.mPolicy == DEC_MIN_TRUNC_POLICY_FLOOR);
    BOOST_TEST(aSelf.mFracPart == 123);
    BOOST_TEST(aSelf.mIntPart == 456);
    BOOST_TEST(aSelf.mExp == 78);

    get_dec_real(&aSelf, &retSign, &retFracPart, &retIntPart, &retExpPart, 
        &retPolicy);

    BOOST_TEST(retFracPart == 123);
    BOOST_TEST(retIntPart == 456);
    BOOST_TEST(retExpPart == 78);
    BOOST_TEST(retPolicy == DEC_MIN_TRUNC_POLICY_FLOOR);    

    set_dec_real_frac(&aSelf, 987);
    set_dec_real_int(&aSelf, 654);
    set_dec_real_exp(&aSelf, 32);
    set_dec_real_policy(&aSelf, DEC_MIN_TRUNC_POLICY_CEILING);
    
    get_dec_real_int(&aSelf, &retIntPart);
    get_dec_real_frac(&aSelf, &retFracPart);
    get_dec_real_exp(&aSelf, &retExpPart);
    get_dec_real_policy(&aSelf, &retPolicy);    
                
                BOOST_TEST(retFracPart == 987);
                BOOST_TEST(retIntPart == 654);
                BOOST_TEST(retExpPart == 32);
                BOOST_TEST(retPolicy == DEC_MIN_TRUNC_POLICY_CEILING);

    retFracPart = 0;
    retIntPart =  0;
    retExpPart =  0;
    retPolicy =  DEC_MIN_TRUNC_POLICY_CEILING;    

    retFracPart = dec_real_frac(&aSelf);
    retIntPart =  dec_real_int(&aSelf);
    retExpPart =  dec_real_exp(&aSelf);
    retPolicy =  dec_real_policy(&aSelf);    

                BOOST_TEST(retFracPart == 987);
                BOOST_TEST(retIntPart == 654);
                BOOST_TEST(retExpPart == 32);
                BOOST_TEST(retPolicy == DEC_MIN_TRUNC_POLICY_CEILING);

    release_dec_real(&aSelf);
    release_dec_real(&aOther);    
    // 여기까지 실제 테스트할 코드를 작성한다.
}                
   
BOOST_AUTO_TEST_CASE(dec_real_tester_shift,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_shift"))
    * ut::fixture<Logger>(std::string("dec_real_tester_shift") )
    ){

std::vector<int> aSignVector;
    aSignVector = {1, -1};

boost::random::random_device rng;

boost::random::uniform_int_distribution<> rndExp(-15, 15);
boost::random::uniform_int_distribution<> rndPlusExp(0, 10);
boost::random::uniform_int_distribution<> rndMinusExp(-10, 0);
boost::random::uniform_int_distribution<> rndSign(0, 1);

boost::random::uniform_int_distribution<> rndInt(0, 999999);
boost::random::uniform_int_distribution<> rndFrac(1, 999999);

boost::random::uniform_real_distribution<> rndReal(1.0e-15, 1.0e15);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

UDEC aFrac;
UDEC aInt;
DEC aExp;
DEC aSign;
DEC aSignValue;

REAL aSelfValue;


DEC aShiftExp;

    aSelf = create_dec_real_default();
    aOther = create_dec_real_default();

for(int i=0; i< 1000; i++){

    aSign = rndSign(rng);
    aFrac = rndInt(rng) * 1000;
    // aFrac = rndFrac(rng) * 100000;
    aInt = rndInt(rng);
    aExp = rndExp(rng);
    set_dec_real(aSelf, aSign, aFrac, aInt, aExp, DEC_MIN_TRUNC_POLICY_ROUND);
    
    aSignValue = aSignVector[aSign];

    // aShiftExp = rndMinusExp(rng);
    aShiftExp = rndExp(rng);

    BOOST_TEST( 0,
    std::setw(20) << "before shift : " << 
    std::setw(5) << aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    shift_dec_real(aSelf, aShiftExp);

    BOOST_TEST( 0,
    std::setw(20) << "after shift : " << 
    std::setw(5) << aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    shift_dec_real(aSelf, -aShiftExp);

    BOOST_TEST( 0,
    std::setw(20) << " to org : " << 
    std::setw(5) << -aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    }

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_to_from,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_to_from"))
    * ut::fixture<Logger>(std::string("dec_real_tester_to_from") )
    ){

std::vector<int> aSignVector;
aSignVector = {-1, 1};

boost::random::random_device rng;

boost::random::uniform_int_distribution<> rndSign(0, 1);
boost::random::uniform_real_distribution<> rndReal(-1.0e-30, 1.0e30);    
boost::random::uniform_int_distribution<> rndInt(-9, 9);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;
REAL aTol = 0.000001;

DEC aSign;
UDEC aFrac;
UDEC aInt;
DEC aExp;

DecMinTruncPolicy aPolicy;

// aSelf = create_dec_real(aSign, aFrac, aInt, aExp, aPolicy);
aSelf = create_dec_real_default();
aOther = create_dec_real_default();
    // Progress aPrs(999,1);
    for(int i=0; i<1000; i++){

            // aSign = aSignVector[rndSign(rng)];
            // aFrac = rndInt(rng);
            // aInt = rndInt(rng);
            // aExp = rndExp(rng);
            // aSelfValue = rndReal(rng);
            aSelfValue = rndInt(rng);
            set_dec_real_from(aSelf, aSelfValue);

            BOOST_TEST(dec_real_to(aSelf) == aSelfValue,

            std::setw(15) << "self value:" << 
            std::setw(20) << aSelfValue <<
            std::setw(15) << ", dec_real_member_var: " <<
            std::setw(1) << (dec_real_sign(aSelf)?"-":" ") <<
            std::setw(9) << dec_real_int(aSelf) << "." <<
            std::setw(9) << dec_real_frac(aSelf) <<
            std::setw(5) << " * 10^" <<
            std::setw(2) << dec_real_exp(aSelf) <<
            std::setw(10) << ", dec_real_to:" << 
            std::setw(20) << dec_real_to(aSelf)

            );

            // aPrs.tick();

        // }
    }

delete_dec_real(aOther);
delete_dec_real(aSelf);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_fix,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_fix"))
    * ut::fixture<Logger>(std::string("dec_real_tester_fix") )
    ){

std::vector<int> aSignVector;
    aSignVector = {1, -1};

boost::random::random_device rng;

boost::random::uniform_int_distribution<> rndExp(-15, 15);
boost::random::uniform_int_distribution<> rndSign(0, 1);

boost::random::uniform_int_distribution<> rndInt(0, 999999);

boost::random::uniform_real_distribution<> rndReal(1.0e-15, 1.0e15);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

UDEC aFrac;
UDEC aInt;
DEC aExp;
DEC aSign;
DEC aSignValue;

REAL aSelfValue;

bool retSign;
UDEC retInt;
UDEC retFrac;
DEC retExp;

DEC aShiftExp;

    aSelf = create_dec_real_default();
    aOther = create_dec_real_default();

for(int i=0; i< 1000; i++){

    aSign = rndSign(rng);
    aFrac = rndInt(rng) * 1000;
    // aFrac = rndFrac(rng) * 100000;
    aInt = rndInt(rng);
    aExp = rndExp(rng);
    set_dec_real(aSelf, aSign, aFrac, aInt, aExp, DEC_MIN_TRUNC_POLICY_ROUND);
    
    aSignValue = aSignVector[aSign];

    // aShiftExp = rndMinusExp(rng);
    aShiftExp = rndExp(rng);
    retExp = aShiftExp;

    BOOST_TEST( 0,
    std::setw(20) << "before fix : " << 
    std::setw(5) << retExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    dec_real_fix_exp_ret(aSelf, &retExp, &retSign, &retFrac, &retInt);
    set_dec_real(aSelf, retSign, retFrac, retInt, retExp, 
        DEC_MIN_TRUNC_POLICY_ROUND);

    BOOST_TEST( 0,
    std::setw(20) << "after fix : " << 
    std::setw(5) << retExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    dec_real_fix_exp_ret(aSelf, &aExp, &retSign, &retFrac, &retInt);
    set_dec_real(aSelf, retSign, retFrac, retInt, aExp, 
        DEC_MIN_TRUNC_POLICY_ROUND);

    BOOST_TEST( 0,
    std::setw(20) << " to org : " << 
    std::setw(5) << aExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    }

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_add,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_add"))
    * ut::fixture<Logger>(std::string("dec_real_tester_add"))
    ){

std::vector<int> aSignVector = {-1, 1};

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(1.0e-3, 1.0e3);
boost::random::uniform_int_distribution<> rndInt(-1000, 1000);

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    add_dec_real(aSelf, aOther);
    BOOST_TEST( dec_real_to(aSelf) == aSelfValue+aOtherValue,
    "add result:" << dec_real_to(aSelf) << 
    ", self value:" << aSelfValue <<
    ", other value:" << aOtherValue
    );
}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
    }

BOOST_AUTO_TEST_CASE(dec_real_tester_sub,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_sub"))
    * ut::fixture<Logger>(std::string("dec_real_tester_sub"))
    ){

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(
    1.0e-3, 1.0e3);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    sub_dec_real(aSelf, aOther);
    BOOST_TEST( dec_real_to(aSelf) == aSelfValue-aOtherValue,
    "sub result:" << dec_real_to(aSelf) << 
    ", self value:" << aSelfValue <<
    ", other value:" << aOtherValue
    );
}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
    }

BOOST_AUTO_TEST_CASE(dec_real_tester_mul,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_mul"))
    * ut::fixture<Logger>(std::string("dec_real_tester_mul"))
    ){

std::vector<int> aSignVector = {-1, 1};

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(
    1.0e-3, 1.0e3);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    mul_dec_real(aSelf, aOther);
    BOOST_TEST( dec_real_to(aSelf) == aSelfValue*aOtherValue,
    "mul result:" << dec_real_to(aSelf) << 
    ", self value:" << aSelfValue <<
    ", other value:" << aOtherValue
    );
}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);

    }

BOOST_AUTO_TEST_CASE(dec_real_tester_div,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_div"))
    * ut::fixture<Logger>(std::string("dec_real_tester_div"))
    ){

std::vector<int> aSignVector = {-1, 1};

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(
    1.0e-3, 1.0e3);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    div_dec_real(aSelf, aOther);
    BOOST_TEST( dec_real_to(aSelf) == aSelfValue/aOtherValue,
    "div result:" << dec_real_to(aSelf) << 
    ", self value:" << aSelfValue <<
    ", other value:" << aOtherValue
    );
}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);

    }


BOOST_AUTO_TEST_CASE(dec_real_tester_shift_trunc,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_shift_trunc"))
    * ut::fixture<Logger>(std::string("dec_real_tester_shift_trunc") )
    ){

std::vector<int> aSignVector;
    aSignVector = {1, -1};

boost::random::random_device rng;

boost::random::uniform_int_distribution<> rndExp(-15, 15);
boost::random::uniform_int_distribution<> rndPlusExp(0, 10);
boost::random::uniform_int_distribution<> rndMinusExp(-10, 0);
boost::random::uniform_int_distribution<> rndSign(0, 1);

boost::random::uniform_int_distribution<> rndInt(0, 999999);
boost::random::uniform_int_distribution<> rndFrac(1, 999999);

boost::random::uniform_real_distribution<> rndReal(1.0e-15, 1.0e15);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

UDEC aFrac;
UDEC aInt;
DEC aExp;
DEC aSign;
DEC aSignValue;

REAL aSelfValue;


DEC aShiftExp;

    aSelf = create_dec_real_default();
    aOther = create_dec_real_default();

for(int i=0; i< 1000; i++){

    aSign = rndSign(rng);
    aFrac = rndInt(rng) * 1000;
    // aFrac = rndFrac(rng) * 100000;
    aInt = rndInt(rng);
    aExp = rndExp(rng);
    set_dec_real(aSelf, aSign, aFrac, aInt, aExp, DEC_MIN_TRUNC_POLICY_ROUND);
    
    aSignValue = aSignVector[aSign];

    // aShiftExp = rndMinusExp(rng);
    aShiftExp = rndExp(rng);

    BOOST_TEST( 0,
    std::setw(20) << "before shift : " << 
    std::setw(5) << aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    shift_dec_real_trunc(aSelf, aShiftExp);

    BOOST_TEST( 0,
    std::setw(20) << "after shift : " << 
    std::setw(5) << aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    shift_dec_real_trunc(aSelf, -aShiftExp);

    BOOST_TEST( 0,
    std::setw(20) << " to org : " << 
    std::setw(5) << -aShiftExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    }

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}


BOOST_AUTO_TEST_CASE(dec_real_tester_fix_trunc,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_fix_trunc"))
    * ut::fixture<Logger>(std::string("dec_real_tester_fix_trunc") )
    ){

std::vector<int> aSignVector;
    aSignVector = {1, -1};

boost::random::random_device rng;

boost::random::uniform_int_distribution<> rndExp(-15, 15);
boost::random::uniform_int_distribution<> rndSign(0, 1);

boost::random::uniform_int_distribution<> rndInt(0, 999999999);

boost::random::uniform_real_distribution<> rndReal(1.0e-15, 1.0e15);    

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

UDEC aFrac;
UDEC aInt;
DEC aExp;
DEC aSign;
DEC aSignValue;

REAL aSelfValue;

bool retSign;
UDEC retInt;
UDEC retFrac;
DEC retExp;

DEC aShiftExp;

    aSelf = create_dec_real_default();
    aOther = create_dec_real_default();

for(int i=0; i< 1000; i++){

    aSign = rndSign(rng);
    aFrac = rndInt(rng) * 1;
    // aFrac = rndFrac(rng) * 100000;
    aInt = rndInt(rng);
    aExp = rndExp(rng);
    set_dec_real(aSelf, aSign, aFrac, aInt, aExp, DEC_MIN_TRUNC_POLICY_ROUND);
    
    aSignValue = aSignVector[aSign];

    // aShiftExp = rndMinusExp(rng);
    aShiftExp = rndExp(rng);
    retExp = aShiftExp;

    BOOST_TEST( 0,
    std::setw(20) << "before fix : " << 
    std::setw(5) << retExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    dec_real_fix_exp_trunc_ret(aSelf, &retExp, &retSign, &retFrac, &retInt);
    set_dec_real(aSelf, retSign, retFrac, retInt, retExp, 
        DEC_MIN_TRUNC_POLICY_ROUND);

    BOOST_TEST( 0,
    std::setw(20) << "after fix : " << 
    std::setw(5) << retExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    dec_real_fix_exp_trunc_ret(aSelf, &aExp, &retSign, &retFrac, &retInt);
set_dec_real(aSelf, retSign, retFrac, retInt, aExp, 
        DEC_MIN_TRUNC_POLICY_ROUND);

    BOOST_TEST( 0,
    std::setw(20) << " to org : " << 
    std::setw(5) << aExp << ", " <<
    (aSelf->mSign?"-":" ") << 
    std::setw(9) << aSelf->mIntPart << "." << 
    std::setw(9) << aSelf->mFracPart << " * 10^" << aSelf->mExp
        );

    }

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_equal,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_equal"))
    * ut::fixture<Logger>(std::string("dec_real_tester_equal") )
    ){

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(
    1.0e-3, 1.0e3);    

boost::random::uniform_real_distribution<> rndTol(
    0.000001, 0.001);

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;
REAL aTol;
REAL aOffset;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aTol = rndTol(rng);
    aOffset = rndTol(rng);
    // aOffset = rndReal(rng)/100;
    aOtherValue = aSelfValue + aOffset;
    // aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    BOOST_TEST( dec_real_equal(aSelf, aOther, aTol),
    "dec_real_equal" <<
    ", self value: " << aSelfValue <<
    ", other value: " << aOtherValue <<
    ", offset: " << aOffset <<
    ", tol: " << aTol
    );

}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_equal_rel,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_equal_rel"))
    * ut::fixture<Logger>(std::string("dec_real_tester_equal_rel") )
    ){

boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(
    1.0e-3, 1.0e3);    

boost::random::uniform_real_distribution<> rndTol(
    0.000001, 0.001);

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;
REAL aTol;
REAL aOffset;

aSelf = create_dec_real_default();
aOther = create_dec_real_default();

// Progress aPrs(999,1);
for(int i=0; i< 1000; i++){

    aSelfValue = rndReal(rng);
    aTol = rndTol(rng);
    aOffset = rndTol(rng);
    // aOffset = rndReal(rng)/100;
    aOtherValue = aSelfValue + aOffset;
    // aOtherValue = rndReal(rng);
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    BOOST_TEST( dec_real_equal_rel(aSelf, aOther, aTol),
    "dec_real_equal" <<
    ", self value: " << aSelfValue <<
    ", other value: " << aOtherValue <<
    ", offset: " << aOffset <<
    ", tol: " << aTol
    );

}

    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}


BOOST_AUTO_TEST_SUITE(dec_real_tester_ext_suite,
    * ut::fixture<Stopwatch>(std::string("dec_real_tester_ext_suite"))
    * ut::fixture<Logger>(std::string("dec_real_tester_ext_suite") )
)


boost::random::random_device rng;

boost::random::uniform_real_distribution<> rndReal(1.0e-3, 1.0e3);
boost::random::uniform_int_distribution<> rndInt(-1000, 1000);

DecReal* aSelf = NULL;
DecReal* aOther = NULL;

REAL aSelfValue;
REAL aOtherValue;

BOOST_AUTO_TEST_CASE(dec_real_tester_ext_begin){
aSelf = create_dec_real_default();
aOther = create_dec_real_default();
}

BOOST_DATA_TEST_CASE(dec_real_tester_ext, 
    bdata::xrange(1000) ^
    bdata::random(
        bdata::distribution=std::uniform_real_distribution<>(-1.0e-3, 1.0e3)) ^
    bdata::random(
        bdata::distribution=std::uniform_int_distribution<>(-1000, 1000)),
    xr,self,other){

    aSelfValue = self;
    aOtherValue = other;
    set_dec_real_from(aSelf, aSelfValue);
    set_dec_real_from(aOther, aOtherValue);

    add_dec_real(aSelf, aOther);
    BOOST_TEST_INFO("add result:" << dec_real_to(aSelf) << 
        ", self value:" << aSelfValue <<
        ", other value:" << aOtherValue);
   
    BOOST_TEST( dec_real_to(aSelf) == aSelfValue+aOtherValue);
    // BOOST_TEST(false);
}

BOOST_AUTO_TEST_CASE(dec_real_tester_ext_end){
    delete_dec_real(aSelf);
    delete_dec_real(aOther);
}

BOOST_AUTO_TEST_SUITE_END()