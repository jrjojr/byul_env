/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* quantity
*
***************************************************************************/

#define BOOST_TEST_MODULE quantity tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "quantity.h"
#include "quantity_tester.hpp"

#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace ut = boost::unit_test;
BOOST_AUTO_TEST_CASE(quantity_tester_test,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_test"))
    *ut::fixture<Logger>(std::string("quantity_tester_test"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    aSelf = create_quantity_default();
    aOther = create_quantity_default();

    BOOST_TEST(dec_real_to(&aSelf->mDecReal) == 0.0);
    BOOST_TEST(unit_is_equal(&aSelf->mUnits.mUnit, &EmptyUnit));

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_first,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_first"))
    *ut::fixture<Logger>(std::string("quantity_tester_first"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    aDecReal = create_dec_real_from(120.0456);

    aUnits = create_units_default();
    assign_unit(&aUnits->mUnit, &Work);

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);

    set_quantity_dec_real(aSelf, aDecReal);
    BOOST_TEST(dec_real_equal_rel(&quantity_dec_real(aSelf), aDecReal, 0.001),
        dec_real_to(&quantity_dec_real(aSelf)) << ", " <<
        dec_real_to(aDecReal)
         );

    set_quantity_units(aSelf, aUnits);
    BOOST_TEST(units_equal(&aSelf->mUnits, aUnits));

    get_quantity_dec_real(aSelf, &aRetDecReal);

    get_quantity_units(aSelf, &aRetUnits);

    BOOST_TEST(dec_real_to(&aRetDecReal) == 120.0456);
    BOOST_TEST(units_equal(aUnits, &aRetUnits));

    set_dec_real_from(&aRetDecReal, 0.0);

    aRetDecReal = quantity_dec_real(aSelf);
    aRetUnits = quantity_units(aSelf);

    BOOST_TEST(dec_real_to(&aRetDecReal) == 120.0456);
    BOOST_TEST(units_equal(aUnits, &aRetUnits));    

    delete_quantity(aSelf);
    aSelf = create_quantity_default();

    assign_quantity(aSelf, aOther);

    BOOST_TEST(quantity_equal_rel(aSelf, aOther, 0.001));
    BOOST_TEST(dec_real_equal_rel(&quantity_dec_real(aSelf), aDecReal, 0.001),
            dec_real_to(&quantity_dec_real(aSelf)) << ", " <<
        dec_real_to(aDecReal)
         );
    BOOST_TEST(units_equal(&aSelf->mUnits, aUnits));

    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_second,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_second"))
    *ut::fixture<Logger>(std::string("quantity_tester_second"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    for(int i=0; i<1000; i++){
        aSelfValue = rndReal(rng);
        aOtherValue = rndReal(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_add(aSelf, aOther);
        // add_quantity(aSelf, aOther);
        BOOST_TEST_INFO("add result : "  << 
            dec_real_to(&aResult.mDecReal) <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(dec_real_to(&aResult.mDecReal) == aSelfValue+aOtherValue);


        aResult = quantity_sub(aSelf, aOther);
        BOOST_TEST_INFO("sub result : "  << 
            dec_real_to(&aResult.mDecReal) <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(dec_real_to(&aResult.mDecReal) == aSelfValue-aOtherValue);

        aResult = quantity_mul(aSelf, aOther);
        BOOST_TEST_INFO("mul result : "  << 
            dec_real_to(&aResult.mDecReal) <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);        
        BOOST_TEST(dec_real_to(&aResult.mDecReal) == aSelfValue*aOtherValue);        

        aResult = quantity_div(aSelf, aOther);
        BOOST_TEST_INFO("div result : "  << 
            dec_real_to(&aResult.mDecReal) <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);        
        BOOST_TEST(dec_real_to(&aResult.mDecReal) == aSelfValue/aOtherValue);
    }

    aResult = quantity_pow(aSelf, 3.0);
    BOOST_TEST(dec_real_to(&aResult.mDecReal) == pow(aSelfValue, 3.0));        
    // BOOST_TEST(
    //     dec_real_equal_rel( &aResult.mDecReal, 
    //     &dec_real_from(pow(aSelfValue, 3.0)), 0.0001 ),
    //     dec_real_to(&aResult.mDecReal) << ", " <<
    //     pow(aSelfValue, 3.0)
    //        );     


    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_add,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_add"))
    *ut::fixture<Logger>(std::string("quantity_tester_add"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    std::vector<REAL> aOpA;
    std::vector<REAL> aOpB;
    std::vector<REAL> aOpC;
    REAL aResultValue;

    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_add(aSelf, aOther);
        // add_quantity(aSelf, aOther);
        // BOOST_TEST_INFO("add result : "  << 
        //     dec_real_to(&aResult.mDecReal) <<
        //     ", self value : " << aSelfValue <<
        //     ", other value : " << aOtherValue);
        aResultValue = quantity_value(&aResult);
        BOOST_TEST_INFO("a error op result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        // BOOST_TEST(aResultValue == aSelfValue+aOtherValue);        
        if(!value_equal_rel(aResultValue, aSelfValue+aOtherValue, 0.000001)){
            aOpA.push_back(aSelfValue);
            aOpB.push_back(aOtherValue);
            aOpC.push_back(aResultValue);
        }

    }

    std::cout << "a error op result size : " << aOpA.size() << std::endl;
    for(int i=0; i<aOpA.size(); i++){
        aSelfValue = aOpA[i];
        aOtherValue = aOpB[i];

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_add(aSelf, aOther);
        // add_quantity(aSelf, aOther);
        // BOOST_TEST_INFO("add result : "  << 
        //     dec_real_to(&aResult.mDecReal) <<
        //     ", self value : " << aSelfValue <<
        //     ", other value : " << aOtherValue);
        aResultValue = quantity_value(&aResult);

        BOOST_TEST_INFO("a error op result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == aSelfValue+aOtherValue);
    }    

    std::vector<REAL> aHitErrorA;
    std::vector<REAL> aHitErrorB;

    aHitErrorA.push_back(672);
    aHitErrorA.push_back(473);
    aHitErrorA.push_back(906);

    aHitErrorA.push_back(0);
    aHitErrorA.push_back(868);
    aHitErrorA.push_back(153);
    aHitErrorA.push_back(713);

    aHitErrorB.push_back(-672);
    aHitErrorB.push_back(0);
    aHitErrorB.push_back(-906);

    aHitErrorB.push_back(-612);
    aHitErrorB.push_back(0);
    aHitErrorB.push_back(0);
    aHitErrorB.push_back(0);

    for(int i=0; i<7; i++){
        aSelfValue = aHitErrorA[i];
        aOtherValue = aHitErrorB[i];

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_add(aSelf, aOther);
        // add_quantity(aSelf, aOther);
        // BOOST_TEST_INFO("add result : "  << 
        //     dec_real_to(&aResult.mDecReal) <<
        //     ", self value : " << aSelfValue <<
        //     ", other value : " << aOtherValue);
        aResultValue = quantity_value(&aResult);

        BOOST_TEST_INFO("a hit error op result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == aSelfValue+aOtherValue);        
    }

    // a error op result : 672, self value : 672, other value : -672
    // a error op result : -199, self value : 473, other value : 0
    // a error op result : 906, self value : 906, other value : -906

    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_sub,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_sub"))
    *ut::fixture<Logger>(std::string("quantity_tester_sub"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    std::vector<REAL> aOpA;
    std::vector<REAL> aOpB;
    std::vector<REAL> aOpC;
    REAL aResultValue;

    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_sub(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        BOOST_TEST_INFO("sub result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == aSelfValue-aOtherValue);        
        if(aResultValue != aSelfValue+aOtherValue){
            aOpA.push_back(aSelfValue);
            aOpB.push_back(aOtherValue);
            aOpC.push_back(aResultValue);
        }
    }
    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_mul,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_mul"))
    *ut::fixture<Logger>(std::string("quantity_tester_mul"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    std::vector<REAL> aOpA;
    std::vector<REAL> aOpB;
    std::vector<REAL> aOpC;
    REAL aResultValue;

    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_mul(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        // BOOST_TEST_INFO("mul result : "  << 
        //     aResultValue <<
        //     ", self value : " << aSelfValue <<
        //     ", other value : " << aOtherValue);
        // BOOST_TEST(aResultValue == aSelfValue*aOtherValue);        
        if(aResultValue != aSelfValue*aOtherValue){
            aOpA.push_back(aSelfValue);
            aOpB.push_back(aOtherValue);
            aOpC.push_back(aResultValue);
        }
    }

    std::cout << "a error mul op result size : " << aOpA.size() << std::endl;
    for(int i=0; i<aOpA.size(); i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        // aSelfValue = rndInt(rng);
        // aOtherValue = rndInt(rng);
        aSelfValue = aOpA[i];
        aOtherValue = aOpB[i];

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_mul(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        BOOST_TEST_INFO("mul result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == aSelfValue*aOtherValue);        
    }

    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_div,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_div"))
    *ut::fixture<Logger>(std::string("quantity_tester_div"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    std::vector<REAL> aOpA;
    std::vector<REAL> aOpB;
    std::vector<REAL> aOpC;
    REAL aResultValue;

    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_div(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        BOOST_TEST_INFO("div result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == aSelfValue/aOtherValue);        
        if(aResultValue != aSelfValue+aOtherValue){
            aOpA.push_back(aSelfValue);
            aOpB.push_back(aOtherValue);
            aOpC.push_back(aResultValue);
        }
    }
    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_pow,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_pow"))
    *ut::fixture<Logger>(std::string("quantity_tester_pow"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-3, 3);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-6, 1.0e6);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    std::vector<REAL> aOpA;
    std::vector<REAL> aOpB;
    std::vector<REAL> aOpC;
    REAL aResultValue;

    for(int i=0; i<1000; i++){
        aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        // aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        aResult = quantity_pow(aSelf, aOtherValue);
        aResultValue = quantity_value(&aResult);
        BOOST_TEST_INFO("pow result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        BOOST_TEST(aResultValue == pow(aSelfValue,aOtherValue));        
    }
    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_add_str,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_add_str"))
    *ut::fixture<Logger>(std::string("quantity_tester_add_str"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;
    REAL aResultValue;

    char buf[64];

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    assign_unit(&aSelf->mUnits.mUnit, &RadiantFlux);
    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);        

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        strcpy(buf, quantity_to(aSelf, buf));
        BOOST_TEST_INFO("before add : self : " << buf);

        strcpy(buf, quantity_to(aOther, buf));
        BOOST_TEST_INFO("before add : other : " << buf);        

        aResult = quantity_add(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        // add_quantity(aSelf, aOther);
        BOOST_TEST_INFO("add result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        // BOOST_TEST(dec_real_equal_rel(&aResult.mDecReal, 
        //     &dec_real_add(&aSelf->mDecReal,&aOther->mDecReal), 0.001));

        strcpy(buf, quantity_to(&aResult, buf));
        BOOST_TEST_INFO("after add : self : " << buf);
        BOOST_TEST(false);
    }

    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(quantity_tester_mul_str,
    *ut::fixture<Stopwatch>(std::string("quantity_tester_mul_str"))
    *ut::fixture<Logger>(std::string("quantity_tester_mul_str"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    Quantity* aSelf = NULL;
    Quantity* aOther = NULL;

    Quantity aResult;

    DecReal* aDecReal = NULL;
    Units* aUnits = NULL;

    DecReal aRetDecReal;
    Units aRetUnits;

    REAL aSelfValue;
    REAL aOtherValue;
    REAL aResultValue;

    char buf[64];

    boost::random::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-1000, 1000);
    boost::random::uniform_real_distribution<> rndReal(-1.0e-12, 1.0e12);

    aDecReal = create_dec_real_default();
    aUnits = create_units_default();

    aSelf = create_quantity_default();
    aOther = create_quantity(aDecReal, aUnits);
    // aOther = create_quantity_default();
    // assign_dec_real(&aOther->mDecReal, aDecReal);
    // assign_units(&aOther->mUnits, aUnits);

    // assign_unit(&aSelf->mUnits.mUnit, &RadiantFlux);
    assign_unit(&aSelf->mUnits.mUnit, &Force);

    // assign_unit(&aOther->mUnits.mUnit, &Force);
    assign_unit(&aOther->mUnits.mUnit, &Pressure);
    // assign_unit(&aOther->mUnits.mUnit, &Work);
    for(int i=0; i<1000; i++){
        // aSelfValue = rndReal(rng);
        // aOtherValue = rndReal(rng);
        aSelfValue = rndInt(rng);
        aOtherValue = rndInt(rng);        

        set_dec_real_from(&aSelf->mDecReal, aSelfValue);
        set_dec_real_from(&aOther->mDecReal, aOtherValue);

        strcpy(buf, quantity_to(aSelf, buf));
        BOOST_TEST_INFO("before mul : self : " << buf);

        strcpy(buf, quantity_to(aOther, buf));
        BOOST_TEST_INFO("before mul : other : " << buf);        

        aResult = quantity_mul(aSelf, aOther);
        aResultValue = quantity_value(&aResult);
        // mul_quantity(aSelf, aOther);
        BOOST_TEST_INFO("mul result : "  << 
            aResultValue <<
            ", self value : " << aSelfValue <<
            ", other value : " << aOtherValue);
        // BOOST_TEST(dec_real_equal_rel(&aResult.mDecReal, 
        //     &dec_real_mul(&aSelf->mDecReal,&aOther->mDecReal), 0.001));

        strcpy(buf, quantity_to(&aResult, buf));
        BOOST_TEST_INFO("after mul : self : " << buf);
        BOOST_TEST(false);
    }

    // aResult = quantity_pow(aSelf, 3.0);
    // BOOST_TEST(quantity_value(&aResult) == pow(aSelfValue, 3.0));        
    // BOOST_TEST(
    //     dec_real_equal_rel( &aResult, 
    //     &dec_real_from(pow(aSelfValue, 3.0)), 0.0001 ),
    //     quantity_value(&aResult) << ", " <<
    //     pow(aSelfValue, 3.0)
    //        );     


    delete_units(aUnits);
    delete_dec_real(aDecReal);

    delete_quantity(aOther);
    delete_quantity(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}