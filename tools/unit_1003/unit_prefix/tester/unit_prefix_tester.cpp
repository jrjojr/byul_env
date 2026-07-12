/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit_prefix
*
***************************************************************************/

#define BOOST_TEST_MODULE unit_prefix tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "unit_prefix.h"
#include "unit_prefix_tester.hpp"

#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace ut = boost::unit_test;

BOOST_AUTO_TEST_CASE(test_unit_prefix_tester){
    std::cout << "test_unit_prefix_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "unit_prefix version is below." << std::endl;
    print_unit_prefix_version();

    // 여기부터 실제 테스트할 코드를 작성한다.
    UnitPrefixEnum aEnumSelf = UNIT_PREFIX_MILLI;
    UnitPrefixEnum aEnumOther = UNIT_PREFIX_CENTI;

    UnitPrefixEnum aLengthEnum;
    UnitPrefixEnum aMassEnum;
    UnitPrefixEnum aTimeEnum;
    UnitPrefixEnum aAngleEnum;

    UnitPrefixSnapPolicy aPolicySelf = UNIT_PREFIX_SNAP_POLICY_NORMAL;
    
    UnitPrefixSnapPolicy aPolicyRet;

    // char* aUnitPrefixName;
    // aUnitPrefixName = unit_prefix_name(aEnumSelf);

    add_unit_prefix_enum(&aEnumSelf, aEnumOther, aPolicySelf);
    BOOST_TEST(aEnumSelf == UNIT_PREFIX_MILLI);

   
    BOOST_TEST(aLengthEnum == UNIT_PREFIX_GIGA);
    BOOST_TEST(aMassEnum == UNIT_PREFIX_MEGA);
    BOOST_TEST(aTimeEnum == UNIT_PREFIX_MICRO);
    BOOST_TEST(aAngleEnum == UNIT_PREFIX_NANO);



    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start unit_prefix test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   unit_prefix test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}

BOOST_AUTO_TEST_CASE(unit_prefix_tester_snap,
    * ut::fixture<Stopwatch>(std::string("unit_prefix_tester_snap"))
    * ut::fixture<Logger>(std::string("unit_prefix_tester_snap"))
){
    // 여기부터 실제 테스트할 코드를 작성한다.
    boost::random_device rng;
    boost::random::uniform_int_distribution<> rndInt(-40, 40);
    int aExp;
    int aSnapExp;

    for(int i=0; i<100; i++){
        aExp = rndInt(rng);
        aSnapExp = unit_prefix_enum_snap_from(aExp, 
            UNIT_PREFIX_SNAP_POLICY_ZERO_NEAREST);
        BOOST_TEST(0,
        "aExp : " << aExp <<
        ", aSnapExp : " << aSnapExp
        );
    }

    // 여기까지 실제 테스트할 코드를 작성한다.
}
