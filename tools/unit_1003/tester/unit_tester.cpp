/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit
*
***************************************************************************/

#define BOOST_TEST_MODULE unit tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "unit.h"

BOOST_AUTO_TEST_CASE(test_unit_tester){
    std::cout << "test_unit_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "unit version is below." << std::endl;
    print_unit_version();

    char buf[16];
    printf("unit_version is : %s\n", unit_version(buf));

    // 여기부터 실제 테스트할 코드를 작성한다.

    Unit aUnit;

    init_unit(&aUnit);
    
    BOOST_TEST(unit_is_equal(&aUnit, &EmptyUnit) == true);
    assign_unit(&aUnit, &Length);
    BOOST_TEST(unit_is_equal(&aUnit, &Length) == true);

    assign_unit(&aUnit, &Mass);
    BOOST_TEST(unit_is_equal(&aUnit, &Mass) == true);

    assign_unit(&aUnit, &Time);
    BOOST_TEST(unit_is_equal(&aUnit, &Time) == true);
    
    assign_unit(&aUnit, &Angle);
    BOOST_TEST(unit_is_equal(&aUnit, &Angle) == true);

    BOOST_TEST(unit_is_equal(&Length, &Angle) ==false);
    BOOST_TEST(unit_is_equal(&Angle, &Mass) ==false);
    BOOST_TEST(unit_is_equal(&EmptyUnit, &Time) ==false);
    BOOST_TEST(unit_is_equal(&Time, &Length) ==false);

    release_unit(&aUnit);

    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start unit test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   unit test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}

BOOST_AUTO_TEST_CASE(test_unit_op_tester){
    std::cout << "test_unit_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();

    // 여기부터 실제 테스트할 코드를 작성한다.

    Unit aUnitSelf;
    Unit aUnitOther;
    Unit aUnitOrgSelf;

    init_unit(&aUnitSelf);
    init_unit(&aUnitOther);
    init_unit(&aUnitOrgSelf);

    assign_unit(&aUnitOther, &Length);
    
    assign_unit(&aUnitOrgSelf, &aUnitSelf);
    add_unit(&aUnitSelf, &aUnitOther);
    BOOST_TEST(unit_is_equal(&aUnitSelf, &aUnitOrgSelf));

    assign_unit(&aUnitOrgSelf, &aUnitSelf);
    sub_unit(&aUnitSelf, &aUnitOther);
    BOOST_TEST(unit_is_equal(&aUnitSelf, &aUnitOrgSelf));
    DEBUG_PRINT("\n");

    bool aIsUnitEqual;
    for(int i=0; i<10; i++){
        assign_unit(&aUnitOrgSelf, &aUnitSelf);
        if (mul_unit(&aUnitSelf, &aUnitOther) != UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = unit_is_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \n");
                print_unit(&aUnitSelf);

                printf("aUnit Other : \n");
                print_unit(&aUnitOther);

                printf("aUnit OrgSelf : \n");
                print_unit(&aUnitOrgSelf);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 10;
        }
    }
    DEBUG_PRINT("\n");

    for(int i=0; i<20; i++){
        assign_unit(&aUnitOrgSelf, &aUnitSelf);
        if (div_unit(&aUnitSelf, &aUnitOther) != UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = unit_is_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \n");
                print_unit(&aUnitSelf);

                printf("aUnit Other : \n");
                print_unit(&aUnitOther);

                printf("aUnit Org Self : \n");
                print_unit(&aUnitOrgSelf);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 20;
        }
    }
    DEBUG_PRINT("\n");

    release_unit(&aUnitSelf);
    release_unit(&aUnitOther);
    release_unit(&aUnitOrgSelf);
    
    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start unit test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   unit test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}
