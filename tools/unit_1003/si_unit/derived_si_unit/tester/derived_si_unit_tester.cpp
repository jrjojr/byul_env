/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* derived_si_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE derived_si_unit tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "derived_si_unit.h"

BOOST_AUTO_TEST_CASE(test_derived_si_unit_tester){
    std::cout << "test_derived_si_unit_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "derived_si_unit version is below." << std::endl;

    // 여기부터 실제 테스트할 코드를 작성한다.
    DerivedSiUnit aUnitSelf;
    DerivedSiUnit aUnitOther;
    DerivedSiUnit aUnitOrgSelf;

    SiUnitPrefixList aGetPrefixList;
    SiUnit aGetSiUnit;

    SiUnit aSiUnit;
    init_si_unit(&aSiUnit);
    assign_si_unit_unit(&aSiUnit, &Power);

    SiUnitPrefixList aPrefixList;
    init_si_prefix_list(&aPrefixList);

    init_derived_si_unit(&aUnitSelf);
    SiUnit aSiUnitFromSelfDerived = get_derived_si_unit_si_unit(&aUnitSelf);
    Unit aUnitFromSelfDerived = get_si_unit_unit(&aSiUnitFromSelfDerived);

    SiUnitPrefixList aPrefListFromDerived = 
        get_derived_si_prefix_list(&aUnitSelf);

    BOOST_TEST(unit_is_equal(&aUnitFromSelfDerived,&EmptyUnit));

    BOOST_TEST(
        is_si_prefix_list_equal(&aPrefListFromDerived, &aPrefixList) );

    assign_derived_si_unit_unit(&aUnitOther, &Force);

    assign_derived_si_unit(&aUnitSelf, &aUnitOther);
    aUnitFromSelfDerived = get_si_unit_unit(&aSiUnitFromSelfDerived);
    
    BOOST_TEST( unit_is_equal(&aUnitFromSelfDerived, 
        &get_si_unit_unit(&aSiUnitFromSelfDerived) ) );

    set_derived_si_prefix_list(&aUnitSelf, &aPrefixList);

    BOOST_TEST( is_si_prefix_list_equal( 
        &get_derived_si_prefix_list(&aUnitSelf), &aPrefixList) );

    set_derived_si_unit_si_unit(&aUnitSelf, &aSiUnit);
    BOOST_TEST(is_si_unit_equal(
        &get_derived_si_unit_si_unit(&aUnitSelf), &aSiUnit));

    assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
    add_derived_si_unit(&aUnitSelf, &aUnitOther);

    SiUnit aSiUnitSelf, aSiUnitOther;
    aSiUnitSelf = get_derived_si_unit_si_unit(&aUnitSelf);
    aSiUnitOther = get_derived_si_unit_si_unit(&aUnitOther);
    add_si_unit(&aSiUnitSelf, &aSiUnitOther);
    BOOST_TEST(is_si_unit_equal(
        &get_derived_si_unit_si_unit(&aUnitSelf), &aSiUnitSelf)  );

    bool aIsUnitEqual;

    for(int i=0; i<10; i++){
        assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (add_derived_si_unit(&aUnitSelf, &aUnitOther) != 
            UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_derived_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \t\t");
                print_si_unit(&aUnitSelf.mSiUnit);

                printf("aUnit Other : \t\t");
                print_si_unit(&aUnitOther.mSiUnit);

                printf("aUnit OrgSelf : \t");
                print_si_unit(&aUnitOrgSelf.mSiUnit);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 10;
        }
    }
    DEBUG_PRINT("\n");

    for(int i=0; i<10; i++){
        assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (sub_derived_si_unit(&aUnitSelf, &aUnitOther) != 
            UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_derived_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \t\t");
                print_si_unit(&aUnitSelf.mSiUnit);

                printf("aUnit Other : \t\t");
                print_si_unit(&aUnitOther.mSiUnit);

                printf("aUnit OrgSelf : \t");
                print_si_unit(&aUnitOrgSelf.mSiUnit);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 10;
        }
    }
    DEBUG_PRINT("\n");    

    for(int i=0; i<10; i++){
        assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (mul_derived_si_unit(&aUnitSelf, &aUnitOther) != 
            UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_derived_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \t\t");
                print_si_unit(&aUnitSelf.mSiUnit);

                printf("aUnit Other : \t\t");
                print_si_unit(&aUnitOther.mSiUnit);

                printf("aUnit OrgSelf : \t");
                print_si_unit(&aUnitOrgSelf.mSiUnit);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 10;
        }
    }
    DEBUG_PRINT("\n");

    for(int i=0; i<20; i++){
        assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (div_derived_si_unit(&aUnitSelf, &aUnitOther) != 
            UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_derived_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \t\t");
                print_si_unit(&aUnitSelf.mSiUnit);

                printf("aUnit Other : \t\t");
                print_si_unit(&aUnitOther.mSiUnit);

                printf("aUnit Org Self : \t");
                print_si_unit(&aUnitOrgSelf.mSiUnit);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 20;
        }
    }
    DEBUG_PRINT("\n");        

    assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
    sub_derived_si_unit(&aUnitSelf, &aUnitOther);
    aSiUnitSelf = get_derived_si_unit_si_unit(&aUnitSelf);
    aSiUnitOther = get_derived_si_unit_si_unit(&aUnitOther);
    sub_si_unit(&aSiUnitSelf, &aSiUnitOther);
    BOOST_TEST(is_si_unit_equal(
        &get_derived_si_unit_si_unit(&aUnitSelf), &aSiUnitSelf)  );

    assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
    mul_derived_si_unit(&aUnitSelf, &aUnitOther);
    aSiUnitSelf = get_derived_si_unit_si_unit(&aUnitSelf);
    aSiUnitOther = get_derived_si_unit_si_unit(&aUnitOther);
    mul_si_unit(&aSiUnitSelf, &aSiUnitOther);
    BOOST_TEST(is_si_unit_equal(
        &get_derived_si_unit_si_unit(&aUnitSelf), &aSiUnitSelf)  );

    assign_derived_si_unit(&aUnitOrgSelf, &aUnitSelf);
    div_derived_si_unit(&aUnitSelf, &aUnitOther);
    aSiUnitSelf = get_derived_si_unit_si_unit(&aUnitSelf);
    aSiUnitOther = get_derived_si_unit_si_unit(&aUnitOther);
    div_si_unit(&aSiUnitSelf, &aSiUnitOther);
    BOOST_TEST(is_si_unit_equal(
        &get_derived_si_unit_si_unit(&aUnitSelf), &aSiUnitSelf)  );

    release_derived_si_unit(&aUnitSelf);
    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start derived_si_unit test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   derived_si_unit test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}

