/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit
*
***************************************************************************/

#define BOOST_TEST_MODULE si_unit tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "si_unit.h"

void print_si_unit_values(unsigned long tValue){
    SiUnit aSiUnit;

    init_si_unit(&aSiUnit);    
    set_si_unit_bits(&aSiUnit, tValue);
    DEBUG_PRINT("get_unit_length(&get_si_unit_unit(&aSiUnit))       : %3d\n", 
        get_unit_length(&get_si_unit_unit(&aSiUnit)));

    DEBUG_PRINT("get_unit_mass(&get_si_unit_unit(&aSiUnit))         : %3d\n", 
        get_unit_mass(&get_si_unit_unit(&aSiUnit)));        

    DEBUG_PRINT("get_unit_time(&get_si_unit_unit(&aSiUnit))         : %3d\n", 
        get_unit_time(&get_si_unit_unit(&aSiUnit)));

    DEBUG_PRINT("get_unit_angle(&get_si_unit_unit(&aSiUnit))        : %3d\n", 
        get_unit_angle(&get_si_unit_unit(&aSiUnit)));
        
    DEBUG_PRINT("get_si_unit_electric_current(&aSiUnit)             : %3d\n", 
        get_si_unit_electric_current(&aSiUnit));

    DEBUG_PRINT("get_si_unit_thermodynamic_temperature(&aSiUnit)    : %3d\n", 
        get_si_unit_thermodynamic_temperature(&aSiUnit));

    DEBUG_PRINT("get_si_unit_amount_of_substance(&aSiUnit)          : %3d\n", 
        get_si_unit_amount_of_substance(&aSiUnit));

    DEBUG_PRINT("get_si_unit_luminous_intensity(&aSiUnit)           : %3d\n", 
        get_si_unit_luminous_intensity(&aSiUnit));

    release_si_unit(&aSiUnit);
}

BOOST_AUTO_TEST_CASE(test_si_unit_tester){
    std::cout << "test si_unit starts.\n";
    std::cout << "si_unit version is below." << std::endl;
    print_si_unit_version(char* buf);
    BOOST_TEST(get_si_unit_version(char* buf) == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(get_si_unit_version(char* buf) != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);

    for(int i=0; i<20; i++){
        DEBUG_PRINT("m  (%3d)  : %010X :     %16d\n", i, m(i), m(i));
        DEBUG_PRINT("g  (%3d)  : %010X :     %16d\n", i, g(i), g(i));
        DEBUG_PRINT("s  (%3d)  : %010X :     %16d\n", i, s(i), s(i));
        DEBUG_PRINT("rad(%3d)  : %010X :     %16d\n", i, rad(i), rad(i));
        DEBUG_PRINT("A  (%3d)  : %010X :     %16d\n", i, A(i), A(i));
        DEBUG_PRINT("K  (%3d)  : %010X :     %16d\n", i, K(i), K(i));
        DEBUG_PRINT("mol(%3d)  : %010X :     %16d\n", i, mol(i), mol(i));
        DEBUG_PRINT("cd (%3d)  : %010X :     %16d\n", i, cd(i), cd(i));
    }

    for(int i=0; i<20; i++){
        DEBUG_PRINT("m  (%3d) : %010X :      %16d\n", -i, m(-i), m(-i));
        DEBUG_PRINT("g  (%3d) : %010X :      %16d\n", -i, g(-i), g(-i));
        DEBUG_PRINT("s  (%3d) : %010X :      %16d\n", -i, s(-i), s(-i));
        DEBUG_PRINT("rad(%3d) : %010X :      %16d\n", -i, rad(-i), rad(-i));
        DEBUG_PRINT("A  (%3d) : %010X :      %16d\n", -i, A(-i), A(-i));
        DEBUG_PRINT("K  (%3d) : %010X :      %16d\n", -i, K(-i), K(-i));
        DEBUG_PRINT("mol(%3d) : %010X :      %16d\n", -i, mol(-i), mol(-i));
        DEBUG_PRINT("cd (%3d) : %010X :      %16d\n", -i, cd(-i), cd(-i));

        }

    for(int i=0; i<20; i++){
        DEBUG_PRINT("m  (%3d) | m(%3d) : %010X   : %16d\n", i, i, m(i)     | m(i), m(i)     | m(i));
        DEBUG_PRINT("g  (%3d) | m(%3d) : %010X   : %16d\n", i, i, g(i)     | m(i), g(i)     | m(i));
        DEBUG_PRINT("s  (%3d) | m(%3d) : %010X   : %16d\n", i, i, s(i)     | m(i), s(i)     | m(i));
        DEBUG_PRINT("rad(%3d) | m(%3d) : %010X   : %16d\n", i, i, rad(i) | m(i), rad(i) | m(i));
        DEBUG_PRINT("A  (%3d) | m(%3d) : %010X   : %16d\n", i, i, A(i)     | m(i), A(i)     | m(i));
        DEBUG_PRINT("K  (%3d) | m(%3d) : %010X   : %16d\n", i, i, K(i)     | m(i), K(i)     | m(i));
        DEBUG_PRINT("mol(%3d) | m(%3d) : %010X   : %16d\n", i, i, mol(i) | m(i), mol(i) | m(i));
        DEBUG_PRINT("cd (%3d) | m(%3d) : %010X   : %16d\n", i, i, cd(i)   | m(i), cd(i)   | m(i));
    }        

    SiUnit aSiUnit;
    DEBUG_PRINT("SiUnit sizeof(%zd)", sizeof(SiUnit));
    DEBUG_PRINT("si_unit_t sizeof(%zd)", sizeof(si_unit_t));

    init_si_unit(&aSiUnit);
    DEBUG_PRINT("init_si_unit : %10d, %#" PRIx32 "\n", aSiUnit.mBits, 
        aSiUnit.mBits);

    BOOST_TEST(si_unit_symbol(&aSiUnit) == "");

    // Unit aUnit = get_si_unit_unit(&aSiUnit);
    assign_unit(&aSiUnit.mSiUnit_t.mUnit, &Length);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "m");
 
    assign_unit(&aSiUnit.mSiUnit_t.mUnit, &Mass);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "g");
 
    assign_unit(&aSiUnit.mSiUnit_t.mUnit, &Time);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "s");
 
    assign_unit(&aSiUnit.mSiUnit_t.mUnit, &Angle);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "rad");

    assign_si_unit(&aSiUnit, &ElectricCurrent);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "A");
    // DEBUG_PRINT("cd(1) : %10d", cd(1));
    
    assign_si_unit(&aSiUnit, &ThermodynamicTemperature);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "K");

    assign_si_unit(&aSiUnit, &AmountOfSubstance);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "mol");

    assign_si_unit(&aSiUnit, &LuminousIntensity);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "cd");

    // ElectricCharge                  	s*A
    // const SiUnit ElectricCharge                     = {s(1 || A(1))};
    assign_si_unit(&aSiUnit, &ElectricCharge);
    BOOST_TEST(si_unit_symbol(&aSiUnit) == "cd");

    // set_si_unit_bits(&aSiUnit, m(1));
    print_si_unit_values(m(2));
    print_si_unit_values(m(-2));

    print_si_unit_values(m(2) | s(2));
    print_si_unit_values(m(-2) | s(3)  );    

    print_si_unit_values(m(2) | s(2) | g(1));
    print_si_unit_values(m(-2) | s(3) |  g(1));    

    print_si_unit_values(m(2) | s(-2) | g(1));
    print_si_unit_values(m(-2) | s(-3) |  g(1));    

    release_si_unit(&aSiUnit);
}

BOOST_AUTO_TEST_CASE(test_si_unit_op_tester){
    std::cout << "test_unit_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();

    // 여기부터 실제 테스트할 코드를 작성한다.

    SiUnit aUnitSelf;
    SiUnit aUnitOther;
    SiUnit aUnitOrgSelf;

    init_si_unit(&aUnitSelf);
    init_si_unit(&aUnitOther);
    init_si_unit(&aUnitOrgSelf);

    assign_si_unit_unit(&aUnitOther, &Length);
    
    assign_si_unit(&aUnitOrgSelf, &aUnitSelf);
    add_si_unit(&aUnitSelf, &aUnitOther);
    BOOST_TEST(is_si_unit_equal(&aUnitSelf, &aUnitOrgSelf));

    assign_si_unit(&aUnitOrgSelf, &aUnitSelf);
    sub_si_unit(&aUnitSelf, &aUnitOther);
    BOOST_TEST(is_si_unit_equal(&aUnitSelf, &aUnitOrgSelf));
    DEBUG_PRINT("\n");

    bool aIsUnitEqual;
    for(int i=0; i<10; i++){
        assign_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (mul_si_unit(&aUnitSelf, &aUnitOther) != UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \n");
                print_si_unit(&aUnitSelf);

                printf("aUnit Other : \n");
                print_si_unit(&aUnitOther);

                printf("aUnit OrgSelf : \n");
                print_si_unit(&aUnitOrgSelf);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 10;
        }
    }
    DEBUG_PRINT("\n");

    for(int i=0; i<20; i++){
        assign_si_unit(&aUnitOrgSelf, &aUnitSelf);
        if (div_si_unit(&aUnitSelf, &aUnitOther) != UNIT_OP_RESULT_OVERFLOW){
            aIsUnitEqual = is_si_unit_equal(&aUnitSelf, &aUnitOrgSelf);
            BOOST_TEST(aIsUnitEqual);
            if ( aIsUnitEqual == false){
                printf("aUnit Self : \n");
                print_si_unit(&aUnitSelf);

                printf("aUnit Other : \n");
                print_si_unit(&aUnitOther);

                printf("aUnit Org Self : \n");
                print_si_unit(&aUnitOrgSelf);
            }
        }
        else{
            DEBUG_PRINT("error!, UNIT_OP_RESULT_OVERFLOW at i == %d\n",i);
            i = 20;
        }
    }
    DEBUG_PRINT("\n");

    release_si_unit(&aUnitSelf);
    release_si_unit(&aUnitOther);
    release_si_unit(&aUnitOrgSelf);
    
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
