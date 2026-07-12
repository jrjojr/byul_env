/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_basic_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE si_basic_unit tester
#include <boost/test/included/unit_test.hpp>

#include <iostream>

#include "si_basic_unit.h"

BOOST_AUTO_TEST_CASE(test_si_basic_unit_tester){
    std::cout << "test si_basic_unit starts.\n";
    std::cout << "si_basic_unit version is below." << std::endl;
    print_si_basic_unit_version(char* buf);
    BOOST_TEST(get_si_basic_unit_version(char* buf) == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    // BOOST_TEST(get_version() != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);

    SiBasicUnit aBasicUnit;

    init_si_basic_unit(&aBasicUnit);
    
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &EmptyUnit));
    assign_si_basic_unit(&aBasicUnit, &Length);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &Length));
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "length");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "m");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "metre");


    assign_si_basic_unit(&aBasicUnit, &Mass);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &Mass));
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "mass");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "g");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "gram");

    assign_si_basic_unit(&aBasicUnit, &Time);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &Time) == true);
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "time");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "s");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "second");
    
    assign_si_basic_unit(&aBasicUnit, &ElectricCurrent);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &ElectricCurrent) == true);
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "electric current");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "A");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "ampere");

    assign_si_basic_unit(&aBasicUnit, &ThermodynamicTemperature);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &ThermodynamicTemperature) == true);
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "thermodynamic temperature");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "K");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "kelvin");

    assign_si_basic_unit(&aBasicUnit, &LuminousIntensity);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &LuminousIntensity) == true);
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "luminous intensity");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "cd");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "candela");

    assign_si_basic_unit(&aBasicUnit, &AmountOfSubstance);
    BOOST_TEST(is_si_basic_unit_equal(&aBasicUnit, &AmountOfSubstance) == true);
    BOOST_TEST(get_si_basic_unit_quantity_name(&aBasicUnit) == "amount of substance");
    BOOST_TEST(get_si_basic_unit_symbol(&aBasicUnit) == "mol");
    BOOST_TEST(get_si_basic_unit_name(&aBasicUnit) == "mole");

    BOOST_TEST(is_si_basic_unit_equal(&Length, &AmountOfSubstance) ==false);
    BOOST_TEST(is_si_basic_unit_equal(&ElectricCurrent, &AmountOfSubstance) ==false);
    BOOST_TEST(is_si_basic_unit_equal(&EmptyUnit, &ElectricCurrent) ==false);
    BOOST_TEST(is_si_basic_unit_equal(&LuminousIntensity, &ElectricCurrent) ==true);

    BOOST_TEST(is_si_basic_unit_equal(&Length, &AmountOfSubstance));
    BOOST_TEST(is_si_basic_unit_equal(&ElectricCurrent, &AmountOfSubstance));
    BOOST_TEST(is_si_basic_unit_equal(&EmptyUnit, &ElectricCurrent));    

    release_si_basic_unit(&aBasicUnit);
}

