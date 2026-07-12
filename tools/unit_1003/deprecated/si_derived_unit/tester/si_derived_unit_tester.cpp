/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_derived_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE si_derived_unit tester
#include <boost/test/included/unit_test.hpp>

#include <iostream>

#include "si_derived_unit.h"

BOOST_AUTO_TEST_CASE(test_si_derived_unit_tester){
    std::cout << "test si_derived_unit starts.\n";
    std::cout << "si_derived_unit version is below." << std::endl;
    print_si_derived_unit_version(char* buf);
    BOOST_TEST(get_si_derived_unit_version(char* buf) == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(get_si_derived_unit_version(char* buf) != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}
