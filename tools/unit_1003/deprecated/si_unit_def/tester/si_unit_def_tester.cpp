/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit_def 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE si_unit_def tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "si_unit_def.h"

BOOST_AUTO_TEST_CASE(test_si_unit_def_tester){
    boost::posix_time::ptime datetime;
    datetime = boost::posix_time::microsec_clock::local_time();

    std::cout << "Current Time and Date: " << datetime << std::endl;    
    std::cout << "test si_unit_def starts.\n";
    
    std::cout << "si_unit_def version is below." << std::endl;
    print_si_unit_def_version();
    BOOST_TEST(get_si_unit_def_version() == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(get_si_unit_def_version() != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}
