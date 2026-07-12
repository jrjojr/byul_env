/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_coherent_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE si_coherent_unit tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "si_coherent_unit.h"

BOOST_AUTO_TEST_CASE(test_si_coherent_unit_tester){
    boost::posix_time::ptime datetime;
    datetime = boost::posix_time::microsec_clock::local_time();

    std::cout << "Current Time and Date: " << datetime << std::endl;    
    std::cout << "test si_coherent_unit starts.\n";
    
    std::cout << "si_coherent_unit version is below." << std::endl;
    print_si_coherent_unit_version(char* buf);
    BOOST_TEST(get_si_coherent_unit_version(char* buf) == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(get_si_coherent_unit_version(char* buf) != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}
