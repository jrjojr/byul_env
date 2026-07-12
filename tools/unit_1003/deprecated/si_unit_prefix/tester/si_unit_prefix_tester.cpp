/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit_prefix 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#define BOOST_TEST_MODULE si_unit_prefix tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "si_unit_prefix.h"

BOOST_AUTO_TEST_CASE(test_si_unit_prefix_tester){
    std::cout << "test_si_unit_prefix_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "si_unit_prefix version is below." << std::endl;
    print_si_unit_prefix_version();
    BOOST_TEST(si_unit_prefix_version() == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(si_unit_prefix_version() != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);

    // 여기부터 실제 테스트할 코드를 작성한다.


    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start si_unit_prefix test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   si_unit_prefix test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}
