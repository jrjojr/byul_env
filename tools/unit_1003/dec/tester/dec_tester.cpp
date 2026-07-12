/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec 
*
***************************************************************************/
#define BOOST_TEST_MODULE dec tester
#include <boost/test/included/unit_test.hpp>
#include "dec_tester.hpp"

#include "dec.h"

#include <sstream>

namespace ut = boost::unit_test;

BOOST_AUTO_TEST_CASE(dec_tester_first, 
    * ut::fixture<Stopwatch>(std::string("dec_tester_first"))
    * ut::fixture<Logger>(std::string("dec_tester_first"))
    ){

    // 여기부터 실제 테스트할 코드를 작성한다.

    std::cout << "dec version is below." << std::endl;

    std::stringstream version;
    char buf[16];
    version << DEC_VERSION_MAJOR << "." ;
    version << DEC_VERSION_MINOR << "." ;
    version << DEC_VERSION_PATCH << "." ;
    version << DEC_VERSION_TWEAK;

    BOOST_TEST(dec_version(buf) == version.str());

    // 여기까지 실제 테스트할 코드를 작성한다.
}
