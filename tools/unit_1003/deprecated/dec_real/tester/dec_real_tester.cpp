/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* dec_real 
*
***************************************************************************/

#define BOOST_TEST_MODULE dec_real tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "dec_real.h"

BOOST_AUTO_TEST_CASE(test_dec_real_tester){
    std::cout << "test_dec_real_tester starts.\n";
    boost::posix_time::ptime aStartTime;
    boost::posix_time::ptime aEndTime;
    aStartTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "dec_real version is below." << std::endl;
    dec_real_print_version();
    BOOST_TEST(dec_real_version(char* buf) == 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
    BOOST_TEST(dec_real_version(char* buf) != 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);

    // 여기부터 실제 테스트할 코드를 작성한다.
    DBL_DEC aResult = 0;
    DEC a = 0;
    DEC b = 0;
    DEC retIntPart;
    DEC retFracPart;

    for (int i=1; i<11; i++){
        for (int j=1; j<11; j++){
            div_dec_to(i, j, &retIntPart, &retFracPart);        
            DEBUG_PRINT("a : %d, b : %d, retInt : %d, retFrac : %d\n",
                i, j, retIntPart, retFracPart);
        }
    }
    DEBUG_PRINT("\n");

    for (int i=1; i<11; i++){
        for (int j=1; j<11; j++){
            div_dec_to(-i, j, &retIntPart, &retFracPart);        
            DEBUG_PRINT("a : %d, b : %d, retInt : %d, retFrac : %d\n",
                -i, j, retIntPart, retFracPart);
        }
    }    
    DEBUG_PRINT("\n");

    for (int i=1; i<11; i++){
        for (int j=1; j<11; j++){
            div_dec_to(i, -j, &retIntPart, &retFracPart);        
            DEBUG_PRINT("a : %d, b : %d, retInt : %d, retFrac : %d\n",
                i, -j, retIntPart, retFracPart);
        }
    }    
    DEBUG_PRINT("\n");    

    for (int i=1; i<11; i++){
        for (int j=1; j<11; j++){
            div_dec_to(-i, -j, &retIntPart, &retFracPart);        
            DEBUG_PRINT("a : %d, b : %d, retInt : %d, retFrac : %d\n",
                -i, -j, retIntPart, retFracPart);
        }
    }    

    DEBUG_PRINT("\n");        

    a = -999999999;
    b = -999999999;
    DEBUG_PRINT("-999999999 * -999999999 = %d\n", a* b);

    a = 999999999;
    b = -999999999;
    DEBUG_PRINT("999999999 * -999999999 = %d\n", a* b);    

    a = -999999999;
    b = 999999999;
    DEBUG_PRINT("-999999999 * 999999999 = %d\n", a* b);    

    a = 999999999;
    b = 999999999;
    DEBUG_PRINT("999999999 * 999999999 = %d\n", a* b);        

    // 여기까지 실제 테스트할 코드를 작성한다.

    aEndTime = boost::posix_time::microsec_clock::local_time();
    std::cout << "start dec_real test Time and Date: ";
    std::cout << aStartTime<<std::endl;
    std::cout << "end   dec_real test Time and Date: ";
    std::cout << aEndTime<<std::endl;    
    std::cout << "total elapsed time : ";
    std::cout << aEndTime - aStartTime << std::endl;    
    std::cout << std::endl; 
}
