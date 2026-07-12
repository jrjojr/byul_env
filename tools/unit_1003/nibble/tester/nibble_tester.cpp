/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* nibble
*
***************************************************************************/

#define BOOST_TEST_MODULE nibble tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>

#include "nibble.h"

void print_nibble8_value(unsigned long tBits){
    Nibble8 aNibble8;

    init_nibble8(&aNibble8);    
    set_nibble8(&aNibble8, tBits);

    long aSeven =   get_nibble8_seven(&aNibble8);
    long aSix =     get_nibble8_six(&aNibble8);
    long aFive =    get_nibble8_five(&aNibble8);
    long aFour =    get_nibble8_four(&aNibble8);
    long aThree =   get_nibble8_three(&aNibble8);
    long aTwo =     get_nibble8_two(&aNibble8);
    long aOne =     get_nibble8_one(&aNibble8);
    long aZero =    get_nibble8_zero(&aNibble8);

    DEBUG_PRINT("seven  six five    four    three   two one zero\n");
    DEBUG_PRINT("   %d   %d   %d      %d       %d    %d  %d   %d\n",
                aSeven, aSix,aFive, aFour, aThree, aTwo, aOne, aZero);

    release_nibble8(&aNibble8);
}

BOOST_AUTO_TEST_CASE(test_nibble_tester){
    boost::posix_time::ptime datetime;
    datetime = boost::posix_time::microsec_clock::local_time();

    std::cout << "Current Time and Date: " << datetime << std::endl;    
    std::cout << "test nibble starts.\n";
    
    std::cout << "nibble version is below." << std::endl;

    for(int i=0; i<20; i++){
    DEBUG_PRINT("NIBBLE0(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE0(i)  | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE1(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE1(i)    | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE2(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE2(i)    | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE3(%d) \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE3(i)| NIBBLE7(i));
    DEBUG_PRINT("NIBBLE4(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE4(i)  | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE5(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE5(i)  | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE6(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE6(i)    | NIBBLE7(i));
    DEBUG_PRINT("NIBBLE7(%d) \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE7(i)| NIBBLE7(i));
    }        

    for(int i=0; i<20; i++){
    DEBUG_PRINT("NIBBLE0(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE0(-i)  | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE1(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE1(-i)    | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE2(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE2(-i)    | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE3(%d) \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE3(-i)| NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE4(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE4(-i)  | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE5(%d)  \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE5(-i)  | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE6(%d)   \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE6(-i)    | NIBBLE7(-i));
    DEBUG_PRINT("NIBBLE7(%d) \t| NIBBLE7(%d) \t: %X\n", i, i, NIBBLE7(-i)| NIBBLE7(-i));
    }        
    // #endif // DEBUG

    Nibble8 aNibble8;
    DEBUG_PRINT("Nibble8 sizeof(%zd)\n", sizeof(Nibble8));
    DEBUG_PRINT("nibble8_t sizeof(%zd)\n", sizeof(nibble8_t));

    init_nibble8(&aNibble8);
    BOOST_TEST(aNibble8.mBits == 0);

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE0(i));
        BOOST_TEST(get_nibble8_zero(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE0(%d) \t:\t %X\n", i, NIBBLE0(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE1(i));
        BOOST_TEST(get_nibble8_one(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE1(%d) \t:\t %X\n", i, NIBBLE1(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE2(i));
        BOOST_TEST(get_nibble8_two(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE2(%d) \t:\t %X\n", i, NIBBLE2(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE3(i));
        BOOST_TEST(get_nibble8_three(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE3(%d) \t:\t %X\n", i, NIBBLE3(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE4(i));
        BOOST_TEST(get_nibble8_four(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE4(%d) \t:\t %X\n", i, NIBBLE4(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE5(i));
        BOOST_TEST(get_nibble8_five(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE5(%d) \t:\t %X\n", i, NIBBLE5(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE6(i));
        BOOST_TEST(get_nibble8_six(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE6(%d) \t:\t %X\n", i, NIBBLE6(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE7(i));
        BOOST_TEST(get_nibble8_seven(&aNibble8) == i);

        DEBUG_PRINT("NIBBLE7(%d) \t:\t %X\n", i, NIBBLE7(i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE0(-i));
        BOOST_TEST(get_nibble8_zero(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE0(%d) \t:\t %X\n", -i, NIBBLE0(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }    


    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE1(-i));
        BOOST_TEST(get_nibble8_one(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE1(%d) \t:\t %X\n", -i, NIBBLE1(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }        

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE2(-i));
        BOOST_TEST(get_nibble8_two(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE2(%d) \t:\t %X\n", -i, NIBBLE2(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }                

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE3(-i));
        BOOST_TEST(get_nibble8_three(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE3(%d) \t:\t %X\n", -i, NIBBLE3(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }            

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE4(-i));
        BOOST_TEST(get_nibble8_four(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE4(%d) \t:\t %X\n", -i, NIBBLE4(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }            

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE5(-i));
        BOOST_TEST(get_nibble8_five(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE5(%d) \t:\t %X\n", -i, NIBBLE5(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }                

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE6(-i));
        BOOST_TEST(get_nibble8_six(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE6(%d) \t:\t %X\n", -i, NIBBLE6(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }

    for(int i=0; i<8; i++){
        set_nibble8(&aNibble8, NIBBLE7(-i));
        BOOST_TEST(get_nibble8_seven(&aNibble8) == -i);        

        DEBUG_PRINT("NIBBLE7(%d) \t:\t %X\n", -i, NIBBLE7(-i));
        DEBUG_PRINT("get_nibble8(&aNibble8) : %X\n", get_nibble8(&aNibble8));
        print_nibble8_value(get_nibble8(&aNibble8));
    }    


    release_nibble8(&aNibble8);    
}
