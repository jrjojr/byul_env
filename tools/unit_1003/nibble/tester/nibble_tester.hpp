/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* nibble 
*
***************************************************************************/

#ifndef NIBBLE_HPP
#define NIBBLE_HPP

#define BOOST_TEST_MODULE nibble tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <string>

// 테스트 진행률을 출력한다.
// 테스트에 너무 많은 시간이 걸릴 경우 어디까지 진행 되었는지 알수 있다.
// total count를 exp count만큼 반복한다.
// 맥스 값은 UINT_MAX * 10^UINT_MAX
// 999999를 한 줄로 만드는게 가장 보기 좋다.
// 타이밍이 딱 맞네.
// 999 999 보다 크면 다음 라인으로 넘겨야 한다.
// 지수를 1개 올린다고 보는 것이 맞겠다.
// 루프가 생기면 3차원이나 4차원 또는 더 큰 차원이 생긴다.
// 1차원은 999 999를 고정하고,
// 나머지 고차원들은 999 999 까지 고정하고,
// 그거 반복한다.
// 1차원은 999 999 이다.
class Progress{
public:
    Progress(unsigned int tTotalCount=10000,
        unsigned int tTotalExpCount=100,
        std::string tName="nibble_progress",
        char tFigure = '#'
        ){
            mName = tName;
            mTotalCount = tTotalCount;
            mTotalExpCount = tTotalExpCount;
            mCount = 0;
            mExpCount = 0;
            mFigure = tFigure;
            mOffset = tTotalCount / 50;
            mOffsetCount = 0;
            std::cout << mName << " total " << mTotalCount <<
                " * 10^" << mTotalExpCount << " progress begin.\n";
        }
    virtual ~Progress(){
        // 메모리 할당 된 게 있으면 해제한다.
            std::cout << mName << " total " << mTotalCount <<
                " * 10^" << mTotalExpCount << " progress end.\n";
    }

    // 하나의 과정이 끝날때 호출한다.
    // 콘솔에 진행률이  출력된다.
    void tick(unsigned int tStep=1){
        // 현재카운트/총카운트 * 50 만큼 mFigure를 출력한다.
        // print_figure(50 * mCount/mTotalCount);

        // 현재 카운트에 스텝을 덧셈한다.
        mCount += tStep;

        // 현재 오프셋 카운트에 스텝을 덧셈한다.
        mOffsetCount += tStep;

        // if (mOffsetCount > mOffset){
        //     // 옵셋 카운트와 현재 옵셋의 차이를 확인한다.
        //     // 차이를 옵셋으로 나눠서 나온 정수 만큼 피겨를 출력한다.
        //     int aDiff = mOffsetCount - mOffset;
        //     aDiff /= mOffset;
        //     if (aDiff <= 0){
        //         aDiff = 1;
        //     }
        //     for(int i=0; i<aDiff; i++){
        //         std::cout << mFigure;
        //     }
        //     mOffsetCount = 0;
        // }
        if (mOffsetCount > mOffset){
            std::cout << mFigure;
            mOffsetCount = 0;

            // uint64_t aTotalCount = (uint64_t)pow(mTotalCount, mTotalExpCount);
            if ( mCount > mTotalCount ){
                // 현재 카운트가 맥스에 도달하면,
                // completed_one_exp를 호출한다.
                completed_one_exp();
                // 현재 카운트를 리셋한다.
                mCount = 0;

            }
        }
    }

private:
    std::string mName;
    unsigned int mTotalCount;
    unsigned int mTotalExpCount;
    unsigned int mCount;
    unsigned int mExpCount;
    char mFigure;
    unsigned int mOffset;
    unsigned int mOffsetCount;

    void completed_one_exp(){
        // 현재지수/총지수 * 100 만큼 완료 되었다고 출력한다.
        // 10 % 완료, or 10 % completed.
        std::cout << "\t\t" << 100 * mExpCount/mTotalExpCount << 
            " % completed\n";

        // 지수를 올린다.
        mExpCount++;     
        if (mExpCount > mTotalExpCount){
            // 진행이 완료되었다.
            std::cout << "all completed\n";
        }
   
    }

    // void print_figure(int tCount){
        // figure를 카운트만큼 반복출력한다. 
    // }
};

// 테스트 시작과 끝 시간과 경과 시간을 출력한다.
struct Stopwatch{
    Stopwatch(std::string tName="nibble_stopwatch"){
        std::cout << tName << " begin.\n";
        mStartTime = boost::posix_time::microsec_clock::local_time();      
        mName = tName;  
    }
    ~Stopwatch(){
        mEndTime = boost::posix_time::microsec_clock::local_time();
        std::cout << mName <<" start Time and Date : ";
        std::cout << mStartTime<<std::endl;
        std::cout << mName << " end Time and Date : ";
        std::cout << mEndTime<<std::endl;    
        std::cout << "elapsed time : ";
        std::cout << mEndTime - mStartTime << std::endl;    
        std::cout << mName <<" end.\n";
        std::cout << std::endl; 
    }
    boost::posix_time::ptime mStartTime;
    boost::posix_time::ptime mEndTime;    
    std::string mName;

  void setup(){ 
    // BOOST_TEST_MESSAGE("optional setup " << mName); 
    std::cout << "optional setup " << mName << std::endl;
  }

  void teardown(){ 
    // BOOST_TEST_MESSAGE("optional teardown " << mName); 
    std::cout << "optional teardown " << mName << std::endl;
  }
};

// /***************************************************************************
// Logger fixture는 테스트의 결과를 
// stdout이 아니라 로그파일로 저장하기 위한 코드이다
// 테스트 결과가 너무 많아서 stdout에서 잘리는 경우에 사용하면 아주 유용하다.
struct Logger{
    Logger(std::string tName="nibble_logger") : 
        test_log(tName+".log"){

        boost::unit_test::unit_test_log.set_stream(test_log);
    }
    ~Logger(){
        boost::unit_test::unit_test_log.set_stream(std::cout);
    }
    std::ofstream test_log;
};

// Logger Fixture를 글로발로 설정해야 하나의 파일에 모든 테스트의 결과가 저장된다.
// Logger Fixture를 각각의 테스트 케이스에 설정할 수도 있으나,
// 각각의 테스트를 실행시마다 로그 파일이 덮어쓰기 된다.
// 방지하려면, 로거의 이를을 해당 테스트 케이스마다 다르게 설정해야 한다.
BOOST_TEST_GLOBAL_CONFIGURATION(Logger);
// ***************************************************************************/
#endif // NIBBLE_HPP
