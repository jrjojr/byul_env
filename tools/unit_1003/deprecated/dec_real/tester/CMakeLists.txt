############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# dec_real 1.0.2.2
#
###########################################################################//

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.15)

# 프로젝트명 : 실행파일 이름이 된다.
project(dec_real_tester
  VERSION 1.0.2.2
)

list(APPEND CMAKE_MODULE_PATH "C:/Users/jrjo/Documents/cpp/unit/cmake_modules")
include(starter)

startTester()

  # addSrcs(
    # *.cpp 추가
  # )
  
  # addHeaders( 
    # *.h 추가 
  # )

  addIncDir("C:/Users/jrjo/apps/local/include")
  addIncDir("C:/Users/jrjo/apps/local/bin/include")

  addLibDir("C:/Users/jrjo/apps/local/lib")
  addLibDir("C:/Users/jrjo/apps/local/bin/libs")

  # addCommandWhenBuildExcute( 
    # echo "dec_real BUILD_TYPE is $<CONFIG>" )
  # addCommandWhenBuildExcute(  
    # echo "dec_real PLATFORM_ID is $<PLATFORM_ID>" )

endTester()

# 각종 출력물들은 여기서 실행해야 오류 발생이 적다.
# printSrcs()
# printIncDirs()
# printHeaders()
# printLibDirs()
# printLibs()
