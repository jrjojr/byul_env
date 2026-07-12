############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# real
#
###########################################################################//

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.21)

# 프로젝트명 : 실행파일 이름이 된다.
project(real_tester
VERSION 1.0.0.3
)

file(REAL_PATH "~/apps/cmake_starter" ROOT EXPAND_TILDE)
list(APPEND CMAKE_MODULE_PATH "${ROOT}")
include(starter)

startTester()

  # addSrcs(
    # *.cpp 추가
  # )
  
  # addHeaders( 
    # *.h 추가 
  # )

  # addIncDir("C:/Users/jrjo/apps/local/include")

  # addLibDir("C:/Users/jrjo/apps/local/lib")

  # addCommandWhenBuildExcute(
  #     echo "${{PROJECT_NAME}} BUILD_TYPE is $<CONFIG>" )
  # addCommandWhenBuildExcute('
  #     echo "${{PROJECT_NAME}} PLATFORM_ID is $<PLATFORM_ID>" )

endTester()

# 각종 출력물들은 여기서 실행해야 오류 발생이 적다.
# 빌드 전의 파일 들을 출력한다.
# 빌드 시에 생기는 변화는 감지 못한다.
# printSrcs()
# printHeaders()

# printSubs()
# printIncDirs()
# printLibDirs()
# printLibs()

