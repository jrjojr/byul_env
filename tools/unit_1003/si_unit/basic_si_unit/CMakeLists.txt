############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# basic_si_unit
#
###########################################################################//

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.15)

# 프로젝트명 : 실행파일 이름이 된다.
project(basic_si_unit
  VERSION 1.0.0.3
)

include(CMakeDependentOption)
list(APPEND CMAKE_MODULE_PATH "C:/Users/jrjo/Documents/cpp/unit/cmake_modules")
include(starter)

# 만약, 옵션을 바꾸면, 빌드 디렉토리를 삭제해야한다.
# 내용이 바뀌지 않는다. 빌드 디렉토리의 build/CMakeCache.txt내용이...
set(basic_si_unit_PROJECT_TYPE lib 
  CACHE STRING 
  "project type : cpp | c | lib | dll | flex | bison | flex_main | bison_main" 
  FORCE 
  )

  cmake_dependent_option(basic_si_unit_BUILD_TESTER "build tester exe" ON 
    "NOT HAS_MAIN" OFF)

start_project()

  create_base()
  
  # 소스 추가
  # addSrcs(
    # *.c or *.cpp
  # )  

  # 헤더 추가    
  # addHeaders(
    # *.h or *.hpp
  # )

  # addIncDir("C:/Users/jrjo/apps/local/include")
  # addIncDir("C:/Users/jrjo/apps/local/bin/include")

  # addLibDir("C:/Users/jrjo/apps/local/lib")
  # addLibDir("C:/Users/jrjo/apps/local/bin/libs")

  # addCommandWhenBuildExcute( 
    # echo "basic_si_unitBUILD_TYPE is $<CONFIG>" )
  # addCommandWhenBuildExcute(  
    # echo "basic_si_unitPLATFORM_ID is $<PLATFORM_ID>" )

end_project()

# addSub(subProject)
# add_subdirectory(subOther)

# 각종 출력물들은 여기서 실행해야 오류 발생이 적다.
# 빌드 전의 파일 들을 출력한다.
# 빌드 시에 생기는 변화는 감지 못한다.
# printSrcs()
# printIncDirs()
# printHeaders()
# printLibDirs()
# printLibs()
