# ###########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com>
# All Rights Reserved.
#
# ustring CMakeLists.txt
#
# ###########################################################################

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.21)

# 프로젝트명 : 실행파일 이름이 된다.
project(ustring
  VERSION 1.0.0.3
)

file(REAL_PATH "~/apps/cmake_starter" ROOT EXPAND_TILDE)
list(APPEND CMAKE_MODULE_PATH "${ROOT}")
include(starter)

# tester project를 사용한다.
# 만약 프로젝트가 main을 갖고 있으면,
# 옵션은 무시된다. 따라서 tester 프로젝트는 생성되지 않는다.
option(${PROJECT_NAME}_USE_TESTER "use tester" ON)
                    
# 아래 중에 하나를 선택한다.
# lib , dll , flex , bison , libpp , dllpp,
# main_cpp , main_c , main_flex , main_bison
start_project(lib)

  # 프로젝트에 기본적으로 필요한 헤더, 소스, 기타 등등의 파일을 생성한다.
  # 필요할 때만 호출한다.
  create_base()
  
  # 소스 추가
  # addSrcs(
  #  quantity.cpp
  # )

  # 헤더 추가
  # addHeaders(
  #   quantity.h
  # )

  # addIncDir("C:/Users/jrjo/apps/local/include")
  # addIncDir("C:/Users/jrjo/apps/local/bin/include")
    
  # addLibDir("C:/Users/jrjo/apps/local/lib")
  # addLibDir("C:/Users/jrjo/apps/local/bin/libs")

  # addCommandWhenBuildExcute(
  #   echo "${PROJECT_NAME}_BUILD_TYPE is $<CONFIG> )
  # addCommandWhenBuildExcute(  
  #   echo "${PROJECT_NAME}_PLATFORM_ID is $<PLATFORM_ID> )

  # addSub는 do_project 전에 있어야 한다.
  # addSub의 순서는 중요하다.
  # sub는 전에 추가된 sub만 인식한다.

  # addSub(prev_project)

  # 서브와 현재 프로젝트의 빌드 우선 순위를 결정한다.
  do_project()

  # do_project() 이 후에 추가된 서브들의 디렉토리들은
  # 현재 프로젝트가 빌드 되고 나서 빌드해야 하는 의존성이 있다.

  # addSub(next_project)

# 프로젝트가 끝난다.
end_project()

# 각종 출력물들은 여기서 실행해야 오류 발생이 적다.
# 빌드 전의 파일 들을 출력한다.
# 빌드 시에 생기는 변화는 감지 못한다.
# printSrcs()
# printHeaders()

# printSubs()
# printIncDirs()
# printLibDirs()
# printLibs()

