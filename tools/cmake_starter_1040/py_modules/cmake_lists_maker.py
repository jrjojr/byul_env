'''##########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
# 
# cmake_lists_maker
# 
# CMakeLists.txt 을 생성한다.
# 
##########################################################################'''

import argparse
from pathlib import Path

def make_version_string(tVersion:int=1000)->str:
    aStr = ''

    if(tVersion < 0):
        tVersion *= -1
        pass

    aVersion = str(tVersion)
    aLenVersion = len(aVersion)
    if (aLenVersion == 1):
        aStr = aVersion
        pass
    elif(aLenVersion > 4):
        aStartPos = aLenVersion - 3
        aStr = aVersion[:aStartPos]
        aStr += '.'
        for i in range(aStartPos, aLenVersion-1):
            aStr += aVersion[i]
            aStr += '.'
            pass
        aStr += aVersion[aLenVersion-1]
        pass
    else:
        for i in range(aLenVersion-1):
            aStr += aVersion[i]
            aStr += '.'
            pass
        aStr += aVersion[aLenVersion-1]        
        pass

    return aStr

def make(tName:str='new_project', 
         tVersionMajor:int=1,
         tVersionMinor:int=0,
         tVersionPatch:int=0,
         tVersionTweak:int=0,
         tCMakeStarterDir:str='~/apps/opt/cmake_starter',
       tProject:str='lib', tOnBase:bool=False, tOnTester:bool=False,
       tRelPwd='./', tForce:bool=False):
    """CMakeLists.txt를 생성한다.

    Args:
        tName (str, optional): PROJECT_NAME을 설정한다. 
            Defaults to 'new_project'.
        tVersionMajor (int, optional): 메이저 버전을 설정한다. 
            Defaults to 1
        tVersionMinor (int, optional): 마이너 버전을 설정한다. 
            Defaults to 0
        tVersionPatch (int, optional): 패치 버전을 설정한다. 
            Defaults to 0
        tVersionTweak (int, optional): 트윅 버전을 설정한다. 
            Defaults to 0
        tProject (str, optional): 프로젝트의 타입을 설정한다. 
            아래 중에 하나를 선택한다.
            lib, dll, flex, bison, libpp, dllpp,
            cpp_main, c_main, flex_main, bison_main
            Defaults to 'lib'.
        tOnBase (bool, optional): 프로젝트의 기본 소스와 헤더를 생성한다. 
            Defaults to False.
        tOnTester (bool, optional): main 함수가 없는 경우, 
            테스터 프로젝트를 생성한다. 
            Defaults to False.
        tRelPwd (str, optional): cwd()와 상대적인 parent working dir을 설정한다.
            테스터 프로젝트를 생성한다. 
            Defaults to './'
        tForce (bool, optional): CMakeLists.txt가 있어도 강제로 쓰기 작업한다.
            Defaults to False
    """
    # aVersion = "1.0.0.0"
    # aVersion = make_version_string(tVersion)
    aVersion = f'{tVersionMajor}.{tVersionMinor}.{tVersionPatch}.{tVersionTweak}'

    aOnBase = 'create_base()' if tOnBase else '# create_base()'
    aOnTester = 'ON' if tOnTester else 'OFF'

    aHome = Path.expanduser(Path('~'))
    aIncDir = aHome / tName / 'include'
    aLibDir = aHome / tName / 'lib'
    aStrList = list()
    aStrList.append(f'''\
# ###########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com>
# All Rights Reserved.
#
# {tName} CMakeLists.txt
#
# ###########################################################################

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.21)

# 프로젝트명 : 실행파일 이름이 된다.
project({tName}
  VERSION {aVersion}
)

file(REAL_PATH "{tCMakeStarterDir}" ROOT EXPAND_TILDE)
list(APPEND CMAKE_MODULE_PATH "${{ROOT}}")
include(starter)

# tester project를 사용한다.
# 만약 프로젝트가 main을 갖고 있으면,
# 옵션은 무시된다. 따라서 tester 프로젝트는 생성되지 않는다.
option(${{PROJECT_NAME}}_USE_TESTER "use tester" {aOnTester})

# 아래 중에 하나를 선택한다.
# lib , dll , flex , bison , libpp , dllpp,
# cpp_main , c_main , flex_main , bison_main

start_project({tProject})

  # 프로젝트에 기본적으로 필요한 헤더, 소스, 기타 등등의 파일을 생성한다.
  # 필요할 때만 호출한다.
  {aOnBase}
  
  # 소스 추가 프로젝트 기본 소스파일은 여기서 추가하지 않아도 된다 단지 예시이다
  # addSrcs(
  #  {tProject}.cpp
  # )

  # 헤더 추가 기본 헤더파일은 여기서 추가하지 않아도 된다 단지 예시이다
  # addHeaders(
  #   {tProject}.h
  # )

  # addIncDir("{aIncDir}")
    
  # addLibDir("{aLibDir}")

  # addCommandWhenBuildExcute(
  #   echo "${{PROJECT_NAME}}_BUILD_TYPE is $<CONFIG> )
  # addCommandWhenBuildExcute(  
  #   echo "${{PROJECT_NAME}}_PLATFORM_ID is $<PLATFORM_ID> )

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
''')

    if(Path.cwd() == Path.absolute(Path(tRelPwd))):
        aFilePath = Path('CMakeLists.txt')
        pass
    else:
        # 디렉토리를 생성해야 한다.
        aFilePath = Path.cwd() / tRelPwd / 'CMakeLists.txt'
        pass
    
    if(aFilePath.exists()):
        # 이미 CMakeLists.txt가 있다.
        print('이미 CMakeLists.txt가 있다.')
        if(tForce):
            print('하지만 force 옵션이 있어서 새로 생성한다.')
            with open( aFilePath, 'w', encoding='utf-8') as f:
                f.writelines(aStrList)
                pass        
            pass
        pass
    else:
        # CMakeLists.txt가 없다.
        if(not aFilePath.parent.exists()):
            # dir이 없다.
            aFilePath.parent.mkdir()
            pass
        with open( aFilePath, 'w', encoding='utf-8') as f:
            f.writelines(aStrList)
            pass                
    return

def make_tester(tName:str='new_tester', 
    tVersionMajor:int=1,
    tVersionMinor:int=0,
    tVersionPatch:int=0,
    tVersionTweak:int=0,
    tCMakeStarterDir:str='~/apps/opt/cmake_starter', 
    tForce:bool=False):
    
    aHome = Path.expanduser(Path('~'))
    aIncDir = aHome / tName / 'include'
    aLibDir = aHome / tName / 'lib'

    aStr = f'''\
############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# {tName}
#
###########################################################################//

# CMake의 최소 버전을 지정합니다.
cmake_minimum_required(VERSION 3.21)

# 프로젝트명 : 실행파일 이름이 된다.
project({tName}_tester
  VERSION {tVersionMajor}.{tVersionMinor}.{tVersionPatch}.{tVersionTweak}
)

file(REAL_PATH "{tCMakeStarterDir}" ROOT EXPAND_TILDE)
list(APPEND CMAKE_MODULE_PATH "${{ROOT}}")
include(starter)

startTester()

  # 소스 추가 프로젝트 기본 소스파일은 여기서 추가하지 않아도 된다 단지 예시이다
  # addSrcs(
  #   {tName}_tester.cpp 추가
  # )
  
  # 헤더 추가 프로젝트 기본 헤더파일은 여기서 추가하지 않아도 된다 단지 예시이다
  # addHeaders(
  #   {tName}_tester.h 추가
  # )

  # addIncDir("{aIncDir}")

  # addLibDir("{aLibDir}")

  # addCommandWhenBuildExcute(
  #   echo "${{PROJECT_NAME}} BUILD_TYPE is $<CONFIG>" )
  # addCommandWhenBuildExcute(
  #   echo "${{PROJECT_NAME}} PLATFORM_ID is $<PLATFORM_ID>" )

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
'''

    # 디렉토리를 생성해야 한다.
    aFilePath = Path('tester') / 'CMakeLists.txt'
    
    if(aFilePath.exists()):
        # 이미 CMakeLists.txt가 있다.
        if(tForce):
            with open( aFilePath, 'w', encoding='utf-8') as f:
                f.write(aStr)
                pass        
            pass
        pass
    else:
        # CMakeLists.txt가 없다.
        if(not aFilePath.parent.exists()):
            # dir이 없다.
            aFilePath.parent.mkdir()
            pass
        with open( aFilePath, 'w', encoding='utf-8') as f:
            f.write(aStr)
            pass                
    return

def callback_make(args):
    aName = args.name
    # aVersion = int(args.project_version)
    aVersionMajor = args.version_major
    aVersionMinor = args.version_minor
    aVersionPatch = args.version_patch
    aVersionTweak = args.version_tweak    
    aCMakeStarterDir = args.cmake_starter_dir
    aProject = args.project
    aOnBase = args.on_base
    aOnTester = args.on_tester
    aRelPwd = args.rel_pwd
    aForce = args.force
    make(aName, 
        aVersionMajor, aVersionMinor, aVersionPatch, aVersionTweak,
        aCMakeStarterDir, 
        aProject, aOnBase, aOnTester, aRelPwd, aForce)
    
    # if(aOnTester):
    #     make_tester(aName, aVersionMajor, aVersionMinor,
    #         aVersionPatch, aVersionTweak, aForce)
    #     pass
    return

def callback_make_tester(args):
    aName = args.name
    aVersionMajor = args.version_major
    aVersionMinor = args.version_minor
    aVersionPatch = args.version_patch
    aVersionTweak = args.version_tweak    
    aCMakeStarterDir = args.cmake_starter_dir    
    aForce = args.force

    make_tester(aName, aVersionMajor, aVersionMinor,
        aVersionPatch, aVersionTweak, 
        aCMakeStarterDir, aForce)

    return    

def config_args(tParser:argparse.ArgumentParser):
    aDsc = 'args :\n'
    aDsc += f'{tParser.prog} \n'
    aDsc += f'{tParser.prog}  -bt\n'
    aDsc += f'{tParser.prog}  -bt --project bison\n'
    aDsc += f'{tParser.prog} new_prj\n'
    aDsc += f'{tParser.prog} --project cpp_main new_prj\n'    
    aDsc += f'{tParser.prog} -b --project flex new_prj\n'
    aDsc += f'{tParser.prog} -bt --project c_main new_prj\n'
    aDsc += f'{tParser.prog} -bt --rel-pwd sub new_prj\n'
    aDsc += f'{tParser.prog} --create-tester\n'
    
    tParser.formatter_class=argparse.RawDescriptionHelpFormatter
    aEpilog = '''Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
All Rights Reserved.
cmake_lists_maker

'''
    tParser.epilog=aEpilog

    tParser.description = aDsc

    aProjectName = Path.cwd().name    
    tParser.add_argument('name',
        nargs='?',
        default=aProjectName,
        help='CMakeLists.txt의 project name을 설정한다. \
            (default: %(default)s)')
    
    tParser.add_argument('--force',
        action="store_true",
        default=False,
        help='CMakeLists.txt 이미 존재해도 강제로 생성한다. \
            (default: %(default)s)')    

    aExclusiveGroup = tParser.add_mutually_exclusive_group(required=False)

    aExclusiveGroup.add_argument('--create-tester',
        default=False,
        action="store_true",
        help='tester/CMakeLists.txt를 생성한다.')
    
    aExclusiveGroup.add_argument('--create-generic',
        default=True,
        action="store_false",
        help='CMakeLists.txt를 생성한다.')

    aDsc = '''일반적인 프로젝트를 생성한다.
--create-tester인 경우에는 필요 없는 옵션이다.
'''
    aGroup = tParser.add_argument_group('generic_project', description=aDsc)
    aGroup.add_argument('-b', '--on-base', 
        default=False,
        action="store_true",
        help='project의 기본 소스와 헤더를 생성한다. (default: %(default)s)')
    
    aGroup.add_argument('-t', '--on-tester',
        default=False,
        action="store_true",
        help='project가 메인 함수가 없을 때, tester 프로젝트를 생성한다.\
        (default: %(default)s)')
    
    aGroup.add_argument('--cmake-starter-dir',
        nargs='?',
        default='~/apps/opt/cmake_starter',
        help='cmake_starter의 위치를 설정한다. \
            (default: %(default)s)'                        
        )

    aGroup.add_argument('--project',
        nargs='?',
        default='lib',
        choices=['lib', 'dll', 'flex', 'bison', 'libpp', 'dllpp', 
            'cpp_main', 'c_main', 'flex_main', 'bison_main'],
        help = '원하는 프로젝트 타입을 선택한다. (default: %(default)s)')

    aGroup.add_argument('--rel-pwd',
        nargs='?',
        default='./',
        help='rel-pwd/CMakeLists.txt 마지막 부모 폴더를 설정한다. \
            (default: %(default)s)')

    tParser.add_argument('--version-major',
        nargs='?',
        default=1,
        help = '메이저 버전을 설정한다. (default: %(default)s)')
    
    tParser.add_argument('--version-minor',
        nargs='?',
        default=0,
        help = '마이너 버전을 설정한다. (default: %(default)s)')    
    
    tParser.add_argument('--version-patch',
        nargs='?',
        default=0,
        help = '패치 버전을 설정한다. (default: %(default)s)')
    
    tParser.add_argument('--version-tweak',
        nargs='?',
        default=0,
        help = '트윅 버전을 설정한다. (default: %(default)s)')    
    
    tParser.set_defaults(func=callback_make)    
    tParser.set_defaults(funcTester=callback_make_tester)    

    # args = tParser.parse_args(['--create-tester'])

    # subTester.set_defaults(func=callback_make_tester)

    return tParser

if __name__ == '__main__':
    print("cmake_lists_maker start")

    parser = argparse.ArgumentParser('python3 cmake_lists_maker.py')

    config_args(parser)
    # args = tParser.parse_args()
    args, unknown = parser.parse_known_args()
    if args.create_tester:
        args.funcTester(args)
        pass
    else:
        args.func(args)    
    
    print("cmake_lists_maker end")
