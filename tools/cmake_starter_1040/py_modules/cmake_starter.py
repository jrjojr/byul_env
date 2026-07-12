'''##########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
# 
# cmake_starter
# 
# cmake_starter에서 사용하는 py_modules 를 실행한다.
# 사용하는 py_modules는 다음과 같다.
# cmake_lists_maker.py
# encoding_converter.py
# ext_project_adder.py
# 
# 모듈 실행 시에 입력해야 하는 args
  
-h
maker -h
adder -h
converter -h
 
##########################################################################'''

from pathlib import Path
import argparse

import cmake_lists_maker as maker
import ext_project_adder as adder
import encoding_converter as converter

def config_args(tParser:argparse.ArgumentParser):
    tParser.formatter_class=argparse.RawDescriptionHelpFormatter
    aEpilog = '''Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
All Rights Reserved.
cmake_starter
'''
    tParser.epilog=aEpilog

    aDsc = 'args :\n'
    aDsc += f'{tParser.prog} -h\n'
    aDsc += f'{tParser.prog} maker -h\n'
    aDsc += f'{tParser.prog} adder -h\n'
    aDsc += f'{tParser.prog} converter -h\n'
    
    tParser.description = aDsc
    
    tParser.add_argument('-v', '--verbose', 
                    help = 'print detail progress info',
                    action = 'store_true'
                    )    

    subParsers = tParser.add_subparsers(help='sub-command help', dest='sub')

    subMaker = subParsers.add_parser('maker',
        help='각종 CMakeLists.txt를 생성한다.')
    maker.config_args(subMaker)

    subAdder = subParsers.add_parser('adder',
        help='외부 프로젝트를 참조로 추가한다.')
    adder.config_args(subAdder)

    subConverter = subParsers.add_parser('converter',
        help='''파일이나 폴더의 모든 파일을 
EUC-KR, CP949의 Korean 파일과 utf-8 파일로 상호 변환한다.''')
    converter.config_args(subConverter)

    return tParser

if __name__ == '__main__':
    # try:
    print("cmake_starter start")

    parser = argparse.ArgumentParser('cmake_starter.bat')
    config_args(parser)

    # args = tParser.parse_args()    
    args, unknown = parser.parse_known_args()
    if(args.sub == 'maker'):
        print('maker')
        if args.create_tester:
            args.funcTester(args)
            pass
        else:
            args.func(args)    

    elif(args.sub == 'adder'):
        print('adder')

        args.func(args)

    elif(args.sub == 'converter'):
        print('converter')

        args.func(args)

    else:
        print('##############################')
        parser.parse_args(['--help'])   
        print('##############################')        

    print("cmake_starter end")

    # except argparse.ArgumentError as e:
    #     print (e)
    #     args = parser.parse_args(['-h]'])
