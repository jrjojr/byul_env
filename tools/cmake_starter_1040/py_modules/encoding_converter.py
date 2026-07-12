'''##########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# encoding_converter
# 
# 선택한 파일이나 폴더의 모든 파일을 
# EUC-KR, CP949의 Korean 파일과 utf-8 파일로 상호 변환한다.
# 
##########################################################################'''

import argparse
from pathlib import Path

def detect_encoding(filePath:Path):
    """현재 파일의 인코딩을 알아낸다.

    Args:
        filePath (Path): 조사를 원하는 파일 경로를 입력한다.

    Returns:
        dict: result['encoding']
              result['confidence']
              result['language']
              result["file"]
    """
    from chardet.universaldetector import UniversalDetector

    try:
        # 인코딩 알아내기
        # 텍스트 문서류의 파일만 조사한다.
        # 실행 파일과 라이브스는 제외해야 한다.
        file = filePath
        if (filePath.suffix == '.exe'):
            print(f'{file} 이 exe 라서 그냥 통과한다.')
            return None
        elif (filePath.suffix == '.pdb'):
            print(f'{file} 이 pdb 라서 그냥 통과한다.')
            return None
        elif (filePath.suffix == '.lib'):
            print(f'{file} 이 lib 라서 그냥 통과한다.')
            return None        
        elif (filePath.suffix == '.dll'):
            print(f'{file} 이 dll 라서 그냥 통과한다.')
            return None
        else:
            aStat = filePath.stat()
            if ( aStat.st_size > 4194304):
                print(f'{file} size가 4M보다 커서 그냥 통과한다.')
                return None
            else:
                f = open(file,'rb')

                print(f'{file} detect start.')
                detector = UniversalDetector()
                lines = f.readlines()
                for line in lines:
                    detector.feed(line)
                    if (detector.done):
                        break
                    
                detector.close()
                f.close()    

                result = detector.result
                result['file'] = filePath
                print(f'{file} detect result : {result}')
                # retEncoding = result["encoding"]
                # retConfidence = result['confidence']
                # retLanguage =  result['language']
                # print(f'{result["file"]}')
                # print(f'{result["encoding"]}')
                return result
    except OSError as e:
        print(e)
        return None

def conv_euckr_to_utf8(tFilePath:Path):
    """euc_kr의 파일을 utf-8로 변환한다.

    Args:
        tFilePath (Path): 변환을 원하는 파일
    """
    try:
        if(tFilePath.exists()):
            aFileInfo = detect_encoding(tFilePath)
            if(aFileInfo['encoding'] == 'EUC-KR'):
                # 현재 파일이 euc_kr이다 작업을 시작한다.
                f = open(tFilePath,'rb')
                lines = f.readlines()
                lines2 = []
                for line in lines:
                    lines2.append(line.decode('euc_kr').encode('utf-8','ignore'))
                f.close() 

                f2 = open(tFilePath,'wb')
                f2.writelines(lines2)
                f2.close()
                return
        
    except OSError as e:
        print(e)

def conv_cp949_to_utf8(tFilePath:Path):
    """cp949의 파일을 utf-8로 변환한다.

    Args:
        tFilePath (Path): 변환을 원하는 파일
    """
    try:
        if(tFilePath.exists()):
            aFileInfo = detect_encoding(tFilePath)
            if(aFileInfo['encoding'] == 'CP949'):
                # 현재 파일이 CP949이다 작업을 시작한다.
                f = open(tFilePath,'rb')
                lines = f.readlines()
                lines2 = []
                for line in lines:
                    lines2.append(line.decode('cp949').encode('utf-8','ignore'))
                f.close() 

                f2 = open(tFilePath,'wb')
                f2.writelines(lines2)
                f2.close()
                return
        
    except OSError as e:
        print(e)

# lines2.append(line.decode('utf-8').encode('euc_kr','ignore'))
def conv_utf8_to_euckr(tFilePath:Path):
    """utf-8의 파일을 euc_kr로 변환한다.

    Args:
        tFilePath (Path): 변환을 원하는 파일
    """
    try:
        if(tFilePath.exists()):
            aFileInfo = detect_encoding(tFilePath)
            if(aFileInfo['encoding'] == 'utf-8'):
                # 현재 파일이 euc_kr이다 작업을 시작한다.
                f = open(tFilePath,'rb')
                lines = f.readlines()
                lines2 = []
                for line in lines:
                    lines2.append(line.decode('utf-8').encode('euc_kr','ignore'))
                f.close() 

                f2 = open(tFilePath,'wb')
                f2.writelines(lines2)
                f2.close()
                return
        
    except OSError as e:
        print(e)

def conv_korean_to_utf8(tFilePath:Path):
    """CP949와 EUC-KR의 파일을 utf-8로 변환한다.

    Args:
        tFilePath (Path): 변환을 원하는 파일
    """
    try:
        aFileInfo = detect_encoding(tFilePath)
        if(aFileInfo['language'] == 'Korean'):
            if(aFileInfo['encoding'] == 'CP949'):
                conv_cp949_to_utf8(tFilePath)
            elif(aFileInfo['encoding'] == 'EUC-KR'):
                conv_euckr_to_utf8(tFilePath)
        
    except OSError as e:
        print(e)        

def detect_encoding_all(tDirPath:Path):
    try:
        # filePath
        # encoding
        # confidence
        # language
        aResults = list()
        if(tDirPath.is_dir()):
            for f in tDirPath.iterdir():
                if(f.is_file()):
                    aSubResults = detect_encoding(f)
                    if(aSubResults != None):
                        aResults.append(aSubResults)
                    pass
                elif(f.is_dir()):
                    # f가 폴더이다. 재귀한다.
                    aSubResults = detect_encoding_all(f)
                    if(aSubResults != None):
                        aResults += aSubResults
                else:
                    print(f'{f}는 폴더나 파일이 아니다.')
                    pass
        elif(tDirPath.is_file()):
            aSubResults = detect_encoding(tDirPath)
            if(aSubResults != None):
                aResults.append(aSubResults)

        return aResults
    except OSError as e:
        print(e)
        return None
    
def conv_euckr_to_utf8_all(tDirPath:Path):
    """dir의 모든 파일을 확인해서 euc_kr의 파일을 utf-8로 변환한다.

    Args:
        tDirPath (Path): src dir
    """
    try:
        if(tDirPath.is_dir()):
            # 폴더의 파일들을 알아낸다.
            aFileInfos = detect_encoding_all(tDirPath)
            if(aFileInfos != None):
                # 정보를 얻었다.
                # 필요한 파일을 추출한다.
                # 목표 폴더를 확인한다.

                for aFI in aFileInfos:
                    if(aFI['encoding'] == 'EUC-KR'):
                        conv_euckr_to_utf8(aFI['file'])
                        pass
                    pass
                pass
            pass
        elif(tDirPath.is_file()):
            conv_euckr_to_utf8(tDirPath)
            pass
        else:
            print(f'tDirPath : {tDirPath} 가 폴더도 아니고 파일도 아니고... 종료한다')
            pass
    except OSError as e:
        print(e)    

def conv_utf8_to_euckr_all(tDirPath:Path):
    """dir의 모든 파일을 확인해서 utf-8의 파일을 euc_kr의 파일로 변환한다.

    Args:
        tDirPath (Path): 변환을 원하는 dir
    """
    try:
        if(tDirPath.is_dir()):
            # 폴더의 파일들을 알아낸다.
            aFileInfos = detect_encoding_all(tDirPath)
            if(aFileInfos != None):
                # 정보를 얻었다.
                # 필요한 파일을 추출한다.
                # 목표 폴더를 확인한다.

                for aFI in aFileInfos:
                    if(aFI['encoding'] == 'utf-8'):
                        conv_utf8_to_euckr(aFI['file'])
                        pass
                    pass
                pass
            pass
        elif(tDirPath.is_file()):
            conv_utf8_to_euckr(tDirPath)
            pass
        else:
            print(f'tDirPath : {tDirPath} 가 폴더도 아니고 파일도 아니고... 종료한다')
            pass
    except OSError as e:
        print(e)    

def conv_cp949_to_utf8_all(tDirPath:Path):
    """dir의 모든 파일을 확인해서 CP949의 파일을 utf-8로 변환한다.

    Args:
        tDirPath (Path): src dir
    """
    try:
        if(tDirPath.is_dir()):
            # 폴더의 파일들을 알아낸다.
            aFileInfos = detect_encoding_all(tDirPath)
            if(aFileInfos != None):
                # 정보를 얻었다.
                # 필요한 파일을 추출한다.
                # 목표 폴더를 확인한다.

                for aFI in aFileInfos:
                    if(aFI['encoding'] == 'CP949'):
                        conv_cp949_to_utf8(aFI['file'])
                        pass
                    pass
                pass
            pass
        elif(tDirPath.is_file()):
            conv_cp949_to_utf8(tDirPath)
            pass
        else:
            print(f'tDirPath : {tDirPath} 가 폴더도 아니고 파일도 아니고... 종료한다')
            pass
    except OSError as e:
        print(e)    

def conv_korean_to_utf8_all(tDirPath:Path):
    """dir의 모든 파일을 확인해서 CP949와 EUC-KR의 파일을 utf-8로 변환한다.

    Args:
        tDirPath (Path): src dir
    """
    try:
        if(tDirPath.is_dir()):
            # 폴더의 파일들을 알아낸다.
            aFileInfos = detect_encoding_all(tDirPath)
            if(aFileInfos != None):
                # 정보를 얻었다.
                # 필요한 파일을 추출한다.
                # 목표 폴더를 확인한다.

                for aFI in aFileInfos:
                    if(aFI['language'] == 'Korean'):
                        if(aFI['encoding'] == 'CP949'):
                            conv_cp949_to_utf8(aFI['file'])
                            pass
                        
                        elif(aFI['encoding'] == 'EUC-KR'):
                            conv_euckr_to_utf8(aFI['file'])
                            pass
                        pass
                pass
            pass
        elif(tDirPath.is_file()):
            conv_korean_to_utf8(tDirPath)
            pass
        else:
            print(f'tDirPath : {tDirPath} 가 폴더도 아니고 파일도 아니고... 종료한다')
            pass
    except OSError as e:
        print(e)            

def callback_conv_korean_to_utf8(args):
    srcPath = Path(args.src)
    conv_korean_to_utf8(srcPath)
    return 

def callback_conv_utf8_to_euckr(args):
    srcPath = Path(args.src)
    conv_utf8_to_euckr(srcPath)
    return

def callback_detect_encoding(args):
    srcPath = Path(args.src)
    detect_encoding(srcPath)
    return

def callback_detect_encoding_all(args):
    aPath = Path(args.src)
    detect_encoding_all(aPath)
    return

def callback_conv_utf8_to_euckr_all(args):
    srcPath = Path(args.src)
    conv_utf8_to_euckr_all(srcPath)
    pass

def callback_conv_korean_to_utf8_all(args):
    srcPath = Path(args.src)
    conv_korean_to_utf8_all(srcPath)
    pass

def config_args(tParser:argparse.ArgumentParser):
    tParser.formatter_class=argparse.RawDescriptionHelpFormatter
    aEpilog = '''Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
All Rights Reserved.
encoding_converter

'''
    tParser.epilog=aEpilog

    aDsc = 'args :\n'
    aDsc += f'{tParser.prog} info -h\n'
    aDsc += f'{tParser.prog} info CMakeLists.txt\n'
    aDsc += f'{tParser.prog} info_all -h\n'
    aDsc += f'{tParser.prog} info_all ./\n'
    aDsc += f'{tParser.prog} to_utf8 -h\n'
    aDsc += f'{tParser.prog} to_utf8 CMakeLists.txt\n'
    aDsc += f'{tParser.prog} to_utf8_all -h\n'
    aDsc += f'{tParser.prog} to_utf8_all ./\n'

    tParser.description = aDsc

    tParser.add_argument('-v', '--verbose', 
                    help = 'print detail progress info',
                    action = 'store_true'
                    )
    
    subParsers = tParser.add_subparsers(help='sub-command help', dest='sub')

    subParserInfo = subParsers.add_parser('info',
        help='view %(prog)s encoding info')
    
    subParserInfo.add_argument('src',
        metavar='srcFile',                               
        help='src file의 encoding info를 출력한다.')
    
    subParserInfo.set_defaults(func=callback_detect_encoding)

    subParserToUtf8 = subParsers.add_parser('to_utf8',
        help='to utf8')
    
    subParserToUtf8.add_argument('src',
        metavar='srcFile',
        help='create dst file after convert srcFile from euc_kr to utf-8 (default dst=./dst)'
        )

    subParserToUtf8.add_argument('-t', '--dst',
        nargs = '?',
        default = './dst',
        help = 'output file to dst (default : %(default)s)')
    
    subParserToUtf8.set_defaults(func=callback_conv_korean_to_utf8)

    subParserToEuckr = subParsers.add_parser('to_euckr',
        help='to euckr')
    
    subParserToEuckr.add_argument('src',
        metavar='srcFile',
        help='create dst file after convert srcFile from utf-8 to euc_kr (default dst=./dst)'
        )

    subParserToEuckr.add_argument('-t', '--dst',
        nargs = '?',
        default = './dst',
        help = 'output file to dst (default : %(default)s)')    
    
    subParserToEuckr.set_defaults(func=callback_conv_utf8_to_euckr)

    subParserInfoAll = subParsers.add_parser('info_all',
        help='view %(prog)s encoding info all')
    
    subParserInfoAll.add_argument('src',
        metavar='srcDir',                               
        help='src Dir의 모든 파일의 encoding info를 출력한다.')
    
    subParserInfoAll.set_defaults(func=callback_detect_encoding_all)

    subParserToUtf8All = subParsers.add_parser('to_utf8_all',
        help='to utf8 all')
    
    subParserToUtf8All.add_argument('src',
        metavar='srcFile',
        help='create dst files after convert srcFiles from euc_kr to utf-8 (default dst=./dst)'
        )
    
    subParserToUtf8All.add_argument('-t', '--dst',
        nargs = '?',
        default = './dst',
        help = 'output file to dst (default : %(default)s)')
    
    subParserToUtf8All.set_defaults(func=callback_conv_korean_to_utf8_all)   

    subParserToEuckrAll = subParsers.add_parser('to_euckr_all',
        help='to euc_kr all')
    
    subParserToEuckrAll.add_argument('src',
        metavar='srcFile',
        help='create dst files after convert srcFiles from euc_kr to utf-8 (default dst=./dst)'
        )
    
    subParserToEuckrAll.add_argument('-t', '--dst',
        nargs = '?',
        default = './dst',
        help = 'output file to dst (default : %(default)s)')
    
    subParserToEuckrAll.set_defaults(func=callback_conv_utf8_to_euckr_all)        

    return tParser

if __name__ == '__main__':
    print("encoding_converter start")

    parser = argparse.ArgumentParser(prog = 'encoding_converter', 
        description='%(prog)s do convert euc_kr or utf-8',
        epilog= 'text a the bottom of help'
        )

    config_args(parser)

    # args = parser.parse_args()    
    args, unknown = parser.parse_known_args()
    args.func(args)
    # print(args)
    if(args.sub == 'info'):
        print('info')

        args.func(args)

    elif(args.sub == 'to_utf8'):
        print('to utf8')

        args.func(args)

    elif(args.sub == 'to_euckr'):
        print('to euc kr')

        args.func(args)

    elif(args.sub == 'info_all'):
        print('info_all')

        args.func(args)

    elif(args.sub == 'to_utf8_all'):
        print('to utf8 all')

        args.func(args)

    elif(args.sub == 'to_euckr_all'):
        print('to euc_kr all')

        args.func(args)

    else:
        print('##############################')
        parser.parse_args(['--help'])   
        print('##############################')        

