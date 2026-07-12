'''##########################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# ext_project_adder
#
##########################################################################'''

from pathlib import Path
import argparse
import sys

def ext_project_adder(tSrcDir:Path, tDstDir:Path):
    # sp.run(aArgs, shell=True)
    # aExt = tDstDir / tSrcDir.name
    aExt = tDstDir
    print(f'aExt is {aExt}')
    aExt.symlink_to(tSrcDir, target_is_directory=True)
    return

def callback_ext_project_adder(args):
    srcPath = Path(args.src)
    dstPath = Path(args.dst)
    ext_project_adder(srcPath, dstPath)
    return 

def config_args(tParser:argparse.ArgumentParser):
    tParser.formatter_class=argparse.RawDescriptionHelpFormatter
    aEpilog = '''Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
All Rights Reserved.
ext_project_adder

'''
    tParser.epilog=aEpilog

    aDsc = 'args :\n'
    aDsc += f'{tParser.prog} "C:/Users/jrjo/Documents/cpp/unit_1001"\n'
    aDsc += f'{tParser.prog} -t unit "C:/Users/jrjo/Documents/cpp/unit_1001"\n'
    
    tParser.description = aDsc

    tParser.add_argument('-v', '--verbose', 
                    help = 'print detail progress info',
                    action = 'store_true'
                    )    
    
    tParser.add_argument('-t', '--dst',
        nargs = '?',
        default = './',
        help = 'sym link to src (default : %(default)s)')
        
    tParser.add_argument('src',
        metavar='srcDir',                               
        help='src dir의 sym link를 생성한다.')
    
    tParser.set_defaults(func=callback_ext_project_adder)

    return tParser

if __name__ == '__main__':
    print("ext_project_adder")

    parser = argparse.ArgumentParser('python3 ext_project_adder.py')

    config_args(parser)

    if (len(sys.argv) < 2):
        parser.parse_args(['--help'])
    else:
        # args = tParser.parse_args()
        args, unknown = parser.parse_known_args()
        if(args.verbose):
            print("진행 상황을 출력한다.")
            pass
        args.func(args)
        pass
    pass

