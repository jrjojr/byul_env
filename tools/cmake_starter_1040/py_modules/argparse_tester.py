#############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# 선택한 폴더의 모든 파일을 EUC-KR 과 UTF-8을 상호 변환한다.
#
# argparse_tester
# 
#############################################################################

if __name__ == '__main__':
    import argparse

    # 제로 번째 예제
    parser = argparse.ArgumentParser()
    parser.parse_args()

    # 첫번째 예제
    # parser = argparse.ArgumentParser(
    #                     prog='ProgramName',
    #                     description='What the program does',
    #                     epilog='Text at the bottom of help')

    # parser.add_argument('filename')           # positional argument
    # parser.add_argument('-c', '--count')      # option that takes a value
    # parser.add_argument('-v', '--verbose',
    #                     action='store_true')  # on/off flag

    # args = parser.parse_args()
    # print(args.filename, args.count, args.verbose)

    # # 두번째 예제
    # parser = argparse.ArgumentParser(description='Process some integers.')

    # parser.add_argument('integers', metavar='N', type=int, nargs='+',
    #                     help='an integer for the accumulator')

    # # parser.add_argument('integers', type=int, nargs='+',
    # #                     help='an integer for the accumulator')

    # parser.add_argument('--sum', dest='accumulate', action='store_const',
    #                     const=sum, default=max,
    #                     help='sum the integers (default: find the max)')

    # args = parser.parse_args()
    # print(args.accumulate(args.integers))

    # 세번째 예제
    # create the top-level parser
    # parser = argparse.ArgumentParser(prog='PROG')
    # parser.add_argument('--foo', action='store_true', help='foo help')
    # subparsers = parser.add_subparsers(help='sub-command help')

    # # create the parser for the "a" command
    # parser_a = subparsers.add_parser('a', help='a help')
    # parser_a.add_argument('bar', type=int, help='bar help')

    # # create the parser for the "b" command
    # parser_b = subparsers.add_parser('b', help='b help')
    # parser_b.add_argument('--baz', choices='XYZ', help='baz help')

    # # parse some argument lists
    # args = parser.parse_args(['a', '12'])
    # print(args)

    # args = parser.parse_args(['--foo', 'b', '--baz', 'Z'])
    # print(args)

    # args = parser.parse_args(['--help'])
    # print(args)

    # 네번째 예제.
    # sub-command functions
    # def foo(args):
    #     print(args.x * args.y)

    # def bar(args):
    #     print('((%s))' % args.z)

    # # create the top-level parser
    # parser = argparse.ArgumentParser()
    # subparsers = parser.add_subparsers(title='subcommands',
    #                                    description='valid subcommands',
    #                                    help='additional help',
    #     required=True)

    # # create the parser for the "foo" command
    # parser_foo = subparsers.add_parser('foo', aliases=['fo'])
    # parser_foo.add_argument('-x', type=int, default=1)
    # parser_foo.add_argument('y', type=float)
    # parser_foo.set_defaults(func=foo)

    # # create the parser for the "bar" command
    # parser_bar = subparsers.add_parser('bar')
    # parser_bar.add_argument('z')
    # parser_bar.set_defaults(func=bar)

    # # parse the args and call whatever function was selected
    # args = parser.parse_args('foo 1 -x 2'.split())
    # args.func(args)


    # # parse the args and call whatever function was selected
    # args = parser.parse_args('bar XYZYX'.split())
    # args.func(args)