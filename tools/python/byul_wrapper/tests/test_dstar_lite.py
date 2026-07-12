from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid, NavgridDirMode
from byul_wrapper.dstar_lite import c_dstar_lite
from byul_wrapper.console import c_console
from byul_wrapper.dstar_lite_pqueue import c_dstar_lite_pqueue

if __name__ == "__main__":
    navgrid = c_navgrid(width=10, height=10, mode=NavgridDirMode.DIR_8)
    finder = c_dstar_lite(navgrid)
    finder.set_debug_mode_enabled(True)

    # 1️⃣ 초기 경로
    print("\n▶ 최초 경로:")
    start = c_coord(0,0)
    goal = c_coord(9,9)
    finder.set_start(start)
    finder.set_goal(goal)
    # finder.real_loop_max_retry = 20

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)
    
    # 2️⃣ 장애물 추가
    x = 3
    y = 3
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)
    result.close()

    x = 3
    y = 4
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)    
    result.close()

    x = 3
    y = 2
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)
    result.close()  

    x = 3
    y = 1
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)
    result.close()

    x = 3
    y = 0
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)
    result.close()

    # 4️⃣ 장애물 제거
    x = 3
    y = 3
    print(f"\n▶ ({x}, {y}에 장애물 제거:")
    navgrid.unblock(x, y)
    navgrid.unblock(x, y)

    result = finder.find()
    result.print()
    c_console.print_ascii_update_count(finder, result)    
    result.close()

    # 2️⃣ 장애물 추가
    x = 3
    y = 3
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    with finder.find() as result:
        result.print()
        c_console.print_ascii_update_count(finder, result)

    x = 3
    y = 5
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)

    with finder.find() as result:
        result.print()
        c_console.print_ascii_update_count(finder, result)
        
    x = 3
    y = 6
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)
    
    with finder.find() as result:
        result.print()
        c_console.print_ascii_update_count(finder, result)        
        
    x = 3
    y = 7
    print(f"\n▶ ({x}, {y}에 장애물 추가:")
    navgrid.block(x, y)
    
    with finder.find() as result:
        result.print()
        c_console.print_ascii_update_count(finder, result)        

    start_x = 2
    start_y = 2
    goal_x = 13
    goal_y = 13
    print(f"\n▶ 시작({start_x}, {start_y}로 변경:")
    print(f"\n▶ 목표({goal_x}, {goal_y}로 변경:")

    s = c_coord(start_x, start_y)
    g = c_coord(goal_x, goal_y)

    navgrid.width = goal_x+3
    navgrid.height = goal_y+3

    finder.set_max_range(18)

    print(f"\n▶ 가로 :{goal_x}, 세로 {goal_y}로 변경:")

    with finder.find() as result:
        result.print()
        c_console.print_ascii_update_count(finder, result)


    print('finder1, proto_ route')
    # map1 = c_navgrid(width=20, height=20)
    map1 = c_navgrid()
    start1 = c_coord(-10, -10)
    finder1 = c_dstar_lite(map1, start1)

    finder1.set_goal(goal)
    # finder1.set_debug_mode_enabled(True)
    finder1.find_proto()
    proto = finder1.get_proto_route()
    proto.print()
    c_console.print_ascii_update_count(finder1, proto, 5 )

    finder1.set_start(c_coord(0,0))
    finder1.set_goal(c_coord(13,13))
    # finder1.set_debug_mode_enabled(True)
    finder1.find_proto()
    proto = finder1.get_proto_route()
    proto.print()
    c_console.print_ascii_update_count(finder1, proto, 5 )    

    print('c_dstar_lite 테스트 종료된다.')


