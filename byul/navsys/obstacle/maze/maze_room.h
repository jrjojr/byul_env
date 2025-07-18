#ifndef MAZE_ROOM_H
#define MAZE_ROOM_H

#include "internal/coord_hash.h"
#include "internal/maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Room 구조체 정의
 *
 * 맵 상에 사각형 형태의 공간(방)을 나타냅니다.
 */
typedef struct {
    int x, y;   ///< 방의 좌측 상단 좌표
    int w, h;   ///< 방의 너비와 높이
} room_t;

/**
 * @brief Room + Maze 블렌딩 알고리즘으로 미로를 생성합니다.
 *
 * 이 함수는 **방(Room)** 을 먼저 생성하고,  
 * 그 사이를 복도로 연결한 뒤,  
 * 남은 공간을 **백트래킹 기반 미로 알고리즘**으로 채웁니다.  
 * 
 * 결과적으로 방 + 통로 + 미로가 혼합된 **RPG 게임용 맵 구조**를 생성합니다.
 *
 * ---
 *
 * ### 알고리즘 개요
 * 1. **방 생성 (Room Placement)**  
 *    - 지정된 크기 범위 안에서 무작위 방을 시도하며 배치합니다.  
 *    - 서로 겹치지 않도록 오버랩 검사 수행  
 * 
 * 2. **방 연결 (Corridor Digging)**  
 *    - 각 방의 중심을 기준으로 다음 방과 연결하는 복도를 생성합니다.  
 *    - 복도는 ㄱ자(H/V 순서 랜덤) 형태로 생성됩니다.
 * 
 * 3. **남은 공간 미로화 (Maze Filling)**  
 *    - 위 두 단계에서 생성되지 않은 벽 영역을 대상으로  
 *      백트래킹 기반의 미로 알고리즘으로 채워 넣습니다.
 *    - 전체 지형이 연결되며, 단조롭지 않은 복합 지형이 완성됩니다.
 *
 * ---
 *
 * ### 특징
 * - **방은 개방적**이며, **복도는 길쭉하고 좁게**,  
 *   **미로는 제한된 공간에 정밀하게 채워집니다.**
 * - 미로 생성이 후순위이기 때문에 **트리 형태는 아닙니다.**
 * - Dead-end는 일부 생성되지만 많지 않으며,  
 *   **중앙 집중형 레벨 구성에 적합**합니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()`로 생성되어야 합니다.
 * - `width`, `height`는 **9 이상**의 홀수여야 합니다.
 *
 * ---
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 31, 21, MAZE_TYPE_ROOM_BLEND);
 * maze_make_room_blend(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * ---
 *
 * @param maze 생성할 대상 미로 구조체 포인터
 */
BYUL_API void maze_make_room_blend(maze_t* maze);

#ifdef __cplusplus
}
#endif

#endif // MAZE_ROOM_H
