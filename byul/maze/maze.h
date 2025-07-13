#ifndef MAZE_H
#define MAZE_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_hash.h"
#include "internal/map.h"
#include "internal/maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// 미로 알고리즘 타입
typedef enum {
    MAZE_TYPE_RECURSIVE,
    MAZE_TYPE_PRIM,
    MAZE_TYPE_BINARY,
    MAZE_TYPE_ELLER,
    MAZE_TYPE_ALDOUS_BRODER,
    MAZE_TYPE_WILSON,
    MAZE_TYPE_HUNT_AND_KILL,
    MAZE_TYPE_SIDEWINDER,
    MAZE_TYPE_RECURSIVE_DIVISION,
    MAZE_TYPE_KRUSKAL,
    MAZE_TYPE_ROOM_BLEND
} maze_type_t;

BYUL_API void maze_make(maze_t* maze, maze_type_t type);

/**
 * @brief 재귀적 분할(Recursive Division) 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 주어진 `maze_t` 구조체 내 정의된 영역(
 * `x0`, `y0`, `width`, `height`)을 기반으로,
 * 고전적인 **재귀적 분할 알고리즘**을 통해 벽과 통로의 구조를 생성합니다.
 * 내부적으로 2셀 단위로 벽과 통로를 반복적으로 나누며,
 * 반드시 홀수 크기의 너비와 높이(예: 9x9)를 사용하는 것이 권장됩니다.
 *
 * - 생성된 결과는 `maze->blocked` 내부에 저장되며,
 *   `maze_apply_to_map()` 함수를 통해 `map_t`에 삽입할 수 있습니다.
 * - 이 함수는 `maze_t` 구조체의 `type` 필드가 
 * `MAZE_TYPE_RECURSIVE`일 때에만 호출되어야 합니다.
 * - 미로의 좌표계는 절대 좌표 기준이며, `x0`, `y0`는 미로 좌측 상단 기준점입니다.
 *
 * @note 4방향(상,하,좌,우) 기준으로만 분할되며, 대각선 연결은 생성되지 않습니다.
 * @note 미로는 외벽이 막혀 있는 형태로 생성되며, 출입구는 따로 뚫어야 합니다.
 *
 * @param maze 미로 정보와 결과가 저장될 `maze_t*` 포인터
 *
 * @see maze_new_full()
 * @see maze_apply_to_map()
 */
BYUL_API void maze_make_recursive(maze_t* maze);

/**
 * @brief Prim 알고리즘을 이용하여 미로를 생성합니다.
 *
 * 이 함수는 지정된 maze_t 구조체 내부에 Prim 알고리즘 기반의
 * 미로를 생성합니다. 생성된 미로는 내부 blocked 좌표 집합에 기록되며,
 * 필요 시 `maze_apply_to_map()` 함수를 통해 map_t에 삽입할 수 있습니다.
 *
 * @details
 * Prim 알고리즘은 그래프 최소 신장 트리(MST)를 구하는 방식에서
 * 응용된 미로 생성 방법입니다. 벽으로 둘러싸인 셀들 중 하나에서 시작하여,
 * 인접한 벽을 랜덤하게 선택하면서 셀들을 연결해 나갑니다.
 * 이때 한쪽 셀만 방문된 상태인 벽만 선택하며, 전체를 하나의 연결된 통로로 만듭니다.
 *
 * 미로는 홀수 크기(width, height)로 구성하는 것이 일반적이며,
 * 모든 셀은 홀수 좌표에 위치하며, 그 사이의 짝수 좌표는 벽으로 간주됩니다.
 *
 * @usage
 * ```c
 * maze_t* maze = maze_new_full(0, 0, 21, 21, MAZE_TYPE_PRIM);
 * maze_make_prim(maze);
 * map_t* map = map_new_full(21, 21, MAP_NEIGHBOR_4, NULL);
 * maze_apply_to_map(maze, map);
 * // 이후 map 사용
 * maze_free(maze);
 * map_free(map);
 * ```
 *
 * @param maze 미로를 생성할 maze_t 포인터 (NULL 불가)
 */
BYUL_API void maze_make_prim(maze_t* maze);

/**
 * @brief Binary Tree 알고리즘으로 미로를 생성합니다.
 *
 * 이 함수는 Binary Tree 방식의 미로 생성 알고리즘을 사용하여
 * 주어진 `maze_t` 구조체 내부의 `blocked` 좌표 집합을 설정합니다.
 *
 * Binary Tree 알고리즘은 각 셀에서 북쪽 또는 동쪽으로만 
 * 벽을 허물어가는 매우 단순한 방식이며,
 * 구현이 간단하고 빠르지만, 생성되는 미로는 편향된 경향이 있습니다.
 *
 * - 항상 오른쪽 또는 아래 방향 통로가 존재하게 되어 복잡도가 낮습니다.
 * - 규칙성이 강하므로, 실제 게임에서는 보조 알고리즘이나 후처리가 필요한 경우가 많습니다.
 *
 * @param maze 미로의 좌표 기준, 너비, 높이, 
 *      타입(MAZE_TYPE_BINARY)이 설정된 구조체 포인터입니다.
 *             이 함수는 해당 구조체 내부의 blocked 필드를 수정하여 미로를 생성합니다.
 *
 * @note `maze`는 반드시 유효한 포인터여야 하며, 
 *       `width`와 `height`는 홀수로 설정되어야 합니다.
 *       (미로 벽과 통로를 구분하기 위함입니다.)
 *
 * @see maze_new_full()
 * @see maze_apply_to_map()
 *
 * @example
 * ```c
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_BINARY);
 * maze_make_binary(maze);
 * map_t* map = map_new();
 * maze_apply_to_map(maze, map);
 * ```
 */
BYUL_API void maze_make_binary(maze_t* maze);

/**
 * @brief Eller 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 고전적인 **Eller 알고리즘**을 기반으로, 지정된 `maze_t` 구조체 내부에
 * 미로 패턴을 생성합니다. 미로는 **한 줄씩(row-by-row)** 생성되며, 각 줄마다
 * 셋(set) ID를 관리하고 병합하며 다음 줄로 연결하여 전체 연결된 구조를 만듭니다.
 *
 * ### 알고리즘 개요
 * - 각 홀수 칸마다 고유한 셋(set) ID를 부여합니다.
 * - 좌우 인접 셀을 랜덤하게 병합(연결)하여 수평 통로를 만듭니다.
 * - 각 셋마다 최소 하나 이상은 아래쪽으로 연결되도록 수직 통로를 개방합니다.
 * - 마지막 줄에서는 남은 셋을 전부 병합하여 하나의 연결된 구조를 완성합니다.
 *
 * > ❗ 이 알고리즘은 **외곽 벽을 강제하지 않으며**,  
 * > 외부에서 후처리로 벽을 추가해야 할 수 있습니다.  
 * > 알고리즘 원리에 충실하게 구현되어 있으며, 
 *      외곽 개방 여부도 무작위에 따라 달라질 수 있습니다.
 *
 * ### 사용 조건
 * 이 함수는 반드시 `maze_new_full()` 
 *      함수를 통해 생성된 `maze_t` 구조체를 사용해야 하며,  
 * 다음과 같은 제약을 만족해야 합니다:
 * - 가로(width)와 세로(height)는 **홀수 값**이어야 합니다. (예: 9x9, 11x7)
 * - 가로, 세로는 최소 **3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_ELLER);
 * maze_make_eller(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 실행 후 `maze->blocked` 필드에는 벽 좌표가 등록되며,  
 * `maze->type` 필드는 `MAZE_TYPE_ELLER`로 설정됩니다.
 *
 * @param maze 미로 데이터를 생성할 maze_t 포인터
 */
BYUL_API void maze_make_eller(maze_t* maze);

/**
 * @brief Aldous-Broder 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 **Aldous-Broder 알고리즘**을 사용하여 지정된 `maze_t` 구조체에  
 * 무작위 순회 기반(Random Walk) 방식으로 미로를 생성합니다.  
 * 모든 셀을 무작위로 순회하면서, **방문하지 않은 셀에 도달했을 때만 통로를 개방**합니다.
 *
 * ### 알고리즘 개요
 * - 무작위 셀에서 시작하여 계속해서 이웃 셀로 이동합니다.
 * - **방문하지 않은 셀을 처음 방문했을 때**, 
 *      현재 위치와 다음 셀 사이에 **통로를 개방**합니다.
 * - 이미 방문한 셀로 이동하는 경우는 통로를 만들지 않고 그대로 이동만 합니다.
 * - 모든 셀이 방문될 때까지 반복하며, 이를 통해 완전한 연결성을 갖춘 미로를 만듭니다.
 *
 * 이 알고리즘은 매우 단순하면서도,
 * **모든 가능한 미로 중 하나를 균일한 확률로 생성하는 유일한 알고리즘** 중 하나입니다.
 *
 * > ❗ 단점: 평균적으로 매우 비효율적일 수 있으며,  
 * > 특히 셀 개수가 많을수록 시간이 오래 걸릴 수 있습니다.
 *
 * > ❗ 외곽 벽은 알고리즘 내부에서 강제하지 않습니다.  
 * > 필요하다면 `maze_make()` 또는 후처리 단계에서 별도로 처리하세요.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()` 함수를 통해 생성되어야 합니다.
 * - 가로(`width`)와 세로(`height`)는 **홀수**여야 하며, **최소 3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_ALDOUS_BRODER);
 * maze_make_aldous_broder(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 실행이 완료되면 `maze->blocked`에 벽 좌표가 삽입되고,  
 * `maze->type`은 `MAZE_TYPE_ALDOUS_BRODER`로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_aldous_broder(maze_t* maze);

/**
 * @brief Wilson 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 **Wilson 알고리즘(Wilson's Algorithm)**을 사용하여  
 * 완전한 연결성을 가지며, **편향 없는(uniform)** 미로를 생성합니다.  
 * 알고리즘의 핵심은 **Loop-Erased Random Walk (LERW)** 방식으로,  
 * 중복 경로를 제거하면서 순수한 트리 형태의 미로를 구성합니다.
 *
 * ### 알고리즘 개요
 * - 모든 셀 중 하나를 **기초 셀(initial cell)**로 설정하고 방문 처리합니다.
 * - 나머지 방문하지 않은 셀들 중 하나를 무작위로 선택하여 **무작위 순회**를 시작합니다.
 * - 이미 방문한 셀에 도달할 때까지 이동하며, 
 *      중간에 **루프가 생기면 제거(loop erase)**합니다.
 * - 최종적으로 얻어진 경로를 미로에 **통로(PASSAGE)**로 반영하고, 방문 처리합니다.
 * - 이 과정을 **모든 셀이 포함될 때까지 반복**하여 하나의 
 *      완전한 spanning tree를 만듭니다.
 *
 * > 📌 이 알고리즘은 이론적으로 **가장 공정한(균등 분포)** 미로를 생성합니다.
 * > 단점은 속도가 느릴 수 있고, 복잡한 구현이 필요하다는 점입니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()` 함수를 통해 생성되어야 합니다.
 * - 가로(`width`)와 세로(`height`)는 반드시 **홀수 값**이어야 하며, 
 *      **3 이상**이어야 합니다.
 *
 * ### 외곽 벽 처리
 * - 이 알고리즘은 외곽을 벽으로 강제하지 않으며,  
 *   필요하다면 `maze_make()` 또는 후처리 단계에서 외곽 벽을 추가해야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_WILSON);
 * maze_make_wilson(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 호출이 완료되면 `maze->blocked`에 벽 좌표가 기록되고,  
 * `maze->type` 필드는 `MAZE_TYPE_WILSON`으로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_wilson(maze_t* maze);

/**
 * @brief Hunt-and-Kill 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 **Hunt-and-Kill 알고리즘**을 기반으로 지정된 `maze_t` 구조체에  
 * 깊이 없는 랜덤 워크 방식의 미로를 생성합니다.  
 * DFS(깊이 우선 탐색)보다 간단하며, 
 *      자연스러운 **트리 형태의 미로**를 빠르게 만들 수 있습니다.
 *
 * ### 알고리즘 개요
 * - **Kill Phase**: 무작위로 방향을 선택하여 방문하지 않은 
 *      이웃이 있다면 통로를 뚫고 이동합니다.
 * - 이웃이 없다면 탐색을 종료하고, 다음 단계로 넘어갑니다.
 *
 * - **Hunt Phase**: 전체 셀을 순회하며  
 *   아직 방문되지 않았지만 **인접한 방문 셀이 존재하는 셀**을 찾습니다.  
 *   해당 셀과 연결된 방향으로 통로를 뚫고 다시 Kill Phase로 진입합니다.
 *
 * 이 과정을 **모든 셀이 방문될 때까지 반복**하며,  
 * 결과적으로 하나의 연결된 트리 형태의 미로를 생성합니다.
 *
 * > ❗ 이 알고리즘은 구현이 간단하고 빠르며,  
 * > Dead-end가 많은 전형적인 나무 구조의 미로를 생성합니다.
 *
 * > ❗ 외곽 벽은 알고리즘 내부에서 강제하지 않습니다.  
 * > 필요한 경우 외부에서 후처리로 추가해야 합니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()` 함수를 통해 생성되어야 합니다.
 * - 가로(`width`)와 세로(`height`)는 **홀수 값**이어야 하며, **3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_HUNT_AND_KILL);
 * maze_make_hunt_and_kill(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 호출이 완료되면 `maze->blocked` 필드에는 벽 좌표가 기록되고,  
 * `maze->type` 필드는 `MAZE_TYPE_HUNT_AND_KILL`으로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_hunt_and_kill(maze_t* maze);

/**
 * @brief Sidewinder 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 **Sidewinder 알고리즘**을 기반으로,  
 * 오른쪽 방향 중심의 통로 구조를 가진 미로를 생성합니다.  
 * Binary Tree 알고리즘을 수평 방향으로 확장한 구조이며,  
 * 긴 가로 통로와 적은 수직 연결로 인해 **열린 느낌의 미로**가 만들어집니다.
 *
 * ### 알고리즘 개요
 * - 미로를 **위에서 아래로 줄 단위(row-by-row)**로 처리합니다.
 * - 각 줄마다 **run set**을 유지하면서 오른쪽으로 연결할지 무작위로 결정합니다.
 * - 오른쪽으로 연결하지 않기로 결정되면,  
 *   run set에서 하나를 선택해 **위쪽으로 연결**하고 run을 초기화합니다.
 * - 이 과정을 모든 줄에 대해 반복합니다.
 *
 * > ❗ 이 알고리즘은 사이클이 없는 트리 구조를 보장하며,  
 * > **Dead-end가 거의 없는 부드러운 미로**를 빠르게 생성합니다.
 *
 * > ❗ 외곽 벽은 알고리즘 내부에서 강제하지 않으므로,  
 * > 필요할 경우 외부에서 후처리로 추가해야 합니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()` 함수를 통해 생성되어야 합니다.
 * - 가로(`width`)와 세로(`height`)는 **홀수 값**이어야 하며, **3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_SIDEWINDER);
 * maze_make_sidewinder(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 호출이 완료되면 `maze->blocked` 필드에는 벽 좌표가 기록되며,  
 * `maze->type`은 `MAZE_TYPE_SIDEWINDER`로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_sidewinder(maze_t* maze);

/**
 * @brief Recursive Division 알고리즘을 사용하여 미로를 생성합니다.
 *
 * 이 함수는 **Recursive Division (재귀적 분할)** 알고리즘을 기반으로  
 * 미로 공간을 벽으로 나누며, 각 영역에 무작위로 통로를 만들어  
 * 정돈된 건축적 구조의 미로를 생성합니다.
 *
 * ### 🔹 알고리즘 개요
 * - 미로 전체 공간을 기준으로 무작위 방향(가로 또는 세로)의 벽을 세웁니다.
 * - 해당 벽에는 **통로를 하나 무작위 위치에 생성**합니다.
 * - 벽을 기준으로 나뉜 두 영역에 대해 **재귀적으로 같은 과정을 반복**합니다.
 * - 영역 크기가 충분히 작아지면 분할을 중단합니다.
 *
 * ### ⚠️ 중요한 특성
 * - 이 알고리즘은 **트리 기반 구조가 아니며**,  
 *   **Dead-end(막다른 길)가 거의 없는 개방형 구조**가 됩니다.
 * - 또한 **길의 연결이 보장되지 않습니다.**
 *   → 연결되지 않은 방이나 복도가 존재할 수 있으며,  
 *   이를 해결하려면 별도의 후처리(`fix_disconnected_regions()` 등)가 필요합니다.
 *
 * > 🔧 따라서, 탐색 가능한 단일 경로망이 필요한 경우에는  
 * > `MAZE_TYPE_PRIM`, `MAZE_TYPE_KRUSKAL` 등 
 *      트리 기반 알고리즘을 사용하는 것이 적절합니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()`로 생성되어야 합니다.
 * - `width` 및 `height`는 **홀수이며 3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_RECURSIVE_DIVISION);
 * maze_make_recursive_division(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 실행 후:
 * - `maze->blocked` 필드에 벽 좌표가 기록됩니다.
 * - `maze->type`은 `MAZE_TYPE_RECURSIVE_DIVISION`으로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_recursive_division(maze_t* maze);

/**
 * @brief Kruskal 알고리즘을 사용하여 완전 연결된 미로를 생성합니다.
 *
 * 이 함수는 **Kruskal’s Algorithm (크러스컬 알고리즘)**을 기반으로  
 * 최소 신장 트리(MST)를 구성하여 **사이클이 없는 완전한 미로**를 생성합니다.  
 * 각 통로 셀은 **서로 다른 집합**으로 시작하며,  
 * 인접한 셀을 잇는 벽들을 무작위 순서로 처리하며  
 * **두 셀이 다른 집합일 경우 벽을 허물고 병합**하는 방식으로 작동합니다.
 *
 * ### 🔹 알고리즘 개요
 * - 모든 홀수 좌표 셀을 PASSAGE로 초기화하고, 각각 별도 집합으로 지정합니다.
 * - 인접한 셀 사이의 벽을 간선으로 간주하여 리스트에 저장합니다.
 * - 이 벽 리스트를 **무작위로 섞은 후**, 한 벽씩 선택하면서:
 *   - 벽 양쪽 셀이 서로 다른 집합이면 → 벽을 제거하고 병합(union)
 *   - 같은 집합이면 → 무시
 * - 모든 셀이 하나의 집합으로 병합되면 종료됩니다.
 *
 * > ✅ 이 알고리즘은 항상 **모든 셀이 하나의 경로망으로 연결**되는  
 * > **연결 보장형 미로**를 생성합니다.
 *
 * > ✅ 사이클이 존재하지 않으며, **Dead-end가 풍부한 트리 구조**가 형성됩니다.
 *
 * ---
 *
 * ### 사용 조건
 * - `maze_t`는 반드시 `maze_new_full()`로 생성되어야 합니다.
 * - `width`, `height`는 **홀수이면서 3 이상**이어야 합니다.
 *
 * ### 사용 예시
 * @code
 * maze_t* maze = maze_new_full(0, 0, 9, 9, MAZE_TYPE_KRUSKAL);
 * maze_make_kruskal(maze);
 * map_add_maze(map, maze);
 * @endcode
 *
 * 함수 실행이 완료되면:
 * - `maze->blocked` 필드에 벽 좌표가 저장됩니다.
 * - `maze->type`은 `MAZE_TYPE_KRUSKAL`로 설정됩니다.
 *
 * @param maze 미로를 생성할 대상 maze_t 포인터
 */
BYUL_API void maze_make_kruskal(maze_t* maze);

#ifdef __cplusplus
}
#endif

#endif // MAZE_H
