#include "internal/coord_radar.h"
#include "internal/coord.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <cmath>

struct s_astar_node {
    coord_t* coord;
    int cost;
    int heuristic;
};

static astar_node_t* astar_node_new(
    const coord_t* c, int cost, int heuristic) {
    astar_node_t* node = new astar_node_t;
    node->coord = coord_copy(c);
    node->cost = cost;
    node->heuristic = heuristic;
    return node;
}

static void astar_node_free(astar_node_t* node) {
    if (!node) return;
    coord_free(node->coord);
    delete node;
}

bool find_goal_bfs(const coord_t* start,
                   is_reachable_func is_reachable_fn,
                   void* user_data,
                   int max_range,
                   coord_t* out_result)
{
    if (!start || !out_result || !is_reachable_fn || 
        max_range <= 0 || max_range > MAX_RANGE_LIMIT)
        return false;

    std::queue<coord_t*> q;
    std::unordered_set<uint64_t> visited;

    coord_t* start_copy = coord_copy(start);
    q.push(start_copy);
    visited.insert(coord_pack(start));

    coord_t* offsets[4];
    offsets[0] = coord_new_full(0, -1);
    offsets[1] = coord_new_full(1, 0);
    offsets[2] = coord_new_full(0, 1);
    offsets[3] = coord_new_full(-1, 0);

    while (!q.empty()) {
        coord_t* cur = q.front();
        q.pop();

        if (is_reachable_fn(cur, user_data)) {
            coord_set(out_result, coord_get_x(cur), coord_get_y(cur));
            coord_free(cur);
            while (!q.empty()) {
                coord_free(q.front());
                q.pop();
            }
            for (int i = 0; i < 4; ++i) coord_free(offsets[i]);
            return true;
        }

        for (int i = 0; i < 4; ++i) {
            coord_t* next = coord_new();
            coord_set(next,
                      coord_get_x(cur) + coord_get_x(offsets[i]),
                      coord_get_y(cur) + coord_get_y(offsets[i]));
            if (coord_distance(start, next) > max_range) {
                coord_free(next);
                continue;
            }
            if (!visited.insert(coord_pack(next)).second) {
                coord_free(next);
                continue;
            }
            q.push(next);
        }

        coord_free(cur);
    }

    for (int i = 0; i < 4; ++i) coord_free(offsets[i]);
    coord_set(out_result, -1, -1);
    return false;
}

int astar_node_compare(const astar_node_t* a, const astar_node_t* b) {
    int fa = a->cost + a->heuristic;
    int fb = b->cost + b->heuristic;
    return fa - fb;
}

struct AStarCompare {
    bool operator()(const astar_node_t* a, const astar_node_t* b) const {
        return (a->cost + a->heuristic) > (b->cost + b->heuristic);
    }
};

bool find_goal_astar(const coord_t* start,
                     is_reachable_func is_reachable_fn,
                     void* user_data,
                     int max_range,
                     coord_t* out_result)
{
    if (!start || !out_result || !is_reachable_fn || max_range <= 0)
        return false;

    std::priority_queue<astar_node_t*, std::vector<astar_node_t*>, 
        AStarCompare> open;
    std::unordered_set<uint64_t> visited;

    astar_node_t* start_node = astar_node_new(start, 0, 0);
    open.push(start_node);
    visited.insert(coord_pack(start));

    coord_t* offsets[4];
    offsets[0] = coord_new_full(0, -1);
    offsets[1] = coord_new_full(1, 0);
    offsets[2] = coord_new_full(0, 1);
    offsets[3] = coord_new_full(-1, 0);

    while (!open.empty()) {
        astar_node_t* current = open.top();
        open.pop();

        if (is_reachable_fn(current->coord, user_data)) {
            coord_set(out_result, coord_get_x(current->coord), 
                coord_get_y(current->coord));
            astar_node_free(current);
            while (!open.empty()) {
                astar_node_free(open.top());
                open.pop();
            }
            for (int i = 0; i < 4; ++i) coord_free(offsets[i]);
            return true;
        }

        for (int i = 0; i < 4; ++i) {
            coord_t* next = coord_new();
            coord_set(next,
                      coord_get_x(current->coord) + coord_get_x(offsets[i]),
                      coord_get_y(current->coord) + coord_get_y(offsets[i]));
            if (coord_distance(start, next) > max_range) {
                coord_free(next);
                continue;
            }
            if (!visited.insert(coord_pack(next)).second) {
                coord_free(next);
                continue;
            }
            astar_node_t* node = astar_node_new(next, 
                current->cost + 1, coord_distance(start, next));
                
            open.push(node);
            coord_free(next);  // node 내부에서 복사됨
        }

        astar_node_free(current);
    }

    for (int i = 0; i < 4; ++i) coord_free(offsets[i]);
    coord_set(out_result, -1, -1);
    return false;
}
