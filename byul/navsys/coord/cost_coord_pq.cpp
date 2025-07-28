#include "cost_coord_pq.h"
#include "coord.hpp"   // coord_t, coord_hash, coord_equal
#include <map>
#include <vector>
#include <algorithm>

struct s_cost_coord_pq {
    std::map<float, std::vector<coord_t*>> cost_buckets;
};

cost_coord_pq_t* cost_coord_pq_create() {
    return new s_cost_coord_pq();
}

void cost_coord_pq_destroy(cost_coord_pq_t* pq) {
    if (!pq) return;
    for (auto& [_, vec] : pq->cost_buckets) {
        for (coord_t* c : vec) {
            coord_destroy(c);
        }
    }
    delete pq;
}

void cost_coord_pq_push(cost_coord_pq_t* pq, float cost, const coord_t* c) {
    if (!pq || !c) return;
    coord_t* copy = coord_copy(c);
    pq->cost_buckets[cost].push_back(copy);
}

coord_t* cost_coord_pq_peek(cost_coord_pq_t* pq) {
    if (!pq || pq->cost_buckets.empty()) return nullptr;
    auto& vec = pq->cost_buckets.begin()->second;
    return !vec.empty() ? vec.back() : nullptr;
}

coord_t* cost_coord_pq_pop(cost_coord_pq_t* pq) {
    if (!pq || pq->cost_buckets.empty()) return nullptr;
    auto it = pq->cost_buckets.begin();
    auto& vec = it->second;
    if (vec.empty()) return nullptr;

    coord_t* c = vec.back();
    vec.pop_back();
    if (vec.empty()) pq->cost_buckets.erase(it);
    return c;
}

float cost_coord_pq_peek_cost(cost_coord_pq_t* pq) {
    if (!pq || pq->cost_buckets.empty()) return 0.0f;
    auto it = pq->cost_buckets.begin();
    return !it->second.empty() ? it->first : 0.0f;
}

bool cost_coord_pq_is_empty(cost_coord_pq_t* pq) {
    if (!pq) return true;
    for (const auto& [_, vec] : pq->cost_buckets) {
        if (!vec.empty()) return false;
    }
    return true;
}

bool cost_coord_pq_contains(cost_coord_pq_t* pq, const coord_t* c) {
    if (!pq || !c) return false;
    for (const auto& [_, vec] : pq->cost_buckets) {
        for (coord_t* v : vec) {
            if (coord_equal(v, c)) return true;
        }
    }
    return false;
}

bool cost_coord_pq_remove(cost_coord_pq_t* pq, float cost, const coord_t* c) {
    if (!pq || !c) return false;
    auto it = pq->cost_buckets.find(cost);
    if (it == pq->cost_buckets.end()) return false;

    auto& vec = it->second;
    auto v_it = std::remove_if(vec.begin(), vec.end(), [&](coord_t* v) {
        if (coord_equal(v, c)) {
            coord_destroy(v);
            return true;
        }
        return false;
    });

    bool removed = v_it != vec.end();
    vec.erase(v_it, vec.end());
    if (vec.empty()) pq->cost_buckets.erase(it);
    return removed;
}

int cost_coord_pq_length(cost_coord_pq_t* pq) {
    if (!pq) return 0;
    int count = 0;
    for (const auto& [_, vec] : pq->cost_buckets) {
        count += static_cast<int>(vec.size());
    }
    return count;
}

void cost_coord_pq_trim_worst(cost_coord_pq_t* pq, int n) {
    if (!pq || n <= 0) return;
    int removed = 0;
    auto it = pq->cost_buckets.rbegin();
    while (it != pq->cost_buckets.rend() && removed < n) {
        auto& vec = it->second;
        while (!vec.empty() && removed < n) {
            coord_destroy(vec.back());
            vec.pop_back();
            ++removed;
        }
        if (vec.empty()) {
            auto erase_it = std::next(it).base();
            pq->cost_buckets.erase(erase_it);
        } else {
            ++it;
        }
    }
}
