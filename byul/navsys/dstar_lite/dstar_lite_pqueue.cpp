#include "dstar_lite_pqueue.h"
#include "coord.h"
#include "dstar_lite_key.hpp"
#include "float_common.h"
#include "coord_hash.h"

#include <map>
#include <vector>
#include <algorithm>

struct s_dstar_lite_pqueue {
    std::map<dstar_lite_key_t*, std::vector<coord_t*>, 
        DstarLiteKeyLess> key_to_coords;
        
    coord_hash_t* coord_to_key;
};

dstar_lite_pqueue_t* dstar_lite_pqueue_create() {
    auto* q = new dstar_lite_pqueue_t{};
    q->coord_to_key = coord_hash_create_full(
        (coord_hash_copy_func)dstar_lite_key_copy,
        (coord_hash_destroy_func)dstar_lite_key_destroy
    );
    return q;
}

void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q) {
    if (!q) return;
    for (auto& [key, vec] : q->key_to_coords) {
        for (coord_t* c : vec) coord_destroy(c);
        dstar_lite_key_destroy(key);
    }
    coord_hash_destroy(q->coord_to_key);
    delete q;
}

dstar_lite_pqueue_t* dstar_lite_pqueue_copy(const dstar_lite_pqueue_t* src) {
    if (!src) return nullptr;

    auto* copy = new dstar_lite_pqueue_t{};
    copy->coord_to_key = coord_hash_copy(src->coord_to_key);

    for (const auto& [key, coords] : src->key_to_coords) {
        std::vector<coord_t*> copied_coords;
        for (coord_t* c : coords)
            copied_coords.push_back(coord_copy(c));
        dstar_lite_key_t* copied_key = dstar_lite_key_copy(key);
        copy->key_to_coords[copied_key] = std::move(copied_coords);
    }

    return copy;
}

void dstar_lite_pqueue_push(dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key, const coord_t* c) {
    if (!q || !key || !c) return;

    for (auto& [k, vec] : q->key_to_coords) {
        if (dstar_lite_key_equal(k, key)) {
            vec.push_back(coord_copy(c));
            coord_hash_replace(q->coord_to_key, c, k);
            return;
        }
    }

    dstar_lite_key_t* new_key = dstar_lite_key_copy(key);
    std::vector<coord_t*> vec;
    vec.push_back(coord_copy(c));
    q->key_to_coords[new_key] = vec;
    coord_hash_replace(q->coord_to_key, c, new_key);
}

const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q) {
    if (!q || q->key_to_coords.empty()) return nullptr;
    const auto& entry = *q->key_to_coords.begin();
    return entry.second.empty() ? nullptr : entry.second.front();
}

coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q) {
    if (!q || q->key_to_coords.empty()) return nullptr;

    auto it = q->key_to_coords.begin();
    auto& vec = it->second;

    if (vec.empty()) {
        dstar_lite_key_destroy(it->first);
        q->key_to_coords.erase(it);
        return nullptr;
    }

    coord_t* popped = vec.front();
    vec.erase(vec.begin());
    coord_hash_remove(q->coord_to_key, popped);

    if (vec.empty()) {
        dstar_lite_key_destroy(it->first);
        q->key_to_coords.erase(it);
    }

    return popped;
}

bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q) {
    return !q || q->key_to_coords.empty();
}

bool dstar_lite_pqueue_remove(dstar_lite_pqueue_t* q, const coord_t* u) {
    if (!q || !u) return false;
    auto* key_ptr = static_cast<dstar_lite_key_t*>(coord_hash_get(q->coord_to_key, u));
    if (!key_ptr) return false;

    for (auto it = q->key_to_coords.begin(); it != q->key_to_coords.end(); ++it) {
        if (dstar_lite_key_equal(it->first, key_ptr)) {
            auto& vec = it->second;
            auto found = std::find_if(vec.begin(), vec.end(), [&](coord_t* c) {
                return coord_equal(c, u);
            });
            if (found != vec.end()) {
                coord_destroy(*found);
                vec.erase(found);
                coord_hash_remove(q->coord_to_key, u);
                if (vec.empty()) {
                    dstar_lite_key_destroy(it->first);
                    q->key_to_coords.erase(it);
                }
                return true;
            }
        }
    }
    return false;
}

bool dstar_lite_pqueue_remove_full(dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key, const coord_t* c) {
    if (!q || !key || !c) return false;

    for (auto it = q->key_to_coords.begin(); it != q->key_to_coords.end(); ++it) {
        if (dstar_lite_key_equal(it->first, key)) {
            auto& vec = it->second;
            auto found = std::find_if(vec.begin(), vec.end(), [&](coord_t* item) {
                return coord_equal(item, c);
            });
            if (found != vec.end()) {
                coord_destroy(*found);
                vec.erase(found);
                coord_hash_remove(q->coord_to_key, c);
                if (vec.empty()) {
                    dstar_lite_key_destroy(it->first);
                    q->key_to_coords.erase(it);
                }
                return true;
            }
        }
    }
    return false;
}

dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(dstar_lite_pqueue_t* q, const coord_t* c) {
    if (!q || !c) return nullptr;
    return static_cast<dstar_lite_key_t*>(coord_hash_get(q->coord_to_key, c));
}

dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q) {
    if (!q || q->key_to_coords.empty()) return nullptr;
    return dstar_lite_key_copy(q->key_to_coords.begin()->first);
}

bool dstar_lite_pqueue_contains(dstar_lite_pqueue_t* q, const coord_t* u) {
    if (!q || !u) return false;
    return coord_hash_contains(q->coord_to_key, u);
}
