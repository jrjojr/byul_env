#include "dstar_lite_pqueue.h"
#include "internal/dstar_lite_key_ops.hpp"

#include <map>
#include <set>

namespace {

struct coord_less final {
    bool operator()(const coord_t& lhs, const coord_t& rhs) const noexcept {
        return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    }
};

using key_less = byul::navsys::dstar_lite_detail::key_less;

} // namespace

struct s_dstar_lite_pqueue {
    std::map<dstar_lite_key_t, std::set<coord_t, coord_less>, key_less> entries;
    std::map<coord_t, dstar_lite_key_t, coord_less> keys_by_coord;
};

namespace {

void erase_coord(dstar_lite_pqueue_t& queue, const coord_t& coord) {
    const auto reverse = queue.keys_by_coord.find(coord);
    if (reverse == queue.keys_by_coord.end()) return;
    const auto entry = queue.entries.find(reverse->second);
    if (entry != queue.entries.end()) {
        entry->second.erase(coord);
        if (entry->second.empty()) queue.entries.erase(entry);
    }
    queue.keys_by_coord.erase(reverse);
}

void insert_coord(
    dstar_lite_pqueue_t& queue,
    const coord_t& coord,
    const dstar_lite_key_t& key) {
    erase_coord(queue, coord);
    queue.entries[key].insert(coord);
    queue.keys_by_coord.emplace(coord, key);
}

} // namespace

navsys_status_t dstar_lite_pqueue_create_ex(
    dstar_lite_pqueue_t** out_queue) {
    if (!out_queue) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        dstar_lite_pqueue_t* result = new dstar_lite_pqueue_t{};
        *out_queue = result;
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t dstar_lite_pqueue_copy_ex(
    const dstar_lite_pqueue_t* source,
    dstar_lite_pqueue_t** out_queue) {
    if (!source || !out_queue) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        dstar_lite_pqueue_t* result = new dstar_lite_pqueue_t(*source);
        *out_queue = result;
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t dstar_lite_pqueue_upsert(
    dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    const dstar_lite_key_t* key) {
    if (!queue || !coord || !key) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        dstar_lite_pqueue_t replacement(*queue);
        insert_coord(replacement, *coord, *key);
        queue->entries.swap(replacement.entries);
        queue->keys_by_coord.swap(replacement.keys_by_coord);
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t dstar_lite_pqueue_peek_min(
    const dstar_lite_pqueue_t* queue,
    dstar_lite_pqueue_entry_t* out_entry) {
    if (!queue || !out_entry) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (queue->entries.empty() || queue->entries.begin()->second.empty())
        return NAVSYS_STATUS_NOT_FOUND;
    const dstar_lite_pqueue_entry_t result{
        queue->entries.begin()->first,
        *queue->entries.begin()->second.begin()};
    *out_entry = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_pqueue_pop_min(
    dstar_lite_pqueue_t* queue,
    dstar_lite_pqueue_entry_t* out_entry) {
    if (!queue || !out_entry) return NAVSYS_STATUS_INVALID_ARGUMENT;
    dstar_lite_pqueue_entry_t result{};
    const navsys_status_t status = dstar_lite_pqueue_peek_min(queue, &result);
    if (status != NAVSYS_STATUS_OK) return status;
    erase_coord(*queue, result.coord);
    *out_entry = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_pqueue_find_key(
    const dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    dstar_lite_key_t* out_key,
    bool* out_found) {
    if (!queue || !coord || !out_key || !out_found)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const auto found = queue->keys_by_coord.find(*coord);
    if (found == queue->keys_by_coord.end()) {
        *out_found = false;
        return NAVSYS_STATUS_OK;
    }
    *out_key = found->second;
    *out_found = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_pqueue_remove_ex(
    dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    bool* out_removed) {
    if (!queue || !coord || !out_removed)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const bool removed = queue->keys_by_coord.find(*coord)
        != queue->keys_by_coord.end();
    erase_coord(*queue, *coord);
    *out_removed = removed;
    return NAVSYS_STATUS_OK;
}

size_t dstar_lite_pqueue_size(const dstar_lite_pqueue_t* queue) {
    return queue ? queue->keys_by_coord.size() : 0u;
}

bool dstar_lite_pqueue_empty(const dstar_lite_pqueue_t* queue) {
    return !queue || queue->keys_by_coord.empty();
}

void dstar_lite_pqueue_clear(dstar_lite_pqueue_t* queue) {
    if (!queue) return;
    queue->entries.clear();
    queue->keys_by_coord.clear();
}

dstar_lite_pqueue_t* dstar_lite_pqueue_create(void) {
    dstar_lite_pqueue_t* result = nullptr;
    return dstar_lite_pqueue_create_ex(&result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q) {
    delete q;
}

dstar_lite_pqueue_t* dstar_lite_pqueue_copy(const dstar_lite_pqueue_t* src) {
    dstar_lite_pqueue_t* result = nullptr;
    return dstar_lite_pqueue_copy_ex(src, &result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

void dstar_lite_pqueue_push(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c) {
    (void)dstar_lite_pqueue_upsert(q, c, key);
}

const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q) {
    if (!q || q->entries.empty() || q->entries.begin()->second.empty())
        return nullptr;
    return &*q->entries.begin()->second.begin();
}

coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q) {
    if (!q || q->entries.empty()) return nullptr;
    try {
        auto key_it = q->entries.begin();
        if (key_it->second.empty()) {
            q->entries.erase(key_it);
            return nullptr;
        }
        const coord_t value = *key_it->second.begin();
        coord_t* result = coord_create_full(value.x, value.y);
        if (!result) return nullptr;
        key_it->second.erase(key_it->second.begin());
        q->keys_by_coord.erase(value);
        if (key_it->second.empty()) q->entries.erase(key_it);
        return result;
    } catch (...) {
        return nullptr;
    }
}

bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q) {
    return !q || q->keys_by_coord.empty();
}

bool dstar_lite_pqueue_remove(dstar_lite_pqueue_t* q, const coord_t* u) {
    bool removed = false;
    return dstar_lite_pqueue_remove_ex(q, u, &removed) == NAVSYS_STATUS_OK
        && removed;
}

bool dstar_lite_pqueue_remove_full(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c) {
    if (!q || !key || !c) return false;
    try {
        const auto reverse_it = q->keys_by_coord.find(*c);
        if (reverse_it == q->keys_by_coord.end()
            || !dstar_lite_key_equal_exact(&reverse_it->second, key)) {
            return false;
        }
        return dstar_lite_pqueue_remove(q, c);
    } catch (...) {
        return false;
    }
}

dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(
    dstar_lite_pqueue_t* q,
    const coord_t* c) {
    if (!q || !c) return nullptr;
    try {
        const auto it = q->keys_by_coord.find(*c);
        return it == q->keys_by_coord.end()
            ? nullptr
            : const_cast<dstar_lite_key_t*>(&it->second);
    } catch (...) {
        return nullptr;
    }
}

dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q) {
    if (!q || q->entries.empty()) return nullptr;
    dstar_lite_key_t* result = nullptr;
    return dstar_lite_key_create_ex(
        q->entries.begin()->first.k1,
        q->entries.begin()->first.k2,
        &result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

bool dstar_lite_pqueue_contains(dstar_lite_pqueue_t* q, const coord_t* u) {
    if (!q || !u) return false;
    try {
        return q->keys_by_coord.find(*u) != q->keys_by_coord.end();
    } catch (...) {
        return false;
    }
}
