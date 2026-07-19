#include "cost_coord_pq.h"

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <utility>

namespace {

struct coord_deleter {
    void operator()(coord_t* coord) const noexcept {
        coord_destroy(coord);
    }
};

struct cost_coord_entry {
    float cost;
    coord_t coord;
    std::uint64_t sequence;
    std::unique_ptr<coord_t, coord_deleter> legacy_peek;
};

struct float_total_less {
    bool operator()(float lhs, float rhs) const noexcept {
        const bool lhs_nan = std::isnan(lhs);
        const bool rhs_nan = std::isnan(rhs);
        if (lhs_nan || rhs_nan) {
            return !lhs_nan && rhs_nan;
        }
        return lhs < rhs;
    }
};

float canonical_cost(float cost) noexcept {
    return cost == 0.0f ? 0.0f : cost;
}

bool canonical_cost_valid(float cost) noexcept {
    return std::isfinite(cost) && cost >= 0.0f;
}

}  // namespace

struct s_cost_coord_pq {
    std::map<float, std::deque<cost_coord_entry>, float_total_less> buckets;
    std::uint64_t next_sequence = 0;
    std::size_t size = 0;
};

namespace {

void renumber_sequences(cost_coord_pq_t* queue) noexcept {
    std::uint64_t sequence = 0;
    for (auto& [_, bucket] : queue->buckets) {
        for (auto& entry : bucket) {
            entry.sequence = sequence++;
        }
    }
    queue->next_sequence = sequence;
}

navsys_status_t push_value(
    cost_coord_pq_t* queue,
    float cost,
    const coord_t& coord,
    bool validate_cost) {
    if (!queue || (validate_cost && !canonical_cost_valid(cost))) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    cost = canonical_cost(cost);
    if (queue->next_sequence == std::numeric_limits<std::uint64_t>::max()) {
        renumber_sequences(queue);
        if (queue->next_sequence
            == std::numeric_limits<std::uint64_t>::max()) {
            return NAVSYS_STATUS_LIMIT_REACHED;
        }
    }

    cost_coord_entry entry = {
        cost, coord, queue->next_sequence, nullptr
    };
    try {
        auto existing = queue->buckets.find(cost);
        if (existing != queue->buckets.end()) {
            existing->second.push_back(std::move(entry));
        } else {
            std::deque<cost_coord_entry> bucket;
            bucket.push_back(std::move(entry));
            queue->buckets.emplace(cost, std::move(bucket));
        }
        ++queue->next_sequence;
        ++queue->size;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

auto first_nonempty_bucket(cost_coord_pq_t* queue) {
    auto it = queue->buckets.begin();
    while (it != queue->buckets.end() && it->second.empty()) ++it;
    return it;
}

auto first_nonempty_bucket(const cost_coord_pq_t* queue) {
    auto it = queue->buckets.cbegin();
    while (it != queue->buckets.cend() && it->second.empty()) ++it;
    return it;
}

std::size_t remove_matching_from_bucket(
    std::deque<cost_coord_entry>& bucket,
    const coord_t& coord,
    bool one_only) noexcept {
    std::size_t removed = 0;
    for (auto entry = bucket.begin(); entry != bucket.end();) {
        if (coord_equal(&entry->coord, &coord)) {
            entry = bucket.erase(entry);
            ++removed;
            if (one_only) break;
        } else {
            ++entry;
        }
    }
    return removed;
}

}  // namespace

navsys_status_t cost_coord_pq_create_ex(
    const cost_coord_pq_create_info_t* info,
    cost_coord_pq_t** out_queue) {
    if (!info || !out_queue
        || info->struct_size != sizeof(cost_coord_pq_create_info_t)
        || info->abi_version != BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION
        || info->flags != 0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    try {
        std::unique_ptr<cost_coord_pq_t> queue(new cost_coord_pq_t());
        *out_queue = queue.release();
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t cost_coord_pq_push_ex(
    cost_coord_pq_t* queue, float cost, const coord_t* coord) {
    if (!queue || !coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const coord_t copied = *coord;
    return push_value(queue, cost, copied, true);
}

navsys_status_t cost_coord_pq_peek_min(
    const cost_coord_pq_t* queue,
    float* out_cost,
    coord_t* out_coord) {
    if (!queue || !out_cost || !out_coord) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const auto it = first_nonempty_bucket(queue);
    if (it == queue->buckets.cend()) return NAVSYS_STATUS_NOT_FOUND;
    const float cost = it->second.front().cost;
    const coord_t coord = it->second.front().coord;
    *out_cost = cost;
    *out_coord = coord;
    return NAVSYS_STATUS_OK;
}

navsys_status_t cost_coord_pq_pop_min(
    cost_coord_pq_t* queue,
    float* out_cost,
    coord_t* out_coord) {
    if (!queue || !out_cost || !out_coord) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const auto it = first_nonempty_bucket(queue);
    if (it == queue->buckets.end()) return NAVSYS_STATUS_NOT_FOUND;
    const float cost = it->second.front().cost;
    const coord_t coord = it->second.front().coord;
    it->second.pop_front();
    if (it->second.empty()) queue->buckets.erase(it);
    --queue->size;
    *out_cost = cost;
    *out_coord = coord;
    return NAVSYS_STATUS_OK;
}

size_t cost_coord_pq_size(const cost_coord_pq_t* queue) {
    return queue ? queue->size : 0;
}

bool cost_coord_pq_empty(const cost_coord_pq_t* queue) {
    return !queue || queue->size == 0;
}

navsys_status_t cost_coord_pq_remove_one(
    cost_coord_pq_t* queue,
    float cost,
    const coord_t* coord,
    bool* out_removed) {
    if (!queue || !coord || !out_removed || !canonical_cost_valid(cost)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t copied = *coord;
    const auto bucket = queue->buckets.find(canonical_cost(cost));
    if (bucket == queue->buckets.end()) {
        *out_removed = false;
        return NAVSYS_STATUS_OK;
    }
    const std::size_t removed =
        remove_matching_from_bucket(bucket->second, copied, true);
    queue->size -= removed;
    if (bucket->second.empty()) queue->buckets.erase(bucket);
    *out_removed = removed != 0;
    return NAVSYS_STATUS_OK;
}

navsys_status_t cost_coord_pq_remove_all(
    cost_coord_pq_t* queue,
    const coord_t* coord,
    size_t* out_removed_count) {
    if (!queue || !coord || !out_removed_count) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t copied = *coord;
    std::size_t removed = 0;
    for (auto bucket = queue->buckets.begin();
         bucket != queue->buckets.end();) {
        removed += remove_matching_from_bucket(
            bucket->second, copied, false);
        if (bucket->second.empty()) {
            bucket = queue->buckets.erase(bucket);
        } else {
            ++bucket;
        }
    }
    queue->size -= removed;
    *out_removed_count = removed;
    return NAVSYS_STATUS_OK;
}

void cost_coord_pq_clear(cost_coord_pq_t* queue) {
    if (!queue) return;
    queue->buckets.clear();
    queue->next_sequence = 0;
    queue->size = 0;
}

navsys_status_t cost_coord_pq_trim_to_size(
    cost_coord_pq_t* queue,
    size_t max_size,
    size_t* out_removed_count) {
    if (!queue || !out_removed_count) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::size_t original_size = queue->size;
    while (queue->size > max_size) {
        auto bucket = std::prev(queue->buckets.end());
        bucket->second.pop_back();
        --queue->size;
        if (bucket->second.empty()) queue->buckets.erase(bucket);
    }
    *out_removed_count = original_size - queue->size;
    return NAVSYS_STATUS_OK;
}

cost_coord_pq_t* cost_coord_pq_create(void) {
    const cost_coord_pq_create_info_t info = {
        (uint32_t)sizeof(cost_coord_pq_create_info_t),
        BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION,
        0
    };
    cost_coord_pq_t* queue = nullptr;
    return cost_coord_pq_create_ex(&info, &queue) == NAVSYS_STATUS_OK
        ? queue
        : nullptr;
}

void cost_coord_pq_destroy(cost_coord_pq_t* pq) {
    delete pq;
}

void cost_coord_pq_push(
    cost_coord_pq_t* pq, float cost, const coord_t* c) {
    if (!pq || !c) return;
    const coord_t copied = *c;
    (void)push_value(pq, cost, copied, false);
}

coord_t* cost_coord_pq_peek(cost_coord_pq_t* pq) {
    if (!pq) return nullptr;
    const auto it = first_nonempty_bucket(pq);
    if (it == pq->buckets.end()) return nullptr;
    auto& entry = it->second.back();
    if (!entry.legacy_peek) {
        entry.legacy_peek.reset(coord_copy(&entry.coord));
    }
    return entry.legacy_peek.get();
}

coord_t* cost_coord_pq_pop(cost_coord_pq_t* pq) {
    if (!pq) return nullptr;
    const auto it = first_nonempty_bucket(pq);
    if (it == pq->buckets.end()) return nullptr;
    auto& entry = it->second.back();
    coord_t* copied = entry.legacy_peek
        ? entry.legacy_peek.release()
        : coord_copy(&entry.coord);
    if (!copied) return nullptr;
    it->second.pop_back();
    if (it->second.empty()) pq->buckets.erase(it);
    --pq->size;
    return copied;
}

float cost_coord_pq_peek_cost(cost_coord_pq_t* pq) {
    if (!pq) return 0.0f;
    const auto it = first_nonempty_bucket(pq);
    return it == pq->buckets.end() ? 0.0f : it->first;
}

bool cost_coord_pq_is_empty(cost_coord_pq_t* pq) {
    return cost_coord_pq_empty(pq);
}

bool cost_coord_pq_contains(cost_coord_pq_t* pq, const coord_t* c) {
    if (!pq || !c) return false;
    for (const auto& [_, bucket] : pq->buckets) {
        for (const auto& entry : bucket) {
            if (coord_equal(&entry.coord, c)) return true;
        }
    }
    return false;
}

bool cost_coord_pq_remove(
    cost_coord_pq_t* pq, float cost, const coord_t* c) {
    if (!pq || !c) return false;
    const auto it = pq->buckets.find(canonical_cost(cost));
    if (it == pq->buckets.end()) return false;
    const std::size_t removed =
        remove_matching_from_bucket(it->second, *c, false);
    pq->size -= removed;
    if (it->second.empty()) pq->buckets.erase(it);
    return removed != 0;
}

int cost_coord_pq_length(cost_coord_pq_t* pq) {
    const std::size_t size = cost_coord_pq_size(pq);
    const auto maximum = static_cast<std::size_t>(
        std::numeric_limits<int>::max());
    return size > maximum
        ? std::numeric_limits<int>::max()
        : static_cast<int>(size);
}

void cost_coord_pq_trim_worst(cost_coord_pq_t* pq, int n) {
    if (!pq || n <= 0) return;
    const std::size_t remove_count = static_cast<std::size_t>(n);
    const std::size_t max_size = pq->size > remove_count
        ? pq->size - remove_count
        : 0;
    std::size_t removed = 0;
    (void)cost_coord_pq_trim_to_size(pq, max_size, &removed);
}
