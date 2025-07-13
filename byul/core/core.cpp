#include "core.h"
#include <unordered_set>
#include <cmath>
#include <algorithm>

struct s_hashset {
    std::unordered_set<void*> set;
};

bool float_equal(float a, float b) {
    if (a == b) return true;
    float diff = std::fabs(a - b);
    float largest = std::fmax(std::fabs(a), std::fabs(b));
    return diff <= FLOAT_EPSILON * largest;
}

int float_compare(float a, float b, void*) {
    return float_equal(a, b) ? 0 : (a < b ? -1 : 1);
}

int int_compare(int a, int b, void*) {
    return (a > b) - (a < b);
}

hashset_t* hashset_new() {
    return new hashset_t();
}

void hashset_free(hashset_t* hs) {
    delete hs;
}

bool hashset_add(hashset_t* hs, hashkey item) {
    return hs && hs->set.insert(item).second;
}

bool hashset_contains(const hashset_t* hs, hashkey item) {
    return hs && hs->set.count(item);
}

bool hashset_remove(hashset_t* hs, hashkey item) {
    return hs && hs->set.erase(item);
}

size_t hashset_size(const hashset_t* hs) {
    return hs ? hs->set.size() : 0;
}

void hashset_clear(hashset_t* hs) {
    if (hs) hs->set.clear();
}

hashkey hashset_peek(hashset_t* hs) {
    return (hs && !hs->set.empty()) ? *hs->set.begin() : nullptr;
}

hashkey hashset_pop(hashset_t* hs) {
    if (!hs || hs->set.empty()) return nullptr;
    auto it = hs->set.begin();
    hashkey item = *it;
    hs->set.erase(it);
    return item;
}

void hashset_foreach(hashset_t* hs, hashset_func fn, valueptr userdata) {
    if (!hs || !fn) return;
    for (auto& item : hs->set) {
        fn(item, userdata);
    }
}

hashset_t* hashset_copy(const hashset_t* original) {
    if (!original) return nullptr;
    hashset_t* copy = new hashset_t();
    copy->set = original->set;
    return copy;
}

bool hashset_equal(const hashset_t* a, const hashset_t* b) {
    return a && b && a->set == b->set;
}

uint32_t hashset_hash(const hashkey key) {
    return std::hash<void*>{}(key);
}

hashset_t* hashset_union(const hashset_t* a, const hashset_t* b) {
    if (!a || !b) return nullptr;
    hashset_t* result = hashset_copy(a);
    result->set.insert(b->set.begin(), b->set.end());
    return result;
}

hashset_t* hashset_intersect(const hashset_t* a, const hashset_t* b) {
    if (!a || !b) return nullptr;
    hashset_t* result = new hashset_t();
    for (auto& item : a->set) {
        if (b->set.count(item)) result->set.insert(item);
    }
    return result;
}

hashset_t* hashset_difference(const hashset_t* a, const hashset_t* b) {
    if (!a || !b) return nullptr;
    hashset_t* result = new hashset_t();
    for (auto& item : a->set) {
        if (!b->set.count(item)) result->set.insert(item);
    }
    return result;
}
