#include "strset.h"
#include <unordered_set>
#include <string>
#include <cstring>

struct s_strset {
    std::unordered_set<std::string> set;
};

// ---------------------- 생성/소멸 ----------------------

strset_t* strset_create() {
    return new s_strset();
}

void strset_destroy(strset_t* ss) {
    delete ss;
}

// ---------------------- 기본 연산 ----------------------

bool strset_add(strset_t* ss, const char* item) {
    return ss && item && ss->set.insert(item).second;
}

bool strset_contains(const strset_t* ss, const char* item) {
    return ss && item && ss->set.count(item) > 0;
}

bool strset_remove(strset_t* ss, const char* item) {
    return ss && item && ss->set.erase(item) > 0;
}

size_t strset_size(const strset_t* ss) {
    return ss ? ss->set.size() : 0;
}

void strset_clear(strset_t* ss) {
    if (ss) ss->set.clear();
}

// ---------------------- Peek/Pop ----------------------

const char* strset_peek(const strset_t* ss) {
    if (!ss || ss->set.empty()) return nullptr;
    return ss->set.begin()->c_str();
}

char* strset_pop(strset_t* ss) {
    if (!ss || ss->set.empty()) return nullptr;
    auto it = ss->set.begin();
    std::string item = *it;
    ss->set.erase(it);
    char* result = (char*)malloc(item.size() + 1);
    strcpy(result, item.c_str());
    return result;
}

// ---------------------- 순회 ----------------------

void strset_foreach(strset_t* ss, strset_func fn, void* userdata) {
    if (!ss || !fn) return;
    for (auto& item : ss->set) {
        fn(item.c_str(), userdata);
    }
}

// ---------------------- 복사/비교 ----------------------

strset_t* strset_copy(const strset_t* original) {
    if (!original) return nullptr;
    strset_t* copy = new s_strset();
    copy->set = original->set;
    return copy;
}

bool strset_equal(const strset_t* a, const strset_t* b) {
    return a && b && a->set == b->set;
}

// ---------------------- 집합 연산 ----------------------

strset_t* strset_union(const strset_t* a, const strset_t* b) {
    if (!a || !b) return nullptr;
    strset_t* result = strset_copy(a);
    result->set.insert(b->set.begin(), b->set.end());
    return result;
}

strset_t* strset_intersect(const strset_t* a, const strset_t* b) {
    if (!a || !b) return nullptr;
    strset_t* result = new s_strset();
    for (auto& item : a->set) {
        if (b->set.count(item)) result->set.insert(item);
    }
    return result;
}

strset_t* strset_difference(const strset_t* a, const strset_t* b) {
    if (!a || !b) return nullptr;
    strset_t* result = new s_strset();
    for (auto& item : a->set) {
        if (!b->set.count(item)) result->set.insert(item);
    }
    return result;
}
