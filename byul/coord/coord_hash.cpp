#include "coord.hpp"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include <unordered_map>
#include <functional>
#include <cstdlib>

void* int_copy(const void* p) {
    int* out = new int;
    *out = *reinterpret_cast<const int*>(p);
    return out;
}

void int_free(void* p) {
    delete reinterpret_cast<int*>(p);
}

void* float_copy(const void* p) {
    float* out = new float;
    *out = *reinterpret_cast<const float*>(p);
    return out;
}

void float_free(void* p){
    delete reinterpret_cast<float*>(p);
}

void* double_copy(const void* p){
    double* out = new double;
    *out = *reinterpret_cast<const double*>(p);
    return out;
}

void double_free(void* p){
    delete reinterpret_cast<double*>(p);
}

struct s_coord_hash {
    std::unordered_map<coord_t*, void*, CoordHash, CoordEqual> data;
    coord_hash_copy_func value_copy_func = nullptr;
    coord_hash_free_func value_free_func = nullptr;
};

coord_hash_t* coord_hash_new() {
    return coord_hash_new_full(int_copy, int_free);
}

coord_hash_t* coord_hash_new_full(coord_hash_copy_func copy_func,
                                  coord_hash_free_func free_func) {
    auto* h = new s_coord_hash;
    h->value_copy_func = copy_func;
    h->value_free_func = free_func;
    return h;
}

void coord_hash_free(coord_hash_t* hash) {
    if (!hash) return;
    if (hash->value_free_func) {
        for (auto& [key, val] : hash->data) {
            if (val) hash->value_free_func(val);
        }
    }
    for (auto& [key, _] : hash->data) delete key;
    delete hash;
}

coord_hash_t* coord_hash_copy(const coord_hash_t* original) {
    if (!original) return nullptr;
    coord_hash_t* copy = coord_hash_new();
    copy->data = original->data;
    return copy;
}

bool coord_hash_insert(coord_hash_t* hash, const coord_t* key, void* value) {
    if (!hash || !key || !hash->value_copy_func) return false;

    coord_t* copy_key = coord_copy(key);
    
    void* copy_value = nullptr;
    if (value){
        copy_value = hash->value_copy_func(value);
    } 

    auto result = hash->data.emplace(copy_key, copy_value);
    if (!result.second) {
        delete copy_key;
        if (copy_value){
            if (hash->value_free_func) hash->value_free_func(copy_value);
        }
    }
    return result.second;
}

bool coord_hash_replace(coord_hash_t* hash, const coord_t* key, void* value) {
    if (!hash || !key || !hash->value_copy_func) return false;

    auto it = hash->data.find(const_cast<coord_t*>(key));
    if (it != hash->data.end()) {
        if (value) {
            if (hash->value_free_func) hash->value_free_func(it->second);
            it->second = hash->value_copy_func(value);
            return true;
        }
    }
    return coord_hash_insert(hash, key, value);
}

bool coord_hash_remove(coord_hash_t* hash, const coord_t* key) {
    if (!hash || !key) return false;
    auto it = hash->data.find(const_cast<coord_t*>(key));
    if (it != hash->data.end()) {
        if (hash->value_free_func) hash->value_free_func(it->second);
        delete it->first;
        hash->data.erase(it);
        return true;
    }
    return false;
}

void coord_hash_clear(coord_hash_t* hash) {
    if (!hash) return;
    if (hash->value_free_func) {
        for (auto& [_, val] : hash->data) hash->value_free_func(val);
    }
    for (auto& [key, _] : hash->data) delete key;
    hash->data.clear();
}

int coord_hash_length(const coord_hash_t* hash) {
    return hash ? static_cast<int>(hash->data.size()) : 0;
}

bool coord_hash_is_empty(const coord_hash_t* hash) {
    return !hash || hash->data.empty();
}

void* coord_hash_get(const coord_hash_t* hash, const coord_t* key) {
    if (!hash || !key) return nullptr;
    auto it = hash->data.find(const_cast<coord_t*>(key));
    return it != hash->data.end() ? it->second : nullptr;
}

bool coord_hash_contains(const coord_hash_t* hash, const coord_t* key) {
    return hash && key && hash->data.find(const_cast<coord_t*>(key)) != hash->data.end();
}

void coord_hash_set(coord_hash_t* hash, const coord_t* key, void* value) {
    if (hash && key) hash->data[const_cast<coord_t*>(key)] = value;
}

void coord_hash_remove_all(coord_hash_t* hash) {
    coord_hash_clear(hash);
}

uint32_t coord_hash_hash(const coord_hash_t* h) {
    if (!h) return 0;

    uint32_t hash = 0;
    coord_hash_iter_t* it = coord_hash_iter_new(h);
    coord_t* key;
    void* val;

    while (coord_hash_iter_next(it, &key, &val)) {
        uint32_t h1 = (uint32_t)key->x * 73856093;
        uint32_t h2 = (uint32_t)key->y * 19349663;
        hash ^= (h1 ^ h2);
    }

    coord_hash_iter_free(it);
    return hash;
}

bool coord_hash_equal(const coord_hash_t* a, const coord_hash_t* b) {
    return a && b && a->data == b->data;
}

coord_list_t* coord_hash_keys(const coord_hash_t* h) {
    if (!h) return nullptr;
    coord_list_t* list = coord_list_new();
    for (const auto& [k, _] : h->data) coord_list_push_back(list, k);
    return list;
}

coord_list_t* coord_hash_to_list(const coord_hash_t* hash) {
    if (!hash) return nullptr;
    coord_list_t* list = coord_list_new();
    for (const auto& [key, _] : hash->data) {
        coord_list_push_back(list, key);
    }
    return list;
}

void** coord_hash_values(const coord_hash_t* hash, int* out_count) {
    if (!hash || !out_count) return nullptr;
    int n = static_cast<int>(hash->data.size());
    void** result = static_cast<void**>(malloc(sizeof(void*) * n));
    int i = 0;
    for (const auto& [key, val] : hash->data) {
        result[i++] = val;
    }
    *out_count = n;
    return result;
}

void coord_hash_foreach(
    coord_hash_t* hash, coord_hash_func func, void* user_data) {

    if (!hash || !func) return;
    for (auto& [key, value] : hash->data) {
        func(key, value, user_data);
    }
}

void coord_hash_export(const coord_hash_t* hash, 
    coord_list_t* keys_out, void** values_out, int* count_out) {
        
    if (!hash || !keys_out || !values_out || !count_out) return;
    int i = 0;
    for (const auto& [key, val] : hash->data) {
        coord_list_push_back(keys_out, key);
        values_out[i++] = val;
    }
    *count_out = static_cast<int>(hash->data.size());
}

typedef struct s_coord_hash_iter {
    const std::unordered_map<coord_t*, void*, CoordHash, CoordEqual>* data;
    std::unordered_map<coord_t*, void*, CoordHash, CoordEqual>::const_iterator it;
    std::unordered_map<coord_t*, void*, CoordHash, CoordEqual>::const_iterator end;
} coord_hash_iter_t;

coord_hash_iter_t* coord_hash_iter_new(const coord_hash_t* hash) {
    if (!hash) return nullptr;

    auto* iter = new coord_hash_iter_t;
    iter->data = &hash->data;
    iter->it = hash->data.begin();
    iter->end = hash->data.end();
    return iter;
}

bool coord_hash_iter_next(coord_hash_iter_t* iter, coord_t** key_out, void** val_out) {
    if (!iter || iter->it == iter->end) return false;

    if (key_out) *key_out = iter->it->first;
    if (val_out) *val_out = iter->it->second;

    ++(iter->it);
    return true;
}

void coord_hash_iter_free(coord_hash_iter_t* iter) {
    delete iter;
}
