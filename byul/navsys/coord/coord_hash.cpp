#include "coord.hpp"
#include "coord_list.h"
#include "coord_hash.h"
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <cstring>

void* int_copy(const void* p) {
    if (!p) return nullptr;
    return new int(*reinterpret_cast<const int*>(p));
}

void int_destroy(void* p) {
    if (p) delete reinterpret_cast<int*>(p);
}

void* scalar_copy(const void* p) {
    if (!p) return nullptr;
    return new float(*reinterpret_cast<const float*>(p));
}

void scalar_destroy(void* p) {
    if (p) delete reinterpret_cast<float*>(p);
}

void* double_copy(const void* p) {
    if (!p) return nullptr;
    return new double(*reinterpret_cast<const double*>(p));
}

void double_destroy(void* p) {
    if (p) delete reinterpret_cast<double*>(p);
}

typedef struct s_coord_hash {
    //std::unordered_map<coord_t, void*, CoordHash, CoordEqual> data;
    std::unordered_map<coord_t, void*> data;
    coord_hash_copy_func value_copy_func = nullptr;
    coord_hash_destroy_func value_destroy_func = nullptr;
}coord_hash_t;

coord_hash_t* coord_hash_create() {
    return coord_hash_create_full(int_copy, int_destroy);
}

coord_hash_t* coord_hash_create_full(coord_hash_copy_func copy_func,
    coord_hash_destroy_func destroy_func) {
    auto* h = new s_coord_hash;
    h->data.clear();
    h->value_copy_func = copy_func;
    h->value_destroy_func = destroy_func;
    return h;
}

void coord_hash_destroy(coord_hash_t* hash) {
    if (!hash) return;
    if (hash->value_destroy_func) {
        for (auto& [key, val] : hash->data) {
            if (val) hash->value_destroy_func(val);
        }
    }
    delete hash;
}

coord_hash_t* coord_hash_copy(const coord_hash_t* original) {
    if (!original) return nullptr;
    auto* copy = coord_hash_create_full(
        original->value_copy_func, original->value_destroy_func);

    for (const auto& [key, val] : original->data) {
        void* val_copy = val && copy->value_copy_func ?
            copy->value_copy_func(val) : nullptr;
        copy->data[key] = val_copy;
    }
    return copy;
}

bool coord_hash_insert(coord_hash_t* hash, const coord_t* key, void* value) {
    if (!hash || !key || !hash->value_copy_func) return false;
    auto iter = hash->data.find(*key);
    if (iter != hash->data.end()) {
        if (hash->value_destroy_func && iter->second) {
            hash->value_destroy_func(iter->second);
        }
        iter->second = value ? hash->value_copy_func(value) : nullptr;
        return true;
    }
    void* copy_value = value ? hash->value_copy_func(value) : nullptr;
    hash->data[*key] = copy_value;
    return true;
}

bool coord_hash_insert_xy(coord_hash_t* hash, int x, int y, void* value) {
    coord_t temp = { x, y };
    return coord_hash_insert(hash, &temp, value);
}

bool coord_hash_replace(coord_hash_t* hash, const coord_t* key, void* value) {
    if (!hash || !key) return false;
    return coord_hash_insert(hash, key, value);
}

bool coord_hash_replace_xy(coord_hash_t* hash, int x, int y, void* value) {
    coord_t temp = { x, y };
    return coord_hash_replace(hash, &temp, value);
}

bool coord_hash_remove(coord_hash_t* hash, const coord_t* key) {
    if (!hash || !key) return false;
    auto it = hash->data.find(*key);
    if (it != hash->data.end()) {
        if (it->second && hash->value_destroy_func)
            hash->value_destroy_func(it->second);
        hash->data.erase(it);
        return true;
    }
    return false;
}

void coord_hash_clear(coord_hash_t* hash) {
    if (!hash) return;
    if (hash->value_destroy_func) {
        for (auto& [_, val] : hash->data) hash->value_destroy_func(val);
    }
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
    auto it = hash->data.find(*key);
    return (it != hash->data.end()) ? it->second : nullptr;
}

void* coord_hash_get_xy(const coord_hash_t* hash, int x, int y) {
    if (!hash) return nullptr;
    coord_t key = { x, y };
    auto it = hash->data.find(key);
    return (it != hash->data.end()) ? it->second : nullptr;
}

bool coord_hash_contains(const coord_hash_t* hash, const coord_t* key) {
    return hash && key && hash->data.find(*key) != hash->data.end();
}

void coord_hash_set(coord_hash_t* hash, const coord_t* key, void* value) {
    if (hash && key) hash->data[*key] = value;
}

void coord_hash_remove_all(coord_hash_t* hash) {
    coord_hash_clear(hash);
}

uint32_t coord_hash_hash(const coord_hash_t* h) {
    if (!h) return 0;
    uint32_t hash = 0;
    coord_hash_iter_t* it = coord_hash_iter_create(h);
    coord_t key;
    void* val;
    while (coord_hash_iter_next(it, &key, &val)) {
        uint32_t h1 = (uint32_t)key.x * 73856093;
        uint32_t h2 = (uint32_t)key.y * 19349663;
        hash ^= (h1 ^ h2);
    }
    coord_hash_iter_destroy(it);
    return hash;
}

bool coord_hash_equal(const coord_hash_t* a, const coord_hash_t* b) {
    if (!a || !b || a->data.size() != b->data.size()) return false;
    for (const auto& [key_a, val_a] : a->data) {
        auto it_b = b->data.find(key_a);
        if (it_b == b->data.end()) return false;
        bool a_null = (val_a == nullptr);
        bool b_null = (it_b->second == nullptr);
        if (a_null != b_null) return false;
    }
    return true;
}

coord_list_t* coord_hash_keys(const coord_hash_t* h) {
    if (!h) return nullptr;
    coord_list_t* list = coord_list_create();
    for (const auto& [k, _] : h->data) coord_list_push_back(list, &k);
    return list;
}

coord_list_t* coord_hash_to_list(const coord_hash_t* hash) {
    if (!hash) return nullptr;
    coord_list_t* list = coord_list_create();
    for (const auto& [key, _] : hash->data) {
        coord_list_push_back(list, &key);
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
        func(const_cast<coord_t*>(&key), value, user_data);
    }
}

void coord_hash_export(const coord_hash_t* hash,
    coord_list_t* keys_out, void** values_out, int* count_out) {
    if (!hash || !keys_out || !values_out || !count_out) return;
    int i = 0;
    for (const auto& [key, val] : hash->data) {
        coord_list_push_back(keys_out, &key);
        values_out[i++] = val;
    }
    *count_out = static_cast<int>(hash->data.size());
}

typedef struct s_coord_hash_iter {
    //const std::unordered_map<coord_t, void*, CoordHash, CoordEqual>* data;
    //std::unordered_map<coord_t, void*, CoordHash, CoordEqual>::const_iterator it;
    //std::unordered_map<coord_t, void*, CoordHash, CoordEqual>::const_iterator end;

    const std::unordered_map<coord_t, void*>* data;
    std::unordered_map<coord_t, void*>::const_iterator it;
    std::unordered_map<coord_t, void*>::const_iterator end;
} coord_hash_iter_t;

coord_hash_iter_t* coord_hash_iter_create(const coord_hash_t* hash) {
    if (!hash) return nullptr;
    auto* iter = new coord_hash_iter_t;
    iter->data = &hash->data;
    iter->it = hash->data.begin();
    iter->end = hash->data.end();
    return iter;
}

bool coord_hash_iter_next(coord_hash_iter_t* iter, coord_t* key_out, void** val_out) {
    if (!iter || iter->it == iter->end) return false;
    if (key_out) *key_out = iter->it->first;
    if (val_out) *val_out = iter->it->second;
    ++(iter->it);
    return true;
}

void coord_hash_iter_destroy(coord_hash_iter_t* iter) {
    delete iter;
}

char* coord_hash_to_string(const coord_hash_t* hash) {
    if (!hash) return NULL;
    size_t buf_size = 1024;
    size_t len = 0;
    char* buffer = (char*)malloc(buf_size);
    if (!buffer) return NULL;
    buffer[0] = '\0';
    coord_hash_iter_t* iter = coord_hash_iter_create((coord_hash_t*)hash);
    coord_t key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        char entry[64];
        snprintf(entry, sizeof(entry), "(%d,%d) ", key.x, key.y);
        size_t entry_len = strlen(entry);
        if (len + entry_len + 1 >= buf_size) {
            buf_size *= 2;
            buffer = (char*)realloc(buffer, buf_size);
            if (!buffer) {
                coord_hash_iter_destroy(iter);
                return NULL;
            }
        }
        strcat(buffer, entry);
        len += entry_len;
    }
    coord_hash_iter_destroy(iter);
    return buffer;
}

void coord_hash_print(const coord_hash_t* hash) {
    char* str = coord_hash_to_string(hash);
    int len = coord_hash_length(hash);
    if (str) {
        printf("coords(len: %d): %s\n", len, str);
        free(str);
    }
    else {
        printf("coords: (null or empty)\n");
    }
}
