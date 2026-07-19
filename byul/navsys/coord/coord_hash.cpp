#include "internal/coord_ops.hpp"
#include "coord_list.h"
#include "coord_hash.h"
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <new>
#include <string>
#include <unordered_map>
#include <vector>

struct coord_hash_iteration_state {
    uint64_t generation = 0;
    bool alive = true;
};

void* coord_hash_int_copy(const void* value) {
    if (!value) return nullptr;
    try {
        return new int(*reinterpret_cast<const int*>(value));
    } catch (...) {
        return nullptr;
    }
}

void coord_hash_int_destroy(void* value) {
    delete reinterpret_cast<int*>(value);
}

void* coord_hash_float_copy(const void* value) {
    if (!value) return nullptr;
    try {
        return new float(*reinterpret_cast<const float*>(value));
    } catch (...) {
        return nullptr;
    }
}

void coord_hash_float_destroy(void* value) {
    delete reinterpret_cast<float*>(value);
}

void* coord_hash_double_copy(const void* value) {
    if (!value) return nullptr;
    try {
        return new double(*reinterpret_cast<const double*>(value));
    } catch (...) {
        return nullptr;
    }
}

void coord_hash_double_destroy(void* value) {
    delete reinterpret_cast<double*>(value);
}

void* int_copy(const void* p) {
    return coord_hash_int_copy(p);
}

void int_destroy(void* p) {
    coord_hash_int_destroy(p);
}

void* scalar_copy(const void* p) {
    return coord_hash_float_copy(p);
}

void scalar_destroy(void* p) {
    coord_hash_float_destroy(p);
}

void* double_copy(const void* p) {
    return coord_hash_double_copy(p);
}

void double_destroy(void* p) {
    coord_hash_double_destroy(p);
}

typedef struct s_coord_hash {
    std::unordered_map<
        coord_t,
        void*,
        byul::navsys::detail::coord_hash,
        byul::navsys::detail::coord_equal> data;
    coord_hash_copy_func value_copy_func = nullptr;
    coord_hash_destroy_func value_destroy_func = nullptr;
    coord_hash_value_copy_func_ex value_copy_func_ex = nullptr;
    coord_hash_value_destroy_func_ex value_destroy_func_ex = nullptr;
    coord_hash_value_equal_func_ex value_equal_func_ex = nullptr;
    void* callback_userdata = nullptr;
    std::shared_ptr<coord_hash_iteration_state> iteration_state =
        std::make_shared<coord_hash_iteration_state>();
}coord_hash_t;

namespace {

thread_local const coord_hash_t* active_callback_hash = nullptr;
thread_local const coord_hash_t* secondary_callback_hash = nullptr;

class coord_hash_callback_scope {
public:
    explicit coord_hash_callback_scope(
        const coord_hash_t* hash,
        const coord_hash_t* secondary = nullptr)
        : previous_(active_callback_hash),
          previous_secondary_(secondary_callback_hash) {
        active_callback_hash = hash;
        secondary_callback_hash = secondary;
    }

    ~coord_hash_callback_scope() {
        active_callback_hash = previous_;
        secondary_callback_hash = previous_secondary_;
    }

private:
    const coord_hash_t* previous_;
    const coord_hash_t* previous_secondary_;
};

bool coord_hash_callback_is_active(const coord_hash_t* hash) noexcept {
    return hash
        && (active_callback_hash == hash || secondary_callback_hash == hash);
}

void coord_hash_mark_mutated(coord_hash_t* hash) noexcept {
    if (hash && hash->iteration_state) {
        ++hash->iteration_state->generation;
    }
}

coord_hash_t* coord_hash_allocate() noexcept {
    void* storage = nullptr;
    try {
        storage = ::operator new(sizeof(s_coord_hash));
        return ::new (storage) s_coord_hash;
    } catch (...) {
        ::operator delete(storage);
        return nullptr;
    }
}

void coord_hash_destroy_stored_value(
    coord_hash_t* hash, void* value) noexcept {
    if (!hash || !value) return;
    try {
        coord_hash_callback_scope scope(hash);
        if (hash->value_destroy_func_ex) {
            hash->value_destroy_func_ex(value, hash->callback_userdata);
        } else if (hash->value_destroy_func) {
            hash->value_destroy_func(value);
        }
    } catch (...) {
        // A destroy callback is required to be non-throwing. Keep exceptions
        // inside the C ABI even when a legacy callback violates that contract.
    }
}

navsys_status_t coord_hash_copy_stored_value(
    const coord_hash_t* hash,
    const void* source,
    void** out_copy) noexcept {
    if (!hash || !out_copy) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source) {
        *out_copy = nullptr;
        return NAVSYS_STATUS_OK;
    }

    void* copied = nullptr;
    if (hash->value_copy_func_ex) {
        navsys_status_t status = NAVSYS_STATUS_CALLBACK_FAILED;
        try {
            coord_hash_callback_scope scope(hash);
            status = hash->value_copy_func_ex(
                source, &copied, hash->callback_userdata);
        } catch (...) {
            status = NAVSYS_STATUS_CALLBACK_FAILED;
        }
        if (status != NAVSYS_STATUS_OK) {
            if (copied) {
                coord_hash_destroy_stored_value(
                    const_cast<coord_hash_t*>(hash), copied);
            }
            return status < NAVSYS_STATUS_OK
                ? status
                : NAVSYS_STATUS_CALLBACK_FAILED;
        }
        if (!copied) return NAVSYS_STATUS_CALLBACK_FAILED;
    } else if (hash->value_copy_func) {
        try {
            coord_hash_callback_scope scope(hash);
            copied = hash->value_copy_func(source);
        } catch (const std::bad_alloc&) {
            return NAVSYS_STATUS_OUT_OF_MEMORY;
        } catch (...) {
            return NAVSYS_STATUS_CALLBACK_FAILED;
        }
        if (!copied) return NAVSYS_STATUS_OUT_OF_MEMORY;
    } else {
        return NAVSYS_STATUS_UNSUPPORTED;
    }

    *out_copy = copied;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_hash_commit_new_entry(
    coord_hash_t* hash,
    const coord_t* key,
    void* copied) noexcept {
    try {
        hash->data.emplace(*key, copied);
        coord_hash_mark_mutated(hash);
        return NAVSYS_STATUS_OK;
    } catch (...) {
        coord_hash_destroy_stored_value(hash, copied);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

}

navsys_status_t coord_hash_create_ex(
    const coord_hash_create_info_t* info,
    coord_hash_t** out_hash) {
    if (!info || !out_hash
        || info->struct_size < sizeof(coord_hash_create_info_t)
        || info->abi_version != BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION
        || ((info->copy_value == nullptr)
            != (info->destroy_value == nullptr))) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    coord_hash_t* hash = coord_hash_allocate();
    if (!hash) return NAVSYS_STATUS_OUT_OF_MEMORY;

    hash->value_copy_func_ex = info->copy_value;
    hash->value_destroy_func_ex = info->destroy_value;
    hash->value_equal_func_ex = info->equal_value;
    hash->callback_userdata = info->userdata;
    *out_hash = hash;
    return NAVSYS_STATUS_OK;
}

coord_hash_t* coord_hash_create() {
    return coord_hash_create_full(
        coord_hash_int_copy, coord_hash_int_destroy);
}

coord_hash_t* coord_hash_create_full(coord_hash_copy_func copy_func,
    coord_hash_destroy_func destroy_func) {
    coord_hash_t* hash = coord_hash_allocate();
    if (!hash) return nullptr;
    hash->value_copy_func = copy_func;
    hash->value_destroy_func = destroy_func;
    return hash;
}

void coord_hash_destroy(coord_hash_t* hash) {
    if (!hash || coord_hash_callback_is_active(hash)) return;
    if (hash->iteration_state) {
        hash->iteration_state->alive = false;
        ++hash->iteration_state->generation;
    }
    for (auto& [key, val] : hash->data) {
        coord_hash_destroy_stored_value(hash, val);
    }
    delete hash;
}

coord_hash_t* coord_hash_copy(const coord_hash_t* original) {
    if (!original) return nullptr;
    auto* copy = coord_hash_create_full(
        original->value_copy_func, original->value_destroy_func);
    if (!copy) return nullptr;

    for (const auto& [key, val] : original->data) {
        void* val_copy = val && copy->value_copy_func ?
            copy->value_copy_func(val) : nullptr;
        copy->data[key] = val_copy;
    }
    return copy;
}

navsys_status_t coord_hash_copy_ex(
    const coord_hash_t* source,
    coord_hash_t** out_hash) {
    if (!source || !out_hash) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (coord_hash_callback_is_active(source)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }
    if (!source->value_copy_func_ex && !source->value_copy_func) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }

    coord_hash_t* copied_hash = coord_hash_allocate();
    if (!copied_hash) return NAVSYS_STATUS_OUT_OF_MEMORY;
    copied_hash->value_copy_func = source->value_copy_func;
    copied_hash->value_destroy_func = source->value_destroy_func;
    copied_hash->value_copy_func_ex = source->value_copy_func_ex;
    copied_hash->value_destroy_func_ex = source->value_destroy_func_ex;
    copied_hash->value_equal_func_ex = source->value_equal_func_ex;
    copied_hash->callback_userdata = source->callback_userdata;

    for (const auto& [key, value] : source->data) {
        void* copied_value = nullptr;
        navsys_status_t status =
            coord_hash_copy_stored_value(source, value, &copied_value);
        if (status != NAVSYS_STATUS_OK) {
            coord_hash_destroy(copied_hash);
            return status;
        }
        status = coord_hash_commit_new_entry(
            copied_hash, &key, copied_value);
        if (status != NAVSYS_STATUS_OK) {
            coord_hash_destroy(copied_hash);
            return status;
        }
    }

    *out_hash = copied_hash;
    return NAVSYS_STATUS_OK;
}

bool coord_hash_insert(coord_hash_t* hash, const coord_t* key, void* value) {
    bool inserted = false;
    return coord_hash_upsert_copy(hash, key, value, &inserted)
        == NAVSYS_STATUS_OK;
}

bool coord_hash_insert_xy(coord_hash_t* hash, int x, int y, void* value) {
    coord_t temp = { x, y };
    return coord_hash_upsert_copy(hash, &temp, value, nullptr)
        == NAVSYS_STATUS_OK;
}

bool coord_hash_replace(coord_hash_t* hash, const coord_t* key, void* value) {
    return coord_hash_upsert_copy(hash, key, value, nullptr)
        == NAVSYS_STATUS_OK;
}

bool coord_hash_replace_xy(coord_hash_t* hash, int x, int y, void* value) {
    coord_t temp = { x, y };
    return coord_hash_upsert_copy(hash, &temp, value, nullptr)
        == NAVSYS_STATUS_OK;
}

navsys_status_t coord_hash_insert_copy(
    coord_hash_t* hash,
    const coord_t* key,
    const void* value) {
    if (!hash || !key) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (coord_hash_callback_is_active(hash)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }
    if (hash->data.find(*key) != hash->data.end()) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    void* copied = nullptr;
    navsys_status_t status =
        coord_hash_copy_stored_value(hash, value, &copied);
    if (status != NAVSYS_STATUS_OK) return status;
    return coord_hash_commit_new_entry(hash, key, copied);
}

navsys_status_t coord_hash_replace_copy(
    coord_hash_t* hash,
    const coord_t* key,
    const void* value) {
    if (!hash || !key) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (coord_hash_callback_is_active(hash)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }

    auto iter = hash->data.find(*key);
    if (iter == hash->data.end()) return NAVSYS_STATUS_NOT_FOUND;

    void* copied = nullptr;
    navsys_status_t status =
        coord_hash_copy_stored_value(hash, value, &copied);
    if (status != NAVSYS_STATUS_OK) return status;

    void* replaced = iter->second;
    iter->second = copied;
    coord_hash_mark_mutated(hash);
    coord_hash_destroy_stored_value(hash, replaced);
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_hash_upsert_copy(
    coord_hash_t* hash,
    const coord_t* key,
    const void* value,
    bool* out_inserted) {
    if (!hash || !key) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (coord_hash_callback_is_active(hash)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }

    auto iter = hash->data.find(*key);
    void* copied = nullptr;
    navsys_status_t status =
        coord_hash_copy_stored_value(hash, value, &copied);
    if (status != NAVSYS_STATUS_OK) return status;

    if (iter == hash->data.end()) {
        status = coord_hash_commit_new_entry(hash, key, copied);
        if (status != NAVSYS_STATUS_OK) return status;
        if (out_inserted) *out_inserted = true;
        return NAVSYS_STATUS_OK;
    }

    void* replaced = iter->second;
    iter->second = copied;
    coord_hash_mark_mutated(hash);
    coord_hash_destroy_stored_value(hash, replaced);
    if (out_inserted) *out_inserted = false;
    return NAVSYS_STATUS_OK;
}

bool coord_hash_remove(coord_hash_t* hash, const coord_t* key) {
    if (!hash || !key || coord_hash_callback_is_active(hash)) return false;
    auto it = hash->data.find(*key);
    if (it != hash->data.end()) {
        coord_hash_destroy_stored_value(hash, it->second);
        hash->data.erase(it);
        coord_hash_mark_mutated(hash);
        return true;
    }
    return false;
}

void coord_hash_clear(coord_hash_t* hash) {
    if (!hash || coord_hash_callback_is_active(hash)) return;
    const bool had_entries = !hash->data.empty();
    if (hash->value_destroy_func_ex) {
        for (auto& [_, val] : hash->data) {
            coord_hash_destroy_stored_value(hash, val);
        }
    } else if (hash->value_destroy_func) {
        for (auto& [_, val] : hash->data) {
            try {
                coord_hash_callback_scope scope(hash);
                hash->value_destroy_func(val);
            } catch (...) {
                // Preserve the ABI 1 clear contract while containing exceptions.
            }
        }
    }
    hash->data.clear();
    if (had_entries) coord_hash_mark_mutated(hash);
}

int coord_hash_length(const coord_hash_t* hash) {
    return hash ? static_cast<int>(hash->data.size()) : 0;
}

bool coord_hash_is_empty(const coord_hash_t* hash) {
    return !hash || hash->data.empty();
}

size_t coord_hash_size(const coord_hash_t* hash) {
    return hash ? hash->data.size() : 0;
}

navsys_status_t coord_hash_find(
    const coord_hash_t* hash,
    const coord_t* key,
    const void** out_borrowed_value,
    bool* out_found) {
    if (!hash || !key || !out_borrowed_value || !out_found) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    auto iter = hash->data.find(*key);
    const bool found = iter != hash->data.end();
    const void* value = found ? iter->second : nullptr;
    *out_borrowed_value = value;
    *out_found = found;
    return NAVSYS_STATUS_OK;
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
    if (hash && key && !coord_hash_callback_is_active(hash)) {
        hash->data[*key] = value;
        coord_hash_mark_mutated(hash);
    }
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

navsys_status_t coord_hash_equal_keys(
    const coord_hash_t* a,
    const coord_hash_t* b,
    bool* out_equal) {
    if (!a || !b || !out_equal) return NAVSYS_STATUS_INVALID_ARGUMENT;

    bool equal = a->data.size() == b->data.size();
    if (equal) {
        for (const auto& [key, value] : a->data) {
            (void)value;
            if (b->data.find(key) == b->data.end()) {
                equal = false;
                break;
            }
        }
    }
    *out_equal = equal;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_hash_equal_full(
    const coord_hash_t* a,
    const coord_hash_t* b,
    bool* out_equal) {
    if (!a || !b || !out_equal) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (coord_hash_callback_is_active(a)
        || coord_hash_callback_is_active(b)) {
        return NAVSYS_STATUS_IN_PROGRESS;
    }
    if (!a->value_equal_func_ex
        || a->value_equal_func_ex != b->value_equal_func_ex
        || a->callback_userdata != b->callback_userdata) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }

    bool equal = a->data.size() == b->data.size();
    if (equal) {
        for (const auto& [key, lhs] : a->data) {
            auto rhs_iter = b->data.find(key);
            if (rhs_iter == b->data.end()) {
                equal = false;
                break;
            }
            const void* rhs = rhs_iter->second;
            if (!lhs || !rhs) {
                if (lhs != rhs) {
                    equal = false;
                    break;
                }
                continue;
            }
            try {
                coord_hash_callback_scope scope(a, b);
                if (!a->value_equal_func_ex(
                        lhs, rhs, a->callback_userdata)) {
                    equal = false;
                    break;
                }
            } catch (...) {
                return NAVSYS_STATUS_CALLBACK_FAILED;
            }
        }
    }

    *out_equal = equal;
    return NAVSYS_STATUS_OK;
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
        coord_hash_callback_scope scope(hash);
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

navsys_status_t coord_hash_export_keys(
    const coord_hash_t* hash,
    coord_t* out_keys,
    size_t capacity,
    size_t* out_count) {
    if (!hash || !out_count
        || (!out_keys && capacity != 0)
        || (out_keys && capacity == 0)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    const size_t required = hash->data.size();
    if (!out_keys) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }

    size_t index = 0;
    for (const auto& [key, value] : hash->data) {
        (void)value;
        out_keys[index++] = key;
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_hash_export_entries(
    const coord_hash_t* hash,
    coord_hash_entry_view_t* out_entries,
    size_t capacity,
    size_t* out_count) {
    if (!hash || !out_count
        || (!out_entries && capacity != 0)
        || (out_entries && capacity == 0)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    const size_t required = hash->data.size();
    if (!out_entries) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }

    size_t index = 0;
    for (const auto& [key, value] : hash->data) {
        out_entries[index++] = {key, value};
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

typedef struct s_coord_hash_iter {
    using data_type = std::unordered_map<
        coord_t,
        void*,
        byul::navsys::detail::coord_hash,
        byul::navsys::detail::coord_equal>;

    const data_type* data;
    data_type::const_iterator it;
    data_type::const_iterator end;
    std::shared_ptr<coord_hash_iteration_state> state;
    uint64_t generation;
} coord_hash_iter_t;

coord_hash_iter_t* coord_hash_iter_create(const coord_hash_t* hash) {
    if (!hash) return nullptr;
    try {
        auto* iter = new coord_hash_iter_t;
        iter->data = &hash->data;
        iter->it = hash->data.begin();
        iter->end = hash->data.end();
        iter->state = hash->iteration_state;
        iter->generation = hash->iteration_state->generation;
        return iter;
    } catch (...) {
        return nullptr;
    }
}

navsys_status_t coord_hash_iter_next_ex(
    coord_hash_iter_t* iter,
    coord_t* key_out,
    const void** value_out) {
    if (!iter || !key_out || !value_out) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!iter->state || !iter->state->alive
        || iter->state->generation != iter->generation) {
        return NAVSYS_STATUS_INVALIDATED;
    }
    if (iter->it == iter->end) return NAVSYS_STATUS_NOT_FOUND;

    const coord_t key = iter->it->first;
    const void* value = iter->it->second;
    ++iter->it;
    *key_out = key;
    *value_out = value;
    return NAVSYS_STATUS_OK;
}

bool coord_hash_iter_next(
    coord_hash_iter_t* iter,
    coord_t* key_out,
    void** val_out) {
    if (!iter) return false;
    coord_t ignored_key = {};
    const void* value = nullptr;
    navsys_status_t status = coord_hash_iter_next_ex(
        iter, key_out ? key_out : &ignored_key, &value);
    if (status != NAVSYS_STATUS_OK) return false;
    if (val_out) *val_out = const_cast<void*>(value);
    return true;
}

void coord_hash_iter_destroy(coord_hash_iter_t* iter) {
    delete iter;
}

navsys_status_t coord_hash_format(
    const coord_hash_t* hash,
    char* out_buffer,
    size_t capacity,
    size_t* out_required) {
    if (!hash || !out_required
        || (!out_buffer && capacity != 0)
        || (out_buffer && capacity == 0)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    std::string formatted;
    try {
        std::vector<coord_t> keys;
        keys.reserve(hash->data.size());
        for (const auto& [key, value] : hash->data) {
            (void)value;
            keys.push_back(key);
        }
        std::sort(keys.begin(), keys.end(), [](const coord_t& lhs,
                                               const coord_t& rhs) {
            return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
        });

        for (const coord_t& key : keys) {
            char entry[64];
            const int written =
                snprintf(entry, sizeof(entry), "(%d,%d) ", key.x, key.y);
            if (written < 0
                || static_cast<size_t>(written) >= sizeof(entry)) {
                return NAVSYS_STATUS_CORRUPT_STATE;
            }
            formatted.append(entry, static_cast<size_t>(written));
        }
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }

    const size_t required = formatted.size() + 1;
    if (!out_buffer) {
        *out_required = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_required = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }

    memcpy(out_buffer, formatted.c_str(), required);
    *out_required = required;
    return NAVSYS_STATUS_OK;
}

char* coord_hash_to_string(const coord_hash_t* hash) {
    size_t required = 0;
    if (coord_hash_format(hash, nullptr, 0, &required)
        != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    char* buffer = static_cast<char*>(malloc(required));
    if (!buffer) return nullptr;
    if (coord_hash_format(hash, buffer, required, &required)
        != NAVSYS_STATUS_OK) {
        free(buffer);
        return nullptr;
    }
    return buffer;
}

void coord_hash_buffer_destroy(void* buffer) {
    free(buffer);
}

void coord_hash_print(const coord_hash_t* hash) {
    size_t required = 0;
    if (coord_hash_format(hash, nullptr, 0, &required)
        != NAVSYS_STATUS_OK) {
        printf("coords: (null or empty)\n");
        return;
    }
    std::vector<char> buffer;
    try {
        buffer.resize(required);
    } catch (...) {
        printf("coords: (null or empty)\n");
        return;
    }
    if (coord_hash_format(hash, buffer.data(), buffer.size(), &required)
        != NAVSYS_STATUS_OK) {
        printf("coords: (null or empty)\n");
        return;
    }
    printf("coords(len: %zu): %s\n", coord_hash_size(hash), buffer.data());
}
