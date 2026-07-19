#include "coord.h"
#include "coord_list.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>

struct s_coord_list {
    mutable std::mutex mutex;
    std::vector<coord_t> data;
};

navsys_status_t coord_list_create_ex(coord_list_t** out_list) {
    if (!out_list) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        std::unique_ptr<coord_list_t> list(new coord_list_t());
        *out_list = list.release();
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t coord_list_copy_ex(
    const coord_list_t* source, coord_list_t** out_list) {
    if (!source || !out_list) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        std::unique_ptr<coord_list_t> copied(new coord_list_t());
        {
            const std::lock_guard<std::mutex> lock(source->mutex);
            copied->data = source->data;
        }
        *out_list = copied.release();
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

size_t coord_list_size(const coord_list_t* list) {
    if (!list) return 0;
    const std::lock_guard<std::mutex> lock(list->mutex);
    return list->data.size();
}

navsys_status_t coord_list_fetch(
    const coord_list_t* list, size_t index, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (index >= list->data.size()) return NAVSYS_STATUS_NOT_FOUND;
    *out_coord = list->data[index];
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_fetch_front(
    const coord_list_t* list, coord_t* out_coord) {
    return coord_list_fetch(list, 0, out_coord);
}

navsys_status_t coord_list_fetch_back(
    const coord_list_t* list, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (list->data.empty()) return NAVSYS_STATUS_NOT_FOUND;
    *out_coord = list->data.back();
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_push_back_ex(
    coord_list_t* list, const coord_t* coord) {
    if (!list || !coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const coord_t copied = *coord;
    try {
        const std::lock_guard<std::mutex> lock(list->mutex);
        list->data.push_back(copied);
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t coord_list_try_pop_back(
    coord_list_t* list, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (list->data.empty()) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t removed = list->data.back();
    list->data.pop_back();
    *out_coord = removed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_try_pop_front(
    coord_list_t* list, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (list->data.empty()) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t removed = list->data.front();
    list->data.erase(list->data.begin());
    *out_coord = removed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_insert_ex(
    coord_list_t* list, size_t index, const coord_t* coord) {
    if (!list || !coord || index > list->data.size()) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t copied = *coord;
    try {
        const std::lock_guard<std::mutex> lock(list->mutex);
        list->data.insert(
            list->data.begin() + static_cast<std::ptrdiff_t>(index),
            copied);
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t coord_list_remove_at_ex(
    coord_list_t* list, size_t index, coord_t* out_removed) {
    if (!list || !out_removed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (index >= list->data.size()) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t removed = list->data[index];
    list->data.erase(
        list->data.begin() + static_cast<std::ptrdiff_t>(index));
    *out_removed = removed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_remove_value_ex(
    coord_list_t* list, const coord_t* coord, bool* out_removed) {
    if (!list || !coord || !out_removed) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t copied = *coord;
    const std::lock_guard<std::mutex> lock(list->mutex);
    const auto it = std::find_if(
        list->data.begin(),
        list->data.end(),
        [&](const coord_t& item) {
            return item.x == copied.x && item.y == copied.y;
        });
    if (it == list->data.end()) {
        *out_removed = false;
        return NAVSYS_STATUS_OK;
    }
    list->data.erase(it);
    *out_removed = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_find_ex(
    const coord_list_t* list,
    const coord_t* coord,
    size_t* out_index,
    bool* out_found) {
    if (!list || !coord || !out_index || !out_found) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::lock_guard<std::mutex> lock(list->mutex);
    for (size_t index = 0; index < list->data.size(); ++index) {
        if (list->data[index].x == coord->x
            && list->data[index].y == coord->y) {
            *out_index = index;
            *out_found = true;
            return NAVSYS_STATUS_OK;
        }
    }
    *out_found = false;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_reserve(
    coord_list_t* list, size_t capacity) {
    if (!list) return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        const std::lock_guard<std::mutex> lock(list->mutex);
        list->data.reserve(capacity);
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (const std::length_error&) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t coord_list_create_slice(
    const coord_list_t* source,
    size_t begin,
    size_t end,
    coord_list_t** out_list) {
    if (!source || !out_list || begin > end) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    try {
        std::unique_ptr<coord_list_t> slice(new coord_list_t());
        {
            const std::lock_guard<std::mutex> lock(source->mutex);
            if (end > source->data.size()) {
                return NAVSYS_STATUS_INVALID_ARGUMENT;
            }
            slice->data.assign(
                source->data.begin() + static_cast<std::ptrdiff_t>(begin),
                source->data.begin() + static_cast<std::ptrdiff_t>(end));
        }
        *out_list = slice.release();
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t coord_list_equal(
    const coord_list_t* a,
    const coord_list_t* b,
    bool* out_equal) {
    if (!a || !b || !out_equal) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (a == b) {
        *out_equal = true;
        return NAVSYS_STATUS_OK;
    }
    const std::scoped_lock lock(a->mutex, b->mutex);
    if (a->data.size() != b->data.size()) {
        *out_equal = false;
        return NAVSYS_STATUS_OK;
    }
    for (size_t index = 0; index < a->data.size(); ++index) {
        if (a->data[index].x != b->data[index].x
            || a->data[index].y != b->data[index].y) {
            *out_equal = false;
            return NAVSYS_STATUS_OK;
        }
    }
    *out_equal = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_export(
    const coord_list_t* list,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!list || !out_count
        || (!out_coords && capacity != 0)
        || (out_coords && capacity == 0)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::lock_guard<std::mutex> lock(list->mutex);

    const size_t required = list->data.size();
    if (!out_coords) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    std::copy(list->data.begin(), list->data.end(), out_coords);
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

coord_list_t* coord_list_create() {
    coord_list_t* list = nullptr;
    return coord_list_create_ex(&list) == NAVSYS_STATUS_OK ? list : nullptr;
}

void coord_list_destroy(coord_list_t* list) {
    delete list;
}

coord_list_t* coord_list_copy(const coord_list_t* list) {
    coord_list_t* copy = nullptr;
    return coord_list_copy_ex(list, &copy) == NAVSYS_STATUS_OK
        ? copy
        : nullptr;
}


int coord_list_length(const coord_list_t* list) {
    const size_t size = coord_list_size(list);
    return size > static_cast<size_t>(std::numeric_limits<int>::max())
        ? std::numeric_limits<int>::max()
        : static_cast<int>(size);
}

bool coord_list_empty(const coord_list_t* list) {
    if (!list) return true;
    const std::lock_guard<std::mutex> lock(list->mutex);
    return list->data.empty();
}

const coord_t* coord_list_get(const coord_list_t* list, int index) {
    if (!list || index < 0) {
        return nullptr;
    }
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (static_cast<size_t>(index) >= list->data.size()) return nullptr;
    return &list->data[index];
}

const coord_t* coord_list_front(const coord_list_t* list) {
    if (!list) return nullptr;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (list->data.empty()) return nullptr;
    return &list->data.front();
}

const coord_t* coord_list_back(const coord_list_t* list) {
    if (!list) return nullptr;
    const std::lock_guard<std::mutex> lock(list->mutex);
    if (list->data.empty()) return nullptr;
    return &list->data.back();
}

int coord_list_push_back(coord_list_t* list, const coord_t* c) {
    return coord_list_push_back_ex(list, c) == NAVSYS_STATUS_OK ? 1 : 0;
}

coord_t coord_list_pop_back(coord_list_t* list) {
    coord_t result = { 0, 0 };
    (void)coord_list_try_pop_back(list, &result);
    return result;
}

coord_t coord_list_pop_front(coord_list_t* list) {
    coord_t result = { 0, 0 };
    (void)coord_list_try_pop_front(list, &result);
    return result;
}

int coord_list_insert(coord_list_t* list, int index, const coord_t* c) {
    if (index < 0) return 0;
    return coord_list_insert_ex(list, static_cast<size_t>(index), c)
            == NAVSYS_STATUS_OK
        ? 1
        : 0;
}

void coord_list_remove_at(coord_list_t* list, int index) {
    if (index < 0) return;
    coord_t removed = {0, 0};
    (void)coord_list_remove_at_ex(
        list, static_cast<size_t>(index), &removed);
}

void coord_list_remove_value(coord_list_t* list, const coord_t* c) {
    bool removed = false;
    (void)coord_list_remove_value_ex(list, c, &removed);
}

int coord_list_contains(const coord_list_t* list, const coord_t* c) {
    size_t index = 0;
    bool found = false;
    return coord_list_find_ex(list, c, &index, &found) == NAVSYS_STATUS_OK
        && found
        ? 1
        : 0;
}

int coord_list_find(const coord_list_t* list, const coord_t* c) {
    size_t index = 0;
    bool found = false;
    if (coord_list_find_ex(list, c, &index, &found) != NAVSYS_STATUS_OK
        || !found
        || index > static_cast<size_t>(std::numeric_limits<int>::max())) {
        return -1;
    }
    return static_cast<int>(index);
}

coord_list_t* coord_list_sublist(
    const coord_list_t* list, int start, int end) {
    if (start < 0 || end <= start) return nullptr;
    coord_list_t* sub = nullptr;
    return coord_list_create_slice(
               list,
               static_cast<size_t>(start),
               static_cast<size_t>(end),
               &sub)
            == NAVSYS_STATUS_OK
        ? sub
        : nullptr;
}

bool coord_list_equals(const coord_list_t* a, const coord_list_t* b) {
    bool equal = false;
    return coord_list_equal(a, b, &equal) == NAVSYS_STATUS_OK && equal;
}

void coord_list_clear(coord_list_t* list) {
    if (!list) return;
    const std::lock_guard<std::mutex> lock(list->mutex);
    list->data.clear();
}

void coord_list_reverse(coord_list_t* list) {
    if (!list) return;
    const std::lock_guard<std::mutex> lock(list->mutex);
    std::reverse(list->data.begin(), list->data.end());
}
