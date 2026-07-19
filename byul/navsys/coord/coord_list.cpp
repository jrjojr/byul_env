#include "coord.h"
#include "coord_list.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>

struct s_coord_list {
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
        std::unique_ptr<coord_list_t> copied(new coord_list_t(*source));
        *out_list = copied.release();
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

size_t coord_list_size(const coord_list_t* list) {
    return list ? list->data.size() : 0;
}

navsys_status_t coord_list_fetch(
    const coord_list_t* list, size_t index, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
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
    if (list->data.empty()) return NAVSYS_STATUS_NOT_FOUND;
    *out_coord = list->data.back();
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_push_back_ex(
    coord_list_t* list, const coord_t* coord) {
    if (!list || !coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const coord_t copied = *coord;
    try {
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
    if (list->data.empty()) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t removed = list->data.back();
    list->data.pop_back();
    *out_coord = removed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_list_try_pop_front(
    coord_list_t* list, coord_t* out_coord) {
    if (!list || !out_coord) return NAVSYS_STATUS_INVALID_ARGUMENT;
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

coord_list_t* coord_list_create() {
    return new s_coord_list();
}

void coord_list_destroy(coord_list_t* list) {
    delete list;
}

coord_list_t* coord_list_copy(const coord_list_t* list) {
    if (!list) return nullptr;

    coord_list_t* copy = coord_list_create();

    int len = coord_list_length(list);
    for (int i =0 ; i < len; i++){
        coord_t c = list->data[i];
        copy->data.push_back(c);
    }
    return copy;
}


int coord_list_length(const coord_list_t* list) {
    return list ? static_cast<int>(list->data.size()) : 0;
}

bool coord_list_empty(const coord_list_t* list) {
    return !list || list->data.empty();
}

const coord_t* coord_list_get(const coord_list_t* list, int index) {
    if (!list || index < 0 || index >= static_cast<int>(list->data.size())) {
        return nullptr;
    }
    return &list->data[index];
}

const coord_t* coord_list_front(const coord_list_t* list) {
    if (!list || list->data.empty()) return nullptr;
    return &list->data.front();
}

const coord_t* coord_list_back(const coord_list_t* list) {
    if (!list || list->data.empty()) return nullptr;
    return &list->data.back();
}

int coord_list_push_back(coord_list_t* list, const coord_t* c) {
    if (!list || !c) return 0;
    list->data.push_back(*c);
    return 1;
}

coord_t coord_list_pop_back(coord_list_t* list) {
    coord_t result = { 0, 0 };
    if (list && !list->data.empty()) {
        result = list->data.back();
        list->data.pop_back();
    }
    return result;
}

coord_t coord_list_pop_front(coord_list_t* list) {
    coord_t result = { 0, 0 };
    if (list && !list->data.empty()) {
        result = list->data.front();
        list->data.erase(list->data.begin());
    }
    return result;
}

int coord_list_insert(coord_list_t* list, int index, const coord_t* c) {
    if (!list || !c || index < 0 ||
        index > static_cast<int>(list->data.size())) return 0;

    list->data.insert(list->data.begin() + index, *c);
    return 1;
}

void coord_list_remove_at(coord_list_t* list, int index) {
    if (!list || index < 0 ||
        index >= static_cast<int>(list->data.size())) return;

    list->data.erase(list->data.begin() + index);
}

void coord_list_remove_value(coord_list_t* list, const coord_t* c) {
    if (!list || !c) return;
    auto it = std::find_if(list->data.begin(),
        list->data.end(), [&](const coord_t& item) {
            return item.x == c->x && item.y == c->y;
        });
    if (it != list->data.end()) {
        list->data.erase(it);
    }
}

int coord_list_contains(const coord_list_t* list, const coord_t* c) {
    if (!list || !c) return 0;
    return std::any_of(list->data.begin(), list->data.end(),
        [&](const coord_t& item) {
            return item.x == c->x && item.y == c->y;
        }) ? 1 : 0;
}

int coord_list_find(const coord_list_t* list, const coord_t* c) {
    if (!list || !c) return -1;
    for (size_t i = 0; i < list->data.size(); ++i) {
        if (list->data[i].x == c->x && list->data[i].y == c->y)
            return static_cast<int>(i);
    }
    return -1;
}

coord_list_t* coord_list_sublist(
    const coord_list_t* list, int start, int end) {

    if (!list || start < 0 ||
        end > static_cast<int>(list->data.size()) || end <= start)
        return nullptr;

    coord_list_t* sub = new s_coord_list();
    for (int i = start; i < end; ++i) {
        sub->data.push_back(list->data[i]);
    }
    return sub;
}

bool coord_list_equals(const coord_list_t* a, const coord_list_t* b) {
    if (!a || !b || a->data.size() != b->data.size()) return false;
    for (size_t i = 0; i < a->data.size(); ++i) {
        if (a->data[i].x != b->data[i].x || a->data[i].y != b->data[i].y)
            return false;
    }
    return true;
}

void coord_list_clear(coord_list_t* list) {
    if (!list) return;
    list->data.clear();
}

void coord_list_reverse(coord_list_t* list) {
    if (list) std::reverse(list->data.begin(), list->data.end());
}
