#include "coord.hpp"
#include "coord_list.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

struct s_coord_list {
    std::vector<coord_t*> data;
};

coord_list_t* coord_list_create() {
    return new s_coord_list();
}

void coord_list_destroy(coord_list_t* list) {
    if (!list) return;
    for (coord_t* c : list->data) {
        coord_destroy(c);
    }
    delete list;
}

coord_list_t* coord_list_copy(const coord_list_t* list) {
    if (!list) return nullptr;
    coord_list_t* copy = new s_coord_list();
    for (coord_t* c : list->data) {
        copy->data.push_back(coord_copy(c));
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
    if (!list || index < 0 || index >= static_cast<int>(list->data.size()))
        return nullptr;
    return list->data[index];
}

const coord_t* coord_list_front(const coord_list_t* list) {
    if (!list || list->data.empty()) return nullptr;
    return list->data.front();
}

const coord_t* coord_list_back(const coord_list_t* list) {
    if (!list || list->data.empty()) return nullptr;
    return list->data.back();
}

int coord_list_push_back(coord_list_t* list, const coord_t* c) {
    if (!list || !c) return 0;
    list->data.push_back(coord_copy(c));
    return 1;
}

coord_t* coord_list_pop_back(coord_list_t* list) {
    if (list && !list->data.empty()) {
        coord_t* c = list->data.back();
        list->data.pop_back();
        return c;
    }
    return nullptr;
}

coord_t* coord_list_pop_front(coord_list_t* list) {
    if (list && !list->data.empty()) {
        coord_t* c = list->data.front();
        list->data.erase(list->data.begin());
        return c;
    }
    return nullptr;
}

int coord_list_insert(coord_list_t* list, int index, const coord_t* c) {
    if (!list || !c || index < 0 || 
        index > static_cast<int>(list->data.size())) return 0;

    list->data.insert(list->data.begin() + index, coord_copy(c));
    return 1;
}

void coord_list_remove_at(coord_list_t* list, int index) {
    if (!list || index < 0 || 
        index >= static_cast<int>(list->data.size())) return;

    delete list->data[index];
    list->data.erase(list->data.begin() + index);
}

void coord_list_remove_value(coord_list_t* list, const coord_t* c) {
    if (!list || !c) return;
    auto it = std::find_if(list->data.begin(), 
        list->data.end(), [&](coord_t* item) {
        return item->x == c->x && item->y == c->y;
    });
    if (it != list->data.end()) {
        delete *it;
        list->data.erase(it);
    }
}

int coord_list_contains(const coord_list_t* list, const coord_t* c) {
    if (!list || !c) return 0;
    return std::any_of(list->data.begin(), list->data.end(), 
        [&](coord_t* item) {
        return item->x == c->x && item->y == c->y;
    }) ? 1 : 0;
}

int coord_list_find(const coord_list_t* list, const coord_t* c) {
    if (!list || !c) return -1;
    for (size_t i = 0; i < list->data.size(); ++i) {
        if (list->data[i]->x == c->x && list->data[i]->y == c->y)
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
        sub->data.push_back(coord_copy(list->data[i]));
    }
    return sub;
}

bool coord_list_equals(const coord_list_t* a, const coord_list_t* b) {
    if (!a || !b || a->data.size() != b->data.size()) return false;
    for (size_t i = 0; i < a->data.size(); ++i) {
        if (a->data[i]->x != b->data[i]->x || a->data[i]->y != b->data[i]->y)
            return false;
    }
    return true;
}

void coord_list_clear(coord_list_t* list) {
    if (!list) return;
    for (coord_t* c : list->data) {
        delete c;
    }
    list->data.clear();
}

void coord_list_reverse(coord_list_t* list) {
    if (list) std::reverse(list->data.begin(), list->data.end());
}
