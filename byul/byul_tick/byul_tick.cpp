#include "byul_tick.h"
#include <vector>
#include <mutex>

typedef struct s_tick {
    std::vector<tick_entry_t> entries;
    std::mutex mtx;
}tick_t;

tick_t* tick_create() {
    return new tick_t();
}

void tick_destroy(tick_t* tick) {
    if (!tick) return;
    std::lock_guard<std::mutex> lock(tick->mtx);
    delete tick;
}

void tick_update(tick_t* tick, float dt) {
    if (!tick) return;
    std::lock_guard<std::mutex> lock(tick->mtx);

    for (const auto& entry : tick->entries) {
        if (entry.func) {
            entry.func(entry.context, dt);
        }
    }
}

int tick_attach(tick_t* tick, tick_func func, void* context) {
    if (!tick || !func) return -1;
    std::lock_guard<std::mutex> lock(tick->mtx);

    for (const auto& e : tick->entries) {
        if (e.func == func && e.context == context)
            return -2; // Already exists
    }

    tick_entry_t new_entry = { func, context };
    tick->entries.push_back(new_entry);
    return 0;
}

int tick_detach(tick_t* tick, tick_func func, void* context) {
    if (!tick || !func) return -1;
    std::lock_guard<std::mutex> lock(tick->mtx);

    auto& vec = tick->entries;
    for (auto it = vec.begin(); it != vec.end(); ++it) {
        if (it->func == func && it->context == context) {
            vec.erase(it);
            return 0;
        }
    }
    return -2; // Not found
}

int tick_list_attached(tick_t* tick, tick_entry_t* out, int max_count) {
    if (!tick || !out || max_count <= 0) return 0;
    std::lock_guard<std::mutex> lock(tick->mtx);

    int count = 0;
    for (const auto& e : tick->entries) {
        if (count >= max_count) break;
        out[count++] = e;
    }
    return count;
}
