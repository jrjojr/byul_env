#include "byul_tick.h"
#include <vector>
#include <mutex>
#include <algorithm>

// Extend tick_t to hold pending detach requests
typedef struct s_tick {
    std::vector<tick_entry_t> entries;
    std::vector<tick_entry_t> pending_detach; // new
    std::mutex mtx;
} tick_t;

tick_t* tick_create() {
    return new tick_t();
}

void tick_destroy(tick_t* tick) {
    if (!tick) return;
    // DO NOT lock tick->mtx here. The destructor of std::mutex
    // must not run while it is locked.
    delete tick;
}

void tick_update(tick_t* tick, float dt) {
    if (!tick) return;

    // 1) 먼저 detach 요청을 반영하고, 2) 호출할 목록을 스냅샷으로 복사
    std::vector<tick_entry_t> to_call;
    {
        std::lock_guard<std::mutex> lock(tick->mtx);

        if (!tick->pending_detach.empty()) {
            for (const auto& req : tick->pending_detach) {
                auto it = std::remove_if(
                    tick->entries.begin(), tick->entries.end(),
                    [&](const tick_entry_t& e) {
                        return e.func == req.func && e.context == req.context;
                    });
                tick->entries.erase(it, tick->entries.end());
            }
            tick->pending_detach.clear();
        }

        to_call = tick->entries; // snapshot for unlocked execution
    }

    // 3) 잠금을 풀고 콜백 실행
    for (const auto& entry : to_call) {
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

/**
 * @brief Request to detach a tick entry without removing it immediately.
 * The actual removal will happen during the next tick_update.
 *
 * @param tick Pointer to the tick object.
 * @param func Function pointer to detach.
 * @param context Context pointer to match.
 * @return 0 if request added, -1 if invalid, -2 if already requested.
 */
int tick_request_detach(tick_t* tick, tick_func func, void* context) {
    if (!tick || !func) return -1;
    std::lock_guard<std::mutex> lock(tick->mtx);

    // Prevent duplicate requests
    for (const auto& e : tick->pending_detach) {
        if (e.func == func && e.context == context)
            return -2; // already requested
    }

    tick->pending_detach.push_back({func, context});
    return 0;
}
