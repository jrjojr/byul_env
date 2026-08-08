#include "dstar_lite.h"
#include "internal/dstar_lite_key_ops.hpp"
#include "internal/dstar_lite_planner.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cfloat>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

namespace {

constexpr float infinity = std::numeric_limits<float>::infinity();

struct coord_less final {
    bool operator()(const coord_t& a, const coord_t& b) const noexcept {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    }
};

struct edge_t final { coord_t from; coord_t to; };
struct edge_less final {
    bool operator()(const edge_t& a, const edge_t& b) const noexcept {
        coord_less less;
        return less(a.from, b.from)
            || (!less(b.from, a.from) && less(a.to, b.to));
    }
};

using key_less = byul::navsys::dstar_lite_detail::key_less;

struct planner_state final {
    struct queue_deleter final {
        void operator()(dstar_lite_pqueue_t* value) const noexcept {
            dstar_lite_pqueue_destroy(value);
        }
    };

    planner_state() : open(dstar_lite_pqueue_create()) {}

    navgrid_t* grid = nullptr;
    coord_t start{};
    coord_t last_start{};
    coord_t goal{};
    float km = 0.0f;
    std::map<coord_t, float, coord_less> g;
    std::map<coord_t, float, coord_less> rhs;
    std::unique_ptr<dstar_lite_pqueue_t, queue_deleter> open;
    std::map<edge_t, float, edge_less> edge_costs;
    dstar_lite_cost_callback_t cost_callback = nullptr;
    void* cost_userdata = nullptr;
    dstar_lite_heuristic_callback_t heuristic_callback = nullptr;
    void* heuristic_userdata = nullptr;
    dstar_lite_stats_t stats{};
    std::atomic<bool> cancel_requested{false};
    std::atomic<bool> busy{false};
};

std::mutex registry_mutex;
std::map<const dstar_lite_t*, std::shared_ptr<planner_state>> registry;

bool same_coord(const coord_t& a, const coord_t& b) noexcept {
    return a.x == b.x && a.y == b.y;
}

float table_value(
    const std::map<coord_t, float, coord_less>& table,
    const coord_t& coord) noexcept {
    const auto it = table.find(coord);
    return it == table.end() ? infinity : it->second;
}

void set_table_value(
    std::map<coord_t, float, coord_less>& table,
    const coord_t& coord,
    float value) {
    if (std::isinf(value)) table.erase(coord);
    else table[coord] = value;
}

std::shared_ptr<planner_state> find_state(const dstar_lite_t* planner) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    const auto it = registry.find(planner);
    return it == registry.end() ? nullptr : it->second;
}

void store_state(
    const dstar_lite_t* planner,
    const std::shared_ptr<planner_state>& state) {
    std::lock_guard<std::mutex> lock(registry_mutex);
    registry[planner] = state;
}

navsys_status_t evaluate_heuristic(
    planner_state& state,
    const coord_t& from,
    const coord_t& to,
    float& output) noexcept {
    try {
        float value = 0.0f;
        if (state.heuristic_callback) {
            if (state.heuristic_callback(
                    &from, &to, &value, state.heuristic_userdata)
                != NAVSYS_STATUS_OK) {
                return NAVSYS_STATUS_CALLBACK_FAILED;
            }
        } else {
            const float dx = static_cast<float>(from.x - to.x);
            const float dy = static_cast<float>(from.y - to.y);
            value = std::hypot(dx, dy);
        }
        if (!std::isfinite(value) || value < 0.0f)
            return NAVSYS_STATUS_CALLBACK_FAILED;
        output = value;
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

navsys_status_t evaluate_cost(
    planner_state& state,
    const coord_t& from,
    const coord_t& to,
    float& output) noexcept {
    try {
        const auto override_it = state.edge_costs.find(edge_t{from, to});
        if (override_it != state.edge_costs.end()) {
            output = override_it->second;
            return NAVSYS_STATUS_OK;
        }
        float value = 0.0f;
        if (state.cost_callback) {
            if (state.cost_callback(
                    state.grid, &from, &to, &value, state.cost_userdata)
                != NAVSYS_STATUS_OK) {
                return NAVSYS_STATUS_CALLBACK_FAILED;
            }
        } else {
            value = dstar_lite_cost(state.grid, &from, &to, nullptr);
            if (value >= FLT_MAX) value = infinity;
        }
        if (std::isnan(value) || value < 0.0f)
            return NAVSYS_STATUS_CALLBACK_FAILED;
        output = value;
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

navsys_status_t neighbors(
    planner_state& state,
    const coord_t& center,
    bool predecessors,
    std::vector<coord_t>& output) {
    coord_list_t* list = navgrid_copy_neighbors_all(
        state.grid, center.x, center.y);
    if (!list) return NAVSYS_STATUS_OUT_OF_MEMORY;
    const size_t count = coord_list_size(list);
    try {
        output.clear();
        output.reserve(count + state.edge_costs.size());
        for (size_t index = 0; index < count; ++index) {
            coord_t value{};
            if (coord_list_fetch(list, index, &value) != NAVSYS_STATUS_OK) {
                coord_list_destroy(list);
                return NAVSYS_STATUS_CORRUPT_STATE;
            }
            output.push_back(value);
        }
        for (const auto& item : state.edge_costs) {
            if (predecessors && same_coord(item.first.to, center))
                output.push_back(item.first.from);
            else if (!predecessors && same_coord(item.first.from, center))
                output.push_back(item.first.to);
        }
        std::sort(output.begin(), output.end(), coord_less{});
        output.erase(std::unique(output.begin(), output.end(), same_coord), output.end());
        coord_list_destroy(list);
        return NAVSYS_STATUS_OK;
    } catch (...) {
        coord_list_destroy(list);
        throw;
    }
}

navsys_status_t calculate_key(
    planner_state& state,
    const coord_t& coord,
    dstar_lite_key_t& output) noexcept {
    float heuristic = 0.0f;
    const navsys_status_t status = evaluate_heuristic(
        state, state.start, coord, heuristic);
    if (status != NAVSYS_STATUS_OK) return status;
    const float minimum = std::min(
        table_value(state.g, coord), table_value(state.rhs, coord));
    output = dstar_lite_key_t{minimum + heuristic + state.km, minimum};
    return NAVSYS_STATUS_OK;
}

navsys_status_t open_remove(planner_state& state, const coord_t& coord) {
    bool removed = false;
    return dstar_lite_pqueue_remove_ex(state.open.get(), &coord, &removed);
}

navsys_status_t open_insert(
    planner_state& state,
    const coord_t& coord,
    const dstar_lite_key_t& key) {
    return dstar_lite_pqueue_upsert(state.open.get(), &coord, &key);
}

navsys_status_t update_vertex(planner_state& state, const coord_t& vertex) {
    if (!same_coord(vertex, state.goal)) {
        float best = infinity;
        std::vector<coord_t> successors;
        navsys_status_t status = neighbors(state, vertex, false, successors);
        if (status != NAVSYS_STATUS_OK) return status;
        for (const coord_t& successor : successors) {
            float cost = 0.0f;
            status = evaluate_cost(state, vertex, successor, cost);
            if (status != NAVSYS_STATUS_OK) return status;
            best = std::min(best, cost + table_value(state.g, successor));
        }
        set_table_value(state.rhs, vertex, best);
    }
    navsys_status_t queue_status = open_remove(state, vertex);
    if (queue_status != NAVSYS_STATUS_OK) return queue_status;
    if (table_value(state.g, vertex) != table_value(state.rhs, vertex)) {
        dstar_lite_key_t key{};
        const navsys_status_t status = calculate_key(state, vertex, key);
        if (status != NAVSYS_STATUS_OK) return status;
        queue_status = open_insert(state, vertex, key);
        if (queue_status != NAVSYS_STATUS_OK) return queue_status;
    }
    ++state.stats.vertex_updates;
    return NAVSYS_STATUS_OK;
}

bool cancellation_requested(
    planner_state& state,
    const dstar_lite_replan_options_t& options) noexcept {
    if (state.cancel_requested.load(std::memory_order_relaxed)) return true;
    if (!options.cancel_func) return false;
    try {
        return options.cancel_func(options.cancel_userdata);
    } catch (...) {
        return true;
    }
}

navsys_status_t compute_shortest_path(
    planner_state& state,
    const dstar_lite_replan_options_t& options,
    size_t& expansions) {
    for (;;) {
        dstar_lite_key_t start_key{};
        navsys_status_t status = calculate_key(state, state.start, start_key);
        if (status != NAVSYS_STATUS_OK) return status;
        const bool inconsistent = table_value(state.rhs, state.start)
            != table_value(state.g, state.start);
        dstar_lite_pqueue_entry_t minimum{};
        const navsys_status_t peek_status = dstar_lite_pqueue_peek_min(
            state.open.get(), &minimum);
        if (peek_status == NAVSYS_STATUS_NOT_FOUND
            || (!key_less{}(minimum.key, start_key)
                && !inconsistent)) {
            return NAVSYS_STATUS_OK;
        }
        if (peek_status != NAVSYS_STATUS_OK) return peek_status;
        if (cancellation_requested(state, options))
            return NAVSYS_STATUS_CANCELLED;
        if (options.max_expansions != 0
            && expansions >= options.max_expansions)
            return NAVSYS_STATUS_LIMIT_REACHED;

        dstar_lite_pqueue_entry_t popped{};
        status = dstar_lite_pqueue_pop_min(state.open.get(), &popped);
        if (status != NAVSYS_STATUS_OK) return status;
        const dstar_lite_key_t old_key = popped.key;
        const coord_t vertex = popped.coord;
        dstar_lite_key_t new_key{};
        status = calculate_key(state, vertex, new_key);
        if (status != NAVSYS_STATUS_OK) return status;
        if (key_less{}(old_key, new_key)) {
            status = open_insert(state, vertex, new_key);
            if (status != NAVSYS_STATUS_OK) return status;
        } else if (table_value(state.g, vertex)
                   > table_value(state.rhs, vertex)) {
            set_table_value(state.g, vertex, table_value(state.rhs, vertex));
            std::vector<coord_t> predecessors;
            status = neighbors(state, vertex, true, predecessors);
            if (status != NAVSYS_STATUS_OK) return status;
            for (const coord_t& predecessor : predecessors) {
                status = update_vertex(state, predecessor);
                if (status != NAVSYS_STATUS_OK) return status;
            }
        } else {
            set_table_value(state.g, vertex, infinity);
            std::vector<coord_t> predecessors;
            status = neighbors(state, vertex, true, predecessors);
            if (status != NAVSYS_STATUS_OK) return status;
            status = update_vertex(state, vertex);
            if (status != NAVSYS_STATUS_OK) return status;
            for (const coord_t& predecessor : predecessors) {
                status = update_vertex(state, predecessor);
                if (status != NAVSYS_STATUS_OK) return status;
            }
        }
        ++expansions;
    }
}

navsys_status_t build_route(
    planner_state& state,
    const dstar_lite_replan_options_t& options,
    route_t** output,
    size_t& route_steps) {
    if (std::isinf(table_value(state.g, state.start)))
        return NAVSYS_STATUS_NO_PATH;
    route_builder_t* builder = nullptr;
    navsys_status_t status = route_builder_create(&builder);
    if (status != NAVSYS_STATUS_OK) return status;
    std::set<coord_t, coord_less> visited;
    coord_t current = state.start;
    double total_cost = 0.0;
    for (;;) {
        status = route_builder_push_coord(builder, &current);
        if (status != NAVSYS_STATUS_OK) break;
        if (same_coord(current, state.goal)) {
            status = route_builder_set_total_cost(builder, total_cost);
            if (status == NAVSYS_STATUS_OK)
                status = route_builder_set_completion(
                    builder, ROUTE_COMPLETION_COMPLETE);
            if (status == NAVSYS_STATUS_OK)
                status = route_builder_finish(builder, output);
            route_builder_destroy(builder);
            return status;
        }
        if (!visited.insert(current).second) {
            status = NAVSYS_STATUS_NO_PATH;
            break;
        }
        if (cancellation_requested(state, options)) {
            status = NAVSYS_STATUS_CANCELLED;
            break;
        }
        if (options.max_route_steps != 0
            && route_steps >= options.max_route_steps) {
            status = NAVSYS_STATUS_LIMIT_REACHED;
            break;
        }
        std::vector<coord_t> successors;
        status = neighbors(state, current, false, successors);
        if (status != NAVSYS_STATUS_OK) break;
        float best = infinity;
        float selected_cost = infinity;
        coord_t selected{};
        bool found = false;
        for (const coord_t& successor : successors) {
            float cost = 0.0f;
            status = evaluate_cost(state, current, successor, cost);
            if (status != NAVSYS_STATUS_OK) break;
            const float candidate = cost + table_value(state.g, successor);
            if (!found || candidate < best) {
                best = candidate;
                selected_cost = cost;
                selected = successor;
                found = true;
            }
        }
        if (status != NAVSYS_STATUS_OK) break;
        if (!found || std::isinf(best)) {
            status = NAVSYS_STATUS_NO_PATH;
            break;
        }
        current = selected;
        total_cost += selected_cost;
        ++route_steps;
    }
    route_builder_destroy(builder);
    return status;
}

navsys_status_t reset_state(
    planner_state& state,
    const coord_t& start,
    const coord_t& goal) {
    state.start = start;
    state.last_start = start;
    state.goal = goal;
    state.km = 0.0f;
    state.g.clear();
    state.rhs.clear();
    if (!state.open) return NAVSYS_STATUS_OUT_OF_MEMORY;
    dstar_lite_pqueue_clear(state.open.get());
    state.edge_costs.clear();
    state.stats = dstar_lite_stats_t{};
    state.stats.last_status = NAVSYS_STATUS_INCOMPLETE;
    state.cancel_requested.store(false, std::memory_order_relaxed);
    state.rhs[goal] = 0.0f;
    dstar_lite_key_t goal_key{};
    const navsys_status_t status = calculate_key(state, goal, goal_key);
    if (status != NAVSYS_STATUS_OK) return status;
    return open_insert(state, goal, goal_key);
}

std::shared_ptr<planner_state> ensure_state(const dstar_lite_t* planner) {
    auto existing = find_state(planner);
    if (existing || !planner || !planner->navgrid) return existing;
    auto created = std::make_shared<planner_state>();
    created->grid = planner->navgrid;
    if (reset_state(*created, planner->start, planner->goal)
        != NAVSYS_STATUS_OK) return nullptr;
    store_state(planner, created);
    return created;
}

struct busy_guard final {
    std::atomic<bool>& value;
    ~busy_guard() { value.store(false, std::memory_order_release); }
};

} // namespace

namespace byul::navsys::internal {
void dstar_lite_planner_forget(dstar_lite_t* planner) noexcept {
    try {
        std::lock_guard<std::mutex> lock(registry_mutex);
        registry.erase(planner);
    } catch (...) {
    }
}
} // namespace byul::navsys::internal

navsys_status_t dstar_lite_create_info_init(
    dstar_lite_create_info_t* out_info,
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal) {
    if (!out_info || !navgrid || !start || !goal)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_info = dstar_lite_create_info_t{};
    out_info->struct_size = sizeof(*out_info);
    out_info->version = DSTAR_LITE_CREATE_INFO_VERSION;
    out_info->navgrid = navgrid;
    out_info->start = *start;
    out_info->goal = *goal;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_replan_options_init(
    dstar_lite_replan_options_t* out_options) {
    if (!out_options) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_options = dstar_lite_replan_options_t{};
    out_options->struct_size = sizeof(*out_options);
    out_options->version = DSTAR_LITE_REPLAN_OPTIONS_VERSION;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_create_ex(
    const dstar_lite_create_info_t* info,
    dstar_lite_t** out_planner) {
    if (!info || !out_planner || info->struct_size < sizeof(*info)
        || info->version != DSTAR_LITE_CREATE_INFO_VERSION
        || !info->navgrid) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    try {
        auto state = std::make_shared<planner_state>();
        state->grid = info->navgrid;
        state->cost_callback = info->cost_callback;
        state->cost_userdata = info->cost_userdata;
        state->heuristic_callback = info->heuristic_callback;
        state->heuristic_userdata = info->heuristic_userdata;
        navsys_status_t status = reset_state(*state, info->start, info->goal);
        if (status != NAVSYS_STATUS_OK) return status;
        dstar_lite_t* planner = dstar_lite_create_full(
            info->navgrid, &info->start, &info->goal,
            nullptr, nullptr, false);
        if (!planner) return NAVSYS_STATUS_OUT_OF_MEMORY;
        try {
            store_state(planner, state);
        } catch (...) {
            dstar_lite_destroy(planner);
            throw;
        }
        *out_planner = planner;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t dstar_lite_reset_ex(
    dstar_lite_t* planner,
    const coord_t* start,
    const coord_t* goal) {
    if (!planner || !start || !goal) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->busy.load(std::memory_order_acquire))
        return NAVSYS_STATUS_IN_PROGRESS;
    try {
        const navsys_status_t status = reset_state(*state, *start, *goal);
        if (status == NAVSYS_STATUS_OK) {
            planner->start = *start;
            planner->goal = *goal;
            planner->km = 0.0f;
        }
        return status;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t dstar_lite_bind_cost_callback(
    dstar_lite_t* planner,
    dstar_lite_cost_callback_t callback,
    void* userdata) {
    if (!planner) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->busy.load(std::memory_order_acquire))
        return NAVSYS_STATUS_IN_PROGRESS;
    state->cost_callback = callback;
    state->cost_userdata = userdata;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_bind_heuristic_callback(
    dstar_lite_t* planner,
    dstar_lite_heuristic_callback_t callback,
    void* userdata) {
    if (!planner) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->busy.load(std::memory_order_acquire))
        return NAVSYS_STATUS_IN_PROGRESS;
    state->heuristic_callback = callback;
    state->heuristic_userdata = userdata;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_set_current_start(
    dstar_lite_t* planner,
    const coord_t* start) {
    if (!planner || !start) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->busy.load(std::memory_order_acquire))
        return NAVSYS_STATUS_IN_PROGRESS;
    float delta = 0.0f;
    const navsys_status_t status = evaluate_heuristic(
        *state, state->last_start, *start, delta);
    if (status != NAVSYS_STATUS_OK) return status;
    state->km += delta;
    state->start = *start;
    state->last_start = *start;
    planner->start = *start;
    planner->km = state->km;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_set_goal_ex(
    dstar_lite_t* planner,
    const coord_t* goal) {
    if (!planner || !goal) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    return dstar_lite_reset_ex(planner, &state->start, goal);
}

navsys_status_t dstar_lite_notify_edge_changes(
    dstar_lite_t* planner,
    const dstar_lite_edge_update_t* updates,
    size_t update_count) {
    if (!planner || (!updates && update_count != 0))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->busy.load(std::memory_order_acquire))
        return NAVSYS_STATUS_IN_PROGRESS;
    for (size_t index = 0; index < update_count; ++index) {
        if (std::isnan(updates[index].old_cost)
            || std::isnan(updates[index].new_cost)
            || updates[index].old_cost < 0.0f
            || updates[index].new_cost < 0.0f) {
            return NAVSYS_STATUS_INVALID_ARGUMENT;
        }
    }
    try {
        auto previous_g = state->g;
        auto previous_rhs = state->rhs;
        auto previous_edges = state->edge_costs;
        const dstar_lite_stats_t previous_stats = state->stats;
        dstar_lite_pqueue_t* previous_open_raw = nullptr;
        navsys_status_t status = dstar_lite_pqueue_copy_ex(
            state->open.get(), &previous_open_raw);
        if (status != NAVSYS_STATUS_OK) return status;
        std::unique_ptr<dstar_lite_pqueue_t, planner_state::queue_deleter>
            previous_open(previous_open_raw);

        auto prepared = state->edge_costs;
        for (size_t index = 0; index < update_count; ++index)
            prepared[edge_t{updates[index].from, updates[index].to}]
                = updates[index].new_cost;
        state->edge_costs.swap(prepared);
        const auto rollback = [&]() noexcept {
            state->g.swap(previous_g);
            state->rhs.swap(previous_rhs);
            state->edge_costs.swap(previous_edges);
            state->open.swap(previous_open);
            state->stats = previous_stats;
        };
        try {
            for (size_t index = 0; index < update_count; ++index) {
                status = update_vertex(*state, updates[index].from);
                if (status != NAVSYS_STATUS_OK) {
                    rollback();
                    return status;
                }
            }
        } catch (...) {
            rollback();
            throw;
        }
        state->stats.edge_changes += update_count;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t dstar_lite_replan(
    dstar_lite_t* planner,
    const dstar_lite_replan_options_t* options,
    route_t** out_route,
    dstar_lite_stats_t* out_stats) {
    if (!planner || !out_route) return NAVSYS_STATUS_INVALID_ARGUMENT;
    dstar_lite_replan_options_t effective{};
    dstar_lite_replan_options_init(&effective);
    if (options) {
        if (options->struct_size < sizeof(*options)
            || options->version != DSTAR_LITE_REPLAN_OPTIONS_VERSION)
            return NAVSYS_STATUS_INVALID_ARGUMENT;
        effective = *options;
    }
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    bool expected = false;
    if (!state->busy.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel))
        return NAVSYS_STATUS_IN_PROGRESS;
    busy_guard guard{state->busy};
    navsys_status_t status = NAVSYS_STATUS_CORRUPT_STATE;
    size_t expansions = 0;
    size_t route_steps = 0;
    route_t* route = nullptr;
    try {
        status = compute_shortest_path(*state, effective, expansions);
        if (status == NAVSYS_STATUS_OK)
            status = build_route(*state, effective, &route, route_steps);
    } catch (const std::bad_alloc&) {
        status = NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        status = NAVSYS_STATUS_CORRUPT_STATE;
    }
    state->stats.expansions += expansions;
    state->stats.route_steps = route_steps;
    state->stats.km = state->km;
    state->stats.last_status = status;
    ++state->stats.replans;
    if (out_stats) *out_stats = state->stats;
    if (status == NAVSYS_STATUS_OK) *out_route = route;
    else route_destroy(route);
    return status;
}

navsys_status_t dstar_lite_get_stats(
    const dstar_lite_t* planner,
    dstar_lite_stats_t* out_stats) {
    if (!planner || !out_stats) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    *out_stats = state->stats;
    out_stats->km = state->km;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_request_cancel(dstar_lite_t* planner) {
    if (!planner) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = ensure_state(planner);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    state->cancel_requested.store(true, std::memory_order_relaxed);
    return NAVSYS_STATUS_OK;
}

bool dstar_lite_is_cancel_requested(const dstar_lite_t* planner) {
    auto state = ensure_state(planner);
    return state
        && state->cancel_requested.load(std::memory_order_relaxed);
}
