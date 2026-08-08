#include "dstar_lite_tick.h"
#include "internal/dstar_lite_callback.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <mutex>

namespace {

struct tick_state final {
    tick_t* tick = nullptr;
    dstar_lite_tick_state_t state = DSTAR_LITE_TICK_STATE_DETACHED;
    float accumulated_seconds = 0.0f;
    uint32_t max_steps = BYUL_DSTAR_LITE_TICK_DEFAULT_MAX_STEPS;
    coord_t goal_snapshot{};
};

std::mutex state_mutex;
std::map<const dstar_lite_tick_t*, std::shared_ptr<tick_state>> states;

std::shared_ptr<tick_state> find_state(const dstar_lite_tick_t* controller) {
    std::lock_guard<std::mutex> lock(state_mutex);
    const auto it = states.find(controller);
    return it == states.end() ? nullptr : it->second;
}

void store_state(
    const dstar_lite_tick_t* controller,
    const std::shared_ptr<tick_state>& state) {
    std::lock_guard<std::mutex> lock(state_mutex);
    states[controller] = state;
}

void erase_state(const dstar_lite_tick_t* controller) noexcept {
    try {
        std::lock_guard<std::mutex> lock(state_mutex);
        states.erase(controller);
    } catch (...) {
    }
}

void proxy(void* context, float dt);

void detach(dstar_lite_tick_t* controller, tick_state& state) {
    if (state.tick) (void)tick_detach(state.tick, proxy, controller);
    state.tick = nullptr;
    controller->ticked = false;
}

navsys_status_t append_position(dstar_lite_t* planner, const coord_t& position) {
    route_builder_t* builder = nullptr;
    navsys_status_t status = planner->real_route
        ? route_builder_create_from_route(planner->real_route, &builder)
        : route_builder_create(&builder);
    if (status != NAVSYS_STATUS_OK) return status;
    status = route_builder_push_coord(builder, &position);
    route_t* replacement = nullptr;
    if (status == NAVSYS_STATUS_OK)
        status = route_builder_set_completion(builder, ROUTE_COMPLETION_PARTIAL);
    if (status == NAVSYS_STATUS_OK)
        status = route_builder_finish(builder, &replacement);
    route_builder_destroy(builder);
    if (status != NAVSYS_STATUS_OK) return status;
    route_destroy(planner->real_route);
    planner->real_route = replacement;
    return NAVSYS_STATUS_OK;
}

void mark_completion(dstar_lite_t* planner) {
    if (!planner->real_route) return;
    route_builder_t* builder = nullptr;
    if (route_builder_create_from_route(planner->real_route, &builder)
        != NAVSYS_STATUS_OK) return;
    if (route_builder_set_completion(builder, ROUTE_COMPLETION_COMPLETE)
        != NAVSYS_STATUS_OK) {
        route_builder_destroy(builder);
        return;
    }
    route_t* replacement = nullptr;
    if (route_builder_finish(builder, &replacement) == NAVSYS_STATUS_OK) {
        route_destroy(planner->real_route);
        planner->real_route = replacement;
    }
    route_builder_destroy(builder);
}

void proxy(void* context, float dt) {
    auto* controller = static_cast<dstar_lite_tick_t*>(context);
    const navsys_status_t status = dstar_lite_tick_advance(
        controller, dt, nullptr);
    if (status != NAVSYS_STATUS_OK
        || dstar_lite_tick_get_state(controller)
            == DSTAR_LITE_TICK_STATE_COMPLETED) {
        (void)dstar_lite_tick_stop(controller);
    }
}

bool valid_config(const dstar_lite_tick_create_info_t& info) noexcept {
    return info.struct_size >= sizeof(info)
        && info.abi_version == DSTAR_LITE_TICK_CREATE_INFO_VERSION
        && info.planner
        && std::isfinite(info.tile_size_m) && info.tile_size_m > 0.0f
        && std::isfinite(info.speed_m_per_sec) && info.speed_m_per_sec > 0.0f
        && std::isfinite(info.max_duration_sec)
        && info.max_duration_sec >= 0.0f
        && info.max_steps_per_update > 0;
}

} // namespace

navsys_status_t dstar_lite_tick_create_info_init(
    dstar_lite_tick_create_info_t* out_info,
    dstar_lite_t* planner) {
    if (!out_info || !planner) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_info = dstar_lite_tick_create_info_t{};
    out_info->struct_size = sizeof(*out_info);
    out_info->abi_version = DSTAR_LITE_TICK_CREATE_INFO_VERSION;
    out_info->planner = planner;
    out_info->tile_size_m = 1.0f;
    out_info->speed_m_per_sec = 1.0f;
    out_info->max_duration_sec = 10.0f;
    out_info->max_steps_per_update = BYUL_DSTAR_LITE_TICK_DEFAULT_MAX_STEPS;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_tick_create_ex(
    const dstar_lite_tick_create_info_t* info,
    dstar_lite_tick_t** out_controller) {
    if (!info || !out_controller || !valid_config(*info))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        auto* controller = new dstar_lite_tick_t{};
        controller->base = info->planner;
        controller->max_time = info->max_duration_sec;
        controller->unit_m = info->tile_size_m;
        controller->speed_sec = info->speed_m_per_sec;
        controller->max_elapsed_time = info->tile_size_m
            / info->speed_m_per_sec;
        controller->s_last = info->planner->start;
        auto state = std::make_shared<tick_state>();
        state->max_steps = info->max_steps_per_update;
        state->goal_snapshot = info->planner->goal;
        try {
            store_state(controller, state);
        } catch (...) {
            delete controller;
            throw;
        }
        *out_controller = controller;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t dstar_lite_tick_start(
    dstar_lite_tick_t* controller,
    tick_t* tick) {
    if (!controller || !tick) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = find_state(controller);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    if (state->tick || state->state == DSTAR_LITE_TICK_STATE_ATTACHED
        || state->state == DSTAR_LITE_TICK_STATE_RUNNING)
        return NAVSYS_STATUS_IN_PROGRESS;
    if (state->state != DSTAR_LITE_TICK_STATE_DETACHED)
        return NAVSYS_STATUS_INVALIDATED;
    if (tick_attach(tick, proxy, controller) != 0)
        return NAVSYS_STATUS_CORRUPT_STATE;
    state->tick = tick;
    state->state = DSTAR_LITE_TICK_STATE_ATTACHED;
    controller->ticked = true;
    route_destroy(controller->base->real_route);
    controller->base->real_route = nullptr;
    const navsys_status_t route_status = append_position(
        controller->base, controller->base->start);
    if (route_status != NAVSYS_STATUS_OK) {
        detach(controller, *state);
        state->state = DSTAR_LITE_TICK_STATE_FAILED;
        return route_status;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_tick_stop(dstar_lite_tick_t* controller) {
    if (!controller) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = find_state(controller);
    if (!state) return NAVSYS_STATUS_INVALIDATED;
    detach(controller, *state);
    if (state->state == DSTAR_LITE_TICK_STATE_ATTACHED
        || state->state == DSTAR_LITE_TICK_STATE_RUNNING)
        state->state = DSTAR_LITE_TICK_STATE_DETACHED;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_tick_advance(
    dstar_lite_tick_t* controller,
    float delta_seconds,
    uint32_t* out_steps) {
    if (!controller || !std::isfinite(delta_seconds) || delta_seconds < 0.0f)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto state = find_state(controller);
    if (!state || !controller->base) return NAVSYS_STATUS_INVALIDATED;
    if (state->state == DSTAR_LITE_TICK_STATE_COMPLETED)
        return NAVSYS_STATUS_OK;
    if (state->state == DSTAR_LITE_TICK_STATE_CANCELLED)
        return NAVSYS_STATUS_CANCELLED;
    if (state->state == DSTAR_LITE_TICK_STATE_FAILED)
        return NAVSYS_STATUS_INVALIDATED;
    const bool deterministic_detached = !state->tick
        && state->state == DSTAR_LITE_TICK_STATE_DETACHED;
    state->state = DSTAR_LITE_TICK_STATE_RUNNING;
    state->accumulated_seconds += delta_seconds;
    controller->cur_time += delta_seconds;
    controller->cur_elapsed_time = state->accumulated_seconds;
    uint32_t steps = 0;
    navsys_status_t status = NAVSYS_STATUS_OK;
    const float interval = controller->unit_m / controller->speed_sec;
    if (!coord_equal(&state->goal_snapshot, &controller->base->goal)) {
        status = dstar_lite_set_goal_ex(
            controller->base, &controller->base->goal);
        if (status != NAVSYS_STATUS_OK) {
            state->state = DSTAR_LITE_TICK_STATE_FAILED;
        } else {
            state->goal_snapshot = controller->base->goal;
        }
    }
    if (status == NAVSYS_STATUS_OK
        && coord_equal(&controller->base->start, &controller->base->goal)) {
        mark_completion(controller->base);
        state->state = DSTAR_LITE_TICK_STATE_COMPLETED;
    }
    if (status == NAVSYS_STATUS_OK
        && controller->cur_time > controller->max_time) {
        state->state = DSTAR_LITE_TICK_STATE_FAILED;
        status = NAVSYS_STATUS_LIMIT_REACHED;
    }
    while (state->accumulated_seconds >= interval
        && steps < state->max_steps
        && state->state == DSTAR_LITE_TICK_STATE_RUNNING) {
        if (controller->cur_time > controller->max_time) {
            state->state = DSTAR_LITE_TICK_STATE_FAILED;
            status = NAVSYS_STATUS_LIMIT_REACHED;
            break;
        }
        route_t* route = nullptr;
        if (controller->base->changed_coords_fn) {
            coord_list_t* changed =
                byul::navsys::internal::dstar_lite_invoke_changed_coords(
                    controller->base);
            if (changed) {
                coord_list_destroy(changed);
                status = dstar_lite_reset_ex(
                    controller->base,
                    &controller->base->start,
                    &controller->base->goal);
                if (status != NAVSYS_STATUS_OK) {
                    state->state = DSTAR_LITE_TICK_STATE_FAILED;
                    break;
                }
            }
        }
        status = dstar_lite_replan(controller->base, nullptr, &route, nullptr);
        if (status != NAVSYS_STATUS_OK) {
            state->state = status == NAVSYS_STATUS_CANCELLED
                ? DSTAR_LITE_TICK_STATE_CANCELLED
                : DSTAR_LITE_TICK_STATE_FAILED;
            break;
        }
        const size_t count = route_get_coord_count(route);
        if (count <= 1) {
            route_destroy(route);
            mark_completion(controller->base);
            state->state = DSTAR_LITE_TICK_STATE_COMPLETED;
            break;
        }
        coord_t next{};
        status = route_fetch_coord(route, 1, &next);
        route_destroy(route);
        if (status != NAVSYS_STATUS_OK) {
            state->state = DSTAR_LITE_TICK_STATE_FAILED;
            break;
        }
        status = dstar_lite_set_current_start(controller->base, &next);
        if (status == NAVSYS_STATUS_OK)
            status = append_position(controller->base, next);
        if (status != NAVSYS_STATUS_OK) {
            state->state = DSTAR_LITE_TICK_STATE_FAILED;
            break;
        }
        controller->s_last = next;
        state->accumulated_seconds -= interval;
        controller->cur_elapsed_time = state->accumulated_seconds;
        ++steps;
        byul::navsys::internal::dstar_lite_invoke_move(
            controller->base, &next);
        if (coord_equal(&next, &controller->base->goal)) {
            mark_completion(controller->base);
            state->state = DSTAR_LITE_TICK_STATE_COMPLETED;
            break;
        }
    }
    if (out_steps) *out_steps = steps;
    if (state->state == DSTAR_LITE_TICK_STATE_COMPLETED
        || state->state == DSTAR_LITE_TICK_STATE_CANCELLED
        || state->state == DSTAR_LITE_TICK_STATE_FAILED)
        detach(controller, *state);
    else if (deterministic_detached)
        state->state = DSTAR_LITE_TICK_STATE_DETACHED;
    return status;
}

dstar_lite_tick_state_t dstar_lite_tick_get_state(
    const dstar_lite_tick_t* controller) {
    auto state = find_state(controller);
    return state ? state->state : DSTAR_LITE_TICK_STATE_FAILED;
}

navsys_status_t dstar_lite_tick_fetch_position(
    const dstar_lite_tick_t* controller,
    coord_t* out_position) {
    if (!controller || !out_position || !controller->base)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!find_state(controller)) return NAVSYS_STATUS_INVALIDATED;
    *out_position = controller->base->start;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_tick_get_elapsed_seconds(
    const dstar_lite_tick_t* controller,
    float* out_elapsed_seconds) {
    if (!controller || !out_elapsed_seconds)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!find_state(controller)) return NAVSYS_STATUS_INVALIDATED;
    *out_elapsed_seconds = controller->cur_time;
    return NAVSYS_STATUS_OK;
}

dstar_lite_tick_t* dstar_lite_tick_create(dstar_lite_t* planner) {
    dstar_lite_tick_create_info_t info{};
    dstar_lite_tick_t* result = nullptr;
    return dstar_lite_tick_create_info_init(&info, planner) == NAVSYS_STATUS_OK
        && dstar_lite_tick_create_ex(&info, &result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

dstar_lite_tick_t* dstar_lite_tick_create_full(
    dstar_lite_t* planner, float max_time) {
    dstar_lite_tick_create_info_t info{};
    dstar_lite_tick_t* result = nullptr;
    if (dstar_lite_tick_create_info_init(&info, planner) != NAVSYS_STATUS_OK)
        return nullptr;
    info.max_duration_sec = max_time;
    return dstar_lite_tick_create_ex(&info, &result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

void dstar_lite_tick_destroy(dstar_lite_tick_t* controller) {
    if (!controller) return;
    (void)dstar_lite_tick_stop(controller);
    erase_state(controller);
    delete controller;
}

dstar_lite_tick_t* dstar_lite_tick_copy(const dstar_lite_tick_t* source) {
    if (!source || !source->base) return nullptr;
    auto source_state = find_state(source);
    if (!source_state) return nullptr;
    dstar_lite_tick_create_info_t info{};
    dstar_lite_tick_create_info_init(&info, source->base);
    info.tile_size_m = source->unit_m;
    info.speed_m_per_sec = source->speed_sec;
    info.max_duration_sec = source->max_time;
    info.max_steps_per_update = source_state->max_steps;
    dstar_lite_tick_t* result = nullptr;
    return dstar_lite_tick_create_ex(&info, &result) == NAVSYS_STATUS_OK
        ? result : nullptr;
}

void dstar_lite_tick_reset(dstar_lite_tick_t* controller) {
    if (!controller) return;
    (void)dstar_lite_tick_stop(controller);
    auto state = find_state(controller);
    if (!state) return;
    state->state = DSTAR_LITE_TICK_STATE_DETACHED;
    state->accumulated_seconds = 0.0f;
    controller->cur_time = 0.0f;
    controller->cur_elapsed_time = 0.0f;
    controller->s_last = controller->base->start;
}

void dstar_lite_tick_prepare(dstar_lite_tick_t* controller, tick_t* tick) {
    (void)dstar_lite_tick_start(controller, tick);
}

void dstar_lite_tick_prepare_full(
    dstar_lite_tick_t* controller,
    float unit_m,
    float speed_sec,
    float max_time,
    tick_t* tick) {
    if (!controller || !std::isfinite(unit_m) || unit_m <= 0.0f
        || !std::isfinite(speed_sec) || speed_sec <= 0.0f
        || !std::isfinite(max_time) || max_time < 0.0f) return;
    controller->unit_m = unit_m;
    controller->speed_sec = speed_sec;
    controller->max_time = max_time;
    controller->max_elapsed_time = unit_m / speed_sec;
    (void)dstar_lite_tick_start(controller, tick);
}

void dstar_lite_tick_update(dstar_lite_tick_t* controller, float dt) {
    (void)dstar_lite_tick_advance(controller, dt, nullptr);
}

void dstar_lite_tick_complete(
    dstar_lite_tick_t* controller, tick_t*) {
    (void)dstar_lite_tick_stop(controller);
}
