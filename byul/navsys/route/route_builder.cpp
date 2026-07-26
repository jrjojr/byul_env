#include "route.h"

#include <cmath>
#include <limits>

struct s_route_builder {
    route_t* candidate;
};

namespace {

navsys_status_t builder_candidate(
    route_builder_t* builder,
    route_t** out_candidate) {
    if (!builder || !out_candidate)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!builder->candidate)
        return NAVSYS_STATUS_INVALIDATED;
    *out_candidate = builder->candidate;
    return NAVSYS_STATUS_OK;
}

void reset_edit_metadata(route_t* route) {
    route->cost = 0.0f;
    route->success = false;
    route->total_retry_count = 0;
    coord_list_clear(route->visited_order);
    coord_hash_clear(route->visited_count);
    route->avg_vec_x = 0.0f;
    route->avg_vec_y = 0.0f;
    route->vec_count = 0;
}

navsys_status_t clone_for_edit(
    const route_t* source,
    route_t** out_route) {
    navsys_status_t status = route_clone_ex(source, out_route);
    if (status == NAVSYS_STATUS_OK) {
        coord_list_clear((*out_route)->visited_order);
        coord_hash_clear((*out_route)->visited_count);
        (*out_route)->total_retry_count = 0;
        (*out_route)->avg_vec_x = 0.0f;
        (*out_route)->avg_vec_y = 0.0f;
        (*out_route)->vec_count = 0;
    }
    return status;
}

void commit_candidate(route_builder_t* builder, route_t* candidate) {
    route_t* previous = builder->candidate;
    builder->candidate = candidate;
    route_destroy(previous);
}

navsys_status_t allocate_builder(
    route_t* candidate,
    route_builder_t** out_builder) {
    route_builder_t* builder = nullptr;
    try {
        builder = new route_builder_t{candidate};
    } catch (...) {
        route_destroy(candidate);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    *out_builder = builder;
    return NAVSYS_STATUS_OK;
}

}  // namespace

navsys_status_t route_builder_create(route_builder_t** out_builder) {
    if (!out_builder)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_t* candidate = route_create();
    if (!candidate)
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    return allocate_builder(candidate, out_builder);
}

navsys_status_t route_builder_create_from_route(
    const route_t* source,
    route_builder_t** out_builder) {
    if (!source || !out_builder)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_t* candidate = nullptr;
    navsys_status_t status = clone_for_edit(source, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    return allocate_builder(candidate, out_builder);
}

void route_builder_destroy(route_builder_t* builder) {
    if (!builder)
        return;
    route_destroy(builder->candidate);
    delete builder;
}

navsys_status_t route_builder_push_coord(
    route_builder_t* builder,
    const coord_t* coord) {
    if (!coord)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_t* current = nullptr;
    navsys_status_t status = builder_candidate(builder, &current);
    if (status != NAVSYS_STATUS_OK)
        return status;

    route_t* candidate = nullptr;
    status = clone_for_edit(current, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    status = coord_list_push_back_ex(candidate->coords, coord);
    if (status != NAVSYS_STATUS_OK) {
        route_destroy(candidate);
        return status;
    }
    reset_edit_metadata(candidate);
    commit_candidate(builder, candidate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_insert_coord(
    route_builder_t* builder,
    size_t index,
    const coord_t* coord) {
    if (!coord)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_t* current = nullptr;
    navsys_status_t status = builder_candidate(builder, &current);
    if (status != NAVSYS_STATUS_OK)
        return status;
    if (index > route_get_coord_count(current))
        return NAVSYS_STATUS_INVALID_ARGUMENT;

    route_t* candidate = nullptr;
    status = clone_for_edit(current, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    status = coord_list_insert_ex(candidate->coords, index, coord);
    if (status != NAVSYS_STATUS_OK) {
        route_destroy(candidate);
        return status;
    }
    reset_edit_metadata(candidate);
    commit_candidate(builder, candidate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_remove_coord(
    route_builder_t* builder,
    size_t index,
    coord_t* out_removed) {
    route_t* current = nullptr;
    navsys_status_t status = builder_candidate(builder, &current);
    if (status != NAVSYS_STATUS_OK)
        return status;
    if (index >= route_get_coord_count(current))
        return NAVSYS_STATUS_NOT_FOUND;

    route_t* candidate = nullptr;
    status = clone_for_edit(current, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    coord_t removed{};
    status = coord_list_remove_at_ex(candidate->coords, index, &removed);
    if (status != NAVSYS_STATUS_OK) {
        route_destroy(candidate);
        return status;
    }
    reset_edit_metadata(candidate);
    commit_candidate(builder, candidate);
    if (out_removed)
        *out_removed = removed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_append(
    route_builder_t* builder,
    const route_t* source,
    route_join_policy_t join_policy) {
    if (!source)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (join_policy != ROUTE_JOIN_KEEP_ALL
        && join_policy != ROUTE_JOIN_DEDUP_BOUNDARY) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    route_t* current = nullptr;
    navsys_status_t status = builder_candidate(builder, &current);
    if (status != NAVSYS_STATUS_OK)
        return status;

    const size_t source_count = route_get_coord_count(source);
    size_t start = 0;
    if (join_policy == ROUTE_JOIN_DEDUP_BOUNDARY
        && route_get_coord_count(current) > 0
        && source_count > 0) {
        coord_t left{};
        coord_t right{};
        status = route_fetch_coord(
            current, route_get_coord_count(current) - 1, &left);
        if (status == NAVSYS_STATUS_OK)
            status = route_fetch_coord(source, 0, &right);
        if (status != NAVSYS_STATUS_OK)
            return status;
        if (coord_equal(&left, &right))
            start = 1;
    }
    if (start == source_count)
        return NAVSYS_STATUS_OK;

    route_t* candidate = nullptr;
    status = clone_for_edit(current, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    for (size_t index = start; index < source_count; ++index) {
        coord_t coordinate{};
        status = route_fetch_coord(source, index, &coordinate);
        if (status == NAVSYS_STATUS_OK)
            status = coord_list_push_back_ex(candidate->coords, &coordinate);
        if (status != NAVSYS_STATUS_OK) {
            route_destroy(candidate);
            return status;
        }
    }
    reset_edit_metadata(candidate);
    commit_candidate(builder, candidate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_assign_slice(
    route_builder_t* builder,
    const route_t* source,
    size_t begin,
    size_t end) {
    if (!source)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_t* current = nullptr;
    navsys_status_t status = builder_candidate(builder, &current);
    if (status != NAVSYS_STATUS_OK)
        return status;

    route_t* candidate = nullptr;
    status = route_slice_ex(source, begin, end, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    reset_edit_metadata(candidate);
    commit_candidate(builder, candidate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_set_total_cost(
    route_builder_t* builder,
    double total_cost) {
    route_t* candidate = nullptr;
    navsys_status_t status = builder_candidate(builder, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    if (!std::isfinite(total_cost)
        || total_cost < -std::numeric_limits<float>::max()
        || total_cost > std::numeric_limits<float>::max()) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    candidate->cost = static_cast<float>(total_cost);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_set_completion(
    route_builder_t* builder,
    route_completion_t completion) {
    route_t* candidate = nullptr;
    navsys_status_t status = builder_candidate(builder, &candidate);
    if (status != NAVSYS_STATUS_OK)
        return status;
    const bool empty = route_get_coord_count(candidate) == 0;
    if (completion < ROUTE_COMPLETION_NONE
        || completion > ROUTE_COMPLETION_PARTIAL
        || (empty && completion != ROUTE_COMPLETION_NONE)
        || (!empty && completion == ROUTE_COMPLETION_NONE)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    candidate->success = completion == ROUTE_COMPLETION_COMPLETE;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_builder_finish(
    route_builder_t* builder,
    route_t** out_route) {
    if (!builder || !out_route)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!builder->candidate)
        return NAVSYS_STATUS_INVALIDATED;
    route_t* result = builder->candidate;
    builder->candidate = nullptr;
    *out_route = result;
    return NAVSYS_STATUS_OK;
}
