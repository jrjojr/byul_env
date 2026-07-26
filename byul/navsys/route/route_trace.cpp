#include "route.h"
#include "internal/route_internal.h"

struct s_navsys_search_trace {
    coord_list_t* visited_order;
    coord_hash_t* visited_count;
};

navsys_status_t navsys_search_trace_create(
    navsys_search_trace_t** out_trace) {
    if (!out_trace)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    navsys_search_trace_t* trace = nullptr;
    try {
        trace = new navsys_search_trace_t{};
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    navsys_status_t status = coord_list_create_ex(&trace->visited_order);
    if (status == NAVSYS_STATUS_OK) {
        trace->visited_count = coord_hash_create_full(
            coord_hash_int_copy, coord_hash_int_destroy);
        if (!trace->visited_count)
            status = NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    if (status != NAVSYS_STATUS_OK) {
        navsys_search_trace_destroy(trace);
        return status;
    }
    *out_trace = trace;
    return NAVSYS_STATUS_OK;
}

void navsys_search_trace_destroy(navsys_search_trace_t* trace) {
    if (!trace)
        return;
    coord_list_destroy(trace->visited_order);
    coord_hash_destroy(trace->visited_count);
    delete trace;
}

navsys_status_t navsys_search_trace_clone_ex(
    const navsys_search_trace_t* source,
    navsys_search_trace_t** out_trace) {
    if (!source || !out_trace)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source->visited_order || !source->visited_count)
        return NAVSYS_STATUS_CORRUPT_STATE;

    navsys_search_trace_t* clone = nullptr;
    try {
        clone = new navsys_search_trace_t{};
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    navsys_status_t status =
        coord_list_copy_ex(source->visited_order, &clone->visited_order);
    if (status == NAVSYS_STATUS_OK) {
        status = coord_hash_copy_ex(
            source->visited_count, &clone->visited_count);
    }
    if (status != NAVSYS_STATUS_OK) {
        navsys_search_trace_destroy(clone);
        return status;
    }
    *out_trace = clone;
    return NAVSYS_STATUS_OK;
}

size_t navsys_search_trace_get_visit_count(
    const navsys_search_trace_t* trace) {
    return trace && trace->visited_order
        ? coord_list_size(trace->visited_order)
        : 0;
}

navsys_status_t navsys_search_trace_fetch_visit(
    const navsys_search_trace_t* trace,
    size_t index,
    coord_t* out_coord) {
    if (!trace || !out_coord)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!trace->visited_order)
        return NAVSYS_STATUS_CORRUPT_STATE;
    return coord_list_fetch(trace->visited_order, index, out_coord);
}

navsys_status_t navsys_search_trace_fetch_coord_visit_count(
    const navsys_search_trace_t* trace,
    const coord_t* coord,
    size_t* out_count) {
    if (!trace || !coord || !out_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!trace->visited_count)
        return NAVSYS_STATUS_CORRUPT_STATE;
    const int* count = static_cast<const int*>(
        coord_hash_get(trace->visited_count, coord));
    if (!count)
        return NAVSYS_STATUS_NOT_FOUND;
    *out_count = static_cast<size_t>(*count);
    return NAVSYS_STATUS_OK;
}

navsys_status_t navsys_search_trace_export_visits(
    const navsys_search_trace_t* trace,
    coord_t* output,
    size_t capacity,
    size_t* out_required_count) {
    if (!trace || !out_required_count || (!output && capacity != 0))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!trace->visited_order)
        return NAVSYS_STATUS_CORRUPT_STATE;
    const size_t required = coord_list_size(trace->visited_order);
    *out_required_count = required;
    if (!output)
        return NAVSYS_STATUS_OK;
    if (capacity < required)
        return NAVSYS_STATUS_INCOMPLETE;
    for (size_t index = 0; index < required; ++index) {
        navsys_status_t status = coord_list_fetch(
            trace->visited_order, index, &output[index]);
        if (status != NAVSYS_STATUS_OK)
            return NAVSYS_STATUS_CORRUPT_STATE;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t navsys_search_trace_internal_record(
    navsys_search_trace_t* trace,
    const coord_t* coord) {
    if (!trace || !coord)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!trace->visited_order || !trace->visited_count)
        return NAVSYS_STATUS_CORRUPT_STATE;

    const int* previous = static_cast<const int*>(
        coord_hash_get(trace->visited_count, coord));
    const int count = previous ? *previous + 1 : 1;
    const size_t order_size = coord_list_size(trace->visited_order);
    navsys_status_t status =
        coord_list_push_back_ex(trace->visited_order, coord);
    if (status != NAVSYS_STATUS_OK)
        return status;
    status = coord_hash_upsert_copy(
        trace->visited_count, coord, &count, nullptr);
    if (status != NAVSYS_STATUS_OK) {
        coord_t removed{};
        (void)coord_list_remove_at_ex(
            trace->visited_order, order_size, &removed);
    }
    return status;
}

navsys_status_t navsys_search_trace_internal_clear(
    navsys_search_trace_t* trace) {
    if (!trace)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!trace->visited_order || !trace->visited_count)
        return NAVSYS_STATUS_CORRUPT_STATE;
    coord_list_clear(trace->visited_order);
    coord_hash_clear(trace->visited_count);
    return NAVSYS_STATUS_OK;
}
