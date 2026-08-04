#ifndef BYUL_NAVSYS_ROUTE_INTERNAL_ROUTE_INTERNAL_H
#define BYUL_NAVSYS_ROUTE_INTERNAL_ROUTE_INTERNAL_H

#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

coord_hash_t* route_internal_get_visited_count_mutable(route_t* route);

navsys_status_t route_internal_replace_visited_count(
    route_t* route,
    coord_hash_t* replacement);

navsys_status_t navsys_search_trace_internal_record(
    navsys_search_trace_t* trace,
    const coord_t* coord);

navsys_status_t navsys_search_trace_internal_clear(
    navsys_search_trace_t* trace);

#ifdef __cplusplus
}
#endif

#endif
