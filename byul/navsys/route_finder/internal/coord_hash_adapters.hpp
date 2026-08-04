#ifndef BYUL_ROUTE_FINDER_COORD_HASH_ADAPTERS_HPP
#define BYUL_ROUTE_FINDER_COORD_HASH_ADAPTERS_HPP

#include "coord.h"

inline void* route_finder_coord_copy_for_hash(const void* value) {
    return coord_copy(static_cast<const coord_t*>(value));
}

inline void route_finder_coord_destroy_for_hash(void* value) {
    coord_destroy(static_cast<coord_t*>(value));
}

#endif
