#ifndef BYUL_OBSTACLE_INTERNAL_OBSTACLE_PRIVATE_HPP
#define BYUL_OBSTACLE_INTERNAL_OBSTACLE_PRIVATE_HPP

#include "../obstacle_core.h"
#include "../../coord/coord_hash.h"

struct s_obstacle {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
};

static_assert(
    sizeof(s_obstacle) == (sizeof(void*) == 8 ? 24 : 20),
    "Obstacle ABI 1 binary layout changed");
static_assert(
    alignof(s_obstacle) == alignof(void*),
    "Obstacle ABI 1 alignment changed");

#endif /* BYUL_OBSTACLE_INTERNAL_OBSTACLE_PRIVATE_HPP */
