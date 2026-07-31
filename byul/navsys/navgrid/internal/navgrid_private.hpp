#ifndef BYUL_NAVGRID_INTERNAL_NAVGRID_PRIVATE_HPP
#define BYUL_NAVGRID_INTERNAL_NAVGRID_PRIVATE_HPP

#include "../navgrid.h"

struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;
    coord_hash_t* cell_map;
    is_coord_blocked_func is_coord_blocked_fn;
    void* is_coord_blocked_fn_userdata;
};

static_assert(sizeof(s_navgrid) == 40, "Navgrid ABI 1 binary layout changed");
static_assert(alignof(s_navgrid) == 8, "Navgrid ABI 1 alignment changed");

#endif /* BYUL_NAVGRID_INTERNAL_NAVGRID_PRIVATE_HPP */
