#include "navcell.h"

navcell_t* navcell_create_full(terrain_type_t terrain, int height){
    navcell_t* nc = new navcell_t{};
    nc->terrain = terrain;
    nc->height = height;
    return nc;
}

navcell_t* navcell_create(){
    return navcell_create_full(TERRAIN_TYPE_NORMAL, 0);
}

void navcell_destroy(navcell_t* nc){
    if(nc) delete nc;
}

navcell_t* navcell_copy(const navcell_t* nc){
    if (!nc) return nullptr;

    return navcell_create_full(nc->terrain, nc->height);
}

int navcell_init_full(
    navcell_t* nc, terrain_type_t terrain, int height)
{
    if (!nc) return -1;
    nc->terrain = terrain;
    nc->height = height;
    return 0;
}

int navcell_init(navcell_t* nc)
{
    if (!nc) return -1;
    nc->terrain = TERRAIN_TYPE_NORMAL;
    nc->height = 0;
    return 0;
}

int navcell_assign(navcell_t* nc, const navcell_t* src)
{
    if (!nc || !src) return -1;
    nc->terrain = src->terrain;
    nc->height = src->height;
    return 0;
}
