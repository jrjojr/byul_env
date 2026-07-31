#include "navcell.h"

#include <new>

namespace {

bool terrain_is_supported(terrain_type_t terrain) {
    switch (terrain) {
    case TERRAIN_TYPE_NORMAL:
    case TERRAIN_TYPE_WATER:
    case TERRAIN_TYPE_FOREST:
    case TERRAIN_TYPE_MOUNTAIN:
    case TERRAIN_TYPE_FORBIDDEN:
        return true;
    default:
        return false;
    }
}

} // namespace

navsys_status_t navcell_is_terrain_supported(
    terrain_type_t terrain, bool* out_supported) {
    if (!out_supported) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_supported = terrain_is_supported(terrain);
    return NAVSYS_STATUS_OK;
}

navsys_status_t navcell_validate(const navcell_t* cell) {
    if (!cell) return NAVSYS_STATUS_INVALID_ARGUMENT;
    return terrain_is_supported(cell->terrain)
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_UNSUPPORTED;
}

navsys_status_t navcell_init_checked(
    navcell_t* out_cell, terrain_type_t terrain, int32_t height) {
    if (!out_cell) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const navcell_t candidate = {terrain, static_cast<int>(height)};
    const navsys_status_t status = navcell_validate(&candidate);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_cell = candidate;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navcell_create_checked(
    terrain_type_t terrain, int32_t height, navcell_t** out_cell) {
    if (!out_cell) return NAVSYS_STATUS_INVALID_ARGUMENT;
    navcell_t candidate{};
    const navsys_status_t status =
        navcell_init_checked(&candidate, terrain, height);
    if (status != NAVSYS_STATUS_OK) return status;

    try {
        navcell_t* result = new navcell_t{candidate};
        *out_cell = result;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t navcell_copy_checked(
    const navcell_t* source, navcell_t** out_cell) {
    if (!source || !out_cell) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const navcell_t snapshot = *source;
    return navcell_create_checked(
        snapshot.terrain, static_cast<int32_t>(snapshot.height), out_cell);
}

navsys_status_t navcell_assign_checked(
    navcell_t* out_cell, const navcell_t* source) {
    if (!out_cell || !source) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const navcell_t snapshot = *source;
    const navsys_status_t status = navcell_validate(&snapshot);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_cell = snapshot;
    return NAVSYS_STATUS_OK;
}

navcell_t* navcell_create_full(terrain_type_t terrain, int height){
    navcell_t* result = nullptr;
    return navcell_create_checked(terrain, height, &result)
            == NAVSYS_STATUS_OK
        ? result
        : nullptr;
}

navcell_t* navcell_create(){
    return navcell_create_full(TERRAIN_TYPE_NORMAL, 0);
}

void navcell_destroy(navcell_t* nc){
    delete nc;
}

navcell_t* navcell_copy(const navcell_t* nc){
    navcell_t* result = nullptr;
    return navcell_copy_checked(nc, &result) == NAVSYS_STATUS_OK
        ? result
        : nullptr;
}

int navcell_init_full(
    navcell_t* nc, terrain_type_t terrain, int height)
{
    return static_cast<int>(navcell_init_checked(nc, terrain, height));
}

int navcell_init(navcell_t* nc)
{
    return static_cast<int>(
        navcell_init_checked(nc, TERRAIN_TYPE_NORMAL, 0));
}

int navcell_assign(navcell_t* nc, const navcell_t* src)
{
    return static_cast<int>(navcell_assign_checked(nc, src));
}
