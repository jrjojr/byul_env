#include "internal/route_carver_mutation.hpp"

#include "../navgrid/internal/navgrid_callback.hpp"
#include "../navgrid/internal/navgrid_overlay.hpp"

#include <algorithm>
#include <new>
#include <vector>

namespace byul::navsys::route_carver::internal {
namespace {

bool valid_match(carve_match match) {
    return match == carve_match::stored_forbidden_only
        || match == carve_match::effective_blocked;
}

navsys_status_t poll_cancel(
    carve_cancel_func cancel_func, void* cancel_userdata) {
    if (!cancel_func) return NAVSYS_STATUS_OK;
    try {
        return cancel_func(cancel_userdata)
            ? NAVSYS_STATUS_CANCELLED
            : NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

navsys_status_t collect_stored_forbidden(
    navgrid_t* navgrid,
    const coord_t* candidates,
    size_t count,
    carve_cancel_func cancel_func,
    void* cancel_userdata,
    std::vector<coord_t>& selected) {
    for (size_t index = 0; index < count; ++index) {
        if (index % carve_poll_interval_cells == 0) {
            const navsys_status_t cancel_status = poll_cancel(
                cancel_func, cancel_userdata);
            if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
        }
        navcell_t cell{};
        bool present = false;
        const navsys_status_t status = navgrid_fetch_cell_ex(
            navgrid, candidates[index].x, candidates[index].y,
            &cell, &present);
        if (status != NAVSYS_STATUS_OK) return status;
        if (present && cell.terrain == TERRAIN_TYPE_FORBIDDEN)
            selected.push_back(candidates[index]);
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t collect_effective_builtin(
    navgrid_t* navgrid,
    const coord_t* candidates,
    size_t count,
    carve_cancel_func cancel_func,
    void* cancel_userdata,
    std::vector<coord_t>& selected) {
    is_coord_blocked_func callback = nullptr;
    void* userdata = nullptr;
    const navsys_status_t binding_status =
        navgrid_fetch_is_coord_blocked_binding(
            navgrid, &callback, &userdata);
    if (binding_status != NAVSYS_STATUS_OK) return binding_status;

    const bool mutable_builtin = callback == is_coord_blocked_navgrid;
    for (size_t index = 0; index < count; ++index) {
        if (index % carve_poll_interval_cells == 0) {
            const navsys_status_t cancel_status = poll_cancel(
                cancel_func, cancel_userdata);
            if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
        }
        bool blocked = false;
        const navsys_status_t status =
            byul::navsys::internal::navgrid_invoke_is_coord_blocked_checked(
                navgrid, candidates[index].x, candidates[index].y, &blocked);
        if (status != NAVSYS_STATUS_OK) return status;
        if (mutable_builtin && blocked) selected.push_back(candidates[index]);
    }
    return NAVSYS_STATUS_OK;
}

} // namespace

navsys_status_t mutate_candidates_atomic(
    navgrid_t* navgrid,
    const coord_t* candidates,
    size_t count,
    carve_match match,
    bool dry_run,
    bool atomic,
    carve_cancel_func cancel_func,
    void* cancel_userdata,
    size_t* out_changed_count) {
    if (!navgrid || (!candidates && count != 0) || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!valid_match(match)) return NAVSYS_STATUS_UNSUPPORTED;

    try {
        std::vector<coord_t> selected;
        selected.reserve(count);
        const navsys_status_t status = match == carve_match::stored_forbidden_only
            ? collect_stored_forbidden(
                navgrid, candidates, count,
                cancel_func, cancel_userdata, selected)
            : collect_effective_builtin(
                navgrid, candidates, count,
                cancel_func, cancel_userdata, selected);
        if (status != NAVSYS_STATUS_OK) return status;

        navsys_status_t cancel_status = poll_cancel(
            cancel_func, cancel_userdata);
        if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;

        if (dry_run || atomic) {
            size_t changed = 0;
            const navsys_status_t mutation_status =
                byul::navsys::internal::navgrid_apply_open_overlay_atomic(
                    navgrid,
                    selected.empty() ? nullptr : selected.data(),
                    selected.size(),
                    dry_run,
                    cancel_func,
                    cancel_userdata,
                    &changed);
            if (mutation_status != NAVSYS_STATUS_OK) return mutation_status;
            *out_changed_count = changed;
            return NAVSYS_STATUS_OK;
        }

        size_t changed_total = 0;
        for (size_t offset = 0; offset < selected.size();
             offset += carve_poll_interval_cells) {
            const size_t chunk_count = std::min(
                carve_poll_interval_cells, selected.size() - offset);
            size_t chunk_changed = 0;
            const navsys_status_t mutation_status =
                byul::navsys::internal::navgrid_apply_open_overlay_atomic(
                    navgrid,
                    selected.data() + offset,
                    chunk_count,
                    false,
                    cancel_func,
                    cancel_userdata,
                    &chunk_changed);
            if (mutation_status != NAVSYS_STATUS_OK) {
                *out_changed_count = changed_total;
                return mutation_status;
            }
            changed_total += chunk_changed;
        }
        *out_changed_count = changed_total;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

} // namespace byul::navsys::route_carver::internal
