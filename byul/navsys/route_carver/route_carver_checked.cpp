#include "route_carver.h"

#include "internal/route_carver_geometry.hpp"
#include "internal/route_carver_mutation.hpp"

#include <limits>
#include <new>
#include <vector>

namespace {

namespace internal = byul::navsys::route_carver::internal;

constexpr uint32_t all_flags = NAVGRID_CARVE_INCLUDE_START
    | NAVGRID_CARVE_INCLUDE_END
    | NAVGRID_CARVE_CLIP_TO_EXTENT
    | NAVGRID_CARVE_ATOMIC
    | NAVGRID_CARVE_DRY_RUN;

struct checked_options {
    internal::carve_candidate_options geometry;
    internal::carve_match match;
    bool dry_run;
    bool atomic;
};

navsys_status_t read_options(
    const navgrid_carve_options_t* options,
    bool line,
    checked_options& out) {
    if (!options || options->struct_size < sizeof(*options)
        || options->reserved0 != 0 || options->max_cells == 0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (options->abi_version != NAVGRID_CARVE_OPTIONS_ABI_VERSION)
        return NAVSYS_STATUS_UNSUPPORTED;
    if ((options->flags & ~all_flags) != 0
        || options->metric > NAVGRID_CARVE_EUCLIDEAN_DISK
        || options->line_coverage > NAVGRID_LINE_SUPERCOVER_CELLS
        || options->match > NAVGRID_CARVE_EFFECTIVE_BLOCKED) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (!line
        && ((options->flags
                & (NAVGRID_CARVE_INCLUDE_START | NAVGRID_CARVE_INCLUDE_END))
                != 0
            || options->line_coverage != NAVGRID_LINE_CENTER_CELLS)) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (options->max_cells > NAVGRID_CARVE_MAX_CELLS
        || options->max_cells > std::numeric_limits<size_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }

    out.geometry = {
        options->radius_cells,
        static_cast<internal::carve_metric>(options->metric),
        static_cast<internal::line_coverage>(options->line_coverage),
        (options->flags & NAVGRID_CARVE_INCLUDE_START) != 0,
        (options->flags & NAVGRID_CARVE_INCLUDE_END) != 0,
        (options->flags & NAVGRID_CARVE_CLIP_TO_EXTENT) != 0,
        static_cast<size_t>(options->max_cells),
        options->cancel_func,
        options->cancel_userdata
    };
    out.match = static_cast<internal::carve_match>(options->match);
    out.dry_run = (options->flags & NAVGRID_CARVE_DRY_RUN) != 0;
    out.atomic = (options->flags & NAVGRID_CARVE_ATOMIC) != 0;
    return NAVSYS_STATUS_OK;
}

internal::carve_extent grid_extent(const navgrid_t* navgrid) {
    const int width = navgrid_get_width(navgrid);
    const int height = navgrid_get_height(navgrid);
    return {
        width > 0,
        height > 0,
        0,
        width,
        0,
        height
    };
}

navsys_status_t mutate(
    navgrid_t* navgrid,
    const checked_options& options,
    const std::vector<coord_t>& candidates,
    size_t* out_changed_count) {
    return internal::mutate_candidates_atomic(
        navgrid,
        candidates.empty() ? nullptr : candidates.data(),
        candidates.size(),
        options.match,
        options.dry_run,
        options.atomic,
        options.geometry.cancel_func,
        options.geometry.cancel_userdata,
        out_changed_count);
}

} // namespace

navsys_status_t navgrid_carve_line(
    navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    const navgrid_carve_options_t* options,
    size_t* out_changed_count) {
    if (!navgrid || !start || !goal || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        checked_options parsed{};
        const navsys_status_t option_status = read_options(options, true, parsed);
        if (option_status != NAVSYS_STATUS_OK) return option_status;
        std::vector<coord_t> candidates;
        const navsys_status_t geometry_status =
            internal::enumerate_line_candidates(
                *start, *goal, grid_extent(navgrid),
                parsed.geometry, candidates);
        if (geometry_status != NAVSYS_STATUS_OK) return geometry_status;
        return mutate(navgrid, parsed, candidates, out_changed_count);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t navgrid_carve_area(
    navgrid_t* navgrid,
    const coord_t* center,
    const navgrid_carve_options_t* options,
    size_t* out_changed_count) {
    if (!navgrid || !center || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    try {
        checked_options parsed{};
        const navsys_status_t option_status = read_options(options, false, parsed);
        if (option_status != NAVSYS_STATUS_OK) return option_status;
        std::vector<coord_t> candidates;
        const navsys_status_t geometry_status =
            internal::enumerate_area_candidates(
                *center, grid_extent(navgrid), parsed.geometry, candidates);
        if (geometry_status != NAVSYS_STATUS_OK) return geometry_status;
        return mutate(navgrid, parsed, candidates, out_changed_count);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}
