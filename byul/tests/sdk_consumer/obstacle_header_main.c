#include "obstacle.h"

_Static_assert(BYUL_OBSTACLE_ABI_VERSION == UINT32_C(2),
    "unexpected Obstacle ABI version");
_Static_assert(ENCLOSURE_OPEN_UNKNOWN == 0, "unexpected enclosure enum");
_Static_assert(ENCLOSURE_OPEN_RIGHT == 1, "unexpected enclosure enum");
_Static_assert(ENCLOSURE_OPEN_UP == 2, "unexpected enclosure enum");
_Static_assert(ENCLOSURE_OPEN_LEFT == 3, "unexpected enclosure enum");
_Static_assert(ENCLOSURE_OPEN_DOWN == 4, "unexpected enclosure enum");
_Static_assert(SPIRAL_CLOCKWISE == 0, "unexpected spiral enum");
_Static_assert(SPIRAL_COUNTER_CLOCKWISE == 1, "unexpected spiral enum");
_Static_assert(OBSTACLE_RASTER_CELL_CENTER == 0, "unexpected raster enum");
_Static_assert(OBSTACLE_RASTER_ALL_TOUCHED == 1, "unexpected raster enum");
_Static_assert(OBSTACLE_POLYGON_EVEN_ODD == 0, "unexpected fill enum");
_Static_assert(OBSTACLE_POLYGON_NON_ZERO == 1, "unexpected fill enum");
_Static_assert(OBSTACLE_ENCLOSURE_CLOSED == 0, "unexpected enclosure enum");
_Static_assert(OBSTACLE_ENCLOSURE_OPEN_DOWN == 4, "unexpected enclosure enum");
_Static_assert(OBSTACLE_SPIRAL_CLOCKWISE == 0, "unexpected spiral enum");
_Static_assert(OBSTACLE_SPIRAL_COUNTER_CLOCKWISE == 1, "unexpected spiral enum");
_Static_assert(OBSTACLE_SPIRAL_CLIP_PATH_ONLY == 0, "unexpected clip enum");
_Static_assert(OBSTACLE_SPIRAL_CLIP_OUTPUT == 1, "unexpected clip enum");

static void check_generator_declarations(void) {
    (void)obstacle_generate_options_init;
    (void)obstacle_generate_filled_rect;
    (void)obstacle_generate_rect_outline;
    (void)obstacle_generate_random_rect;
    (void)obstacle_generate_line;
    (void)obstacle_generate_polygon;
    (void)obstacle_generate_polygon_outline;
    (void)obstacle_generate_triangle;
    (void)obstacle_generate_triangle_outline;
    (void)obstacle_enclosure_desc_init;
    (void)obstacle_cross_desc_init;
    (void)obstacle_spiral_desc_init;
    (void)obstacle_generate_enclosure;
    (void)obstacle_generate_cross;
    (void)obstacle_generate_spiral;
    (void)obstacle_make_rect_all_blocked;
    (void)obstacle_make_rect_random_blocked;
    (void)obstacle_make_beam;
    (void)obstacle_make_torus;
    (void)obstacle_make_enclosure;
    (void)obstacle_make_cross;
    (void)obstacle_make_spiral;
    (void)obstacle_make_triangle;
    (void)obstacle_make_triangle_torus;
    (void)obstacle_make_polygon;
    (void)obstacle_make_polygon_torus;
}

int main(void) {
    check_generator_declarations();
    obstacle_generate_options_t options;
    if (obstacle_generate_options_init(&options) != NAVSYS_STATUS_OK
        || options.struct_size != sizeof(options)
        || options.abi_version != OBSTACLE_GENERATE_OPTIONS_ABI_VERSION) {
        return 3;
    }
    obstacle_enclosure_desc_t enclosure;
    obstacle_cross_desc_t cross;
    obstacle_spiral_desc_t spiral;
    if (obstacle_enclosure_desc_init(&enclosure) != NAVSYS_STATUS_OK
        || obstacle_cross_desc_init(&cross) != NAVSYS_STATUS_OK
        || obstacle_spiral_desc_init(&spiral) != NAVSYS_STATUS_OK
        || enclosure.abi_version != OBSTACLE_ENCLOSURE_DESC_ABI_VERSION
        || cross.abi_version != OBSTACLE_CROSS_DESC_ABI_VERSION
        || spiral.abi_version != OBSTACLE_SPIRAL_DESC_ABI_VERSION) {
        return 4;
    }
    obstacle_abi_mismatch_t mismatch = OBSTACLE_ABI_VERSION_MISMATCH;
    obstacle_t* obstacle = obstacle_create_full(1, 2, 7, 9);
    if (obstacle == NULL) {
        return 1;
    }
    coord_t outline_start = {0, 0};
    coord_t outline_goal = {4, 4};
    obstacle_t* legacy_outline = obstacle_make_torus(
        &outline_start, &outline_goal, 1);
    obstacle_t* canonical_outline = NULL;
    const navsys_status_t outline_status = obstacle_generate_rect_outline(
        0, 0, 5, 5, 1, NULL, &canonical_outline);
    const int compatible = legacy_outline != NULL
        && outline_status == NAVSYS_STATUS_OK
        && canonical_outline != NULL
        && obstacle_equal(legacy_outline, canonical_outline);
    const int valid = obstacle_get_abi_version() == BYUL_OBSTACLE_ABI_VERSION
        && obstacle_get_abi_fingerprint() == BYUL_OBSTACLE_ABI_FINGERPRINT
        && obstacle_check_abi(
            BYUL_OBSTACLE_ABI_VERSION,
            BYUL_OBSTACLE_ABI_FINGERPRINT,
            &mismatch) == NAVSYS_STATUS_OK
        && mismatch == OBSTACLE_ABI_MATCH
        && obstacle_get_width(obstacle) == 7
        && obstacle_get_height(obstacle) == 9
        && compatible;
    obstacle_destroy(canonical_outline);
    obstacle_destroy(legacy_outline);
    obstacle_destroy(obstacle);
    return valid ? 0 : 2;
}
