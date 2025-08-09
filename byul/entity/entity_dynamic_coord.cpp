#include "entity_dynamic_coord.h"
#include "coord.h"
#include <math.h>

static inline int round_to_int(float x) {
    // (0.5 -> 1, -0.5 -> -1)
    return (int)llroundf(x);
}

void entity_dynamic_get_world_pos(
    const entity_dynamic_t* ed, float* out_x, float* out_y) {
    if (!ed || !out_x || !out_y) return;

    vec3_t pos;
    xform_get_position(&ed->xf, &pos);

    *out_x = (float)(ed->base.coord.x + round_to_int(pos.x));
    *out_y = (float)(ed->base.coord.y + round_to_int(pos.y));
}

void entity_dynamic_get_world_coord(
    const entity_dynamic_t* ed, coord_t* out) {
    if (!ed || !out) return;

    vec3_t pos;
    xform_get_position(&ed->xf, &pos);

    // coord + round(xform.translation)
    coord_assign(out, &ed->base.coord);
    coord_t delta;
    coord_init_full(&delta, (int)llroundf(pos.x), (int)llroundf(pos.y));
    coord_iadd(out, &delta);
}


void entity_dynamic_commit_coord(entity_dynamic_t* ed) {
    if (!ed) return;

    vec3_t pos;
    xform_get_position(&ed->xf, &pos);

    int dx = round_to_int(pos.x);
    int dy = round_to_int(pos.y);

    if (dx != 0 || dy != 0) {
        coord_t delta;
        coord_init_full(&delta, dx, dy);

        coord_iadd(&ed->base.coord, &delta);

        pos.x -= (float)dx;
        pos.y -= (float)dy;
        xform_set_position(&ed->xf, &pos);
    }
}

float entity_dynamic_coord_distance(
    const entity_dynamic_t* a, const entity_dynamic_t* b) {
    if (!a || !b) return INFINITY;
    return coord_distance(&a->base.coord, &b->base.coord);
}

bool entity_dynamic_coord_in_range(
    const entity_dynamic_t* a, const entity_dynamic_t* b) {
    if (!a || !b) return false;

    float dist = coord_distance(&a->base.coord, &b->base.coord);
    return (dist <= XFORM_MAX_POS);
}
