#include <cstring>
#include <cmath>
#include "entity.h"
#include "vec3.h"
#include "xform.h"
#include "bodyprops.h"
#include "coord.h"

#include <math.h>   // fmaxf
#include <string.h> // memset

void entity_init(entity_t* e)
{
    if (!e) return;
    e->id = -1;
    e->coord = coord_t{0, 0};
    e->owner = NULL;
    e->age = 0.0f;
    e->lifetime = 0.0f;
    e->width_range = 0;
    e->height_range = 0;
    e->influence_ratio = 1.0f;
}

void entity_init_full(
    entity_t* e,
    const coord_t* coord,
    int32_t id,
    void* owner,
    float age,
    float lifetime,
    int width,
    int height,
    float influence
)
{
    if (!e) return;
    e->id = id;
    e->coord = coord ? *coord : coord_t{0, 0};
    e->owner = owner;
    e->age = fmaxf(0.0f, age);
    e->lifetime = fmaxf(0.0f, lifetime);
    e->width_range = (width > 0) ? width : 0;
    e->height_range = (height > 0) ? height : 0;
    e->influence_ratio = (influence >= 0.0f) ? influence : 0.0f;
}

void entity_assign(entity_t* dst, const entity_t* src)
{
    if (!dst || !src) return;
    *dst = *src;
}

bool entity_is_expired(const entity_t* e)
{
    if (!e) return true;
    return (e->lifetime > 0.0f && e->age >= e->lifetime);
}

bool entity_tick(entity_t* e, float dt)
{
    if (!e || dt <= 0.0f) return false;
    e->age += dt;
    return entity_is_expired(e);
}

float entity_size(const entity_t* e)
{
    if (!e) return 0.0f;

    // size: sqrt(1 + width_range^2 + height_range^2)
    float diag = sqrtf(1.0f +
                       (float)(e->width_range * e->width_range) +
                       (float)(e->height_range * e->height_range));

    return diag * e->influence_ratio;
}
