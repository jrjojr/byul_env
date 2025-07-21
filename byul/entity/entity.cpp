#include <cstring>
#include <cmath>
#include "internal/entity.h"
#include "internal/vec3.h"
#include "internal/xform.h"
#include "internal/bodyprops.h"
#include "internal/coord.h"

// ---------------------------------------------------------
// 기본 엔티티 초기화
// ---------------------------------------------------------
void entity_init(entity_t* e)
{
    if (!e) return;
    e->id = -1;
    e->coord = (coord_t){0, 0};
    e->owner = nullptr;
    e->age = 0.0f;
    e->lifetime = 0.0f;
}

void entity_init_full(
    entity_t* e,
    const coord_t* coord,
    int32_t id,
    void* owner,
    float age,
    float lifetime
)
{
    if (!e) return;
    e->id = id;
    e->coord = coord ? *coord : (coord_t){0, 0};
    e->owner = owner;
    e->age = fmaxf(0.0f, age);
    e->lifetime = fmaxf(0.0f, lifetime);
}

void entity_assign(entity_t* dst, const entity_t* src)
{
    if (!dst || !src) return;
    *dst = *src;
}

// ---------------------------------------------------------
// 수명 관리
// ---------------------------------------------------------
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
