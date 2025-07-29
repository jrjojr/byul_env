#include <iostream>
#include "projectile_common.h"
#include <math.h>

void projectile_init(projectile_t* proj)
{
    if (!proj) return;

    entity_dynamic_init(&proj->base);
    proj->on_hit = projectile_default_hit_cb;
    proj->hit_userdata = NULL;
    proj->damage = 1.0f;
}

void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_attr_t attrs,
    float damage,
    projectile_hit_cb on_hit,
    void* hit_userdata
)
{
    if (!proj) return;

    if (base) {
        entity_dynamic_assign(&proj->base, base);
    } else {
        entity_dynamic_init(&proj->base);
    }

    proj->attrs = attrs;
    proj->damage = damage;
    proj->on_hit = on_hit;
    proj->hit_userdata = hit_userdata;
}

void projectile_assign(projectile_t* out, const projectile_t* src)
{
    if (!out || !src) return;
    *out = *src;
}

void projectile_update(projectile_t* proj, float dt)
{
    if (!proj || dt <= 0.0f) return;

    entity_dynamic_update(&proj->base, dt);

    if (proj->base.base.lifetime > 0.0f &&
        proj->base.base.age >= proj->base.base.lifetime)
    {
        if (proj->on_hit) {
            proj->on_hit(proj, proj->hit_userdata);
        }
    }
}

void projectile_default_hit_cb(const void* projectile, void* userdata)
{
    (void)userdata;

    const projectile_t* proj = (const projectile_t*)projectile;
    if (!proj) {
        printf("[projectile] hit callback called with null projectile\n");
        return;
    }

    printf("[projectile] default hit cb damaged : %.2f\n", proj->damage);
}
