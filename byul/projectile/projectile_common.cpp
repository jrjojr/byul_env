#include <iostream>
#include "internal/projectile_common.h"
#include <math.h>

// ---------------------------------------------------------
// 초기화 함수
// ---------------------------------------------------------
void projectile_init(projectile_t* proj)
{
    if (!proj) return;

    entity_dynamic_init(&proj->base);
    proj->on_hit = NULL;
    proj->hit_userdata = NULL;
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

// ---------------------------------------------------------
// 업데이트 함수
// ---------------------------------------------------------
void projectile_update(projectile_t* proj, float dt)
{
    if (!proj || dt <= 0.0f) return;

    entity_dynamic_update(&proj->base, dt);

    // 수명 체크 후 on_hit 호출
    if (proj->base.base.lifetime > 0.0f &&
        proj->base.base.age >= proj->base.base.lifetime)
    {
        if (proj->on_hit) {
            proj->on_hit(proj, proj->hit_userdata);
        }
    }
}

void projectile_default_hit_cb(const projectile_t* proj, void* userdata)
{
    (void)proj;
    (void)userdata;
    std::cout << "[projectile] default hit cb (no effect)\n";
}

