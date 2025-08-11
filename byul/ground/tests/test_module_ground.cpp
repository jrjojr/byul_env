#include "doctest.h"

#include "ground.h"
#include "vec3.h"
#include "plane.h"
#include "bodyprops.h"
#include "coord.h"
#include "coord_hash.h"

#include <cmath>
#include <cstdlib>

// -------------------------- helpers --------------------------

static void make_plane_z(plane_t* p, float z) {
    vec3_t nz; vec3_init_full(&nz, 0.0f, 0.0f, 1.0f);
    // n = (0,0,1), height = z  => plane z = z
    plane_init_normal_height(p, &nz, z);
}

static bodyprops_t make_body(float mass) {
    bodyprops_t b{};
    bodyprops_init_full(&b, mass,
                        /*drag*/0.47f,
                        /*cross_section*/0.1f,
                        /*restitution*/0.5f,
                        /*friction*/0.3f,
                        /*k_magnus*/0.0f,
                        /*k_gyro*/0.0f);
    return b;
}

static void tile_mapper_cell1(const void* ctx, const vec3_t* pos_world, coord_t* out_c) {
    (void)ctx;
    out_c->x = (int)std::floor(pos_world->x);
    out_c->y = (int)std::floor(pos_world->y);
}

// coord_hash copy/destroy for bodyprops_t and plane_t
static void* bodyprops_copy_cb(const void* v) {
    const bodyprops_t* src = (const bodyprops_t*)v;
    bodyprops_t* p = (bodyprops_t*)std::malloc(sizeof(*p));
    *p = *src;
    return p;
}
static void bodyprops_destroy_cb(void* v) {
    std::free(v);
}
static void* plane_copy_cb(const void* v) {
    const plane_t* src = (const plane_t*)v;
    plane_t* p = (plane_t*)std::malloc(sizeof(*p));
    *p = *src;
    return p;
}
static void plane_destroy_cb(void* v) {
    std::free(v);
}

// -------------------------- tests ----------------------------

TEST_CASE("ground: uniform sample and raycast") {
    ground_t g{};
    bodyprops_t bp = make_body(2.0f);
    plane_t pl; make_plane_z(&pl, 0.0f);
    ground_init_uniform(&g, &bp, &pl);

    vec3_t pos; vec3_init_full(&pos, 1.0f, 2.0f, 5.0f);

    vec3_t surf{}, nrm{}; bodyprops_t outb{};
    CHECK(ground_sample_at(&g, &pos, &surf, &nrm, &outb));

    CHECK(doctest::Approx(surf.x).epsilon(1e-5) == 1.0f);
    CHECK(doctest::Approx(surf.y).epsilon(1e-5) == 2.0f);
    CHECK(doctest::Approx(surf.z).epsilon(1e-5) == 0.0f);
    // normal should point +Z
    CHECK(doctest::Approx(nrm.x).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(nrm.y).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(nrm.z).epsilon(1e-5) == -1.0f);
    CHECK(doctest::Approx(outb.mass).epsilon(1e-5) == 2.0f);

    vec3_t org; vec3_init_full(&org, 0.0f, 0.0f, 10.0f);
    vec3_t dir; vec3_init_full(&dir, 0.0f, 0.0f, -1.0f);

    vec3_t hit{}, nh{};
    float t = -1.0f;
    CHECK(ground_raycast(&g, &org, &dir, 100.0f, &hit, &nh, &outb, &t));
    CHECK(doctest::Approx(t).epsilon(1e-5) == 10.0f);
    CHECK(doctest::Approx(hit.x).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(hit.y).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(hit.z).epsilon(1e-5) == 0.0f);
    CHECK(doctest::Approx(nh.z).epsilon(1e-5) == 1.0f);
}

TEST_CASE("ground: heightfield sample and raycast") {
    // Heightfield: z = 0.5 * x (linear in x), cell = 1.0, 3x3
    const int W = 3, H = 3;
    float* Hbuf = (float*)std::malloc(sizeof(float) * W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            Hbuf[y*W + x] = 0.5f * (float)x;
        }
    }
    ground_t g{};
    ground_init_heightfield(&g, W, H, 1.0f, Hbuf, /*owns_buffer*/true);

    // Sample at (x=1.2, y=1.5) => z = 0.5 * 1.2 = 0.6
    vec3_t pos; vec3_init_full(&pos, 1.2f, 1.5f, 10.0f);
    vec3_t surf{}, nrm{};
    CHECK(ground_sample_at(&g, &pos, &surf, &nrm, nullptr));
    CHECK(doctest::Approx(surf.z).epsilon(1e-4) == 0.6f);

    // Expected normal ~ normalize((-dzdx, -dzdy, 1)) = (-0.5, 0, 1)
    vec3_t expect_n = { -0.5f, 0.0f, 1.0f };
    // Normalize expected
    {
        float len = std::sqrt(expect_n.x*expect_n.x + expect_n.y*expect_n.y + expect_n.z*expect_n.z);
        expect_n.x /= len; expect_n.y /= len; expect_n.z /= len;
    }
    // Dot should be close to 1
    float dot = nrm.x*expect_n.x + nrm.y*expect_n.y + nrm.z*expect_n.z;
    CHECK(dot > 0.999f);

    // Raycast from above, straight down
    vec3_t org; vec3_init_full(&org, 1.2f, 1.5f, 5.0f);
    vec3_t dir; vec3_init_full(&dir, 0.0f, 0.0f, -1.0f);
    vec3_t hit{};
    float t = -1.0f;
    CHECK(ground_raycast(&g, &org, &dir, 10.0f, &hit, &nrm, nullptr, &t));
    CHECK(doctest::Approx(hit.z).epsilon(1e-4) == 0.6f);
    CHECK(doctest::Approx(t).epsilon(1e-4) == 4.4f); // 5.0 -> 0.6
    // free implicitly tested via owns_buffer=true in ground_release
    ground_free(&g);
}

TEST_CASE("ground: tiles overrides plane and material") {
    // Uniform base plane z=0 is irrelevant because we will override both plane and body at (1,2)
    ground_t g{};
    // Init tiles with tables
    coord_hash_t* bp_table = coord_hash_create_full(bodyprops_copy_cb, bodyprops_destroy_cb);
    coord_hash_t* pl_table = coord_hash_create_full(plane_copy_cb, plane_destroy_cb);

    // Insert overrides at tile (1,2)
    {
        bodyprops_t special = make_body(3.14f);
        CHECK(coord_hash_insert_xy(bp_table, 1, 2, &special));

        plane_t tile_plane; make_plane_z(&tile_plane, 1.0f); // z = 1 plane
        CHECK(coord_hash_insert_xy(pl_table, 1, 2, &tile_plane));
    }

    ground_init_tiles(&g, bp_table, pl_table, tile_mapper_cell1, nullptr);

    // Sample at world position mapping to (1,2)
    vec3_t pos; vec3_init_full(&pos, 1.2f, 2.3f, 10.0f);
    vec3_t surf{}, nrm{}; bodyprops_t outb{};
    CHECK(ground_sample_at(&g, &pos, &surf, &nrm, &outb));

    // Plane override: z must be 1.0
    CHECK(doctest::Approx(surf.z).epsilon(1e-5) == 1.0f);
    // Bodyprops override: mass must be 3.14
    CHECK(doctest::Approx(outb.mass).epsilon(1e-5) == 3.14f);

    // Raycast down from z=5 should hit at z=1, t=4
    vec3_t org; vec3_init_full(&org, 1.2f, 2.3f, 5.0f);
    vec3_t dir; vec3_init_full(&dir, 0.0f, 0.0f, -1.0f);
    vec3_t hit{}; float t = -1.0f;
    CHECK(ground_raycast(&g, &org, &dir, 10.0f, &hit, &nrm, &outb, &t));
    CHECK(doctest::Approx(hit.z).epsilon(1e-5) == 1.0f);
    CHECK(doctest::Approx(t).epsilon(1e-5) == 4.0f);

    coord_hash_destroy(bp_table);
    coord_hash_destroy(pl_table);
}
