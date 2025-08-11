// ground.cpp
#include "ground.h"
#include "vec3.h"
#include "plane.h"
#include "coord_hash.h"
#include <cmath>
#include <cstring>
#include <cstdlib>

// -----------------------------------------------------------------------------
// Small helpers
static inline int iclamp(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline bool nearly_zero(float v) {
    return std::fabs(v) <= 1e-6f;
}

// -----------------------------------------------------------------------------
// Heightfield sampling
static inline float hf_at_ixiy(const ground_heightfield_t* hf, int ix, int iy) 
{
    ix = iclamp(ix, 0, hf->w - 1);
    iy = iclamp(iy, 0, hf->h - 1);
    return hf->height[iy * hf->w + ix];
}

static float hf_sample_bilinear(const ground_heightfield_t* hf, 
    float xw, float yw) 
{
    if (!hf || hf->w <= 0 || hf->h <= 0 || hf->cell <= 0.0f) return 0.0f;

    float gx = xw / hf->cell;
    float gy = yw / hf->cell;
    int ix = (int)std::floor(gx);
    int iy = (int)std::floor(gy);
    float fx = gx - (float)ix;
    float fy = gy - (float)iy;

    int ix0 = iclamp(ix, 0, hf->w - 1);
    int iy0 = iclamp(iy, 0, hf->h - 1);
    int ix1 = iclamp(ix + 1, 0, hf->w - 1);
    int iy1 = iclamp(iy + 1, 0, hf->h - 1);

    float h00 = hf->height[iy0 * hf->w + ix0];
    float h10 = hf->height[iy0 * hf->w + ix1];
    float h01 = hf->height[iy1 * hf->w + ix0];
    float h11 = hf->height[iy1 * hf->w + ix1];

    float hx0 = h00 + (h10 - h00) 
    * (fx < 0.0f ? 0.0f : (fx > 1.0f ? 1.0f : fx));

    float hx1 = h01 + (h11 - h01) 
    * (fx < 0.0f ? 0.0f : (fx > 1.0f ? 1.0f : fx));

    return hx0 + (hx1 - hx0) * (fy < 0.0f ? 0.0f : (fy > 1.0f ? 1.0f : fy));
}

static void hf_normal_at(const ground_heightfield_t* hf, float xw, float yw, 
    vec3_t* out_n) 
{
    float gx = xw / hf->cell;
    float gy = yw / hf->cell;
    int ix = iclamp((int)std::floor(gx), 0, hf->w - 1);
    int iy = iclamp((int)std::floor(gy), 0, hf->h - 1);

    float hL = hf_at_ixiy(hf, ix - 1, iy);
    float hR = hf_at_ixiy(hf, ix + 1, iy);
    float hD = hf_at_ixiy(hf, ix, iy - 1);
    float hU = hf_at_ixiy(hf, ix, iy + 1);

    float dzdx = (hR - hL) / (2.0f * hf->cell);
    float dzdy = (hU - hD) / (2.0f * hf->cell);

    vec3_t n = { -dzdx, -dzdy, 1.0f };
    vec3_unit(out_n, &n);
}

// -----------------------------------------------------------------------------
// Tiles helpers (expect values stored as bodyprops_t* or plane_t*)
static bool tiles_world_to_coord(
    const ground_tiles_t* t, const vec3_t* pos_world, coord_t* out_c) 
{
    if (!t || !t->map_cb || !out_c) return false;
    t->map_cb(t->map_context, pos_world, out_c);
    return true;
}

static bool tiles_get_bodyprops(const ground_tiles_t* t, const coord_t* c, 
    bodyprops_t* out_body) 
{
    if (!t || !t->bodyprops_table || !out_body) return false;
    void* pv = coord_hash_get(t->bodyprops_table, c);
    if (!pv) return false;
    *out_body = *(const bodyprops_t*)pv;
    return true;
}

static bool tiles_get_plane(const ground_tiles_t* t, const coord_t* c, 
    plane_t* out_plane) 
{
    if (!t || !t->plane_table || !out_plane) return false;
    void* pv = coord_hash_get(t->plane_table, c);
    if (!pv) return false;
    *out_plane = *(const plane_t*)pv;
    return true;
}

// -----------------------------------------------------------------------------
// Plane helpers (do not assume private fields)
static void plane_sample_point(const plane_t* p, const vec3_t* pos_world, 
    vec3_t* out_point) 
{
    plane_project(out_point, p, pos_world);
}

static void plane_estimate_normal(const plane_t* p, const vec3_t* pos_world, 
    vec3_t* out_n) 
{
    vec3_t p0; 
    plane_project(&p0, p, pos_world);

    vec3_t xoff = { 0.25f, 0.0f, 0.0f };
    vec3_t yoff = { 0.0f, 0.25f, 0.0f };
    vec3_t qx, qy;
    vec3_madd(&qx, pos_world, &xoff, 1.0f);
    vec3_madd(&qy, pos_world, &yoff, 1.0f);

    vec3_t px, py;
    plane_project(&px, p, &qx);
    plane_project(&py, p, &qy);

    vec3_t ex, ey;
    vec3_sub(&ex, &px, &p0);
    vec3_sub(&ey, &py, &p0);

    vec3_cross(out_n, &ey, &ex);
    vec3_unit(out_n, out_n);
}

void ground_init(ground_t* g)
{
    bodyprops_t body = {};
    plane_t plane = {};

    bodyprops_init(&body);
    plane_init(&plane);

    ground_init_uniform(g, &body, &plane);
}

// -----------------------------------------------------------------------------
// Lifecycle
void ground_init_uniform(ground_t* g, 
    const bodyprops_t* body, const plane_t* plane) 
{
    if (!g || !body || !plane) return;
    std::memset(g, 0, sizeof(*g));
    g->mode = GROUND_MODE_UNIFORM;
    g->u.uniform.body  = *body;
    g->u.uniform.plane = *plane;
}

void ground_init_tiles(
    ground_t* g,
    coord_hash_t* bodyprops_table,
    coord_hash_t* plane_table,
    ground_tile_mapper_cb map_cb,
    const void* map_context)
{
    if (!g || !map_cb) return;
    std::memset(g, 0, sizeof(*g));
    g->mode = GROUND_MODE_TILES;
    g->u.tiles.bodyprops_table = bodyprops_table;
    g->u.tiles.plane_table     = plane_table;
    g->u.tiles.map_cb          = map_cb;
    g->u.tiles.map_context     = map_context;
}

void ground_init_heightfield(
    ground_t* g,
    int w, int h, float cell,
    float* height,
    bool owns_buffer)
{
    if (!g || !height || w <= 0 || h <= 0 || cell <= 0.0f) return;
    std::memset(g, 0, sizeof(*g));
    g->mode = GROUND_MODE_HEIGHTFIELD;
    g->u.heightfield.w = w;
    g->u.heightfield.h = h;
    g->u.heightfield.cell = cell;
    g->u.heightfield.height = height;
    g->u.heightfield.owns_buffer = owns_buffer;
}

void ground_assign(ground_t* out, const ground_t* src){
    if (!out || !src) return;
    if (out == src) return;

    /* 1) release current resources of 'out' */
    ground_free(out);

    /* 2) deep-copy by mode */
    switch (src->mode) {
    case GROUND_MODE_UNIFORM: {
        out->mode = GROUND_MODE_UNIFORM;
        out->u.uniform = src->u.uniform; /* POD by value */
        break;
    }
    case GROUND_MODE_TILES: {
        out->mode = GROUND_MODE_TILES;

        /* mapper copies (function pointer + opaque context) */
        out->u.tiles.map_cb      = src->u.tiles.map_cb;
        out->u.tiles.map_context = src->u.tiles.map_context;

        /* deep-copy tables: requires coord_hash_copy to clone keys/values
           using the original hash's copy/destroy funcs. */
        out->u.tiles.bodyprops_table = NULL;
        out->u.tiles.plane_table     = NULL;

        if (src->u.tiles.bodyprops_table) {
            coord_hash_t* cp = coord_hash_copy(src->u.tiles.bodyprops_table);
            out->u.tiles.bodyprops_table = cp; /* NULL if copy failed */
        }
        if (src->u.tiles.plane_table) {
            coord_hash_t* cp = coord_hash_copy(src->u.tiles.plane_table);
            out->u.tiles.plane_table = cp;     /* NULL if copy failed */
        }
        break;
    }
    case GROUND_MODE_HEIGHTFIELD: {
        out->mode = GROUND_MODE_HEIGHTFIELD;
        out->u.heightfield.w    = src->u.heightfield.w;
        out->u.heightfield.h    = src->u.heightfield.h;
        out->u.heightfield.cell = src->u.heightfield.cell;

        size_t count = 0;
        float* dst = NULL;
        if (src->u.heightfield.height &&
            src->u.heightfield.w > 0 && src->u.heightfield.h > 0)
        {
            count = (size_t)src->u.heightfield.w * (size_t)src->u.heightfield.h;
            dst = (float*)std::malloc(sizeof(float) * count);
            if (dst) std::memcpy(dst, src->u.heightfield.height, sizeof(float) * count);
        }
        out->u.heightfield.height = dst;
        out->u.heightfield.owns_buffer = (dst != NULL); /* own only if alloc succeeded */
        break;
    }
    default:
        /* Fallback: value-copy; add modes here when extended */
        std::memcpy(out, src, sizeof(*out));
        /* Important: if new modes own heap resources, add deep-copy branches above */
        break;
    }
}

void ground_reset(ground_t* g) {
    if (!g) return;
    std::memset(g, 0, sizeof(*g));
}

void ground_free(ground_t* g) {
    if (!g) return;
    if (g->mode == GROUND_MODE_HEIGHTFIELD) {
        if (g->u.heightfield.owns_buffer && g->u.heightfield.height) {
            std::free(g->u.heightfield.height);
        }
    }
    ground_reset(g);
}

// -----------------------------------------------------------------------------
// Common fallback for materials: tiles override -> uniform
static bool ground_pick_bodyprops_tiles_or_uniform(
    const ground_t* g, const coord_t* c, bodyprops_t* out_body)
{
    if (!g || !out_body) return false;
    if (g->mode == GROUND_MODE_TILES && g->u.tiles.bodyprops_table && c) {
        if (tiles_get_bodyprops(&g->u.tiles, c, out_body)) return true;
    }
    // uniform default
    *out_body = g->u.uniform.body;
    return true;
}

// -----------------------------------------------------------------------------
// Queries
bool ground_sample_at(
    const ground_t* g,
    const vec3_t* pos_world,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body)
{
    if (!g || !pos_world) return false;

    switch (g->mode) {
    case GROUND_MODE_UNIFORM: {
        if (out_point) plane_sample_point(&g->u.uniform.plane, pos_world, out_point);
        if (out_normal) plane_estimate_normal(&g->u.uniform.plane, pos_world, out_normal);
        if (out_body) *out_body = g->u.uniform.body;
        return true;
    }
    case GROUND_MODE_HEIGHTFIELD: {
        float z = hf_sample_bilinear(&g->u.heightfield, pos_world->x, pos_world->y);
        if (out_point) { *out_point = *pos_world; out_point->z = z; }
        if (out_normal) hf_normal_at(&g->u.heightfield, pos_world->x, pos_world->y, out_normal);
        if (out_body) *out_body = g->u.uniform.body; // use uniform unless tiles override outside
        return true;
    }
    case GROUND_MODE_TILES: {
        coord_t c{};
        bool mapped = tiles_world_to_coord(&g->u.tiles, pos_world, &c);

        // Geometry: prefer tile plane if available, else uniform plane
        bool have_geo = false;
        if (mapped && g->u.tiles.plane_table) {
            plane_t pl{};
            if (tiles_get_plane(&g->u.tiles, &c, &pl)) {
                if (out_point) plane_sample_point(&pl, pos_world, out_point);
                if (out_normal) plane_estimate_normal(&pl, pos_world, out_normal);
                have_geo = true;
            }
        }
        if (!have_geo) {
            if (out_point) plane_sample_point(&g->u.uniform.plane, pos_world, out_point);
            if (out_normal) plane_estimate_normal(&g->u.uniform.plane, pos_world, out_normal);
        }

        if (out_body) {
            if (!ground_pick_bodyprops_tiles_or_uniform(g, mapped ? &c : nullptr, out_body)) {
                *out_body = g->u.uniform.body;
            }
        }
        return true;
    }
    default:
        return false;
    }
}

// Analytic raycast for uniform plane
// static bool ground_raycast_uniform(
//     const ground_t* g,
//     const vec3_t* origin,
//     const vec3_t* dir,
//     float max_dist,
//     vec3_t* out_point,
//     vec3_t* out_normal,
//     bodyprops_t* out_body,
//     float* out_t)
// {
//     vec3_t n;
//     plane_estimate_normal(&g->u.uniform.plane, origin, &n);

//     vec3_t p0;
//     plane_sample_point(&g->u.uniform.plane, origin, &p0);

//     vec3_t o_to_p0;
//     vec3_sub(&o_to_p0, &p0, origin);
//     float denom = vec3_dot(&n, dir);
//     if (nearly_zero(denom)) return false;

//     float t = vec3_dot(&n, &o_to_p0) / denom;
//     if (t < 0.0f || t > max_dist) return false;

//     if (out_t) *out_t = t;
//     if (out_point) vec3_madd(out_point, origin, dir, t);
//     if (out_normal) *out_normal = n;
//     if (out_body) *out_body = g->u.uniform.body;
//     return true;
// }

// Analytic raycast for uniform plane (exact)
static bool ground_raycast_uniform(
    const ground_t* g,
    const vec3_t* origin,
    const vec3_t* dir,
    float max_dist,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body,
    float* out_t)
{
    if (!g || !origin || !dir || max_dist <= 0.0f) return false;

    // 1) exact unit normal from plane
    vec3_t n = g->u.uniform.plane.normal_unit; // must be unit-length

    // 2) signed distance of origin to plane
    float sd = plane_signed_distance(&g->u.uniform.plane, origin);

    // 3) rate of approach to plane along ray
    float denom = vec3_dot(&n, dir);
    const float eps = 1e-6f;
    if (fabsf(denom) <= eps) {
        // 평행: 이미 평면 위(sd ~ 0)이면 t=0 히트로 취급할지 정책 결정
        if (fabsf(sd) <= eps) {
            if (out_t) *out_t = 0.0f;
            if (out_point) *out_point = *origin;
            if (out_normal) *out_normal = n;
            if (out_body) *out_body = g->u.uniform.body;
            return true;
        }
        return false;
    }

    // 4) solve for t
    float t = -sd / denom;
    if (t < 0.0f || t > max_dist) return false;

    // 5) outputs
    if (out_t) *out_t = t;
    if (out_point) vec3_madd(out_point, origin, dir, t);
    if (out_normal) *out_normal = n;         // exact normal
    if (out_body) *out_body = g->u.uniform.body;
    return true;
}

// Generic marching + bisection using height difference f(t) = ray_z - ground_z
static bool ground_raycast_marching(
    const ground_t* g,
    const vec3_t* origin,
    const vec3_t* dir,
    float max_dist,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body,
    float* out_t)
{
    float step = 0.25f;
    if (g->mode == GROUND_MODE_HEIGHTFIELD) {
        float base = g->u.heightfield.cell * 0.5f;
        step = (base > 0.05f) ? base : 0.05f;
    }

    float t_prev = 0.0f;
    vec3_t pos_prev = *origin;

    vec3_t surf_prev, n_prev;
    if (!ground_sample_at(g, &pos_prev, &surf_prev, &n_prev, nullptr)) return false;
    float f_prev = pos_prev.z - surf_prev.z;

    for (float t = step; t <= max_dist + 1e-6f; t += step) {
        vec3_t pos;
        vec3_madd(&pos, origin, dir, t);

        vec3_t surf, nrm;
        if (!ground_sample_at(g, &pos, &surf, &nrm, nullptr)) {
            t_prev = t;
            pos_prev = pos;
            f_prev = 0.0f;
            continue;
        }
        float f_curr = pos.z - surf.z;

        if (f_prev >= 0.0f && f_curr < 0.0f) {
            float a = t_prev, b = t;
            for (int i = 0; i < 16; ++i) {
                float m = 0.5f * (a + b);
                vec3_t pm; vec3_madd(&pm, origin, dir, m);
                vec3_t sm, nm;
                ground_sample_at(g, &pm, &sm, &nm, nullptr);
                float fm = pm.z - sm.z;
                if (fm > 0.0f) a = m; else b = m;
            }
            float thit = 0.5f * (a + b);
            vec3_t phit; vec3_madd(&phit, origin, dir, thit);

            vec3_t sh, nh; bodyprops_t bh;
            ground_sample_at(g, &phit, &sh, &nh, &bh);

            if (out_t) *out_t = thit;
            if (out_point) *out_point = sh;
            if (out_normal) *out_normal = nh;
            if (out_body) *out_body = bh;
            return true;
        }

        t_prev = t;
        pos_prev = pos;
        f_prev = f_curr;
    }
    return false;
}

bool ground_raycast(
    const ground_t* g,
    const vec3_t* origin,
    const vec3_t* dir,
    float max_dist,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body,
    float* out_t)
{
    if (!g || !origin || !dir || max_dist <= 0.0f) return false;
    vec3_t dir_unit = *dir;
    vec3_normalize(&dir_unit);

    if (g->mode == GROUND_MODE_UNIFORM) {
        return ground_raycast_uniform(g, origin, &dir_unit, max_dist,
                                      out_point, out_normal, out_body, out_t);
    }
    return ground_raycast_marching(g, origin, &dir_unit, max_dist,
                                   out_point, out_normal, out_body, out_t);
}

bool ground_material_at(
    const ground_t* g,
    const vec3_t* pos_world,
    bodyprops_t* out_body)
{
    if (!g || !pos_world || !out_body) return false;

    if (g->mode == GROUND_MODE_TILES) {
        coord_t c{};
        if (tiles_world_to_coord(&g->u.tiles, pos_world, &c)) {
            if (tiles_get_bodyprops(&g->u.tiles, &c, out_body)) return true;
        }
    }
    *out_body = g->u.uniform.body;
    return true;
}
