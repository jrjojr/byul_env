#include "console.h"
#include "navgrid.h"
#include "float_common.h"
#include "coord_hash.h"
#include "coord_list.h"
#include "route.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <climits>

static char get_navgrid_char(
    const navgrid_t* navgrid, int x, int y,
    const coord_t* start, const coord_t* goal,
    const coord_hash_t* route_coords,
    const coord_hash_t* visited_count
) {
    if (start && coord_get_x(start) == x && coord_get_y(start) == y) 
        return 'S';

    if (goal && coord_get_x(goal) == x && coord_get_y(goal) == y) 
        return 'G';

    if (is_coord_blocked_navgrid(navgrid, x, y, nullptr)) return '#';

    coord_t tmp = {x, y};

    bool is_route 
    = route_coords && coord_hash_contains(route_coords, &tmp);

    bool is_visited 
    = visited_count && coord_hash_contains(visited_count, &tmp);

    if (is_route) return '*';
    if (is_visited) return '+';
    return '.';
}

static const char* get_navgrid_string(
    const navgrid_t* navgrid, int x, int y,
    const coord_t* start, const coord_t* goal,
    const coord_hash_t* route_coords,
    const coord_hash_t* visited_count
) {
    static char buf[16];

    if (start && coord_get_x(start) == x && coord_get_y(start) == y)
        return "  S";
    if (goal && coord_get_x(goal) == x && coord_get_y(goal) == y)
        return "  G";

    if (is_coord_blocked_navgrid(navgrid, x, y, nullptr)) return "  #";

    coord_t tmp = {x, y};

    bool is_route = route_coords && coord_hash_contains(route_coords, &tmp);
    void* val = visited_count ? coord_hash_get(visited_count, &tmp) : NULL;

    if (is_route) return "  *";    

    if (val) {
        int count = *static_cast<int*>(val);
        if (count > 999) count = 999;
        snprintf(buf, sizeof(buf), "%3d", count);
        return buf;
    }

    return "  .";
}

void navgrid_print_ascii(const navgrid_t* navgrid) {
    if (!navgrid) return;

    const coord_hash_t* coords = navgrid_get_cell_map(navgrid);
    int min_x = INT_MAX, min_y = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN;

    int count = coord_hash_length(coords);
    if (count > 0) {
        coord_hash_iter_t* iter 
        = coord_hash_iter_create((coord_hash_t*)coords);

        coord_t key;
        while (coord_hash_iter_next(iter, &key, NULL)) {
            if (key.x < min_x) min_x = key.x;
            if (key.y < min_y) min_y = key.y;
            if (key.x > max_x) max_x = key.x;
            if (key.y > max_y) max_y = key.y;
        }
        coord_hash_iter_destroy(iter);

        if (max_x - min_x + 1 > 100) max_x = min_x + 99;
        if (max_y - min_y + 1 > 100) max_y = min_y + 99;
    } else {
        min_x = 0;
        min_y = 0;
        max_x = 9;
        max_y = 9;
        printf("[AUTO SIZE OVERRIDE: width=0->10, height=0->10]\n");
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    printf("[MAP %dx%d ASCII] (origin = %d,%d)\n", 
        width, height, min_x, min_y);

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            printf("%s", get_navgrid_string(
                navgrid, x, y, NULL, NULL, NULL, NULL));
        }
        putchar('\n');
    }
}

void navgrid_print_ascii_with_route(
    const navgrid_t* navgrid, const route_t* p, int margin) {
    if (!navgrid || !p) return;

    const coord_list_t* list = route_get_coords(p);
    if (!list || coord_list_length(list) == 0) return;

    const coord_t* start = coord_list_get(list, 0);
    const coord_t* goal = coord_list_get(list, coord_list_length(list) - 1);

    coord_hash_t* route_coords = coord_hash_create();

    int min_x = coord_get_x(start), max_x = coord_get_x(start);
    int min_y = coord_get_y(start), max_y = coord_get_y(start);

    for (int i = 0; i < coord_list_length(list); ++i) {
        const coord_t* c = coord_list_get(list, i);
        coord_hash_replace(route_coords, c, NULL);

        int x = coord_get_x(c);
        int y = coord_get_y(c);
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }

    // const int margin = 2;
    if (navgrid->width == 0) {
        min_x -= margin; max_x += margin;
    } else {
        min_x = 0; max_x = navgrid->width - 1;
    }

    if (navgrid->height == 0) {
        min_y -= margin; max_y += margin;
    } else {
        min_y = 0; max_y = navgrid->height - 1;
    }

    printf("MAP %d,%d to %d,%d with Route - total_retry: %d\n", 
        min_x, min_y, max_x, max_y, p->total_retry_count);

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            printf("%s", get_navgrid_string(navgrid, x, y, 
                start, goal, route_coords, NULL));
        }
        putchar('\n');
    }

    coord_hash_destroy(route_coords);
}

void navgrid_print_ascii_with_visited_count(
    const navgrid_t* navgrid, const route_t* p, int margin) {

    if (!navgrid || !p || !route_get_visited_count(p)) return;

    const coord_hash_t* visited = route_get_visited_count(p);
    const coord_list_t* list = route_get_coords(p);
    coord_hash_t* route_coords = coord_hash_create();

    if (!list || coord_list_length(list) == 0) {
        coord_hash_destroy(route_coords);
        return;
    }

    const coord_t* start = coord_list_get(list, 0);
    const coord_t* goal = coord_list_get(list, coord_list_length(list) - 1);

    int min_x = coord_get_x(start), max_x = coord_get_x(start);
    int min_y = coord_get_y(start), max_y = coord_get_y(start);

    int len = coord_list_length(list);
    for (int i = 0; i < len; ++i) {
        const coord_t* c = coord_list_get(list, i);
        coord_hash_replace(route_coords, c, NULL);

        int x = coord_get_x(c);
        int y = coord_get_y(c);
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }

    if (navgrid->width == 0) {
        min_x -= margin; max_x += margin;
    } else {
        min_x = 0; max_x = navgrid->width - 1;
    }

    if (navgrid->height == 0) {
        min_y -= margin; max_y += margin;
    } else {
        min_y = 0; max_y = navgrid->height - 1;
    }

    printf("MAP %d,%d to %d,%d with Route and Visit Counts - total_retry: %d\n", 
        min_x, min_y, max_x, max_y, p->total_retry_count);

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            printf("%s", get_navgrid_string(navgrid, x, y, 
                start, goal, route_coords, visited));
        }
        putchar('\n');
    }

    coord_hash_destroy(route_coords);
}
