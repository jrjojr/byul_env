#include "doctest.h"
#include <cmath>

#include "coord.h"
#include "scalar.h"

TEST_CASE("Wrap-Around Test") {
    coord_t a;
    coord_init_full(&a, COORD_MAX, 0);

    coord_t b;
    coord_init_full(&b, 1, 0);

    coord_t result;
    coord_add(&result, &a, &b);

    CHECK(result.x == COORD_MIN);
    CHECK(result.y == 0);
}

TEST_CASE("Distance and Angle Test") {
    coord_t origin;
    coord_init(&origin);

    coord_t east;
    coord_init_full(&east, 1, 0);

    coord_t north;
    coord_init_full(&north, 0, 1);

    coord_t west;
    coord_init_full(&west, -1, 0);

    coord_t south;
    coord_init_full(&south, 0, -1);

     CHECK(doctest::Approx(coord_distance(&origin, &east)).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(coord_distance(&origin, &north)).epsilon(1e-6) == 1.0f);

     CHECK(doctest::Approx(coord_degree(&origin, &east)).epsilon(1e-6) == 0.0);
    CHECK(doctest::Approx(coord_degree(&origin, &north)).epsilon(1e-6) == 90.0);
    CHECK(doctest::Approx(coord_degree(&origin, &west)).epsilon(1e-6) == 180.0);
    CHECK(doctest::Approx(coord_degree(&origin, &south)).epsilon(1e-6) == 270.0);
}

TEST_CASE("Add/Sub Test") {
    coord_t a;
    coord_init_full(&a, 5, 10);

    coord_t b;
    coord_init_full(&b, 3, -4);

    coord_t result;

    coord_add(&result, &a, &b);
    CHECK(result.x == 8);
    CHECK(result.y == 6);

    coord_sub(&result, &a, &b);
    CHECK(result.x == 2);
    CHECK(result.y == 14);
}

TEST_CASE("Next To Goal Test") {
    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal;
    coord_init_full(&goal, 3, 4);

    coord_t next;
    coord_next_to_goal(&next, &start, &goal);

    CHECK(next.x == 1);
    CHECK(next.y == 1);
}


TEST_CASE("Angle in Radian Test") {
    coord_t origin;
    coord_init(&origin);

    coord_t east, north, west, south;
    coord_init_full(&east, 1, 0);
    coord_init_full(&north, 0, 1);
    coord_init_full(&west, -1, 0);
    coord_init_full(&south, 0, -1);

     CHECK(doctest::Approx(coord_angle(&origin, &east)).epsilon(SCALAR_EPSILON) == 0.0);
    CHECK(doctest::Approx(coord_angle(&origin, &north)).epsilon(SCALAR_EPSILON) == M_PI / 2.0);
    CHECK(doctest::Approx(coord_angle(&origin, &west)).epsilon(SCALAR_EPSILON) == M_PI);
    CHECK(doctest::Approx(coord_angle(&origin, &south)).epsilon(SCALAR_EPSILON) == 3.0 * M_PI / 2.0);
}