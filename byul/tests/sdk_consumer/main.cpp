#include <cassert>
#include <cstddef>
#include <cstring>
#include <type_traits>

#include "coord.h"

int main() {
    static_assert(std::is_standard_layout_v<coord_t>);
    assert(sizeof(coord_t) == coord_sizeof());
    assert(alignof(coord_t) == coord_alignof());
    assert(offsetof(coord_t, x) == coord_offsetof_x());
    assert(offsetof(coord_t, y) == coord_offsetof_y());

    coord_t value{};
    assert(coord_init_checked(&value, -3, 7) == NAVSYS_STATUS_OK);

    std::size_t required = 0;
    assert(coord_format(&value, nullptr, 0, &required) == NAVSYS_STATUS_OK);
    assert(required == sizeof("(-3, 7)"));

    char buffer[sizeof("(-3, 7)")]{};
    assert(
        coord_format(&value, buffer, sizeof(buffer), &required)
        == NAVSYS_STATUS_OK);
    assert(std::strcmp(buffer, "(-3, 7)") == 0);
    return 0;
}
