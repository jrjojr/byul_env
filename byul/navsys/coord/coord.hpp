#ifndef COORD_HPP
#define COORD_HPP

#include "coord.h"
#include <cstddef>     // for size_t
#include <functional>  // for std::hash
#include <ostream>     // for operator<<

// ------------------------ Comparison Operators ------------------------

/// @brief Equality comparison (uses coord_equal())
inline bool operator==(const coord_t& a, const coord_t& b) {
    return coord_equal(&a, &b);
}

/// @brief Inequality comparison
inline bool operator!=(const coord_t& a, const coord_t& b) {
    return !coord_equal(&a, &b);
}

/// @brief Comparison operator for sorting (x first, then y if equal)
inline bool operator<(const coord_t& a, const coord_t& b) {
    return coord_compare(&a, &b) < 0;
}

/// @brief Debug output operator
inline std::ostream& operator<<(std::ostream& os, const coord_t& c) {
    int x = coord_get_x(&c);
    int y = coord_get_y(&c);
    return os << "(" << x << ", " << y << ")";
}

struct CoordHash {
    size_t operator()(const coord_t* c) const {
        return coord_hash(c);
    }
};

struct CoordEqual {
    bool operator()(const coord_t* a, const coord_t* b) const {
        return coord_equal(a, b);
    }
};

// ------------------------ Hash operator specialization must be in std ------------------------

namespace std {
    template<>
    struct hash<coord_t> {
        std::size_t operator()(const coord_t& c) const {
            return static_cast<std::size_t>(coord_hash(&c));
        }
    };
}

#endif // COORD_HPP
