#ifndef COORD_HPP
#define COORD_HPP

#include "internal/coord.h"
#include <cstddef>     // for size_t
#include <functional>  // for std::hash
#include <ostream>     // for operator<<

// ------------------------ 비교 연산자 ------------------------

/// @brief 값 동등 비교 (coord_equal() 사용)
inline bool operator==(const coord_t& a, const coord_t& b) {
    return coord_equal(&a, &b);
}

/// @brief 값 비동등 비교
inline bool operator!=(const coord_t& a, const coord_t& b) {
    return !coord_equal(&a, &b);
}

/// @brief 정렬용 비교 연산자 (x 우선, 같으면 y)
inline bool operator<(const coord_t& a, const coord_t& b) {
    return coord_compare(&a, &b) < 0;
}

/// @brief 디버깅 출력용
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

// ------------------------ 해시 연산자는 std에 있어야 함 ------------------------

namespace std {
    template<>
    struct hash<coord_t> {
        std::size_t operator()(const coord_t& c) const {
            return static_cast<std::size_t>(coord_hash(&c));
        }
    };
}

#endif // COORD_HPP
