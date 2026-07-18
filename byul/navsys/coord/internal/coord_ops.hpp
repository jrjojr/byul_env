/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file coord_ops.hpp
 * @brief Internal C++ value functors for coord_t containers.
 */

#ifndef BYUL_NAVSYS_COORD_INTERNAL_COORD_OPS_HPP
#define BYUL_NAVSYS_COORD_INTERNAL_COORD_OPS_HPP

#include "coord.h"

#include <cstddef>

namespace byul::navsys::detail {

struct coord_equal final {
    [[nodiscard]] bool operator()(
        const coord_t& lhs, const coord_t& rhs) const noexcept {
        return ::coord_equal(&lhs, &rhs);
    }
};

struct coord_less final {
    [[nodiscard]] bool operator()(
        const coord_t& lhs, const coord_t& rhs) const noexcept {
        return ::coord_compare(&lhs, &rhs) < 0;
    }
};

struct coord_hash final {
    [[nodiscard]] std::size_t operator()(
        const coord_t& value) const noexcept {
        return static_cast<std::size_t>(::coord_hash(&value));
    }
};

} // namespace byul::navsys::detail

#endif /* BYUL_NAVSYS_COORD_INTERNAL_COORD_OPS_HPP */
