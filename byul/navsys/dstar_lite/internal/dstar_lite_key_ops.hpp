/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file dstar_lite_key_ops.hpp
 * @brief Canonical D* Lite key를 위한 구현 전용 C++ ordering functor를 제공한다.
 *
 * 유효한 dstar_lite_key_t 값을 exact lexicographic relation으로 비교하는 private
 * container helper를 제공한다.
 */

#ifndef BYUL_NAVSYS_DSTAR_LITE_INTERNAL_DSTAR_LITE_KEY_OPS_HPP
#define BYUL_NAVSYS_DSTAR_LITE_INTERNAL_DSTAR_LITE_KEY_OPS_HPP

#include "../dstar_lite_key.h"

namespace byul::navsys::dstar_lite_detail {

struct key_less final {
    [[nodiscard]] bool operator()(
        const dstar_lite_key_t& lhs,
        const dstar_lite_key_t& rhs) const noexcept {
        int comparison = 0;
        return ::dstar_lite_key_compare_exact(
                &lhs, &rhs, &comparison) == NAVSYS_STATUS_OK
            && comparison < 0;
    }
};

} // namespace byul::navsys::dstar_lite_detail

#endif /* BYUL_NAVSYS_DSTAR_LITE_INTERNAL_DSTAR_LITE_KEY_OPS_HPP */
