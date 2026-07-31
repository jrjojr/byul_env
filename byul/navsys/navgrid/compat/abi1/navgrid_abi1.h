/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file navgrid_abi1.h
 * @brief Navgrid ABI 1 공개 layout을 사용하는 기존 source용 호환 선언이다.
 *
 * 새 source는 이 header를 include하지 않고 opaque navgrid_t API만 사용한다. 이 선언은
 * ABI 1 layout에 직접 접근하던 기존 source를 별도 compatibility package에서 다시
 * 빌드하거나 기존 binary layout을 검증할 때만 제공한다.
 */

#ifndef BYUL_COMPAT_ABI1_NAVGRID_H
#define BYUL_COMPAT_ABI1_NAVGRID_H

#include <stdint.h>

#include "navgrid.h"

#define BYUL_NAVGRID_ABI1_VERSION UINT32_C(1)
#define BYUL_NAVGRID_ABI1_FINGERPRINT UINT64_C(0x4e47524944010028)

/**
 * @deprecated ABI 1 direct field access는 compatibility component에서만 제공된다.
 * 새 source는 navgrid.h의 opaque handle과 accessor/export API를 사용한다.
 */
struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;
    coord_hash_t* cell_map;
    is_coord_blocked_func is_coord_blocked_fn;
    void* is_coord_blocked_fn_userdata;
};

#endif /* BYUL_COMPAT_ABI1_NAVGRID_H */
