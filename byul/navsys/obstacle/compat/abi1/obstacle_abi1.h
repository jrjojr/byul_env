/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file obstacle_abi1.h
 * @brief Obstacle ABI 1 공개 layout을 사용하는 기존 source용 호환 선언이다.
 *
 * 새 source는 이 header를 include하지 않고 obstacle_core.h의 opaque handle과
 * accessor/export API를 사용한다. 이 선언은 별도 compatibility component에서만
 * 설치되며 기본 SDK에는 포함되지 않는다.
 */

#ifndef BYUL_COMPAT_ABI1_OBSTACLE_H
#define BYUL_COMPAT_ABI1_OBSTACLE_H

#include <stdint.h>

#include "coord_hash.h"
#include "obstacle_core.h"

#define BYUL_OBSTACLE_ABI1_VERSION UINT32_C(1)
#define BYUL_OBSTACLE_ABI1_FINGERPRINT UINT64_C(0x4f42535401000018)

/**
 * @deprecated ABI 1 direct field access는 compatibility component에서만 제공된다.
 * 새 source는 obstacle_core.h의 opaque handle과 accessor/export API를 사용한다.
 */
struct s_obstacle {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
};

#endif /* BYUL_COMPAT_ABI1_OBSTACLE_H */
