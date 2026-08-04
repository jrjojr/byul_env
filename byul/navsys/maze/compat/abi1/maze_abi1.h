/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_abi1.h
 * @brief Opt-in source compatibility declaration for the Maze ABI 1 layout.
 *
 * New source must include maze_core.h and use its opaque handle and checked
 * accessors. This header is installed only by the compatibility component.
 */

#ifndef BYUL_COMPAT_ABI1_MAZE_H
#define BYUL_COMPAT_ABI1_MAZE_H

#include <stdint.h>

#include "coord_hash.h"
#include "maze_core.h"

#define BYUL_MAZE_ABI1_VERSION UINT32_C(1)
#define BYUL_MAZE_ABI1_FINGERPRINT UINT64_C(0x4d415a4501000018)

/**
 * @deprecated Direct Maze fields are available only for ABI 1 migration.
 * New source uses the opaque handle declared by maze_core.h.
 */
struct s_maze {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
};

#endif /* BYUL_COMPAT_ABI1_MAZE_H */
