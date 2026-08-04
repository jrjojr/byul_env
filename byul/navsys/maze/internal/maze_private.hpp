/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file maze_private.hpp
 * @brief Defines the private Maze representation used by native sources.
 */

#ifndef BYUL_MAZE_INTERNAL_MAZE_PRIVATE_HPP
#define BYUL_MAZE_INTERNAL_MAZE_PRIVATE_HPP

#include <cstdint>

#include "../maze.h"
#include "../maze_binary.h"
#include "../../coord/coord_hash.h"

struct s_maze {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
};

static_assert(
    sizeof(s_maze) == (sizeof(void*) == 8 ? 24 : 20),
    "Maze ABI 1 binary layout changed");
static_assert(
    alignof(s_maze) == alignof(void*),
    "Maze ABI 1 alignment changed");

class byul_maze_generation_context final {
public:
    byul_maze_generation_context(
        uint64_t seed,
        uint64_t max_steps,
        byul_maze_generate_cancel_func cancel_func,
        void* cancel_userdata) noexcept;

    uint32_t next_u32() noexcept;
    uint32_t bounded(uint32_t bound) noexcept;

    navsys_status_t poll() const noexcept;
    navsys_status_t begin_step() noexcept;
    uint64_t steps() const noexcept;

private:
    uint64_t state_;
    uint64_t increment_;
    uint64_t steps_;
    uint64_t max_steps_;
    byul_maze_generate_cancel_func cancel_func_;
    void* cancel_userdata_;
};

uint64_t byul_maze_generation_legacy_seed() noexcept;

navsys_status_t byul_maze_generate_recursive_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_prim_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_binary_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_binary_with_bias_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_binary_bias_t bias,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_eller_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_hunt_and_kill_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_sidewinder_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_recursive_division_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_room_blend_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_aldous_broder_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_wilson_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

navsys_status_t byul_maze_generate_kruskal_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept;

#endif /* BYUL_MAZE_INTERNAL_MAZE_PRIVATE_HPP */
