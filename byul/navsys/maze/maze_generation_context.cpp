/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#include "internal/maze_private.hpp"

#include <atomic>
#include <chrono>
#include <random>

namespace {

constexpr uint64_t pcg32_multiplier = UINT64_C(6364136223846793005);
constexpr uint64_t maze_stream_sequence = UINT64_C(0x4d415a455f5631);

} // namespace

byul_maze_generation_context::byul_maze_generation_context(
    uint64_t seed,
    uint64_t max_steps,
    byul_maze_generate_cancel_func cancel_func,
    void* cancel_userdata) noexcept
    : state_(0),
      increment_((maze_stream_sequence << 1u) | UINT64_C(1)),
      steps_(0),
      max_steps_(max_steps),
      cancel_func_(cancel_func),
      cancel_userdata_(cancel_userdata) {
    static_cast<void>(next_u32());
    state_ += seed;
    static_cast<void>(next_u32());
}

uint32_t byul_maze_generation_context::next_u32() noexcept {
    const uint64_t oldstate = state_;
    state_ = oldstate * pcg32_multiplier + increment_;
    const uint32_t xorshifted = static_cast<uint32_t>(
        ((oldstate >> 18u) ^ oldstate) >> 27u);
    const uint32_t rotation = static_cast<uint32_t>(oldstate >> 59u);
    return (xorshifted >> rotation)
        | (xorshifted << ((0u - rotation) & 31u));
}

uint32_t byul_maze_generation_context::bounded(uint32_t bound) noexcept {
    if (bound == 0) return 0;
    const uint32_t threshold = (0u - bound) % bound;
    for (;;) {
        const uint32_t value = next_u32();
        if (value >= threshold) return value % bound;
    }
}

navsys_status_t byul_maze_generation_context::poll() const noexcept {
    if (!cancel_func_) return NAVSYS_STATUS_OK;
    try {
        return cancel_func_(cancel_userdata_)
            ? NAVSYS_STATUS_CANCELLED
            : NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

navsys_status_t byul_maze_generation_context::begin_step() noexcept {
    const navsys_status_t poll_status = poll();
    if (poll_status != NAVSYS_STATUS_OK) return poll_status;
    if (max_steps_ != 0 && steps_ >= max_steps_) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    ++steps_;
    return NAVSYS_STATUS_OK;
}

uint64_t byul_maze_generation_context::steps() const noexcept {
    return steps_;
}

uint64_t byul_maze_generation_legacy_seed() noexcept {
    static std::atomic<uint64_t> sequence{UINT64_C(0)};
    uint64_t seed = static_cast<uint64_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
    seed ^= ++sequence * UINT64_C(0x9e3779b97f4a7c15);
    try {
        std::random_device source;
        seed ^= static_cast<uint64_t>(source()) << 32u;
        seed ^= static_cast<uint64_t>(source());
    } catch (...) {
        // Time and the monotonic process-local sequence remain nondeterministic
        // enough for the explicitly non-replayable legacy adapter.
    }
    return seed;
}
