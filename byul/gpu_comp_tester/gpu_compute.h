#ifndef GPU_COMPUTE_H
#define GPU_COMPUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ----------------------------------------
// Compute Shader Loading and Execution
// ----------------------------------------

// Load compute shader only (no vs/fs)
unsigned int gpu_load_compute_shader(const char* cs_path);

// Dispatch compute (X, Y, Z thread groups)
void gpu_dispatch_compute(unsigned int program, int x, int y, int z);

// Memory barrier (handles GL_SHADER_STORAGE_BARRIER_BIT internally)
void gpu_memory_barrier(void);

// ----------------------------------------
// SSBO (Shader Storage Buffer Object) Management
// ----------------------------------------

// Create SSBO (binding index corresponds to layout(binding=...))
unsigned int gpu_create_ssbo(uint32_t size, uint32_t binding_index);

// Upload CPU data to SSBO
void gpu_update_ssbo(unsigned int ssbo, const void* data, uint32_t size);

// Map SSBO (for read access)
void* gpu_navgrid_ssbo(unsigned int ssbo);

// Unmap SSBO (must be called after mapping)
void gpu_unnavgrid_ssbo(unsigned int ssbo);

#ifdef __cplusplus
}
#endif

#endif // GPU_COMPUTE_H
