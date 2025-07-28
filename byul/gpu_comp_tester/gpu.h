#ifndef GPU_H
#define GPU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "quat.h"

// ----------------------------------------
// 🖼️ GPU 초기화 / 종료
// ----------------------------------------

int  gpu_init(int width, int height, const char* title);
void gpu_terminate(void);
void* gpu_get_window(void); // 내부적으로는 SDL_Window*

// ----------------------------------------
// 🔧 셰이더 로딩 / GLSL 관리
// ----------------------------------------

unsigned int gpu_load_shader(const char* vs_path, const char* fs_path);
unsigned int gpu_create_ubo(uint32_t size, uint32_t binding_index);
void         gpu_update_ubo(unsigned int ubo, const void* data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif // GPU_H
