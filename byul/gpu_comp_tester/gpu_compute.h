#ifndef GPU_COMPUTE_H
#define GPU_COMPUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ----------------------------------------
// ⚙️ Compute Shader 로딩 및 실행
// ----------------------------------------

// 컴퓨트 셰이더만 로딩 (vs/fs 없음)
unsigned int gpu_load_compute_shader(const char* cs_path);

// 컴퓨트 디스패치 (X, Y, Z 스레드 그룹)
void gpu_dispatch_compute(unsigned int program, int x, int y, int z);

// 메모리 배리어 (GL_SHADER_STORAGE_BARRIER_BIT 등 내부 처리)
void gpu_memory_barrier(void);

// ----------------------------------------
// 📦 SSBO (Shader Storage Buffer Object) 관리
// ----------------------------------------

// SSBO 생성 (binding index는 layout(binding=...)과 대응됨)
unsigned int gpu_create_ssbo(uint32_t size, uint32_t binding_index);

// SSBO에 CPU 데이터 업로드
void gpu_update_ssbo(unsigned int ssbo, const void* data, uint32_t size);

// SSBO 매핑 (읽기용)
void* gpu_navgrid_ssbo(unsigned int ssbo);

// SSBO 언매핑 (매핑 해제 시 반드시 호출)
void  gpu_unnavgrid_ssbo(unsigned int ssbo);

#ifdef __cplusplus
}
#endif

#endif // GPU_COMPUTE_H
