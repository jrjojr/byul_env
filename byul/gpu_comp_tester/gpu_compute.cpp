#include "internal/gpu_compute.h"
#include <glad/gl.h>
#include <stdio.h>
#include <stdlib.h>

static char* read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    rewind(f);
    char* buffer = (char*)malloc(len + 1);
    fread(buffer, 1, len, f);
    buffer[len] = '\0';
    fclose(f);
    return buffer;
}

unsigned int gpu_load_compute_shader(const char* cs_path) {
    char* source = read_file(cs_path);
    if (!source) {
        fprintf(stderr, "[GPU] Failed to read compute shader: %s\n", cs_path);
        return 0;
    }

    GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(shader, 1, (const char**)&source, NULL);
    glCompileShader(shader);
    free(source);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        fprintf(stderr, "[GPU] Compute shader compile error:\n%s\n", log);
        glDeleteShader(shader);
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, shader);
    glLinkProgram(program);
    glDeleteShader(shader);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), NULL, log);
        fprintf(stderr, "[GPU] Program link error:\n%s\n", log);
        glDeleteProgram(program);
        return 0;
    }

    return program;
}

void gpu_dispatch_compute(unsigned int program, int x, int y, int z) {
    glUseProgram(program);
    glDispatchCompute(x, y, z);
}

void gpu_memory_barrier(void) {
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

unsigned int gpu_create_ssbo(uint32_t size, uint32_t binding_index) {
    GLuint ssbo = 0;
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, NULL, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding_index, ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return ssbo;
}

void gpu_update_ssbo(unsigned int ssbo, const void* data, uint32_t size) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, size, data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void* gpu_navgrid_ssbo(unsigned int ssbo) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    return glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
}

void gpu_unnavgrid_ssbo(unsigned int ssbo) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
