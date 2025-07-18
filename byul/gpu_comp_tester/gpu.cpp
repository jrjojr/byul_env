#include "internal/gpu.h"
#ifdef USE_SDL3
    #include <SDL3/SDL.h>
#else
    #include <SDL.h>
#endif

#include <glad/gl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

// ----------------------------------------
// ✅ 내부 상태
// ----------------------------------------

static SDL_Window* window = nullptr;
static SDL_GLContext gl_context = nullptr;

// ----------------------------------------
// 🖼️ GPU 초기화 / 종료
// ----------------------------------------

// int gpu_init(int width, int height, const char* title) {
//     if (SDL_Init(SDL_INIT_VIDEO) != 0) {
//         fprintf(stderr, "[GPU] SDL_Init failed: %s\n", SDL_GetError());
//         return -1;
//     }

//     SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
//     SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
//     SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

//     window = SDL_CreateWindow(title,
//                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
//                               width, height, SDL_WINDOW_OPENGL);
//     if (!window) {
//         fprintf(stderr, "[GPU] SDL_CreateWindow failed\n");
//         return -1;
//     }

//     gl_context = SDL_GL_CreateContext(window);
//     if (!gl_context) {
//         fprintf(stderr, "[GPU] SDL_GL_CreateContext failed\n");
//         return -1;
//     }

//     if (!gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress)) {
//         fprintf(stderr, "[GPU] gladLoadGL failed\n");
//         return -1;
//     }

//     return 0;
// }

int gpu_init(int width, int height, const char* title) {
#ifdef USE_SDL3
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "[GPU] SDL3_Init failed: %s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window* win = SDL_CreateWindow(
        title,
        width,
        height,
        SDL_WINDOW_OPENGL
    );
    if (!win) {
        fprintf(stderr, "[GPU] SDL3_CreateWindow failed: %s\n", SDL_GetError());
        return -1;
    }
    window = win;

#else // SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "[GPU] SDL2_Init failed: %s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    window = SDL_CreateWindow(title,
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              width, height, SDL_WINDOW_OPENGL);
    if (!window) {
        fprintf(stderr, "[GPU] SDL2_CreateWindow failed\n");
        return -1;
    }
#endif

    gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) {
        fprintf(stderr, "[GPU] SDL_GL_CreateContext failed\n");
        return -1;
    }

    if (!gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress)) {
        fprintf(stderr, "[GPU] gladLoadGL failed\n");
        return -1;
    }

    return 0;
}


// void gpu_terminate(void) {
//     if (gl_context) SDL_GL_DeleteContext(gl_context);
//     if (window) SDL_DestroyWindow(window);
//     SDL_Quit();
// }

void gpu_terminate(void) {
#ifdef USE_SDL3
    if (window) {
        SDL_DestroyWindow(window);  // OpenGL context 포함 파괴됨
        window = NULL;
    }
    SDL_Quit();
#else
    if (gl_context) {
        SDL_GL_DeleteContext(gl_context);
        gl_context = NULL;
    }
    if (window) {
        SDL_DestroyWindow(window);
        window = NULL;
    }
    SDL_Quit();
#endif
}

void* gpu_get_window(void) {
    return (void*)window;
}

// ----------------------------------------
// 🔧 셰이더 로딩 유틸
// ----------------------------------------

static char* read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    char* buffer = (char*)malloc(size + 1);
    fread(buffer, 1, size, f);
    buffer[size] = 0;
    fclose(f);
    return buffer;
}

static void check_compile_error(GLuint shader, const char* label) {
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info[512];
        glGetShaderInfoLog(shader, 512, NULL, info);
        fprintf(stderr, "[GPU] Shader compile error (%s): %s\n", label, info);
    }
}

static void check_link_error(GLuint program) {
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info[512];
        glGetProgramInfoLog(program, 512, NULL, info);
        fprintf(stderr, "[GPU] Program link error: %s\n", info);
    }
}

unsigned int gpu_load_shader(const char* vs_path, const char* fs_path) {
    const char* vs_src = read_file(vs_path);
    const char* fs_src = read_file(fs_path);
    if (!vs_src || !fs_src) {
        fprintf(stderr, "[GPU] Failed to read shader files\n");
        return 0;
    }

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vs_src, NULL);
    glCompileShader(vs);
    check_compile_error(vs, "vertex");

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fs_src, NULL);
    glCompileShader(fs);
    check_compile_error(fs, "fragment");

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    check_link_error(program);

    glDeleteShader(vs);
    glDeleteShader(fs);
    free((void*)vs_src);
    free((void*)fs_src);

    return program;
}

unsigned int gpu_create_ubo(uint32_t size, uint32_t binding_index) {
    GLuint ubo;
    glGenBuffers(1, &ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, ubo);
    glBufferData(GL_UNIFORM_BUFFER, size, NULL, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, binding_index, ubo);
    return ubo;
}

void gpu_update_ubo(unsigned int ubo, const void* data, uint32_t size) {
    glBindBuffer(GL_UNIFORM_BUFFER, ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, size, data);
}
