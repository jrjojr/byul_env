#include "gpu.h"
#include "quat.hpp"
#include "vec3.hpp"
#ifdef USE_SDL3
    #include <SDL3/SDL.h>
#else
    #include <SDL.h>
#endif
#include <glad/gl.h>
#include <chrono>
#include <cmath>
#include <cstdio>

#if defined(_WIN32)
    #include <windows.h>
#endif

#ifndef SHADER_DIR
#define SHADER_DIR "../glsl"
#endif

#include "float_common.h"
#include <filesystem>

static GLuint vao = 0, vbo = 0, vbo_type = 0;
static const int HAND_COUNT = 2;
static float vertices[HAND_COUNT * 2 * 3];  // 2점 × 3좌표 × 2라인
static int types[HAND_COUNT * 2];          // 각 정점의 타입 (0: SLERP, 1: LERP)

static void update_vertices(const vec3_t& slerp, const vec3_t& lerp) {
    // SLERP (빨강)
    vertices[0] = 0.0f; vertices[1] = 0.0f; vertices[2] = 0.0f;
    vertices[3] = slerp.x; vertices[4] = slerp.y; vertices[5] = slerp.z;

    // LERP (파랑)
    vertices[6] = 0.0f; vertices[7] = 0.0f; vertices[8] = 0.0f;
    vertices[9] = lerp.x; vertices[10] = lerp.y; vertices[11] = lerp.z;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
}

static void init_vao() {
    for (int i = 0; i < HAND_COUNT * 2; ++i)
        types[i] = (i < 2) ? 0 : 1;  // 앞 2개: SLERP, 뒤 2개: LERP

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &vbo_type);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
    glBufferData(GL_ARRAY_BUFFER, sizeof(types), types, GL_STATIC_DRAW);
    glVertexAttribIPointer(1, 1, GL_INT, sizeof(int), nullptr);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

static std::filesystem::path get_executable_dir() {
#if defined(_WIN32)
    char path[MAX_PATH];
    GetModuleFileNameA(NULL, path, MAX_PATH);
    return std::filesystem::path(path).parent_path();
#else
    std::filesystem::path exe_symlink = "/proc/self/exe";
    return std::filesystem::read_symlink(exe_symlink).parent_path();
#endif
}

static std::string shader_path(const char* name) {
    return (get_executable_dir() / SHADER_DIR / name).string();
}

int main(int argc, char** argv) {
    if (gpu_init(800, 600, "[BYUL] LERP vs SLERP") != 0)
        return -1;

    GLuint shader = gpu_load_shader(
        shader_path("shader.vert").c_str(),
        shader_path("shader.frag").c_str());

    if (!shader) return -1;

    glUseProgram(shader);
    init_vao();

    SDL_Window* window = (SDL_Window*)gpu_get_window();
    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now(), last_time = start_time;

    int running = 1, frames = 0;
    vec3_t origin = {0.0f, 0.5f, 0.0f};

        while (running) {
#ifdef USE_SDL3
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) {
            running = 0;
        } else if (event.type == SDL_EVENT_KEY_DOWN) {
            // SDL3는 keysym 구조체 없이 바로 keycode
            if (event.key.key == SDLK_ESCAPE) {
                running = 0;
            }
        }
    }
            #else
                    SDL_Event event;
                    while (SDL_PollEvent(&event)) {
                        if (event.type == SDL_QUIT ||
                            (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                            running = 0;
                        }
                    }
            #endif


        auto now = clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        float base_sec = floorf(fmodf(elapsed, 60.0f));
        float t = elapsed - base_sec;

        float angle_a = -base_sec * (float)(M_PI / 30.0f);
        float angle_b = -(base_sec + 1.0f) * (float)(M_PI / 30.0f);

        vec3_t axis = {0.0f, 0.0f, 1.0f};
        quat_t ra;
        quat_init_axis_angle(&ra, &axis, angle_a);

        quat_t rb;
        quat_init_axis_angle(&rb, &axis, angle_b);

        quat_t rs;
        quat_init(&rs);
        quat_slerp(&rs, &ra, &rb, t);

        vec3_t rotated_slerp, rotated_lerp, target;
        quat_rotate_vector(&rs, &origin, &rotated_slerp);
        quat_rotate_vector(&rb, &origin, &target);

        // LERP 계산 (벡터 직접 보간)
        rotated_lerp.x = origin.x * (1 - t) + target.x * t;
        rotated_lerp.y = origin.y * (1 - t) + target.y * t;
        rotated_lerp.z = origin.z * (1 - t) + target.z * t;

        update_vertices(rotated_slerp, rotated_lerp);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shader);

        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, 4);
        glBindVertexArray(0);

        SDL_GL_SwapWindow(window);

        // FPS 측정
        frames++;
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count() >= 1) {
            printf("FPS: %d\n", frames);
            frames = 0;
            last_time = now;
        }
    }

    gpu_terminate();
    return 0;
}
