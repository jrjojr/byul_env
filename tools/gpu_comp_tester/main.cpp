#include "gpu.h"
#include "gpu_compute.h"
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
#include <array>
#include <cstdio>
#include <cstring>

#if defined(_WIN32)
    #include <windows.h>
#endif

#ifndef SHADER_DIR
#define SHADER_DIR "../glsl"
#endif

#include "scalar.h"
#include <filesystem>

static GLuint vao = 0, vbo = 0, vbo_type = 0;
static const int HAND_COUNT = 2;
static float vertices[HAND_COUNT * 2 * 3];  // 2 points * 3 coordinates * 2 lines
static int types[HAND_COUNT * 2];           // Vertex type (0: GPU SLERP, 1: CPU LERP)

struct alignas(16) ComputeInput {
    float a[4];
    float b[4];
    float params[4];
};

struct alignas(16) ComputeResult {
    float rotated[4];
};

struct ComputeDemo {
    const char* file;
    const char* name;
    const char* description;
    GLuint program;
};

static std::array<ComputeDemo, 4> compute_demos = {{
    {"clock.comp", "Quaternion SLERP", "smooth quaternion interpolation", 0},
    {"nlerp.comp", "Quaternion NLERP", "normalized linear quaternion interpolation", 0},
    {"pulse.comp", "Pulse", "pulsating radial motion", 0},
    {"lissajous.comp", "Lissajous", "independent figure-eight motion", 0}
}};

static void select_demo(SDL_Window* window, int& selected, int next) {
    const int count = static_cast<int>(compute_demos.size());
    selected = (next % count + count) % count;
    const ComputeDemo& demo = compute_demos[selected];
    char title[160];
    snprintf(title, sizeof(title), "[BYUL] GPU comp %d/%d - %s",
             selected + 1, count, demo.name);
    SDL_SetWindowTitle(window, title);
    printf("[comp %d] %s (%s): %s\n",
           selected + 1, demo.name, demo.file, demo.description);
}

static void pack_quat(float out[4], const quat_t& q) {
    // GLSL quaternion helpers use the (x, y, z, w) convention.
    out[0] = q.x;
    out[1] = q.y;
    out[2] = q.z;
    out[3] = q.w;
}

static void update_vertices(const vec3_t& slerp, const vec3_t& lerp) {
    // SLERP (red)
    vertices[0] = 0.0f; vertices[1] = 0.0f; vertices[2] = 0.0f;
    vertices[3] = slerp.x; vertices[4] = slerp.y; vertices[5] = slerp.z;

    // LERP (blue)
    vertices[6] = 0.0f; vertices[7] = 0.0f; vertices[8] = 0.0f;
    vertices[9] = lerp.x; vertices[10] = lerp.y; vertices[11] = lerp.z;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
}

static void init_vao() {
    for (int i = 0; i < HAND_COUNT * 2; ++i)
        types[i] = (i < 2) ? 0 : 1;  // First 2: SLERP, next 2: LERP

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
    const std::filesystem::path executable_dir = get_executable_dir();
    const std::filesystem::path candidates[] = {
        executable_dir / "glsl" / name,          // portable package
        executable_dir / ".." / "glsl" / name, // install or single-config build
        executable_dir / ".." / ".." / "glsl" / name,
        std::filesystem::path(SHADER_DIR) / name
    };

    for (const auto& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate))
            return candidate.lexically_normal().string();
    }

    fprintf(stderr, "[GPU] Shader was not found: %s\n", name);
    return candidates[0].string();
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    if (gpu_init(800, 600, "[BYUL] GPU SLERP vs CPU LERP") != 0)
        return -1;

    GLuint render_program = gpu_load_shader(
        shader_path("shader.vert").c_str(),
        shader_path("shader.frag").c_str());
    bool compute_load_failed = false;
    for (ComputeDemo& demo : compute_demos) {
        demo.program = gpu_load_compute_shader(shader_path(demo.file).c_str());
        compute_load_failed = compute_load_failed || !demo.program;
    }

    if (!render_program || compute_load_failed) {
        glDeleteProgram(render_program);
        for (const ComputeDemo& demo : compute_demos)
            glDeleteProgram(demo.program);
        gpu_terminate();
        return -1;
    }

    GLuint input_ubo = gpu_create_ubo(sizeof(ComputeInput), 0);
    GLuint result_ssbo = gpu_create_ssbo(sizeof(ComputeResult), 1);

    glUseProgram(render_program);
    init_vao();

    SDL_Window* window = (SDL_Window*)gpu_get_window();
    int selected_demo = 0;
    select_demo(window, selected_demo, selected_demo);
    printf("Select comp: 1-4 = direct select, Left/Right = previous/next, Esc = quit\n");
    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now(), last_time = start_time;

    int running = 1, frames = 0;
    float max_gpu_error = 0.0f;
    vec3_t origin = {0.0f, 0.5f, 0.0f};

    while (running) {
#ifdef USE_SDL3
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = 0;
            } else if (event.type == SDL_EVENT_KEY_DOWN) {
                // SDL3 uses keycode directly (no keysym struct)
                if (event.key.key == SDLK_ESCAPE) {
                    running = 0;
                } else if (event.key.key >= SDLK_1 && event.key.key <= SDLK_4) {
                    select_demo(window, selected_demo,
                                static_cast<int>(event.key.key - SDLK_1));
                } else if (event.key.key == SDLK_LEFT) {
                    select_demo(window, selected_demo, selected_demo - 1);
                } else if (event.key.key == SDLK_RIGHT) {
                    select_demo(window, selected_demo, selected_demo + 1);
                }
            }
        }
#else
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_KEYDOWN) {
                SDL_Keycode key = event.key.keysym.sym;
                if (key == SDLK_ESCAPE) {
                    running = 0;
                } else if (key >= SDLK_1 && key <= SDLK_4) {
                    select_demo(window, selected_demo, static_cast<int>(key - SDLK_1));
                } else if (key == SDLK_LEFT) {
                    select_demo(window, selected_demo, selected_demo - 1);
                } else if (key == SDLK_RIGHT) {
                    select_demo(window, selected_demo, selected_demo + 1);
                }
            }
        }
#endif

        auto now = clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        float cycle_time = fmodf(elapsed, 60.0f);
        float base_sec = floorf(cycle_time);
        float t = cycle_time - base_sec;

        float angle_a = -base_sec * (float)(M_PI / 30.0f);
        float angle_b = -(base_sec + 1.0f) * (float)(M_PI / 30.0f);

        vec3_t axis = {0.0f, 0.0f, 1.0f};
        quat_t ra;
        quat_init_axis_angle(&ra, &axis, angle_a);

        quat_t rb;
        quat_init_axis_angle(&rb, &axis, angle_b);

        ComputeInput input = {};
        pack_quat(input.a, ra);
        pack_quat(input.b, rb);
        input.params[0] = t;
        input.params[1] = elapsed;
        gpu_update_ubo(input_ubo, &input, sizeof(input));
        gpu_dispatch_compute(compute_demos[selected_demo].program, 1, 1, 1);
        gpu_memory_barrier();

        ComputeResult gpu_result = {};
        void* mapped = gpu_map_ssbo(result_ssbo);
        if (!mapped) {
            fprintf(stderr, "[GPU] Failed to map compute result buffer\n");
            running = 0;
            continue;
        }
        memcpy(&gpu_result, mapped, sizeof(gpu_result));
        gpu_unmap_ssbo(result_ssbo);

        vec3_t rotated_gpu = {
            gpu_result.rotated[0],
            gpu_result.rotated[1],
            gpu_result.rotated[2]
        };

        vec3_t rotated_a, rotated_b, rotated_lerp;
        quat_rotate_vector(&ra, &origin, &rotated_a);
        quat_rotate_vector(&rb, &origin, &rotated_b);
        rotated_lerp.x = rotated_a.x * (1.0f - t) + rotated_b.x * t;
        rotated_lerp.y = rotated_a.y * (1.0f - t) + rotated_b.y * t;
        rotated_lerp.z = rotated_a.z * (1.0f - t) + rotated_b.z * t;

        quat_t cpu_slerp;
        quat_slerp(&cpu_slerp, &ra, &rb, t);
        vec3_t rotated_cpu_slerp;
        quat_rotate_vector(&cpu_slerp, &origin, &rotated_cpu_slerp);
        float dx = rotated_gpu.x - rotated_cpu_slerp.x;
        float dy = rotated_gpu.y - rotated_cpu_slerp.y;
        float dz = rotated_gpu.z - rotated_cpu_slerp.z;
        float gpu_error = sqrtf(dx * dx + dy * dy + dz * dz);
        if (gpu_error > max_gpu_error)
            max_gpu_error = gpu_error;

        update_vertices(rotated_gpu, rotated_lerp);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(render_program);

        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, 4);
        glBindVertexArray(0);

        SDL_GL_SwapWindow(window);

        // FPS measurement
        frames++;
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_time).count() >= 1) {
            printf("FPS: %d | comp: %s | max CPU SLERP deviation: %.8f\n",
                   frames, compute_demos[selected_demo].name, max_gpu_error);
            frames = 0;
            max_gpu_error = 0.0f;
            last_time = now;
        }
    }

    glDeleteBuffers(1, &result_ssbo);
    glDeleteBuffers(1, &input_ubo);
    glDeleteBuffers(1, &vbo_type);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    for (const ComputeDemo& demo : compute_demos)
        glDeleteProgram(demo.program);
    glDeleteProgram(render_program);
    gpu_terminate();
    return 0;
}
