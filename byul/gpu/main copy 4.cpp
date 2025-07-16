#include "internal/gpu.h"
#include "internal/gpu_compute.h"
#include <stdio.h>
#include <glad/gl.h>
#include <SDL.h>
#include <chrono>
#include <cmath>

static const int WORKGROUP_SIZE = 1;

int main(int argc, char** argv) {
    if (gpu_init(800, 600, "[BYUL] Compute Clock") != 0) return -1;

    GLuint compute_program = gpu_load_compute_shader("clock.comp");
    if (!compute_program) return -1;

    GLuint ubo = gpu_create_ubo(sizeof(rotator_gpu_lerp_t), 0);
    GLuint ssbo = gpu_create_ssbo(sizeof(float) * 4, 1);

    GLuint vao = 0, vbo = 0;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);

    float vertices[6] = {
        0.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f
    };

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    GLuint shader = gpu_load_shader("shader.vert", "shader.frag");
    if (!shader) return -1;

    SDL_Window* window = (SDL_Window*)gpu_get_window();
    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now();

    int running = 1;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = 0;
            }
        }

        float elapsed = std::chrono::duration<float>(clock::now() - start_time).count();
        rotator_gpu_lerp_t udata;
        // udata.a = {0.0f, 0.0f, 1.0f, -elapsed * (float)(M_PI / 30.0f)};
        // udata.b = {0.0f, 0.0f, 1.0f, -(elapsed + 1.0f) * (float)(M_PI / 30.0f)};

float angle_a = -elapsed * (float)(M_PI / 30.0f);
float angle_b = -(elapsed + 1.0f) * (float)(M_PI / 30.0f);

float axis_angle_a[4] = {0.0f, 0.0f, 1.0f, angle_a};
float axis_angle_b[4] = {0.0f, 0.0f, 1.0f, angle_b};

memcpy(udata.a, axis_angle_a, sizeof(float) * 4);
memcpy(udata.b, axis_angle_b, sizeof(float) * 4);

        udata.t = fmodf(elapsed, 1.0f);
        udata.pad[0] = udata.pad[1] = udata.pad[2] = 0.0f;

        gpu_update_ubo(ubo, &udata, sizeof(udata));

        gpu_dispatch_compute(compute_program, WORKGROUP_SIZE, 1, 1);
        gpu_memory_barrier();

        float* result = (float*)gpu_map_ssbo(ssbo);
        vertices[3] = result[0];
        vertices[4] = result[1];
        vertices[5] = result[2];
        printf("[GPU] rotated = (%.3f, %.3f, %.3f)\n", result[0], result[1], result[2]);
        gpu_unmap_ssbo(ssbo);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shader);
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, 2);
        glBindVertexArray(0);

        SDL_GL_SwapWindow(window);
    }

    gpu_terminate();
    return 0;
}
