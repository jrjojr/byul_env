#include "internal/gpu.h"
#include <stdio.h>
#include <glad/gl.h>
#include <SDL.h>
#include <chrono>
#include <cmath>

static GLuint vao = 0, vbo = 0, vbo_type = 0;

static void init_two_hands() {
    float vertices[2 * 3 * 2] = {
        // SLERP (빨강)
        0.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f,

        // LERP (파랑)
        0.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f
    };

    int types[4] = {
        0, 0, // SLERP
        1, 1  // LERP
    };

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glGenBuffers(1, &vbo_type);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
    glBufferData(GL_ARRAY_BUFFER, sizeof(types), types, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribIPointer(1, 1, GL_INT, sizeof(int), (void*)0);

    glBindVertexArray(0);
}

int main(int argc, char** argv) {
    if (gpu_init(800, 600, "[BYUL] LERP vs SLERP") != 0) return -1;

    GLuint shader = gpu_load_shader("shader.vert", "shader.frag");
    if (!shader) return -1;

    glUseProgram(shader);
    init_two_hands();

    SDL_Window* window = (SDL_Window*)gpu_get_window();
    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now();
    auto last_time = start_time;

    vec3_t origin = { 0.0f, 0.5f, 0.0f };
    vec3_t rotated_slerp, rotated_lerp;

    int running = 1, frames = 0;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = 0;
            }
        }

        auto now = clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();

        float total_seconds = fmod(elapsed, 60.0f);
        float base_sec = floorf(total_seconds);
        float t = total_seconds - base_sec;

        float angle_rad_a = -base_sec * (M_PI / 30.0f);
        float angle_rad_b = -(base_sec + 1) * (M_PI / 30.0f);

        vec3_t axis = { 0.0f, 0.0f, 1.0f };
        rotator_t* ra = rotator_from_axis_angle(&axis, angle_rad_a);
        rotator_t* rb = rotator_from_axis_angle(&axis, angle_rad_b);

        rotator_t* rs = rotator_new();
        rotator_slerp(rs, ra, rb, t);
        rotator_apply_to_vec3(&rotated_slerp, rs, &origin);

        vec3_t target;
        rotator_apply_to_vec3(&target, rb, &origin);
        rotated_lerp.x = origin.x * (1.0f - t) + target.x * t;
        rotated_lerp.y = origin.y * (1.0f - t) + target.y * t;
        rotated_lerp.z = origin.z * (1.0f - t) + target.z * t;

        float vertices[] = {
            0.0f, 0.0f, 0.0f,
            rotated_slerp.x, rotated_slerp.y, rotated_slerp.z,
            0.0f, 0.0f, 0.0f,
            rotated_lerp.x, rotated_lerp.y, rotated_lerp.z
        };

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shader);

        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, 4);
        glBindVertexArray(0);

        SDL_GL_SwapWindow(window);

        rotator_free(ra);
        rotator_free(rb);
        rotator_free(rs);

        frames++;
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time);
        if (duration.count() >= 1) {
            printf("FPS: %d\n", frames);
            frames = 0;
            last_time = now;
        }
    }

    gpu_terminate();
    return 0;
}
