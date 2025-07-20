#include "internal/gpu.h"
#include <stdio.h>
#include <glad/gl.h>
#include <SDL.h>
#include <chrono>
#include <cmath>

static GLuint vao = 0, vbo = 0;

static void init_clock_hand() {
    float vertices[] = {
        0.0f, 0.0f, 0.0f,  // 시계 중심
        0.0f, 0.5f, 0.0f   // 초침 끝 (12시 방향)
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

int main(int argc, char** argv) {
    if (gpu_init(800, 600, "[BYUL] Analog Clock") != 0) {
        fprintf(stderr, "[MAIN] GPU init failed\n");
        return -1;
    }

    GLuint shader = gpu_load_shader("shader.vert", "shader.frag");
    if (!shader) {
        fprintf(stderr, "[MAIN] Failed to load shader\n");
        return -1;
    }
    glUseProgram(shader);

    init_clock_hand();
    SDL_Window* window = (SDL_Window*)gpu_get_window();

    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now();
    auto last_time = start_time;
    int frames = 0;

    vec3_t origin = vec3_t{ 0.0f, 0.5f, 0.0f }; // 초침 벡터
    vec3_t rotated = vec3_t{0,0,0};

    int running = 1;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_KEYDOWN &&
                 event.key.keysym.sym == SDLK_ESCAPE)) {
                running = 0;
            }
        }

        auto now = clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();

        // 초 단위 회전 각도 계산 (-6도 * 초)
        float seconds = fmod(elapsed, 60.0f);
        float angle_rad = -seconds * (M_PI / 30.0f); // 360도 / 60 = 6도

        // Z축 기준 회전 쿼터니언
        vec3_t axis = vec3_t{0,0,1};
        rotator_t* r = rotator_from_axis_angle(&axis, angle_rad);

        // 회전 적용
        rotator_apply_to_vec3(&rotated, r, &origin);

        float vertices[] = {
            0.0f, 0.0f, 0.0f,
            rotated.x, rotated.y, rotated.z
        };

        // VBO 갱신
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

        // 렌더링
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shader);
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, 2);
        glBindVertexArray(0);

        SDL_GL_SwapWindow(window);

        rotator_destroy(r);

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
