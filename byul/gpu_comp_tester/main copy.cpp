#include "internal/gpu.h"
#include <stdio.h>
#include <glad/gl.h>   // OpenGL 함수 선언
#include <SDL.h>
#include <chrono>
#include <cmath>

static GLuint vao[2] = {0}, vbo[2] = {0};

// 원본 선분과 보간된 선분 두 개의 VAO/VBO 초기화
static void init_line_geometries() {
    float vertices[] = {
        -0.5f, 0.0f, 0.0f,
         0.5f, 0.0f, 0.0f,
    };

    glGenVertexArrays(2, vao);
    glGenBuffers(2, vbo);

    for (int i = 0; i < 2; ++i) {
        glBindVertexArray(vao[i]);
        glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

        // 선 ID 고정값 (0 = 원본, 1 = 보간된 선)
        glEnableVertexAttribArray(1);
        glVertexAttribI1i(1, i); // 정수 속성으로 고정 설정
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

int main(int argc, char** argv) {
    if (gpu_init(800, 600, "[BYUL] GPU Test Window") != 0) {
        fprintf(stderr, "[MAIN] GPU init failed\n");
        return -1;
    }

    GLuint shader = gpu_load_shader("shader.vert", "shader.frag");
    if (!shader) {
        fprintf(stderr, "[MAIN] Failed to load shader\n");
        return -1;
    }
    glUseProgram(shader);

    GLuint ubo = gpu_create_ubo(sizeof(rotator_gpu_lerp_t), 0);

    rotator_t* rot_a = rotator_new_full(1, 0, 0, 0);
    rotator_t* rot_b = rotator_new_full(0, 1, 0, 0);

    rotator_gpu_lerp_t udata;
    init_line_geometries();

    int running = 1;
    SDL_Window* window = (SDL_Window*)gpu_get_window();

    using clock = std::chrono::high_resolution_clock;
    auto start_time = clock::now();
    auto last_time = start_time;
    int frames = 0;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_KEYDOWN && 
                    event.key.keysym.sym == SDLK_ESCAPE)) {
                running = 0;
            }
        }

        // ⏱ 시간 기반 t 계산 (0~1 사이를 반복)
        auto now = clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        float t = (sinf(elapsed) * 0.5f) + 0.5f; // 0 ~ 1

        // UBO 업데이트 (시간에 따라 보간 변화)
        rotator_to_gpu_lerp(&udata, rot_a, rot_b, t);
        gpu_update_ubo(ubo, &udata, sizeof(udata));

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shader);

        glBindVertexArray(vao[0]);
        glDrawArrays(GL_LINES, 0, 2);

        glBindVertexArray(vao[1]);
        glDrawArrays(GL_LINES, 0, 2);

        glBindVertexArray(0);
        SDL_GL_SwapWindow(window);

        frames++;
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time);
        if (duration.count() >= 1) {
            printf("FPS: %d\n", frames);
            frames = 0;
            last_time = now;
        }
    }

    rotator_free(rot_a);
    rotator_free(rot_b);
    gpu_terminate();
    return 0;
}
