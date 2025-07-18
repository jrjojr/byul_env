#include "internal/trajectory.h"
#include <vector>
#include <cstring>  // memset
#include <stdexcept>

// ---------------------------------------------------------
// trajectory 초기화
// ---------------------------------------------------------
bool trajectory_init(trajectory_t* traj, int capacity) {
    if (!traj || capacity <= 0) return false;

    // 기존 메모리 해제 (안전)
    if (traj->samples) {
        delete[] traj->samples;
        traj->samples = nullptr;
    }

    try {
        traj->samples = new trajectory_sample_t[capacity];
        traj->count = 0;
        traj->capacity = capacity;
        return true;
    } catch (const std::bad_alloc&) {
        traj->samples = nullptr;
        traj->count = 0;
        traj->capacity = 0;
        return false;
    }
}

// ---------------------------------------------------------
// trajectory 메모리 해제
// ---------------------------------------------------------
void trajectory_free(trajectory_t* traj) {
    if (!traj) return;
    if (traj->samples) {
        delete[] traj->samples;
        traj->samples = nullptr;
    }
    traj->count = 0;
    traj->capacity = 0;
}

// ---------------------------------------------------------
// trajectory 샘플 추가
// ---------------------------------------------------------
bool trajectory_add_sample(
    trajectory_t* traj, float t, const motion_state_t* state) {

    if (!traj || !state) return false;
    if (traj->count >= traj->capacity) return false;

    trajectory_sample_t& sample = traj->samples[traj->count++];
    sample.t = t;
    sample.state = *state;
    return true;
}

// ---------------------------------------------------------
// trajectory 초기화 (0으로 세팅)
// ---------------------------------------------------------
void trajectory_clear(trajectory_t* traj) {
    if (!traj) return;
    traj->count = 0;
    if (traj->samples) {
        std::memset(
            traj->samples, 0, sizeof(trajectory_sample_t) * traj->capacity);
    }
}

int trajectory_length(const trajectory_t* traj) {
    if (!traj) return 0;
    return traj->count;
}

int trajectory_capacity(const trajectory_t* traj) {
    if (!traj) return 0;
    return traj->capacity;
}

// ---------------------------------------------------------
// 문자열 버퍼로 trajectory 출력
// ---------------------------------------------------------
char* trajectory_to_string(const trajectory_t* traj, char* buffer, size_t size) {
    if (!traj || !buffer || size == 0) return buffer;

    size_t offset = 0;
    int n = snprintf(buffer + offset, size - offset,
                     "---- Trajectory Samples (count=%d) ----\n", traj->count);
    if (n < 0) return buffer;
    offset += (size_t)n;

    for (int i = 0; i < traj->count && offset < size; ++i) {
        const trajectory_sample_t* s = &traj->samples[i];
        const vec3_t* p = &s->state.linear.position;
        const vec3_t* v = &s->state.linear.velocity;

        n = snprintf(buffer + offset, size - offset,
                     " t=%.3f  pos=(%.3f, %.3f, %.3f)  vel=(%.3f, %.3f, %.3f)\n",
                     s->t, p->x, p->y, p->z, v->x, v->y, v->z);

        if (n < 0) break;
        offset += (size_t)n;
    }
    return buffer;
}

// ---------------------------------------------------------
// 콘솔로 trajectory 출력
// ---------------------------------------------------------
void trajectory_print(const trajectory_t* traj) {
    if (!traj) return;

    printf("---- Trajectory Samples (count=%d) ----\n", traj->count);
    printf("    t(s)        pos(x,y,z)              vel(x,y,z)\n");
    printf("-----------------------------------------------------------\n");

    for (int i = 0; i < traj->count; ++i) {
        const trajectory_sample_t* s = &traj->samples[i];
        const vec3_t* p = &s->state.linear.position;
        const vec3_t* v = &s->state.linear.velocity;

        printf(" %6.3f   (%.3f, %.3f, %.3f)   (%.3f, %.3f, %.3f)\n",
               s->t, p->x, p->y, p->z, v->x, v->y, v->z);
    }
    printf("-----------------------------------------------------------\n");
}

// ---------------------------------------------------------
// 위치 벡터 리스트 추출
// ---------------------------------------------------------
int trajectory_get_positions(const trajectory_t* traj, vec3_t* out_list, int max) {
    if (!traj || !out_list || max <= 0) return 0;

    int count = (traj->count < max) ? traj->count : max;
    for (int i = 0; i < count; ++i) {
        out_list[i] = traj->samples[i].state.linear.position;
    }
    return count;
}

// ---------------------------------------------------------
// 속력 리스트 추출
// ---------------------------------------------------------
int trajectory_get_speeds(const trajectory_t* traj, float* out_list, int max) {
    if (!traj || !out_list || max <= 0) return 0;

    int count = (traj->count < max) ? traj->count : max;
    for (int i = 0; i < count; ++i) {
        const vec3_t* v = &traj->samples[i].state.linear.velocity;
        out_list[i] = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
    }
    return count;
}

// 특정 시간의 타겟 위치를 보간
bool trajectory_sample_position(const trajectory_t* traj, float t, vec3_t* out_pos) {
    if (!traj || traj->count < 1) return false;
    // 경계 체크
    if (t <= traj->samples[0].t) {
        *out_pos = traj->samples[0].state.linear.position;
        return true;
    }
    if (t >= traj->samples[traj->count - 1].t) {
        *out_pos = traj->samples[traj->count - 1].state.linear.position;
        return true;
    }
    // 이진 탐색 또는 선형 탐색으로 적절한 구간 찾기
    for (int i = 0; i < traj->count - 1; i++) {
        const trajectory_sample_t* s1 = &traj->samples[i];
        const trajectory_sample_t* s2 = &traj->samples[i + 1];
        if (t >= s1->t && t <= s2->t) {
            float alpha = (t - s1->t) / (s2->t - s1->t);
            vec3_lerp(out_pos, &s1->state.linear.position, &s2->state.linear.position, alpha);
            return true;
        }
    }
    return false;
}
