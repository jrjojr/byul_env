#include "doctest.h"

extern "C" {
    #include "internal/numeq_kalman.h"
}

// -------------------- Scalar Kalman --------------------

TEST_CASE("Scalar Kalman: Initialization and update") {
    kalman_filter_t kf;
    kalman_init_full(&kf, 0.0f, 1.0f, 0.01f, 1.0f);  // x=0, p=1

    CHECK(kf.x == doctest::Approx(0.0f));
    CHECK(kf.p == doctest::Approx(1.0f));

    float m1 = 1.0f;
    float x1 = kalman_measurement_update(&kf, m1);  // 측정값 = 1.0
    CHECK(x1 > 0.0f);
    CHECK(x1 < 1.0f);  // 초기 추정이 0 → 보정 후 중간값
}

TEST_CASE("Scalar Kalman: Predict should increase uncertainty") {
    kalman_filter_t kf;
    kalman_init_full(&kf, 5.0f, 1.0f, 0.1f, 1.0f);

    float prev_p = kf.p;
    kalman_time_update(&kf);
    CHECK(kf.p > prev_p);  // 예측은 불확실성을 늘림
}

// -------------------- Vector Kalman --------------------

TEST_CASE("Vec3 Kalman: Initialization and basic time_update") {
    kalman_filter_vec3_t kf;
    vec3_t init_pos = {0.0f, 0.0f, 0.0f};
    vec3_t init_vel = {1.0f, 2.0f, 3.0f};

    kalman_vec3_init_full(&kf, &init_pos, &init_vel, 0.1f, 1.0f, 1.0f);  // dt = 1.0

    kalman_vec3_time_update(&kf);  // 위치 += 속도 * dt

    CHECK(kf.position.x == doctest::Approx(1.0f));
    CHECK(kf.position.y == doctest::Approx(2.0f));
    CHECK(kf.position.z == doctest::Approx(3.0f));
}

TEST_CASE("Vec3 Kalman: Update brings time_update closer to measurement") {
    kalman_filter_vec3_t kf;
    vec3_t init_pos = {0.0f, 0.0f, 0.0f};
    vec3_t init_vel = {1.0f, 0.0f, 0.0f};
    kalman_vec3_init_full(&kf, &init_pos, &init_vel, 0.1f, 1.0f, 1.0f);

    kalman_vec3_time_update(&kf);

    vec3_t measured = {2.0f, 0.0f, 0.0f};  // 측정값이 더 멀리 있음
    kalman_vec3_measurement_update(&kf, &measured);

    CHECK(kf.position.x > 1.0f);   // 보정 후 위치는 예측과 측정 사이
    CHECK(kf.position.x < 2.0f);
}

TEST_CASE("Vec3 Kalman: Project future position") {
    kalman_filter_vec3_t kf;
    vec3_t pos = {1.0f, 2.0f, 3.0f};
    vec3_t vel = {0.5f, 0.5f, 0.0f};
    kalman_vec3_init_full(&kf, &pos, &vel, 0.1f, 1.0f, 1.0f);

    vec3_t out = {0};
    kalman_vec3_project(&kf, 2.0f, &out);

    CHECK(out.x == doctest::Approx(2.0f));
    CHECK(out.y == doctest::Approx(3.0f));
    CHECK(out.z == doctest::Approx(3.0f));  // 속도 0이므로 유지
}

TEST_CASE("Vec3 Kalman: Copy state") {
    kalman_filter_vec3_t kf1, kf2;
    vec3_t pos = {1.0f, 1.0f, 1.0f};
    vec3_t vel = {0.0f, 1.0f, 0.0f};

    kalman_vec3_init_full(&kf1, &pos, &vel, 0.2f, 0.5f, 0.1f);
    kalman_vec3_assign(&kf2, &kf1);

    CHECK(kf2.position.x == doctest::Approx(1.0f));
    CHECK(kf2.velocity.y == doctest::Approx(1.0f));
    CHECK(kf2.q == doctest::Approx(0.2f));
    CHECK(kf2.r == doctest::Approx(0.5f));
    CHECK(kf2.dt == doctest::Approx(0.1f));
}

#include <random>

TEST_CASE("Scalar Kalman: Covariance P decreases after update") {
    kalman_filter_t kf;
    kalman_init_full(&kf, 0.0f, 10.0f, 0.1f, 1.0f);  // 큰 초기 오차 공분산

    float p_before = kf.p;
    float z = 5.0f;
    kalman_measurement_update(&kf, z);
    float p_after = kf.p;

    CHECK(p_after < p_before);  // update 후에는 항상 줄어야 함
}

TEST_CASE("Scalar Kalman: Random noisy measurements converge to true value") {
    kalman_filter_t kf;
    kalman_init_full(&kf, 0.0f, 1.0f, 0.01f, 0.5f);

    std::default_random_engine rng(42);
    std::normal_distribution<float> noise(0.0f, 0.5f);  // 측정 노이즈

    const float true_value = 3.14f;
    float last_estimate = 0.0f;

    for (int i = 0; i < 100; ++i) {
        float noisy_measured = true_value + noise(rng);
        last_estimate = kalman_measurement_update(&kf, noisy_measured);
    }

    CHECK(last_estimate == doctest::Approx(true_value).epsilon(0.05));  // 약 5% 오차 이내 수렴
}

TEST_CASE("Vec3 Kalman: error_p decreases after update") {
    kalman_filter_vec3_t kf;
    vec3_t pos = {0.0f, 0.0f, 0.0f};
    vec3_t vel = {1.0f, 0.0f, 0.0f};
    kalman_vec3_init_full(&kf, &pos, &vel, 0.1f, 1.0f, 1.0f);

    kalman_vec3_time_update(&kf);

    vec3_t p_before = kf.error_p;
    vec3_t measured = {2.0f, 0.0f, 0.0f};
    kalman_vec3_measurement_update(&kf, &measured);
    vec3_t p_after = kf.error_p;

    CHECK(p_after.x < p_before.x);
    CHECK(p_after.y < p_before.y);
    CHECK(p_after.z < p_before.z);
}
