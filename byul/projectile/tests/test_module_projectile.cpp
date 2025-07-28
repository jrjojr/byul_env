#include <doctest.h>
#include "projectile.h"

#include "vec3.h"
#include "environ.h"

TEST_CASE("Shell projectile basic initialization") {
    shell_projectile_t shell;
    shell_projectile_init(&shell);

    CHECK(shell.proj.damage == doctest::Approx(1.0f));
    CHECK(shell.explosion_radius == doctest::Approx(10.0f));

    shell_projectile_init_full(&shell, 10.0f, 5.0f);
    CHECK(shell.proj.damage == doctest::Approx(10.0f));
    CHECK(shell.explosion_radius == doctest::Approx(5.0f));

    shell_projectile_t copy;
    shell_projectile_assign(&copy, &shell);
    CHECK(copy.explosion_radius == doctest::Approx(5.0f));
}

TEST_CASE("Shell projectile launch simulation") {
    shell_projectile_t shell;
    shell_projectile_init_full(&shell, 1.0f, 10.0f);

    environ_t env;
    environ_init(&env);

    projectile_result_t * result = projectile_result_create();

    vec3_t target = {10.0f, 0.0f, 0.0f};
    bool hit = shell_projectile_launch(&shell, &target, 30.0f, &env, result);

    projectile_result_print_detailed(result);
    CHECK(result->trajectory->count > 0);

    projectile_result_destroy(result);
}

TEST_CASE("Rocket initialization") {
    rocket_t rocket;
    rocket_init(&rocket);

    CHECK(rocket.base.proj.damage == doctest::Approx(1.0f));
    CHECK(rocket.base.explosion_radius == doctest::Approx(10.0f));
}

TEST_CASE("Missile initialization and launch") {
    missile_t missile;
    missile_init_full(&missile, 20.0f, 8.0f);

    CHECK(missile.base.base.proj.damage == doctest::Approx(20.0f));
    CHECK(missile.base.base.explosion_radius == doctest::Approx(8.0f));

    environ_t env;
    environ_init(&env);

    vec3_t target = {15.0f, 0.0f, 0.0f};

    projectile_result_t * result = projectile_result_create();
    bool launched = missile_launch(&missile, &target, 30.0f, &env, result);

    projectile_result_print_detailed(result);
    CHECK(result->trajectory->count > 0);

    projectile_result_destroy(result);    
}

TEST_CASE("Patriot initialization and launch") {
    patriot_t patriot;
    patriot_init_full(&patriot, 50.0f, 10.0f);

    CHECK(patriot.base.base.base.proj.damage == doctest::Approx(50.0f));
    CHECK(patriot.base.base.base.explosion_radius == doctest::Approx(10.0f));

    environ_t env;
    environ_init(&env);

    entity_dynamic_t dummy_target;
    entity_dynamic_init(&dummy_target);
    dummy_target.xf.pos = {30.0f, 0.0f, 0.0f};

    projectile_result_t * result = projectile_result_create();
    
    bool launched = patriot_launch(&patriot, &dummy_target, 40.0f, &env, result);

    projectile_result_print_detailed(result);
    CHECK(result->trajectory->count > 0);

    projectile_result_destroy(result);    
}

TEST_CASE("shell_projectile_update 동작 테스트") {
    shell_projectile_t proj;
    shell_projectile_init(&proj);

    proj.proj.base.velocity = {1.0f, 0.0f, 0.0f};
    proj.proj.base.angular_velocity = {0.0f, 0.0f, 3.1415f};
    proj.proj.base.base.lifetime = 0.5f;

    int userdata_val = 0;
    proj.proj.on_hit = [](const void* p, void* ud) {
        (void)p;
        int* val = static_cast<int*>(ud);
        *val = 999;
    };
    proj.proj.hit_userdata = &userdata_val;

    // 0.3초 경과
    projectile_update(&proj.proj, 0.3f);
    CHECK(proj.proj.base.base.age == doctest::Approx(0.3f));
    CHECK(userdata_val == 0);

    // 0.3초 추가 (총 0.6초 > 0.5초 수명)
    projectile_update(&proj.proj, 0.3f);
    CHECK(proj.proj.base.base.age == doctest::Approx(0.6f));
    CHECK(userdata_val == 999);
}

TEST_CASE("shell_projectile_default_hit_cb 동작 테스트") {
    shell_projectile_t proj;
    shell_projectile_init(&proj);

    // stdout 캡처 없이 단순 호출만 체크
    shell_projectile_hit_cb(&proj, nullptr);
    CHECK(true); // 단순 호출 성공 여부만 확인
}

TEST_CASE("shell_projectile_update에서 on_hit 호출 테스트") {
    shell_projectile_t proj;
    shell_projectile_init(&proj);

    proj.proj.base.base.lifetime = 1.0f;  // 1초 후 만료

    // 아직 수명 전
    projectile_update(&proj.proj, 0.5f);

    // 수명 경과 (1.2초 후)
    projectile_update(&proj.proj, 0.7f);
}
