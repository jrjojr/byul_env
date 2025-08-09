#include "doctest.h"
#include "byul_tick.h"

#include <vector>

struct test_ctx_t {
    float acc = 0.0f;
    int call_count = 0;
};

static void sample_tick_func(void* context, float dt) {
    test_ctx_t* ctx = static_cast<test_ctx_t*>(context);
    if (ctx) {
        ctx->acc += dt;
        ctx->call_count++;
    }
}

TEST_CASE("tick_create and destroy") {
    tick_t* tk = tick_create();
    CHECK(tk != nullptr);
    tick_destroy(tk);
}

TEST_CASE("tick_attach and tick") {
    tick_t* tk = tick_create();
    test_ctx_t ctx;

    int ret = tick_attach(tk, sample_tick_func, &ctx);
    CHECK(ret == 0);

    tick_update(tk, 0.5f);
    CHECK(ctx.acc == doctest::Approx(0.5f));
    CHECK(ctx.call_count == 1);

    tick_update(tk, 1.0f);
    CHECK(ctx.acc == doctest::Approx(1.5f));
    CHECK(ctx.call_count == 2);

    tick_destroy(tk);
}

TEST_CASE("tick_attach duplicate") {
    tick_t* tk = tick_create();
    test_ctx_t ctx;

    CHECK(tick_attach(tk, sample_tick_func, &ctx) == 0);
    CHECK(tick_attach(tk, sample_tick_func, &ctx) == -2); // duplicate

    tick_destroy(tk);
}

TEST_CASE("tick_detach removes correctly") {
    tick_t* tk = tick_create();
    test_ctx_t ctx;

    CHECK(tick_attach(tk, sample_tick_func, &ctx) == 0);
    CHECK(tick_detach(tk, sample_tick_func, &ctx) == 0);

    tick_update(tk, 1.0f); // should not be called
    CHECK(ctx.acc == doctest::Approx(0.0f));
    CHECK(ctx.call_count == 0);

    tick_destroy(tk);
}

TEST_CASE("tick_list_attached returns correct entries") {
    tick_t* tk = tick_create();
    test_ctx_t ctx1, ctx2;

    CHECK(tick_attach(tk, sample_tick_func, &ctx1) == 0);
    CHECK(tick_attach(tk, sample_tick_func, &ctx2) == 0);

    tick_entry_t entries[4];
    int count = tick_list_attached(tk, entries, 4);
    CHECK(count == 2);

    bool found1 = false, found2 = false;
    for (int i = 0; i < count; ++i) {
        if (entries[i].context == &ctx1) found1 = true;
        if (entries[i].context == &ctx2) found2 = true;
        CHECK(entries[i].func == sample_tick_func);
    }
    CHECK(found1);
    CHECK(found2);

    tick_destroy(tk);
}
