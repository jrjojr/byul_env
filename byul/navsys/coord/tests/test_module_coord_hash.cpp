#include "doctest.h"

#include <stdexcept>

extern "C" {
#include "coord.h"
#include "coord_hash.h"
}

int* make_int(int value) {
    return new int(value);
}

namespace {

struct copy_destroy_counts {
    int copy_calls;
    int destroy_calls;
    int null_destroy_calls;
};

copy_destroy_counts g_counts = {};

void reset_copy_destroy_counts() {
    g_counts = {};
}

void* counted_int_copy(const void* value) {
    ++g_counts.copy_calls;
    if (!value) {
        return nullptr;
    }
    return new int(*static_cast<const int*>(value));
}

void counted_int_destroy(void* value) {
    ++g_counts.destroy_calls;
    if (!value) {
        ++g_counts.null_destroy_calls;
        return;
    }
    delete static_cast<int*>(value);
}

struct canonical_callback_context {
    int copy_calls = 0;
    int destroy_calls = 0;
    int null_destroy_calls = 0;
    navsys_status_t next_copy_status = NAVSYS_STATUS_OK;
    bool throw_on_copy = false;
    bool write_copy_on_failure = false;
    coord_hash_t* reentry_hash = nullptr;
    navsys_status_t reentry_status = NAVSYS_STATUS_OK;
};

navsys_status_t canonical_int_copy(
    const void* source,
    void** out_copy,
    void* userdata) {
    auto* context = static_cast<canonical_callback_context*>(userdata);
    ++context->copy_calls;
    if (context->reentry_hash) {
        coord_t reentry_key = {99, 99};
        int reentry_value = 99;
        context->reentry_status = coord_hash_insert_copy(
            context->reentry_hash, &reentry_key, &reentry_value);
        coord_hash_destroy(context->reentry_hash);
    }
    if (context->throw_on_copy) {
        throw std::runtime_error("copy failure");
    }
    if (context->next_copy_status != NAVSYS_STATUS_OK) {
        if (context->write_copy_on_failure) {
            *out_copy = new int(*static_cast<const int*>(source));
        }
        return context->next_copy_status;
    }
    *out_copy = new int(*static_cast<const int*>(source));
    return NAVSYS_STATUS_OK;
}

void canonical_int_destroy(void* value, void* userdata) {
    auto* context = static_cast<canonical_callback_context*>(userdata);
    ++context->destroy_calls;
    if (!value) {
        ++context->null_destroy_calls;
        return;
    }
    delete static_cast<int*>(value);
}

coord_hash_create_info_t canonical_create_info(
    canonical_callback_context* context) {
    coord_hash_create_info_t info = {};
    info.struct_size = sizeof(info);
    info.abi_version = BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION;
    info.copy_value = canonical_int_copy;
    info.destroy_value = canonical_int_destroy;
    info.userdata = context;
    return info;
}

}

TEST_CASE("coord_hash canonical create validates versioned callback binding") {
    canonical_callback_context context;
    coord_hash_create_info_t info = canonical_create_info(&context);
    coord_hash_t* sentinel = coord_hash_create();
    REQUIRE(sentinel != nullptr);
    coord_hash_t* output = sentinel;

    info.struct_size = sizeof(info) - 1;
    CHECK(coord_hash_create_ex(&info, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == sentinel);

    info = canonical_create_info(&context);
    ++info.abi_version;
    CHECK(coord_hash_create_ex(&info, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == sentinel);

    info = canonical_create_info(&context);
    info.destroy_value = nullptr;
    CHECK(coord_hash_create_ex(&info, &output)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(output == sentinel);

    coord_hash_destroy(sentinel);
}

TEST_CASE("coord_hash canonical mutations are insert-only replace-only and upsert") {
    canonical_callback_context context;
    coord_hash_create_info_t info = canonical_create_info(&context);
    coord_hash_t* hash = nullptr;
    REQUIRE(coord_hash_create_ex(&info, &hash) == NAVSYS_STATUS_OK);
    REQUIRE(hash != nullptr);

    coord_t first = {11, 12};
    coord_t second = {13, 14};
    int first_value = 110;
    int second_value = 120;
    int third_value = 130;
    bool inserted = false;

    CHECK(coord_hash_insert_copy(hash, &first, &first_value)
        == NAVSYS_STATUS_OK);
    CHECK(context.copy_calls == 1);
    CHECK(coord_hash_insert_copy(hash, &first, &second_value)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(context.copy_calls == 1);

    CHECK(coord_hash_replace_copy(hash, &second, &second_value)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(context.copy_calls == 1);
    CHECK(coord_hash_replace_copy(hash, &first, &second_value)
        == NAVSYS_STATUS_OK);
    CHECK(context.copy_calls == 2);
    CHECK(context.destroy_calls == 1);

    CHECK(coord_hash_upsert_copy(hash, &second, &third_value, &inserted)
        == NAVSYS_STATUS_OK);
    CHECK(inserted);
    CHECK(coord_hash_upsert_copy(hash, &second, &first_value, &inserted)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(inserted);
    CHECK(context.copy_calls == 4);
    CHECK(context.destroy_calls == 2);

    coord_hash_destroy(hash);
    CHECK(context.destroy_calls == 4);
    CHECK(context.null_destroy_calls == 0);
}

TEST_CASE("coord_hash canonical callback failure preserves table and outputs") {
    canonical_callback_context context;
    coord_hash_create_info_t info = canonical_create_info(&context);
    coord_hash_t* hash = nullptr;
    REQUIRE(coord_hash_create_ex(&info, &hash) == NAVSYS_STATUS_OK);

    coord_t key = {15, 16};
    int original_value = 150;
    int replacement_value = 160;
    REQUIRE(coord_hash_insert_copy(hash, &key, &original_value)
        == NAVSYS_STATUS_OK);
    void* original_stored = coord_hash_get(hash, &key);
    REQUIRE(original_stored != nullptr);

    context.next_copy_status = NAVSYS_STATUS_OUT_OF_MEMORY;
    context.write_copy_on_failure = true;
    bool inserted = true;
    CHECK(coord_hash_upsert_copy(hash, &key, &replacement_value, &inserted)
        == NAVSYS_STATUS_OUT_OF_MEMORY);
    CHECK(inserted);
    CHECK(coord_hash_get(hash, &key) == original_stored);
    CHECK(*static_cast<int*>(coord_hash_get(hash, &key)) == 150);
    CHECK(context.destroy_calls == 1);

    context.next_copy_status = NAVSYS_STATUS_OK;
    context.write_copy_on_failure = false;
    context.throw_on_copy = true;
    CHECK(coord_hash_replace_copy(hash, &key, &replacement_value)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(coord_hash_get(hash, &key) == original_stored);

    coord_hash_destroy(hash);
    CHECK(context.destroy_calls == 2);
}

TEST_CASE("coord_hash canonical copy is deep and rejects missing copy callback") {
    canonical_callback_context context;
    coord_hash_create_info_t info = canonical_create_info(&context);
    coord_hash_t* source = nullptr;
    REQUIRE(coord_hash_create_ex(&info, &source) == NAVSYS_STATUS_OK);

    coord_t key = {17, 18};
    int value = 170;
    REQUIRE(coord_hash_insert_copy(source, &key, &value)
        == NAVSYS_STATUS_OK);

    coord_hash_t* copied = nullptr;
    CHECK(coord_hash_copy_ex(source, &copied) == NAVSYS_STATUS_OK);
    REQUIRE(copied != nullptr);
    REQUIRE(coord_hash_get(source, &key) != nullptr);
    REQUIRE(coord_hash_get(copied, &key) != nullptr);
    CHECK(coord_hash_get(source, &key) != coord_hash_get(copied, &key));
    CHECK(*static_cast<int*>(coord_hash_get(copied, &key)) == 170);

    coord_hash_destroy(copied);
    coord_hash_destroy(source);
    CHECK(context.copy_calls == 2);
    CHECK(context.destroy_calls == 2);

    coord_hash_create_info_t null_info = {};
    null_info.struct_size = sizeof(null_info);
    null_info.abi_version = BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION;
    coord_hash_t* null_only = nullptr;
    REQUIRE(coord_hash_create_ex(&null_info, &null_only) == NAVSYS_STATUS_OK);
    coord_hash_t* unchanged = null_only;
    CHECK(coord_hash_copy_ex(null_only, &unchanged)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(unchanged == null_only);
    coord_hash_destroy(null_only);
}

TEST_CASE("coord_hash canonical callback rejects same-table reentry") {
    canonical_callback_context context;
    coord_hash_create_info_t info = canonical_create_info(&context);
    coord_hash_t* hash = nullptr;
    REQUIRE(coord_hash_create_ex(&info, &hash) == NAVSYS_STATUS_OK);
    context.reentry_hash = hash;

    coord_t key = {19, 20};
    int value = 190;
    CHECK(coord_hash_insert_copy(hash, &key, &value) == NAVSYS_STATUS_OK);
    CHECK(context.reentry_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(coord_hash_length(hash) == 1);

    context.reentry_hash = nullptr;
    coord_hash_destroy(hash);
    CHECK(context.destroy_calls == 1);
}

TEST_CASE("coord_hash legacy insert and replace are copy-upsert") {
    reset_copy_destroy_counts();
    coord_hash_t* hash =
        coord_hash_create_full(counted_int_copy, counted_int_destroy);
    REQUIRE(hash != nullptr);

    coord_t first = {1, 2};
    coord_t second = {3, 4};
    int first_value = 10;
    int second_value = 20;
    int third_value = 30;
    int fourth_value = 40;

    CHECK(coord_hash_insert(hash, &first, &first_value));
    CHECK(g_counts.copy_calls == 1);
    CHECK(g_counts.destroy_calls == 0);

    CHECK(coord_hash_insert(hash, &first, &second_value));
    CHECK(g_counts.copy_calls == 2);
    CHECK(g_counts.destroy_calls == 1);
    REQUIRE(coord_hash_get(hash, &first) != nullptr);
    CHECK(*static_cast<int*>(coord_hash_get(hash, &first)) == 20);

    CHECK(coord_hash_replace(hash, &second, &third_value));
    CHECK(g_counts.copy_calls == 3);
    CHECK(g_counts.destroy_calls == 1);
    CHECK(coord_hash_length(hash) == 2);

    CHECK(coord_hash_replace(hash, &second, &fourth_value));
    CHECK(g_counts.copy_calls == 4);
    CHECK(g_counts.destroy_calls == 2);

    CHECK(coord_hash_remove(hash, &first));
    CHECK(g_counts.destroy_calls == 3);
    coord_hash_clear(hash);
    CHECK(g_counts.destroy_calls == 4);
    CHECK(coord_hash_is_empty(hash));

    coord_hash_destroy(hash);
    CHECK(g_counts.destroy_calls == 4);
}

TEST_CASE("coord_hash legacy set stores exact pointer and leaks replaced owner") {
    reset_copy_destroy_counts();
    coord_hash_t* hash =
        coord_hash_create_full(counted_int_copy, counted_int_destroy);
    REQUIRE(hash != nullptr);

    coord_t key = {5, 6};
    int* replaced_owner = new int(50);
    int* current_owner = new int(60);

    coord_hash_set(hash, &key, replaced_owner);
    CHECK(coord_hash_get(hash, &key) == replaced_owner);
    CHECK(g_counts.copy_calls == 0);
    CHECK(g_counts.destroy_calls == 0);

    coord_hash_set(hash, &key, current_owner);
    CHECK(coord_hash_get(hash, &key) == current_owner);
    CHECK(g_counts.copy_calls == 0);
    CHECK(g_counts.destroy_calls == 0);

    delete replaced_owner;
    coord_hash_destroy(hash);
    CHECK(g_counts.destroy_calls == 1);
    CHECK(g_counts.null_destroy_calls == 0);
}

TEST_CASE("coord_hash copy without copy callback substitutes null values") {
    reset_copy_destroy_counts();
    coord_hash_t* original =
        coord_hash_create_full(nullptr, counted_int_destroy);
    REQUIRE(original != nullptr);

    coord_t key = {7, 8};
    int* owned_value = new int(70);
    coord_hash_set(original, &key, owned_value);

    coord_hash_t* copied = coord_hash_copy(original);
    REQUIRE(copied != nullptr);
    CHECK(coord_hash_contains(copied, &key));
    CHECK(coord_hash_get(copied, &key) == nullptr);
    CHECK(g_counts.copy_calls == 0);

    coord_hash_destroy(copied);
    CHECK(g_counts.destroy_calls == 0);
    coord_hash_destroy(original);
    CHECK(g_counts.destroy_calls == 1);
}

TEST_CASE("coord_hash clear and destroy disagree on stored null callback") {
    reset_copy_destroy_counts();
    coord_t key = {9, 10};

    coord_hash_t* cleared =
        coord_hash_create_full(counted_int_copy, counted_int_destroy);
    REQUIRE(cleared != nullptr);
    CHECK(coord_hash_insert(cleared, &key, nullptr));
    CHECK(g_counts.copy_calls == 0);
    coord_hash_clear(cleared);
    CHECK(g_counts.destroy_calls == 1);
    CHECK(g_counts.null_destroy_calls == 1);
    coord_hash_destroy(cleared);
    CHECK(g_counts.destroy_calls == 1);

    reset_copy_destroy_counts();
    coord_hash_t* destroyed =
        coord_hash_create_full(counted_int_copy, counted_int_destroy);
    REQUIRE(destroyed != nullptr);
    CHECK(coord_hash_insert(destroyed, &key, nullptr));
    coord_hash_destroy(destroyed);
    CHECK(g_counts.destroy_calls == 0);
    CHECK(g_counts.null_destroy_calls == 0);
}

TEST_CASE("coord_hash: replace with int values (new/delete)") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);
    coord_t* c = coord_create_full(1, 1);

    int* v1 = make_int(100);
    int* v2 = make_int(200);
    int* v3 = make_int(300);

    CHECK(coord_hash_replace(hash, c, v1));
    int* found1 = (int*)coord_hash_get(hash, c);
    REQUIRE(found1 != nullptr);
    CHECK(*found1 == 100);

    CHECK(coord_hash_replace(hash, c, v2));
    int* found2 = (int*)coord_hash_get(hash, c);
    REQUIRE(found2 != nullptr);
    CHECK(*found2 == 200);

    CHECK(coord_hash_replace(hash, c, v3));
    int* found3 = (int*)coord_hash_get(hash, c);
    REQUIRE(found3 != nullptr);
    CHECK(*found3 == 300);

    coord_destroy(c);
    delete v1;
    delete v2;
    delete v3;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: insert and get") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c1 = coord_create_full(2, 3);
    int* v = make_int(42);

    CHECK(coord_hash_insert(hash, c1, v));
    CHECK(coord_hash_length(hash) == 1);

    int* got = (int*)coord_hash_get(hash, c1);
    REQUIRE(got != nullptr);
    CHECK(*got == 42);

    coord_destroy(c1);
    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: contains and remove") {
    coord_hash_t* hash = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c1 = coord_create_full(5, 5);
    coord_t* c2 = coord_create_full(6, 6);
    int* v = make_int(123);

    CHECK_FALSE(coord_hash_contains(hash, c1));
    CHECK(coord_hash_insert(hash, c1, v));
    CHECK(coord_hash_contains(hash, c1));
    CHECK_FALSE(coord_hash_contains(hash, c2));

    CHECK(coord_hash_remove(hash, c1));
    CHECK_FALSE(coord_hash_contains(hash, c1));
    CHECK(coord_hash_length(hash) == 0);

    coord_destroy(c1);
    coord_destroy(c2);
    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: clear and empty") {
    coord_hash_t* hash = coord_hash_create();

    coord_t* c1 = coord_create_full(7, 8);
    coord_t* c2 = coord_create_full(9, 10);

    int* v1 = make_int(11);
    int* v2 = make_int(22);

    CHECK(coord_hash_insert(hash, c1, v1));
    CHECK(coord_hash_insert(hash, c2, v2));
    CHECK(coord_hash_length(hash) == 2);

    coord_hash_clear(hash);
    CHECK(coord_hash_is_empty(hash));
    CHECK(coord_hash_length(hash) == 0);

    coord_destroy(c1);
    coord_destroy(c2);
    delete v1;
    delete v2;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: deep copy") {
    coord_hash_t* a = coord_hash_create_full(int_copy, int_destroy);

    coord_t* c = coord_create_full(1, 2);
    int* v = make_int(55);

    coord_hash_insert(a, c, v);
    coord_hash_t* b = coord_hash_copy(a);

    int* a_val = (int*)coord_hash_get(a, c);
    int* b_val = (int*)coord_hash_get(b, c);

    REQUIRE(a_val);
    REQUIRE(b_val);
    CHECK(*a_val == 55);
    CHECK(*b_val == 55);
    CHECK(a_val != b_val);

    coord_destroy(c);
    delete v;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}

TEST_CASE("coord_hash: equality with null-check and key presence only") {
    coord_hash_t* a = coord_hash_create();
    coord_hash_t* b = coord_hash_create();

    coord_t* c = coord_create_full(1, 1);
    int* v1 = make_int(999);
    int* v2 = make_int(999);

    coord_hash_insert(a, c, v1);
    coord_hash_insert(b, c, v1);
    CHECK(coord_hash_equal(a, b));

    coord_hash_replace(b, c, v2);
    CHECK(coord_hash_equal(a, b));

    coord_hash_replace(b, c, nullptr);
    CHECK_FALSE(coord_hash_equal(a, b));

    coord_hash_replace(a, c, nullptr);
    CHECK(coord_hash_equal(a, b));

    coord_destroy(c);
    delete v1;
    delete v2;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}


TEST_CASE("coord_hash: iteration test") {
    coord_hash_t* hash = coord_hash_create();

    for (int i = 0; i < 10; ++i) {
        coord_t* c = coord_create_full(i, i + 1);
        int* v = make_int(i * 10);
        coord_hash_insert(hash, c, v);
        coord_destroy(c);
        delete v;
    }

    int count = 0;
    coord_hash_foreach(hash, [](const coord_t* key, void* val, void* userdata) {
        int* cnt = (int*)userdata;
        (*cnt)++;
        REQUIRE(key != nullptr);
        REQUIRE(val != nullptr);
        }, &count);

    CHECK(count == 10);
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: insert copies key and value") {
    coord_hash_t* hash = coord_hash_create();

    coord_t* c = coord_create_full(1, 1);
    int* v = new int(123);

    // The table stores the key by value and copies the input value.
    coord_hash_insert(hash, c, v);
    coord_destroy(c);  
    delete v;          

    coord_hash_foreach(hash, [](const coord_t* key, void* val, void* userdata) {
        CHECK(key != nullptr);
        CHECK(val != nullptr);
        }, nullptr);

    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: copied hash owns independent key and value storage") {
    coord_hash_t* a = coord_hash_create();
    coord_t* c = coord_create_full(2, 2);
    int* v = new int(456);

    coord_hash_insert(a, c, v);
    coord_hash_t* b = coord_hash_copy(a);

    coord_destroy(c);

    // Keys are stored by value and copied values are independently owned.
    CHECK(coord_hash_equal(a, b));

    delete v;
    coord_hash_destroy(a);
    coord_hash_destroy(b);
}
TEST_CASE("coord_hash: lookup compares coord keys by value") {
    coord_hash_t* hash = coord_hash_create();

    coord_t c;
    coord_init_full(&c, 7, 7);
    int* v = new int(77);

    coord_hash_insert(hash, &c, v);

    // A distinct object with the same components resolves the stored key.
    coord_t probe;
    coord_init_full(&probe, 7, 7);

    void* val = coord_hash_get(hash, &probe);
    CHECK(*(int*)val == *v);

    delete v;
    coord_hash_destroy(hash);
}

TEST_CASE("coord_hash: lookup remains valid after input key lifetime ends") {
    coord_hash_t* hash = coord_hash_create();
    REQUIRE(hash != nullptr);

    {
        coord_t input_key = {5, 5};
        int input_value = 55;
        CHECK(coord_hash_insert(hash, &input_key, &input_value));
    }

    coord_t probe = {5, 5};
    void* result = coord_hash_get(hash, &probe);
    REQUIRE(result != nullptr);
    CHECK(*static_cast<int*>(result) == 55);

    coord_hash_destroy(hash);
}
