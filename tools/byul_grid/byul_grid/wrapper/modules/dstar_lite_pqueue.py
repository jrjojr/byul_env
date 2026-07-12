from ffi_core import ffi, C

from coord import c_coord
from dstar_lite_key import c_dstar_lite_key

import weakref

ffi.cdef("""
typedef struct s_dstar_lite_pqueue dstar_lite_pqueue_t;

dstar_lite_pqueue_t* dstar_lite_pqueue_create(void);

void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q);

dstar_lite_pqueue_t* dstar_lite_pqueue_copy(const dstar_lite_pqueue_t* src);

void dstar_lite_pqueue_push(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q);

coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q);

bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q);

bool dstar_lite_pqueue_remove(dstar_lite_pqueue_t* q, const coord_t* u);

bool dstar_lite_pqueue_remove_full(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(
    dstar_lite_pqueue_t* q, const coord_t* c);

dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q);

bool dstar_lite_pqueue_contains(
    dstar_lite_pqueue_t* q, const coord_t* u);

""")

class c_dstar_lite_pqueue:
    def __init__(self, raw_ptr=None, own=False):
        if raw_ptr:
            self._c = raw_ptr
            self._own = own
        else:
            self._c = C.dstar_lite_pqueue_create()
            if not self._c:
                raise MemoryError("dstar_lite_pqueue allocation failed")
            self._own = True

        if self._own:
            self._finalizer = weakref.finalize(self, C.dstar_lite_pqueue_destroy, self._c)
        else:
            self._finalizer = None

    def push(self, key: c_dstar_lite_key, coord: c_coord):
        C.dstar_lite_pqueue_push(self._c, key.ptr(), coord.ptr())

    def peek(self):
        ptr = C.dstar_lite_pqueue_peek(self._c)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def pop(self):
        ptr = C.dstar_lite_pqueue_pop(self._c)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def top_key(self):
        ptr = C.dstar_lite_pqueue_top_key(self._c)
        return c_dstar_lite_key(raw_ptr=ptr, own=True) if ptr != ffi.NULL else None

    def find_key_by_coord(self, coord: c_coord):
        ptr = C.dstar_lite_pqueue_get_key_by_coord(self._c, coord.ptr())
        return c_dstar_lite_key(raw_ptr=ptr, own=True) if ptr != ffi.NULL else None

    def remove(self, coord: c_coord):
        return bool(C.dstar_lite_pqueue_remove(self._c, coord.ptr()))

    def remove_full(self, key: c_dstar_lite_key, coord: c_coord):
        return bool(C.dstar_lite_pqueue_remove_full(self._c, key.ptr(), coord.ptr()))

    def contains(self, coord: c_coord):
        return bool(C.dstar_lite_pqueue_contains(self._c, coord.ptr()))

    def is_empty(self):
        return bool(C.dstar_lite_pqueue_is_empty(self._c))

    def ptr(self):
        return self._c

    def __repr__(self):
        return f"c_dstar_lite_pqueue(empty={self.is_empty()})"

    def __del__(self):
        self.close()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def copy(self):
        return c_dstar_lite_pqueue(raw_ptr=C.dstar_lite_pqueue_copy(self._c), own=True)
