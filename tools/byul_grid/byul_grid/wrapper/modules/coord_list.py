from ffi_core import ffi, C
import weakref

from coord import c_coord

ffi.cdef("""
// Opaque structure definition
typedef struct s_coord_list coord_list_t;

// Creation/Destruction
coord_list_t* coord_list_create(void);
void coord_list_destroy(coord_list_t* list);
coord_list_t* coord_list_copy(const coord_list_t* list);

// Information
int coord_list_length(const coord_list_t* list);
bool coord_list_empty(const coord_list_t* list);
const coord_t* coord_list_get(const coord_list_t* list, int index);
const coord_t* coord_list_front(const coord_list_t* list);
const coord_t* coord_list_back(const coord_list_t* list);

// Modification
int coord_list_push_back(coord_list_t* list, const coord_t* c);

/// @brief Removes and returns the last element of the list (NULL if none)
coord_t coord_list_pop_back(coord_list_t* list);

/// @brief Removes and returns the first element of the list (NULL if none)
coord_t coord_list_pop_front(coord_list_t* list);

int coord_list_insert(coord_list_t* list, int index, const coord_t* c);
void coord_list_remove_at(coord_list_t* list, int index);
void coord_list_remove_value(coord_list_t* list, const coord_t* c);
void coord_list_clear(coord_list_t* list);
void coord_list_reverse(coord_list_t* list);

// Search
int  coord_list_contains(const coord_list_t* list, const coord_t* c);  // returns 1 if contains
int  coord_list_find(const coord_list_t* list, const coord_t* c);      // returns index, -1 if not found

// Sublist extraction
coord_list_t* coord_list_sublist(const coord_list_t* list, int start, int end);

// Comparison
bool coord_list_equals(const coord_list_t* a, const coord_list_t* b);

""")

class c_coord_list:
    def __init__(self, raw_ptr=None, own=False):
        if raw_ptr is not None:
            self._c = raw_ptr
            self._own = own
        else:
            self._c = C.coord_list_create()
            if not self._c:
                raise MemoryError("coord_list allocation failed")
            self._own = True

        if self._own:
            self._finalizer = weakref.finalize(self, C.coord_list_destroy, self._c)
        else:
            self._finalizer = None

    def __len__(self):
        return C.coord_list_length(self._c)

    def __getitem__(self, index):
        if index < 0 or index >= len(self):
            raise IndexError("coord_list index out of range")
        ptr = C.coord_list_get(self._c, index)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def __iter__(self):
        for i in range(len(self)):
            yield self[i]

    def __contains__(self, item):
        return isinstance(item, c_coord) and C.coord_list_contains(self._c, item.ptr()) != 0

    def index(self, item):
        if not isinstance(item, c_coord):
            raise TypeError("index() expects a c_coord object")
        return C.coord_list_find(self._c, item.ptr())

    def front(self):
        ptr = C.coord_list_front(self._c)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def back(self):
        ptr = C.coord_list_back(self._c)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def append(self, coord):
        if not isinstance(coord, c_coord):
            raise TypeError("append expects a c_coord object")
        return C.coord_list_push_back(self._c, coord.ptr())

    def pop(self):
        val = C.coord_list_pop_back(self._c)
        return c_coord(raw_ptr=ffi.new("coord_t *", val))

    def pop_front(self):
        val = C.coord_list_pop_front(self._c)
        return c_coord(raw_ptr=ffi.new("coord_t *", val))

    def insert(self, index, coord):
        if not isinstance(coord, c_coord):
            raise TypeError("insert expects a c_coord object")
        return C.coord_list_insert(self._c, index, coord.ptr())

    def remove_at(self, index):
        C.coord_list_remove_at(self._c, index)

    def remove_value(self, coord):
        if not isinstance(coord, c_coord):
            raise TypeError("remove_value expects a c_coord object")
        C.coord_list_remove_value(self._c, coord.ptr())

    def clear(self):
        C.coord_list_clear(self._c)

    def reverse(self):
        C.coord_list_reverse(self._c)

    def copy(self):
        return c_coord_list(raw_ptr=C.coord_list_copy(self._c), own=True)

    def sublist(self, start, end):
        return c_coord_list(raw_ptr=C.coord_list_sublist(self._c, start, end), own=True)

    def equals(self, other):
        return isinstance(other, c_coord_list) and C.coord_list_equals(self._c, other._c)

    def empty(self):
        return C.coord_list_empty(self._c)

    def ptr(self):
        return self._c

    def __str__(self):
        return "[" + ", ".join(str(c) for c in self) + "]"

    def __repr__(self):
        return f"c_coord_list(len={len(self)})"

    def __del__(self):
        self.close()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def to_list(self):
        return [c.copy() for c in self]

    @classmethod
    def from_list(cls, lst):
        clist = cls()
        for c in lst:
            if not isinstance(c, c_coord):
                raise TypeError("from_list() expects only c_coord elements")
            clist.append(c)
        return clist
