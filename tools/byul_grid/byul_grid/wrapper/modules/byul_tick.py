from ffi_core import ffi, C

import weakref

ffi.cdef("""

/**
 * @brief Function pointer type for tick callbacks.
 * Each tick function is called with its registered context and delta time.
 */
typedef void (*tick_func)(void* context, float dt);

/**
 * @brief Entry in the tick list.
 */
typedef struct s_tick_entry {
    tick_func func;       ///< The function to call.
    void* context;        ///< The context to pass to the function.
} tick_entry_t;

/**
 * @brief Opaque tick system that manages all registered tick functions.
 */
typedef struct s_tick tick_t;

tick_t* tick_create();

void tick_destroy(tick_t* tick);

/**
 * @brief Calls all registered tick functions with the given delta time.
 * @param tick Tick system handle
 * @param dt Delta time (in seconds)
 */
void tick_update(tick_t* tick, float dt);

/**
 * @brief Registers a tick function with an associated context.
 * @param tick Tick system handle
 * @param func Tick function to call each frame
 * @param context The context passed to the function on each call
 * @return 0 on success, non-zero on duplicate or error
 */
int tick_attach(tick_t* tick, tick_func func, void* context);

/**
 * @brief Unregisters a specific tick function and context pair.
 * @param tick Tick system handle
 * @param func Function to detach
 * @param context Context that was originally registered with the function
 * @return 0 on success, non-zero if not found
 */
int tick_detach(tick_t* tick, tick_func func, void* context);

/**
 * @brief Lists all currently attached tick functions.
 * @param tick Tick system handle
 * @param out Array to store tick_entry_t entries
 * @param max_count Maximum number of entries to write
 * @return Actual number of entries written
 */
int tick_list_attached(tick_t* tick, tick_entry_t* out, int max_count);

""")

class c_tick:
    def __init__(self):
        self._c = C.tick_create()
        if self._c == ffi.NULL:
            raise MemoryError("tick_create() failed")
        self._finalizer = weakref.finalize(self, C.tick_destroy, self._c)
        self._callbacks = []

    def attach(self, pyfunc, context=None):
        """
        pyfunc: Python function with signature (ctx: any, dt: float)
        context: Optional Python object passed to callback
        """
        # Wrap context in ffi.new_handle
        py_ctx = ffi.new_handle(context)

        @ffi.callback("void(void*, float)")
        def c_func(c_ctx, dt):
            user_ctx = ffi.from_handle(c_ctx)
            pyfunc(user_ctx, dt)

        # Register C-side callback
        result = C.tick_attach(self._c, c_func, py_ctx)
        if result != 0:
            raise RuntimeError("tick_attach failed (maybe duplicate?)")

        # Keep references to prevent GC
        self._callbacks.append((c_func, py_ctx))

    def detach(self, pyfunc=None, context=None):
        """
        Removes a callback if it matches.
        """
        for (c_func, py_ctx) in self._callbacks:
            user_ctx = ffi.from_handle(py_ctx)
            if user_ctx == context:
                C.tick_detach(self._c, c_func, py_ctx)
                self._callbacks.remove((c_func, py_ctx))
                return
        raise RuntimeError("tick_detach failed (not found)")

    def update(self, dt: float):
        C.tick_update(self._c, dt)

    def list_attached(self, max_count=64):
        buf = ffi.new("tick_entry_t[]", max_count)
        count = C.tick_list_attached(self._c, buf, max_count)
        result = []
        for i in range(count):
            result.append({
                "func": buf[i].func,
                "context": ffi.from_handle(buf[i].context)
            })
        return result

    def close(self):
        if self._finalizer.alive:
            self._finalizer()
        self._callbacks.clear()

    def __del__(self):
        self.close()
