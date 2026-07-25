#ifndef DSTAR_LITE_KEY_HPP
#define DSTAR_LITE_KEY_HPP

#include "byul_config.h"
#include "coord.h"
#include "dstar_lite_key.h"

#include <functional>
#include <cstddef>
#include <vector>
#include <map>

struct DstarLiteKeyHash {
    std::size_t operator()(const dstar_lite_key_t* key) const {
        return dstar_lite_key_hash_exact(key);
    }
};

struct DstarLiteKeyEqual {
    bool operator()(const dstar_lite_key_t* a, const dstar_lite_key_t* b) const {
        return dstar_lite_key_equal_exact(a, b);
    }
};

struct DstarLiteKeyLess {
    bool operator()(const dstar_lite_key_t* a, const dstar_lite_key_t* b) const {
        if (a->k1 < b->k1) return true;
        if (a->k1 > b->k1) return false;
        return a->k2 < b->k2;
    }
};

namespace std {
    template<>
    struct hash<dstar_lite_key_t> {
        std::size_t operator()(const dstar_lite_key_t& key) const {
            return static_cast<std::size_t>(dstar_lite_key_hash_exact(&key));
        }
    };
}

struct dstar_lite_key_ptr_less {
    bool operator()(
        const dstar_lite_key_t* a, const dstar_lite_key_t* b) const {
        int result = 0;
        return dstar_lite_key_compare_exact(a, b, &result)
                == NAVSYS_STATUS_OK
            && result < 0;
    }
};

#endif // DSTAR_LITE_KEY_HPP
