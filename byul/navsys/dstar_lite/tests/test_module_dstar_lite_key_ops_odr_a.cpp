#include "internal/dstar_lite_key_ops.hpp"

bool dstar_lite_key_ops_odr_a(
    const dstar_lite_key_t& lhs,
    const dstar_lite_key_t& rhs) {
    return byul::navsys::dstar_lite_detail::key_less{}(lhs, rhs);
}
