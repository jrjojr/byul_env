#ifndef BYUL_DSTAR_LITE_PLANNER_INTERNAL_HPP
#define BYUL_DSTAR_LITE_PLANNER_INTERNAL_HPP

struct s_dstar_lite;

namespace byul::navsys::internal {
void dstar_lite_planner_forget(s_dstar_lite* planner) noexcept;
}

#endif
