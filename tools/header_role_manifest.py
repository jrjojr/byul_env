#!/usr/bin/env python3
"""Build and validate the approved BYUL header-role manifest.

The stage-1 inventory is evidence.  This tool applies the reviewed stage-2
boundary policy without rewriting the immutable stage-1 baseline.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "docs/ko/todo/header-refactor-current"
SOURCE_MANIFEST_NAME = "header-inventory.json"
ROLE_MANIFEST_NAME = "header-role-manifest.json"
INSTALL_REPORT_NAME = "header-install-intent-report.json"

PUBLIC_ROLES = {
    "public-foundation",
    "public-module-entry",
    "public-aggregate",
    "public-component",
}
PRIMARY_ROLES = PUBLIC_ROLES | {
    "compatibility-forwarder",
    "cpp-facade",
    "internal",
    "generated",
    "tool-only",
    "reference",
    "deprecated-reference",
}
WRAPPER_MODES = {"generated", "manual", "excluded"}

DEPRECATED_SINCE = "1.1.0"
REMOVAL_VERSION = "2.0.0"

MATH_ENGINE_SUCCESSORS: tuple[dict[str, Any], ...] = (
    {
        "name": "damping",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-bodyprops-extraction.org",
        "sources": ("byul/balix/bodyprops/bodyprops.h",),
    },
    {
        "name": "environment",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-environ-extraction.org",
        "sources": ("byul/balix/environ/environ.h",),
    },
    {
        "name": "collision_query",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-collision-query-extraction.org",
        "sources": (
            "byul/balix/collision/collision.h",
            "byul/entity/entity_interaction.h",
            "byul/projectile/projectile_predict.h",
            "byul/projectile/projectile_tick.h",
        ),
    },
    {
        "name": "transform",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-transform-extraction.org",
        "sources": ("byul/balix/xform/xform.h",),
    },
    {
        "name": "trajectory_analysis",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-trajectory-analysis-extraction.org",
        "sources": (
            "byul/balix/trajectory/trajectory.h",
            "byul/projectile/projectile_predict.h",
        ),
    },
    {
        "name": "root_solver",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-solver-extraction.org",
        "sources": (
            "byul/balix/numeq/numeq_solver.h",
            "byul/projectile/projectile_core.h",
            "byul/projectile/guidance.h",
        ),
    },
    {
        "name": "integrator",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-integrator-extraction.org",
        "sources": (
            "byul/balix/numeq/numeq_integrator.h",
            "byul/projectile/projectile_predict.h",
        ),
    },
    {
        "name": "physics_model",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-physics-model-extraction.org",
        "sources": (
            "byul/balix/bodyprops/bodyprops.h",
            "byul/balix/numeq/numeq_model.h",
            "byul/entity/entity_dynamic.h",
        ),
    },
    {
        "name": "motion_model",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-motion-model-extraction.org",
        "sources": (
            "byul/balix/numeq/numeq_model_motion.h",
            "byul/entity/entity_dynamic.h",
        ),
    },
    {
        "name": "pid",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-pid-extraction.org",
        "sources": (
            "byul/balix/numeq/numeq_pid.h",
            "byul/balix/numeq/numeq_pid_vec3.h",
        ),
    },
    {
        "name": "mpc",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-mpc-extraction.org",
        "sources": ("byul/balix/numeq/numeq_mpc.h",),
    },
    {
        "name": "estimation",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-kalman-extraction.org",
        "sources": (
            "byul/balix/numeq/filters/numeq_filters.h",
            "byul/balix/numeq/filters/numeq_kalman.h",
        ),
    },
    {
        "name": "motion_query",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-motion-state-extraction.org",
        "sources": (
            "byul/balix/motion_state/motion_state.h",
            "byul/entity/entity_dynamic_coord.h",
        ),
    },
    {
        "name": "control",
        "design_todo": "docs/ko/todo/math-engine/todo-balix-controller-adapter.org",
        "sources": (
            "byul/balix/controller/controller.h",
            "byul/projectile/propulsion.h",
        ),
    },
    {
        "name": "entity_dynamics",
        "design_todo": "docs/ko/todo/math-engine/todo-entity-math-extraction.org",
        "sources": (
            "byul/entity/entity_dynamic.h",
            "byul/entity/entity_dynamic_coord.h",
            "byul/entity/entity_interaction.h",
        ),
    },
    {
        "name": "force_network",
        "design_todo": "docs/ko/todo/math-engine/todo-entity-math-extraction.org",
        "sources": ("byul/entity/entity_spring.h",),
    },
    {
        "name": "projectile_launch",
        "design_todo": "docs/ko/todo/math-engine/todo-projectile-math-extraction.org",
        "sources": (
            "byul/projectile/projectile.h",
            "byul/projectile/projectile_core.h",
        ),
    },
    {
        "name": "guidance",
        "design_todo": "docs/ko/todo/math-engine/todo-projectile-math-extraction.org",
        "sources": (
            "byul/projectile/guidance.h",
            "byul/projectile/projectile_predict.h",
        ),
    },
    {
        "name": "propulsion",
        "design_todo": "docs/ko/todo/math-engine/todo-projectile-math-extraction.org",
        "sources": (
            "byul/projectile/projectile_tick.h",
            "byul/projectile/propulsion.h",
        ),
    },
    {
        "name": "terrain_query",
        "design_todo": "docs/ko/todo/math-engine/todo-ground-geometry-extraction.org",
        "sources": ("byul/ground/ground.h",),
    },
)

SIMULATION_RESOURCE_DESIGN_TODO = (
    "docs/ko/todo/todo-리팩토링-simulation-consumer-headers.org"
)

PUBLIC_RESOURCE_MIGRATIONS: tuple[dict[str, Any], ...] = (
    {
        "resource": "controller_t",
        "source": "byul/balix/controller/controller.h",
        "abi1_additions": (
            "controller_get_type",
            "controller_set_userdata",
            "controller_get_userdata",
        ),
        "abi2_header": "byul/balix/controller/balix_controller.h",
        "abi2_type": "balix_controller_handle_t",
    },
    {
        "resource": "environ_t",
        "source": "byul/balix/environ/environ.h",
        "abi1_additions": (
            "environ_get_gravity",
            "environ_set_gravity",
            "environ_get_wind_velocity",
            "environ_set_wind_velocity",
            "environ_set_callback",
            "environ_get_callback_userdata",
        ),
        "abi2_header": "byul/balix/environ/balix_environment.h",
        "abi2_type": "balix_environment_handle_t",
    },
    {
        "resource": "filter_interface_t",
        "source": "byul/balix/numeq/filters/numeq_filters.h",
        "abi1_additions": (
            "filter_interface_bind",
            "filter_interface_time_update",
            "filter_interface_measurement_update",
            "filter_interface_get_state",
        ),
        "abi2_header": "byul/balix/numeq/filters/balix_filter.h",
        "abi2_type": "balix_filter_handle_t",
    },
    {
        "resource": "integrator_t",
        "source": "byul/balix/numeq/numeq_integrator.h",
        "abi1_additions": (
            "integrator_get_type",
            "integrator_set_type",
            "integrator_get_state",
            "integrator_set_state",
        ),
        "abi2_header": "byul/balix/numeq/balix_integrator.h",
        "abi2_type": "balix_integrator_handle_t",
    },
    {
        "resource": "trajectory_t",
        "source": "byul/balix/trajectory/trajectory.h",
        "abi1_additions": (
            "trajectory_data",
            "trajectory_size",
            "trajectory_capacity",
            "trajectory_reserve",
        ),
        "abi2_header": "byul/balix/trajectory/balix_trajectory.h",
        "abi2_type": "balix_trajectory_handle_t",
    },
    {
        "resource": "projectile_t",
        "source": "byul/projectile/projectile_core.h",
        "abi1_additions": (
            "projectile_set_hit_callback",
            "projectile_get_hit_callback_userdata",
        ),
        "abi2_header": "byul/projectile/projectile_state.h",
        "abi2_type": "projectile_state_handle_t",
    },
    {
        "resource": "projectile_result_t",
        "source": "byul/projectile/projectile_predict.h",
        "abi1_additions": (
            "projectile_result_get_trajectory",
            "projectile_result_detach_trajectory",
            "projectile_result_reset",
        ),
        "abi2_header": "byul/projectile/projectile_result.h",
        "abi2_type": "projectile_result_handle_t",
    },
    {
        "resource": "projectile_tick_t",
        "source": "byul/projectile/projectile_tick.h",
        "abi1_additions": (
            "projectile_tick_bind_environment",
            "projectile_tick_bind_ground",
            "projectile_tick_bind_propulsion",
            "projectile_tick_set_guidance",
            "projectile_tick_get_trajectory",
        ),
        "abi2_header": "byul/projectile/projectile_tick_resource.h",
        "abi2_type": "projectile_tick_handle_t",
    },
    {
        "resource": "propulsion_t",
        "source": "byul/projectile/propulsion.h",
        "abi1_additions": (
            "propulsion_bind_controller",
            "propulsion_get_controller",
        ),
        "abi2_header": "byul/projectile/projectile_propulsion.h",
        "abi2_type": "projectile_propulsion_handle_t",
    },
    {
        "resource": "ground_t",
        "source": "byul/ground/ground.h",
        "abi1_additions": (
            "ground_get_mode",
            "ground_set_tile_mapper",
            "ground_get_heightfield_view",
            "ground_set_heightfield_copy",
        ),
        "abi2_header": "byul/ground/ground_resource.h",
        "abi2_type": "ground_handle_t",
    },
)


ROLE_OVERRIDES: dict[str, tuple[str, str | None, str | None, str]] = {
    "byul/aerial/aerial.h": (
        "deprecated-reference",
        "aerial",
        None,
        "Header declarations and aerial.cpp definitions do not match; the module is absent from the root shared/install graph.",
    ),
    "byul/balix/bodyprops/bodyprops.h": (
        "public-component",
        "balix",
        "balix",
        "The word generated describes a physical effect; this header is handwritten public ABI.",
    ),
    "byul/balix/numeq/numeq.h": (
        "public-aggregate",
        "balix",
        "numeq",
        "Include-only aggregate for the stable numeq public module boundary.",
    ),
    "byul/byul_config.h": (
        "public-foundation",
        "byul",
        "byul",
        "Root export, platform and version foundation shared by installed headers.",
    ),
    "byul/entity/entity_avoidance.h": (
        "deprecated-reference",
        "entity",
        None,
        "Experimental declarations have no production definition, CMake inventory or installed ABI.",
    ),
    "byul/entity/entity_encirclement.h": (
        "deprecated-reference",
        "entity",
        None,
        "Experimental declarations have no production definition, CMake inventory or installed ABI.",
    ),
    "byul/navsys/dstar_lite/dstar_lite_key.hpp": (
        "compatibility-forwarder",
        "navsys",
        "dstar_lite",
        "Installed C++ implementation helpers move behind the D* Lite internal boundary; the old include path remains through ABI 1.x.",
    ),
    "byul/navsys/maze/maze.h": (
        "public-aggregate",
        "navsys",
        "maze",
        "Canonical maze aggregate above maze_core and algorithm components.",
    ),
    "byul/navsys/route_carver/route_carver.h": (
        "public-module-entry",
        "navsys",
        "route_carver",
        "Canonical entry for the route-carver public module.",
    ),
    "byul/navsys/route_finder/route_finder.h": (
        "public-module-entry",
        "navsys",
        "route_finder",
        "Canonical entry for route-finder dispatch and configuration.",
    ),
    "byul/navsys/route_finder/route_finder_core.h": (
        "public-component",
        "navsys",
        "route_finder",
        "Public cost and heuristic callback/evaluation component, not a shared type foundation.",
    ),
    "byul/numal/dualquat.hpp": (
        "cpp-facade",
        "numal",
        "numal",
        "Installed C++17 source facade retained separately from the stable C ABI.",
    ),
    "byul/numal/quat.hpp": (
        "cpp-facade",
        "numal",
        "numal",
        "Installed C++17 source facade retained separately from the stable C ABI.",
    ),
    "byul/numal/vec3.hpp": (
        "cpp-facade",
        "numal",
        "numal",
        "Installed C++17 source facade retained separately from the stable C ABI.",
    ),
    "byul/number_theory/byul_base_primes_65535.h": (
        "generated",
        "number_theory",
        None,
        "Generator-owned implementation table; never part of the SDK or wrapper surface.",
    ),
    "byul/number_theory/byul_compiler.h": (
        "deprecated-reference",
        "number_theory",
        None,
        "Unreferenced compiler macro experiment with no build, install or export consumer.",
    ),
}

for _maze_name in (
    "maze_aldous_broder.h",
    "maze_binary.h",
    "maze_eller.h",
    "maze_hunt_and_kill.h",
    "maze_kruskal.h",
    "maze_prim.h",
    "maze_recursive.h",
    "maze_recursive_division.h",
    "maze_room_blend.h",
    "maze_sidewinder.h",
    "maze_wilson.h",
):
    ROLE_OVERRIDES[f"byul/navsys/maze/{_maze_name}"] = (
        "public-component",
        "navsys",
        "maze",
        "Handwritten public maze algorithm component; generator wording is not file provenance.",
    )

ROLE_OVERRIDES.update(
    {
        "byul/rng/rng.h": (
            "public-module-entry",
            "rng",
            "rng",
            "Approved Math Engine RNG module entry; current root/install omission is a tracked implementation gap.",
        ),
        "byul/rng/rng_core.h": (
            "public-foundation",
            "rng",
            "rng",
            "Deterministic state, stream and uniform primitives shared by RNG components.",
        ),
        "byul/rng/rng_fill.h": (
            "public-component",
            "rng",
            "rng",
            "Typed caller-buffer fill API in the approved RNG public module.",
        ),
        "byul/rng/distributions.h": (
            "public-component",
            "rng",
            "rng",
            "Distribution sampling component; pure probability functions remain a separate Mathfunc responsibility.",
        ),
        "byul/rng/roll.h": (
            "public-component",
            "rng",
            "rng",
            "Discrete and weighted sampling component.",
        ),
        "byul/rng/shuffle.h": (
            "public-component",
            "rng",
            "rng",
            "Permutation component using caller-owned RNG state.",
        ),
    }
)


def forward(
    target: str, reason: str, *, canonical_install: bool = True
) -> dict[str, Any]:
    return {
        "disposition": "forward",
        "canonical_path": target,
        "canonical_install": canonical_install,
        "deprecated_since": DEPRECATED_SINCE,
        "remove_in": REMOVAL_VERSION,
        "reason": reason,
    }


NAMING_OVERRIDES: dict[str, dict[str, Any]] = {
    "byul/balix/collision/collision.h": forward(
        "byul/balix/collision/balix_collision.h",
        "The flat SDK component needs its stable balix module boundary.",
    ),
    "byul/console/dstar_lite_console.h": forward(
        "byul/console/console_dstar_lite.h",
        "The diagnostic adapter is owned by console; the old basename remains a compatibility include.",
    ),
    "byul/navsys/dstar_lite/dstar_lite_key.hpp": forward(
        "byul/navsys/dstar_lite/internal/dstar_lite_key_ops.hpp",
        "C++ ordering/hash helpers are implementation-only; retain the installed legacy include through ABI 1.x.",
        canonical_install=False,
    ),
    "byul/navsys/route_finder/route_finder_core.h": forward(
        "byul/navsys/route_finder/route_finder_evaluation.h",
        "The header owns cost/heuristic callbacks and defaults, not a public core resource.",
    ),
    "byul/numal/scalar.h": forward(
        "byul/numal/numal_scalar.h",
        "Generic scalar.h is ambiguous in the flat SDK and must expose the stable numal boundary.",
    ),
    "byul/number_theory/byul_prime32.h": forward(
        "byul/number_theory/number_theory_prime_u32.h",
        "The project prefix is already represented by include/byul; use the number_theory module boundary.",
    ),
    "byul/number_theory/byul_prime_segmented_u32.h": forward(
        "byul/number_theory/number_theory_prime_segmented_u32.h",
        "The project prefix is already represented by include/byul; use the number_theory module boundary.",
    ),
    "byul/number_theory/byul_base_primes_65535.h": {
        "disposition": "generated-regenerate",
        "canonical_path": "byul/number_theory/internal/base_primes_65535.h",
        "canonical_install": False,
        "deprecated_since": None,
        "remove_in": None,
        "reason": "Move the generator output, template reference and consuming source together behind the internal boundary.",
    },
    "byul/rng/distributions.h": forward(
        "byul/rng/rng_distributions.h",
        "Generic distributions.h needs the approved RNG module boundary.",
    ),
    "byul/rng/roll.h": forward(
        "byul/rng/rng_roll.h",
        "Generic roll.h needs the approved RNG module boundary.",
    ),
    "byul/rng/shuffle.h": forward(
        "byul/rng/rng_shuffle.h",
        "Generic shuffle.h needs the approved RNG module boundary.",
    ),
}

for _algorithm in (
    "astar",
    "bfs",
    "dfs",
    "dijkstra",
    "fast_marching",
    "fringe_search",
    "greedy_best_first",
    "ida_star",
    "rta_star",
    "sma_star",
    "weighted_astar",
):
    _path = f"byul/navsys/route_finder/{_algorithm}.h"
    NAMING_OVERRIDES[_path] = forward(
        f"byul/navsys/route_finder/route_finder_{_algorithm}.h",
        "Algorithm components require the route_finder module boundary in the flat SDK.",
    )


KEEP_FALSE_POSITIVES = {
    "byul/balix/numeq/numeq.h",
    "byul/balix/numeq/numeq_model.h",
    "byul/balix/numeq/numeq_model_motion.h",
    "byul/balix/numeq/numeq_solver.h",
    "byul/byul_config.h",
    "byul/navsys/maze/maze.h",
    "byul/navsys/route_carver/route_carver.h",
    "byul/numal/dualnumber.h",
}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    fd, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def default_role(row: dict[str, Any]) -> tuple[str, str, str | None, str]:
    path = row["current_path"]
    candidate = row["role_candidate"]
    if path.startswith("tools/gpu_comp_tester/"):
        return (
            "tool-only",
            "gpu_comp_tester",
            None,
            "Private executable-tool header; excluded from the BYUL SDK.",
        )
    if path.startswith("tools/unit_1003/deprecated/"):
        return (
            "deprecated-reference",
            "unit_1003",
            None,
            "Immutable deprecated reference with original provenance and license.",
        )
    if path.startswith("tools/unit_1003/"):
        return (
            "reference",
            "unit_1003",
            None,
            "Regression/requirements reference; not a BYUL-owned public ABI.",
        )
    mapping = {
        "public-module-entry": "public-module-entry",
        "public-component": "public-component",
        "private-implementation": "internal",
    }
    if candidate not in mapping:
        raise ValueError(f"{path}: role candidate {candidate!r} needs an explicit approval")
    return (
        mapping[candidate],
        row["current_owner"],
        row["public_module_boundary_candidate"],
        "Stage-1 evidence supports the current public or internal boundary.",
    )


def naming_decision(row: dict[str, Any]) -> dict[str, Any]:
    path = row["current_path"]
    if path in NAMING_OVERRIDES:
        result = dict(NAMING_OVERRIDES[path])
    elif path.startswith("tools/"):
        result = {
            "disposition": "keep",
            "canonical_path": path,
            "canonical_install": True,
            "deprecated_since": None,
            "remove_in": None,
            "reason": "Tool/reference paths retain their owning corpus layout and never enter the flat SDK.",
        }
    elif row["naming_disposition"] == "keep" or path in KEEP_FALSE_POSITIVES:
        result = {
            "disposition": "keep",
            "canonical_path": path,
            "canonical_install": True,
            "deprecated_since": None,
            "remove_in": None,
            "reason": "Approved stable module/responsibility name or documented foundation/aggregate exception.",
        }
    elif path == "byul/navsys/coord/internal/coord_ops.hpp":
        result = {
            "disposition": "keep",
            "canonical_path": path,
            "canonical_install": False,
            "deprecated_since": None,
            "remove_in": None,
            "reason": "Already migrated to the canonical internal implementation path.",
        }
    elif row["naming_disposition"] == "generated-regenerate":
        # All false-positive generated candidates are explicitly reclassified.
        result = {
            "disposition": "keep",
            "canonical_path": path,
            "canonical_install": True,
            "deprecated_since": None,
            "remove_in": None,
            "reason": "Handwritten public component; generator wording was a stage-1 heuristic false positive.",
        }
    elif row["naming_disposition"] == "decision-required":
        result = {
            "disposition": "keep",
            "canonical_path": path,
            "canonical_install": True,
            "deprecated_since": None,
            "remove_in": None,
            "reason": "Explicit stage-2 role decision makes the current non-SDK or C++ facade path terminal.",
        }
    else:
        raise ValueError(
            f"{path}: naming candidate {row['naming_disposition']!r} needs an explicit approval"
        )
    if result["disposition"] == "forward":
        result["compatibility_path"] = path
    else:
        result["compatibility_path"] = None
    return result


def wrapper_decision(row: dict[str, Any], role: str) -> dict[str, str]:
    path = row["current_path"]
    if role == "cpp-facade":
        return {
            "mode": "excluded",
            "reason": "CFFI consumes the C ABI; this is a separately supported C++17 source facade.",
        }
    if role == "compatibility-forwarder":
        return {
            "mode": "excluded",
            "reason": "C++ compatibility include has no independent C wrapper surface.",
        }
    if role not in PUBLIC_ROLES:
        return {
            "mode": "excluded",
            "reason": f"{role} headers are outside the installed C ABI wrapper input.",
        }
    if path.startswith("byul/rng/"):
        return {
            "mode": "manual",
            "reason": "Approved future public RNG ABI; export/status metadata and generated support are completed in the RNG design TODO.",
        }
    if row["wrapper_modules_registered"]:
        return {
            "mode": "generated",
            "reason": "Registered in the canonical wrapper generator manifest.",
        }
    if row["exported_symbols"]:
        return {
            "mode": "manual",
            "reason": "Public legacy declarations require explicit ownership/error metadata before automatic generation.",
        }
    return {
        "mode": "excluded",
        "reason": "Include-only aggregate/foundation with no independent exported function declarations.",
    }


def approved_install(role: str, row: dict[str, Any]) -> tuple[bool, str]:
    if role in PUBLIC_ROLES or role in {"cpp-facade", "compatibility-forwarder"}:
        if row["current_path"].startswith("byul/rng/"):
            return (
                True,
                "Approved public RNG boundary; current install omission is an implementation gap.",
            )
        return True, "Public or compatibility SDK surface."
    return False, f"{role} is excluded from the SDK install inventory."


def find_include_cycles(rows: list[dict[str, Any]]) -> tuple[int, list[list[str]]]:
    paths = {row["current_path"] for row in rows}
    edges: set[tuple[str, str]] = set()
    for included in rows:
        for consumer in included.get("umbrella_consumers", []):
            if consumer in paths:
                edges.add((consumer, included["current_path"]))
    graph: dict[str, list[str]] = {path: [] for path in paths}
    for source, target in edges:
        graph[source].append(target)

    state: dict[str, int] = {}
    stack: list[str] = []
    cycles: list[list[str]] = []

    def visit(node: str) -> None:
        state[node] = 1
        stack.append(node)
        for target in graph[node]:
            if state.get(target, 0) == 0:
                visit(target)
            elif state.get(target) == 1:
                start = stack.index(target)
                cycle = stack[start:] + [target]
                if cycle not in cycles:
                    cycles.append(cycle)
        stack.pop()
        state[node] = 2

    for node in sorted(graph):
        if state.get(node, 0) == 0:
            visit(node)
    return len(edges), cycles


def build_outputs(source: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    approved_rows: list[dict[str, Any]] = []
    source_rows = source["headers"]
    source_paths = {row["current_path"] for row in source_rows}
    successor_rows: list[dict[str, Any]] = []
    successor_links: dict[str, list[str]] = {path: [] for path in source_paths}

    for successor in MATH_ENGINE_SUCCESSORS:
        canonical_path = (
            f"byul/math_engine/math_engine_{successor['name']}.h"
        )
        asset_id = f"created:{canonical_path}"
        missing_sources = set(successor["sources"]) - source_paths
        if missing_sources:
            raise ValueError(
                f"{asset_id}: unknown source assets {sorted(missing_sources)}"
            )
        design_todo = REPOSITORY_ROOT / successor["design_todo"]
        if not design_todo.is_file():
            raise ValueError(f"{asset_id}: missing design TODO {successor['design_todo']}")
        for source_path in successor["sources"]:
            successor_links[source_path].append(asset_id)
        successor_rows.append(
            {
                "asset_id": asset_id,
                "current_path": None,
                "canonical_path": canonical_path,
                "primary_role": "public-component",
                "owner_module": "math_engine",
                "public_module": "math_engine",
                "approved_install": True,
                "install_target": "byul_math_engine",
                "export_target": "byul_math_engine",
                "wrapper": {
                    "mode": "manual",
                    "reason": "Created C17 ABI requires approved export/status metadata before generator registration.",
                },
                "design_todo": successor["design_todo"],
                "source_assets": [
                    f"initial:{source_path}" for source_path in successor["sources"]
                ],
                "symbol_prefix": "byul_math_",
                "decision_status": "approved-design",
            }
        )

    successor_paths = [row["canonical_path"] for row in successor_rows]
    if len(successor_paths) != len(set(successor_paths)):
        raise ValueError("created successor canonical paths must be unique")

    resource_migrations: list[dict[str, Any]] = []
    resource_names: set[str] = set()
    abi2_headers: set[str] = set()
    abi2_types: set[str] = set()
    for migration in PUBLIC_RESOURCE_MIGRATIONS:
        resource = migration["resource"]
        source_path = migration["source"]
        abi2_header = migration["abi2_header"]
        abi2_type = migration["abi2_type"]
        if source_path not in source_paths:
            raise ValueError(f"{resource}: unknown source header {source_path}")
        if resource in resource_names:
            raise ValueError(f"duplicate resource migration: {resource}")
        if abi2_header in abi2_headers or abi2_type in abi2_types:
            raise ValueError(f"{resource}: duplicate ABI-2 header or type")
        additions = list(migration["abi1_additions"])
        if not additions or len(additions) != len(set(additions)):
            raise ValueError(f"{resource}: invalid ABI-1 additive API")
        resource_names.add(resource)
        abi2_headers.add(abi2_header)
        abi2_types.add(abi2_type)
        resource_migrations.append(
            {
                "resource": resource,
                "source_asset": f"initial:{source_path}",
                "design_owner": "simulation_consumer",
                "design_todo": SIMULATION_RESOURCE_DESIGN_TODO,
                "abi_1_x": {
                    "layout": "preserve",
                    "existing_symbols": "preserve",
                    "additive_api": additions,
                    "deprecation": "direct field access is deprecated only after every supported operation has an accessor",
                },
                "abi_2_0": {
                    "canonical_header": abi2_header,
                    "opaque_type": abi2_type,
                    "transition": "replace the exposed layout only at the explicit ABI-major 2.0 gate",
                },
                "compatibility_conditions": [
                    "ABI-1 layout fingerprint remains unchanged",
                    "old direct-layout and new accessor consumers pass together",
                    "ownership, borrow, retain, copy, invalidation and destroy behavior is documented and tested",
                    "ABI-2 removal is separately approved after migration documentation is published",
                ],
                "decision_status": "approved-design",
            }
        )

    for row in source_rows:
        path = row["current_path"]
        if path in ROLE_OVERRIDES:
            role, owner, module, role_reason = ROLE_OVERRIDES[path]
        else:
            role, owner, module, role_reason = default_role(row)
        if role not in PRIMARY_ROLES:
            raise ValueError(f"{path}: invalid primary role {role!r}")

        naming = naming_decision(row)
        install, install_reason = approved_install(role, row)
        wrapper = wrapper_decision(row, role)
        if wrapper["mode"] not in WRAPPER_MODES or not wrapper["reason"]:
            raise ValueError(f"{path}: incomplete wrapper decision")

        approved_rows.append(
            {
                "asset_id": row["asset_id"],
                "current_path": path,
                "sha256": row["sha256"],
                "primary_role": role,
                "role_reason": role_reason,
                "owner_module": owner,
                "public_module": module,
                "approved_install": install,
                "install_reason": install_reason,
                "wrapper": wrapper,
                "naming": naming,
                "successor_headers": sorted(successor_links[path]),
                "source_evidence": {
                    "module_header_groups": row["module_header_groups"],
                    "root_header_inventory": row["root_header_inventory"],
                    "install_inventory": row["install_inventory"],
                    "wrapper_modules_registered": row[
                        "wrapper_modules_registered"
                    ],
                    "exported_symbols": row["exported_symbols"],
                    "source_definition_candidates": row[
                        "source_definition_candidates"
                    ],
                },
                "decision_status": "approved",
            }
        )

    paths = [row["current_path"] for row in approved_rows]
    if len(paths) != len(set(paths)) or len(paths) != 135:
        raise ValueError("approved manifest must contain exactly 135 unique current paths")

    intended_paths: list[str] = []
    for row in approved_rows:
        if not row["approved_install"]:
            continue
        if row["naming"]["canonical_install"]:
            intended_paths.append(row["naming"]["canonical_path"])
        if row["naming"]["compatibility_path"]:
            intended_paths.append(row["naming"]["compatibility_path"])
    intended_paths = sorted(set(intended_paths))
    intended_paths = sorted(set(intended_paths) | set(successor_paths))
    basenames: dict[str, list[str]] = {}
    for path in intended_paths:
        basenames.setdefault(Path(path).name, []).append(path)
    collisions = {
        name: entries for name, entries in basenames.items() if len(entries) > 1
    }
    if collisions:
        raise ValueError(f"approved SDK basename collision: {collisions}")
    future_paths = intended_paths + sorted(abi2_headers)
    future_basenames: dict[str, list[str]] = {}
    for path in future_paths:
        future_basenames.setdefault(Path(path).name, []).append(path)
    future_collisions = {
        name: entries
        for name, entries in future_basenames.items()
        if len(entries) > 1
    }
    if future_collisions:
        raise ValueError(
            f"approved ABI-2 SDK basename collision: {future_collisions}"
        )

    edge_count, cycles = find_include_cycles(source_rows)
    if cycles:
        raise ValueError(f"approved current include graph contains cycles: {cycles}")

    current_install = sorted(
        row["current_path"] for row in source_rows if row["install_inventory"]
    )
    current_set = set(current_install)
    intended_set = set(intended_paths)
    report = {
        "schema_version": 1,
        "source_manifest": SOURCE_MANIFEST_NAME,
        "current_install_paths": current_install,
        "approved_intent_paths": intended_paths,
        "additions": sorted(intended_set - current_set),
        "retained": sorted(intended_set & current_set),
        "removals": sorted(current_set - intended_set),
        "counts": {
            "current": len(current_set),
            "approved_intent": len(intended_set),
            "additions": len(intended_set - current_set),
            "retained": len(intended_set & current_set),
            "removals": len(current_set - intended_set),
        },
    }

    manifest = {
        "schema_version": 1,
        "approval_stage": 2,
        "approval_status": "approved-stage-2-boundaries",
        "source_manifest": SOURCE_MANIFEST_NAME,
        "compatibility_policy": {
            "deprecated_since": DEPRECATED_SINCE,
            "remove_in": REMOVAL_VERSION,
            "removal_conditions": [
                "canonical header and legacy forwarding consumer fixtures pass",
                "release migration documentation is published",
                "ABI-major 2.0 removal gate is explicitly approved",
            ],
        },
        "summary": {
            "headers": len(approved_rows),
            "approved": sum(
                row["decision_status"] == "approved" for row in approved_rows
            ),
            "approved_install": sum(
                row["approved_install"] for row in approved_rows
            ),
            "wrapper_generated": sum(
                row["wrapper"]["mode"] == "generated" for row in approved_rows
            ),
            "wrapper_manual": sum(
                row["wrapper"]["mode"] == "manual" for row in approved_rows
            ),
            "wrapper_excluded": sum(
                row["wrapper"]["mode"] == "excluded" for row in approved_rows
            ),
            "compatibility_forwarders": sum(
                row["naming"]["disposition"] == "forward"
                for row in approved_rows
            ),
            "successor_headers": len(successor_rows),
            "successor_links": sum(
                len(row["successor_headers"]) for row in approved_rows
            ),
            "public_resource_migrations": len(resource_migrations),
            "sdk_basename_collisions": len(collisions),
            "abi_2_sdk_basename_collisions": len(future_collisions),
            "current_include_edges": edge_count,
            "current_include_cycles": len(cycles),
        },
        "remaining_stage_2_design_gate": [],
        "created_successors": successor_rows,
        "public_resource_migrations": resource_migrations,
        "headers": approved_rows,
    }
    return manifest, report


def compare_or_write(
    path: Path, expected: dict[str, Any], apply: bool, label: str
) -> bool:
    if apply:
        atomic_write_json(path, expected)
        print(f"[WRITTEN] {path.relative_to(REPOSITORY_ROOT)}")
        return True
    if not path.exists():
        print(f"[STALE] missing {label}: {path.relative_to(REPOSITORY_ROOT)}")
        return False
    actual = load_json(path)
    if actual != expected:
        print(f"[STALE] {label} differs: {path.relative_to(REPOSITORY_ROOT)}")
        return False
    print(f"[OK] {label}: {path.relative_to(REPOSITORY_ROOT)}")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate/check the approved BYUL stage-2 header role manifest."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true", help="write generated JSON")
    mode.add_argument("--check", action="store_true", help="check generated JSON")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="directory containing the stage-1 inventory and stage-2 outputs",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir
    if not output_dir.is_absolute():
        output_dir = REPOSITORY_ROOT / output_dir
    source = load_json(output_dir / SOURCE_MANIFEST_NAME)
    manifest, report = build_outputs(source)
    print(
        "[SUMMARY] "
        f"headers={manifest['summary']['headers']} "
        f"approved={manifest['summary']['approved']} "
        f"install={manifest['summary']['approved_install']} "
        f"forwarders={manifest['summary']['compatibility_forwarders']} "
        f"remaining-gates={len(manifest['remaining_stage_2_design_gate'])}"
    )
    apply = args.apply
    ok_manifest = compare_or_write(
        output_dir / ROLE_MANIFEST_NAME, manifest, apply, "role manifest"
    )
    ok_report = compare_or_write(
        output_dir / INSTALL_REPORT_NAME, report, apply, "install intent report"
    )
    return 0 if ok_manifest and ok_report else 1


if __name__ == "__main__":
    raise SystemExit(main())
