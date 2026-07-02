"""
Manual timing breakdown of DUE solve() cpp=True path.
Reproduces the solve() loop externally with per-step timing.
"""

import time
import random
import warnings
import numpy as np
from collections import defaultdict

import uxsim
from uxsim import World
from uxsim.Utilities import enumerate_k_random_routes
from uxsim.DTAsolvers.DTAsolvers import (
    _make_cpp_func_world,
    _build_name_id_maps,
    _convert_od_routes_to_ids,
    _build_enforce_routes_input,
    _convert_cpp_result_to_names,
)

MAX_ITER = 20
SEED = 0
N_ROUTES = 10
SWAP_PROB = 0.05


def _create_due_world(seed, cpp):
    """9x9 grid scenario (same as benchtestdta.py)."""
    W = uxsim.World(
        name="", deltan=10, tmax=4800,
        duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=0,
        random_seed=seed, cpp=cpp,
    )
    imax, jmax = 9, 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i, j] = W.addNode(f"n{(i, j)}", i, j, flow_capacity=1.6)
    for i in range(imax):
        for j in range(jmax):
            free_flow_speed = 10
            if i != imax - 1:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i+1, j)}", nodes[i, j], nodes[i+1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if i != 0:
                if j == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i-1, j)}", nodes[i, j], nodes[i-1, j],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != jmax - 1:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j+1)}", nodes[i, j], nodes[i, j+1],
                           length=1000, free_flow_speed=free_flow_speed)
            free_flow_speed = 10
            if j != 0:
                if i == id_center:
                    free_flow_speed = 20
                W.addLink(f"l{(i, j, i, j-1)}", nodes[i, j], nodes[i, j-1],
                           length=1000, free_flow_speed=free_flow_speed)

    demand_flow = 0.08
    demand_duration = 2400
    outer_ids = 3
    for n1 in [(0, j) for j in range(outer_ids, jmax - outer_ids)]:
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
    for n1 in [(i, 0) for i in range(outer_ids, imax - outer_ids)]:
        for n2 in [(i, jmax - 1) for i in range(outer_ids, imax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)
        for n2 in [(imax - 1, j) for j in range(outer_ids, jmax - outer_ids)]:
            W.adddemand(nodes[n1], nodes[n2], 0, demand_duration, demand_flow)

    return W


if __name__ == "__main__":
    output_lines = []
    def log(msg=""):
        print(msg)
        output_lines.append(msg)

    log("=" * 70)
    log(f"DUE Timing Breakdown (cpp=True, max_iter={MAX_ITER}, seed={SEED})")
    log("=" * 70)

    # ---- Pre-loop setup ----
    base_func_World = lambda: _create_due_world(SEED, cpp=False)
    func_World = _make_cpp_func_world(base_func_World)

    t0 = time.perf_counter()
    W_orig = func_World()
    t_world_orig = time.perf_counter() - t0
    log(f"\n[Pre-loop] World_orig creation: {t_world_orig:.4f}s")

    dict_od_to_vehid = defaultdict(lambda: [])
    for key, veh in W_orig.VEHICLES.items():
        o = veh.orig.name
        d = veh.dest.name
        dict_od_to_vehid[o, d].append(key)

    t0 = time.perf_counter()
    if W_orig.finalized == False:
        W_orig.finalize_scenario()
    t_finalize_orig = time.perf_counter() - t0
    log(f"[Pre-loop] finalize_scenario (orig): {t_finalize_orig:.4f}s")

    t0 = time.perf_counter()
    dict_od_to_routes = enumerate_k_random_routes(W_orig, k=N_ROUTES)
    t_enum_routes = time.perf_counter() - t0
    log(f"[Pre-loop] enumerate_k_random_routes: {t_enum_routes:.4f}s")
    log(f"  OD pairs: {len(dict_od_to_routes)}, total routes: {sum(len(v) for v in dict_od_to_routes.values())}")

    # ---- Per-iteration timings ----
    step_names = [
        "func_World",
        "build_name_id_maps",
        "convert_od_routes_to_ids",
        "build_enforce_input",
        "batch_enforce_routes",
        "exec_simulation",
        "analyzer_stats",
        "route_swap_due_cpp",
        "convert_result_to_names",
        "misc_logging",
    ]
    timings = {name: [] for name in step_names}
    iter_totals = []

    routes_specified_data = None

    log(f"\n{'iter':>4} | " + " | ".join(f"{n[:12]:>12}" for n in step_names) + " | total")
    log("-" * (7 + 15 * len(step_names) + 10))

    for i in range(MAX_ITER):
        t_iter_start = time.perf_counter()
        step_times = {}

        # 1. func_World()
        t0 = time.perf_counter()
        W = func_World()
        if i != MAX_ITER - 1:
            W.vehicle_logging_timestep_interval = -1
        step_times["func_World"] = time.perf_counter() - t0

        # 2. build_name_id_maps
        t0 = time.perf_counter()
        link_name_to_id, link_id_to_name, node_name_to_id = _build_name_id_maps(W)
        step_times["build_name_id_maps"] = time.perf_counter() - t0

        # 3. convert_od_routes_to_ids
        t0 = time.perf_counter()
        od_route_sets_ids = _convert_od_routes_to_ids(dict_od_to_routes, node_name_to_id, link_name_to_id)
        step_times["convert_od_routes_to_ids"] = time.perf_counter() - t0

        # 4+5. batch_enforce_routes (including _build_enforce_routes_input)
        if i != 0 and routes_specified_data is not None:
            t0 = time.perf_counter()
            enforce_input = _build_enforce_routes_input(W, routes_specified_data, link_name_to_id)
            step_times["build_enforce_input"] = time.perf_counter() - t0

            t0 = time.perf_counter()
            W._cpp_world.batch_enforce_routes(enforce_input)
            step_times["batch_enforce_routes"] = time.perf_counter() - t0
        else:
            step_times["build_enforce_input"] = 0.0
            step_times["batch_enforce_routes"] = 0.0

        # 6. exec_simulation
        t0 = time.perf_counter()
        W.exec_simulation()
        step_times["exec_simulation"] = time.perf_counter() - t0

        # 7. analyzer stats
        t0 = time.perf_counter()
        W.analyzer.print_simple_stats()
        unfinished = W.analyzer.trip_all - W.analyzer.trip_completed
        df_link = W.analyzer.link_to_pandas()
        step_times["analyzer_stats"] = time.perf_counter() - t0

        # 8. route_swap_due (C++ call)
        t0 = time.perf_counter()
        rng_seed = random.randint(0, 2**31 - 1)
        cpp_result = W._cpp_world.route_swap_due(
            od_route_sets_ids, SWAP_PROB, False, rng_seed
        )
        step_times["route_swap_due_cpp"] = time.perf_counter() - t0

        # 9. convert result to names
        t0 = time.perf_counter()
        result = _convert_cpp_result_to_names(cpp_result, W, link_id_to_name)
        routes_specified_data = result['routes_specified']
        route_actual = result['route_actual']
        cost_actual = result['cost_actual']
        n_swap = result['n_swap']
        potential_n_swap = result['potential_n_swap']
        total_t_gap = result['total_t_gap']
        step_times["convert_result_to_names"] = time.perf_counter() - t0

        # 10. misc logging
        t0 = time.perf_counter()
        W.dict_od_to_routes = dict_od_to_routes
        # (would normally append to solver lists)
        step_times["misc_logging"] = time.perf_counter() - t0

        t_iter_total = time.perf_counter() - t_iter_start
        iter_totals.append(t_iter_total)

        for name in step_names:
            timings[name].append(step_times[name])

        row = f"{i:4d} | " + " | ".join(f"{step_times[n]*1000:12.2f}" for n in step_names) + f" | {t_iter_total*1000:.1f}"
        log(row)

    # ---- Summary ----
    log("\n" + "=" * 70)
    log("SUMMARY (mean over iterations, ms)")
    log("=" * 70)

    total_mean = np.mean(iter_totals) * 1000
    for name in step_names:
        vals = timings[name]
        mean_ms = np.mean(vals) * 1000
        std_ms = np.std(vals) * 1000
        pct = mean_ms / total_mean * 100
        log(f"  {name:30s}: {mean_ms:8.2f} ms (std={std_ms:6.2f}) [{pct:5.1f}%]")
    log(f"  {'TOTAL':30s}: {total_mean:8.2f} ms")

    log(f"\n  Pre-loop overhead:")
    log(f"    World_orig creation:        {t_world_orig*1000:.2f} ms")
    log(f"    finalize_scenario:          {t_finalize_orig*1000:.2f} ms")
    log(f"    enumerate_k_random_routes:  {t_enum_routes*1000:.2f} ms")
    log(f"    Total pre-loop:             {(t_world_orig+t_finalize_orig+t_enum_routes)*1000:.2f} ms")

    log(f"\n  Total solve time: {sum(iter_totals):.2f}s (loop) + {t_world_orig+t_finalize_orig+t_enum_routes:.2f}s (pre-loop) = {sum(iter_totals)+t_world_orig+t_finalize_orig+t_enum_routes:.2f}s")

    with open("devlog/timing_breakdown.txt", "w") as f:
        f.write("\n".join(output_lines) + "\n")
    log("\nSaved to devlog/timing_breakdown.txt")
