"""
DTA Solver benchmark: cpp=False vs cpp=True
DUE and DSO (D2D) solvers, max_iter=10, N_SEEDS=1
Single-thread: run with OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
"""

import time
import sys
import numpy as np

import uxsim
from uxsim import World
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D

N_SEEDS = 1
SEEDS = list(range(N_SEEDS))
MAX_ITER = 10


def _create_due_world(seed, cpp):
    """9x9 grid scenario for DUE/DSO (same as benchtestdta.py)."""
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


def run_duo(seed, cpp):
    """Single DUO simulation (baseline)."""
    W = _create_due_world(seed, cpp)
    t0 = time.perf_counter()
    W.exec_simulation()
    elapsed = time.perf_counter() - t0
    df = W.analyzer.basic_to_pandas()
    return {
        "elapsed": elapsed,
        "ttt": df["total_travel_time"].sum(),
    }


def run_due(seed, cpp):
    """DUE solver. When cpp=True, passes cpp=True to solve() to use C++ batch functions."""
    create_world = lambda: _create_due_world(seed, cpp)
    solver = SolverDUE(create_world)
    t0 = time.perf_counter()
    solver.solve(max_iter=MAX_ITER, print_progress=False, cpp=cpp)
    elapsed = time.perf_counter() - t0
    df = solver.W_sol.analyzer.basic_to_pandas()
    return {
        "elapsed": elapsed,
        "ttt": df["total_travel_time"].sum(),
        "ttts": solver.ttts,
    }


def run_dso(seed, cpp):
    """DSO (D2D) solver. When cpp=True, passes cpp=True to solve() to use C++ batch functions."""
    create_world = lambda: _create_due_world(seed, cpp)
    solver = SolverDSO_D2D(create_world)
    t0 = time.perf_counter()
    solver.solve(max_iter=MAX_ITER, print_progress=False, cpp=cpp)
    elapsed = time.perf_counter() - t0
    df = solver.W_sol.analyzer.basic_to_pandas()
    return {
        "elapsed": elapsed,
        "ttt": df["total_travel_time"].sum(),
        "ttts": solver.ttts,
    }


if __name__ == "__main__":
    output_lines = []
    def log(msg=""):
        print(msg)
        output_lines.append(msg)

    log("=" * 70)
    log("DTA Solver Benchmark: Python vs C++")
    log(f"  N_SEEDS={N_SEEDS}, MAX_ITER={MAX_ITER}")
    log(f"  Scenario: 9x9 grid, deltan=10, tmax=4800")
    log("=" * 70)

    results = {}

    for solver_name, run_fn in [("DUO", run_duo), ("DUE", run_due), ("DSO", run_dso)]:
        log(f"\n--- {solver_name} ---")
        for cpp in [False, True]:
            mode = "C++" if cpp else "Python"
            times = []
            ttts = []
            ttt_series_list = []
            for seed in SEEDS:
                res = run_fn(seed, cpp)
                times.append(res["elapsed"])
                ttts.append(res["ttt"])
                if "ttts" in res:
                    ttt_series_list.append(res["ttts"])

            elapsed_mean = np.mean(times)
            elapsed_std = np.std(times)
            ttt_mean = np.mean(ttts)

            results[(solver_name, mode)] = {
                "elapsed_mean": elapsed_mean,
                "elapsed_std": elapsed_std,
                "ttt_mean": ttt_mean,
                "ttts": ttt_series_list,
            }

            log(f"  {mode:8s}: elapsed={elapsed_mean:.2f}s (std={elapsed_std:.2f}), TTT={ttt_mean:.0f}")
            if ttt_series_list:
                log(f"           TTT per iter: {[int(x) for x in ttt_series_list[0]]}")

    # Speedup summary
    log("\n" + "=" * 70)
    log("SPEEDUP SUMMARY (Python time / C++ time)")
    log("=" * 70)
    for solver_name in ["DUO", "DUE", "DSO"]:
        py = results[(solver_name, "Python")]["elapsed_mean"]
        cpp = results[(solver_name, "C++")]["elapsed_mean"]
        ratio = py / cpp if cpp > 0 else float("inf")
        log(f"  {solver_name}: Python={py:.2f}s, C++={cpp:.2f}s, speedup={ratio:.2f}x")

    # TTT comparison
    log("\n" + "=" * 70)
    log("TTT COMPARISON (Python vs C++)")
    log("=" * 70)
    for solver_name in ["DUO", "DUE", "DSO"]:
        py_ttt = results[(solver_name, "Python")]["ttt_mean"]
        cpp_ttt = results[(solver_name, "C++")]["ttt_mean"]
        diff_pct = (cpp_ttt - py_ttt) / py_ttt * 100 if py_ttt > 0 else 0
        log(f"  {solver_name}: Python={py_ttt:.0f}, C++={cpp_ttt:.0f}, diff={diff_pct:+.2f}%")

    # Save results
    with open("devlog/bench_dta_result.txt", "w") as f:
        f.write("\n".join(output_lines) + "\n")
    log("\nResults saved to devlog/bench_dta_result.txt")
