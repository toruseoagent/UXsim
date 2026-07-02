"""
Precise DTA benchmark after optimizations.
10 seeds, max_iter=100, DUE + DSO, Python vs C++.
Reports median and std.
"""

import time
import numpy as np
import warnings
warnings.filterwarnings("ignore")

import uxsim
from uxsim import World
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D

N_SEEDS = 10
SEEDS = list(range(N_SEEDS))
MAX_ITER = 100


def _create_due_world(seed, cpp):
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
    log(f"DTA Precise Benchmark (post-optimization)")
    log(f"  N_SEEDS={N_SEEDS}, MAX_ITER={MAX_ITER}")
    log(f"  Scenario: 9x9 grid, deltan=10, tmax=4800")
    log("=" * 70)

    configs = [
        ("DUE", SolverDUE),
        ("DSO", SolverDSO_D2D),
    ]

    results = {}

    for solver_name, SolverClass in configs:
        for cpp in [False, True]:
            mode = "C++" if cpp else "Python"
            label = f"{solver_name} {mode}"
            times = []
            ttts = []

            for seed in SEEDS:
                create_world = lambda s=seed, c=cpp: _create_due_world(s, c)
                solver = SolverClass(create_world)
                t0 = time.perf_counter()
                solver.solve(max_iter=MAX_ITER, print_progress=False, cpp=cpp)
                elapsed = time.perf_counter() - t0
                ttt = solver.W_sol.analyzer.total_travel_time
                times.append(elapsed)
                ttts.append(ttt)
                log(f"  {label} seed={seed}: {elapsed:.2f}s, TTT={ttt:.0f}")

            results[label] = {"times": times, "ttts": ttts}
            log(f"  {label} => median={np.median(times):.2f}s, std={np.std(times):.2f}s, TTT median={np.median(ttts):.0f}")
            log("")

    # Summary
    log("\n" + "=" * 70)
    log("SUMMARY")
    log("=" * 70)
    log(f"{'':30s} {'median(s)':>10} {'std(s)':>10} {'TTT median':>12}")
    log("-" * 65)

    for solver_name in ["DUE", "DSO"]:
        for mode in ["Python", "C++"]:
            label = f"{solver_name} {mode}"
            r = results[label]
            med = np.median(r["times"])
            std = np.std(r["times"])
            ttt_med = np.median(r["ttts"])
            log(f"  {label:28s} {med:10.2f} {std:10.2f} {ttt_med:12.0f}")

    log("")
    log("SPEEDUP (median Python / median C++)")
    log("-" * 40)
    for solver_name in ["DUE", "DSO"]:
        py_med = np.median(results[f"{solver_name} Python"]["times"])
        cpp_med = np.median(results[f"{solver_name} C++"]["times"])
        ratio = py_med / cpp_med if cpp_med > 0 else float("inf")
        log(f"  {solver_name}: {py_med:.2f}s / {cpp_med:.2f}s = {ratio:.2f}x")

    log("")
    log("TTT COMPARISON (median)")
    log("-" * 40)
    for solver_name in ["DUE", "DSO"]:
        py_ttt = np.median(results[f"{solver_name} Python"]["ttts"])
        cpp_ttt = np.median(results[f"{solver_name} C++"]["ttts"])
        diff = (cpp_ttt - py_ttt) / py_ttt * 100
        log(f"  {solver_name}: Python={py_ttt:.0f}, C++={cpp_ttt:.0f}, diff={diff:+.2f}%")

    with open("devlog/bench_dta_opt.txt", "w") as f:
        f.write("\n".join(output_lines) + "\n")
    log("\nSaved to devlog/bench_dta_opt.txt")
