"""
DTA solver validation: Python vs C++ on 9x9 grid scenario.
seed=0, max_iter=20 for SolverDUE and SolverDSO_D2D.
"""
import time
import sys
import numpy as np
import uxsim
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D


def _create_due_world(seed, cpp):
    """Helper: build the 9x9 DUE/DSO scenario world (from benchtestdta.py)."""
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


def run_solver(solver_cls, name, seed, cpp, max_iter):
    mode = "C++" if cpp else "Python"
    label = f"{name} ({mode})"
    print(f"\n{'='*60}")
    print(f"Running {label}, seed={seed}, max_iter={max_iter}")
    print(f"{'='*60}")

    t0 = time.perf_counter()
    solver = solver_cls(lambda: _create_due_world(seed, cpp))
    solver.solve(max_iter=max_iter, print_progress=False)
    elapsed = time.perf_counter() - t0

    W_sol = solver.W_sol
    W_sol.analyzer.print_simple_stats(force_print=True)

    print(f"\nElapsed: {elapsed:.2f}s")
    print(f"Final TTT: {solver.ttts[-1]:.1f}")
    print(f"Trip completed: {W_sol.analyzer.trip_completed}")
    print(f"\nTTT per iteration:")
    for i, ttt in enumerate(solver.ttts):
        print(f"  iter {i:3d}: TTT={ttt:.1f}")
    print(f"\nn_swaps per iteration:")
    for i, ns in enumerate(solver.n_swaps):
        print(f"  iter {i:3d}: n_swaps={ns}")

    return {
        "label": label,
        "elapsed": elapsed,
        "ttts": solver.ttts,
        "n_swaps": solver.n_swaps,
        "final_ttt": solver.ttts[-1],
        "trip_completed": W_sol.analyzer.trip_completed,
    }


if __name__ == "__main__":
    seed = 0
    max_iter = 20
    results = {}

    # DUO baseline
    for cpp in [False, True]:
        mode = "C++" if cpp else "Python"
        print(f"\n{'='*60}")
        print(f"DUO baseline ({mode})")
        print(f"{'='*60}")
        t0 = time.perf_counter()
        W = _create_due_world(seed, cpp)
        W.exec_simulation()
        elapsed = time.perf_counter() - t0
        W.analyzer.print_simple_stats(force_print=True)
        ttt = W.analyzer.total_travel_time
        print(f"Elapsed: {elapsed:.2f}s, TTT: {ttt:.1f}, Trips: {W.analyzer.trip_completed}")
        results[f"DUO_{mode}"] = {"elapsed": elapsed, "ttt": ttt, "trips": W.analyzer.trip_completed}

    # DUE
    for cpp in [False, True]:
        r = run_solver(SolverDUE, "DUE", seed, cpp, max_iter)
        results[r["label"]] = r

    # DSO D2D
    for cpp in [False, True]:
        r = run_solver(SolverDSO_D2D, "DSO_D2D", seed, cpp, max_iter)
        results[r["label"]] = r

    # Summary comparison
    print(f"\n{'='*60}")
    print("SUMMARY COMPARISON")
    print(f"{'='*60}")

    print(f"\n--- DUO baseline ---")
    for mode in ["Python", "C++"]:
        r = results[f"DUO_{mode}"]
        print(f"  {mode:8s}: TTT={r['ttt']:.1f}, trips={r['trips']}, time={r['elapsed']:.2f}s")
    duo_py = results["DUO_Python"]["ttt"]
    duo_cpp = results["DUO_C++"]["ttt"]
    print(f"  Rel diff: {abs(duo_cpp - duo_py) / duo_py * 100:.2f}%")

    for solver_name in ["DUE", "DSO_D2D"]:
        print(f"\n--- {solver_name} ---")
        py_r = results[f"{solver_name} (Python)"]
        cpp_r = results[f"{solver_name} (C++)"]
        print(f"  Python:  final TTT={py_r['final_ttt']:.1f}, trips={py_r['trip_completed']}, time={py_r['elapsed']:.2f}s")
        print(f"  C++:     final TTT={cpp_r['final_ttt']:.1f}, trips={cpp_r['trip_completed']}, time={cpp_r['elapsed']:.2f}s")
        rel_diff = abs(cpp_r['final_ttt'] - py_r['final_ttt']) / py_r['final_ttt'] * 100
        speedup = py_r['elapsed'] / cpp_r['elapsed'] if cpp_r['elapsed'] > 0 else float('inf')
        print(f"  TTT rel diff: {rel_diff:.2f}%")
        print(f"  Speedup: {speedup:.2f}x")

        # TTT trajectory comparison
        print(f"\n  TTT trajectory (Python vs C++):")
        print(f"  {'iter':>4s}  {'Python':>12s}  {'C++':>12s}  {'rel_diff%':>10s}")
        for i in range(min(len(py_r['ttts']), len(cpp_r['ttts']))):
            py_ttt = py_r['ttts'][i]
            cpp_ttt = cpp_r['ttts'][i]
            rd = abs(cpp_ttt - py_ttt) / py_ttt * 100 if py_ttt > 0 else 0
            print(f"  {i:4d}  {py_ttt:12.1f}  {cpp_ttt:12.1f}  {rd:10.2f}%")
