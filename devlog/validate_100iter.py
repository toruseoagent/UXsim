"""
DTA solver validation: Python vs C++ on 9x9 grid scenario.
seed=0, max_iter=100 for SolverDUE and SolverDSO_D2D.
"""
import time
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
    solver = solver_cls(lambda: _create_due_world(seed, cpp), cpp=cpp)
    solver.solve(max_iter=max_iter, print_progress=False)
    elapsed = time.perf_counter() - t0

    W_sol = solver.W_sol
    W_sol.analyzer.print_simple_stats(force_print=True)

    ttts = solver.ttts
    last20_avg = np.mean(ttts[-20:])
    min_ttt = np.min(ttts)
    min_iter = int(np.argmin(ttts))

    # W_intermid_solution TTT (best solution found during iteration)
    intermid_ttt = None
    if solver.W_intermid_solution is not None:
        intermid_ttt = solver.W_intermid_solution.analyzer.total_travel_time

    print(f"\nElapsed: {elapsed:.2f}s")
    print(f"Final TTT (iter 99): {ttts[-1]:.1f}")
    print(f"Last 20 iter avg TTT: {last20_avg:.1f}")
    print(f"Min TTT: {min_ttt:.1f} (iter {min_iter})")
    if intermid_ttt is not None:
        print(f"W_intermid_solution TTT: {intermid_ttt:.1f}")
    print(f"Trip completed: {W_sol.analyzer.trip_completed}")
    print(f"\nTTT per iteration:")
    for i, ttt in enumerate(ttts):
        print(f"  iter {i:3d}: TTT={ttt:.1f}")

    return {
        "label": label,
        "elapsed": elapsed,
        "ttts": ttts,
        "n_swaps": solver.n_swaps,
        "final_ttt": ttts[-1],
        "last20_avg": last20_avg,
        "min_ttt": min_ttt,
        "min_iter": min_iter,
        "intermid_ttt": intermid_ttt,
        "trip_completed": W_sol.analyzer.trip_completed,
    }


if __name__ == "__main__":
    seed = 0
    max_iter = 100
    results = {}

    # (A) DUE Python
    results["DUE_Py"] = run_solver(SolverDUE, "DUE", seed, False, max_iter)

    # (B) DUE C++
    results["DUE_Cpp"] = run_solver(SolverDUE, "DUE", seed, True, max_iter)

    # (C) DSO Python
    results["DSO_Py"] = run_solver(SolverDSO_D2D, "DSO_D2D", seed, False, max_iter)

    # (D) DSO C++
    results["DSO_Cpp"] = run_solver(SolverDSO_D2D, "DSO_D2D", seed, True, max_iter)

    # ============================================================
    # Summary
    # ============================================================
    print(f"\n{'='*60}")
    print("SUMMARY COMPARISON (seed=0, max_iter=100)")
    print(f"{'='*60}")

    # DUE comparison: last 20 iter avg TTT
    print(f"\n--- DUE (comparison metric: last 20 iter avg TTT) ---")
    py_r = results["DUE_Py"]
    cpp_r = results["DUE_Cpp"]
    print(f"  Python:  last20 avg={py_r['last20_avg']:.1f}, final={py_r['final_ttt']:.1f}, trips={py_r['trip_completed']}, time={py_r['elapsed']:.2f}s")
    print(f"  C++:     last20 avg={cpp_r['last20_avg']:.1f}, final={cpp_r['final_ttt']:.1f}, trips={cpp_r['trip_completed']}, time={cpp_r['elapsed']:.2f}s")
    avg_diff = abs(cpp_r['last20_avg'] - py_r['last20_avg']) / py_r['last20_avg'] * 100
    speedup = py_r['elapsed'] / cpp_r['elapsed'] if cpp_r['elapsed'] > 0 else float('inf')
    print(f"  Last 20 avg TTT rel diff: {avg_diff:.2f}%")
    print(f"  Speedup: {speedup:.2f}x")

    py_last20 = py_r['ttts'][-20:]
    cpp_last20 = cpp_r['ttts'][-20:]
    print(f"\n  Last 20 iters statistics:")
    print(f"  Python: mean={np.mean(py_last20):.1f}, std={np.std(py_last20):.1f}, min={np.min(py_last20):.1f}, max={np.max(py_last20):.1f}")
    print(f"  C++:    mean={np.mean(cpp_last20):.1f}, std={np.std(cpp_last20):.1f}, min={np.min(cpp_last20):.1f}, max={np.max(cpp_last20):.1f}")

    # DSO comparison: min TTT (W_intermid_solution)
    print(f"\n--- DSO_D2D (comparison metric: min TTT / W_intermid_solution) ---")
    py_r = results["DSO_Py"]
    cpp_r = results["DSO_Cpp"]
    print(f"  Python:  min TTT={py_r['min_ttt']:.1f} (iter {py_r['min_iter']}), intermid={py_r['intermid_ttt']}, trips={py_r['trip_completed']}, time={py_r['elapsed']:.2f}s")
    print(f"  C++:     min TTT={cpp_r['min_ttt']:.1f} (iter {cpp_r['min_iter']}), intermid={cpp_r['intermid_ttt']}, trips={cpp_r['trip_completed']}, time={cpp_r['elapsed']:.2f}s")
    min_diff = abs(cpp_r['min_ttt'] - py_r['min_ttt']) / py_r['min_ttt'] * 100
    speedup = py_r['elapsed'] / cpp_r['elapsed'] if cpp_r['elapsed'] > 0 else float('inf')
    print(f"  Min TTT rel diff: {min_diff:.2f}%")
    if py_r['intermid_ttt'] and cpp_r['intermid_ttt']:
        intermid_diff = abs(cpp_r['intermid_ttt'] - py_r['intermid_ttt']) / py_r['intermid_ttt'] * 100
        print(f"  W_intermid TTT rel diff: {intermid_diff:.2f}%")
    print(f"  Speedup: {speedup:.2f}x")

    py_last20 = py_r['ttts'][-20:]
    cpp_last20 = cpp_r['ttts'][-20:]
    print(f"\n  Last 20 iters statistics:")
    print(f"  Python: mean={np.mean(py_last20):.1f}, std={np.std(py_last20):.1f}, min={np.min(py_last20):.1f}, max={np.max(py_last20):.1f}")
    print(f"  C++:    mean={np.mean(cpp_last20):.1f}, std={np.std(cpp_last20):.1f}, min={np.min(cpp_last20):.1f}, max={np.max(cpp_last20):.1f}")

    # TTT trajectory every 10 iters for both solvers
    for solver_name, py_key, cpp_key in [("DUE", "DUE_Py", "DUE_Cpp"), ("DSO_D2D", "DSO_Py", "DSO_Cpp")]:
        py_r = results[py_key]
        cpp_r = results[cpp_key]
        print(f"\n  {solver_name} TTT trajectory (every 10 iters):")
        print(f"  {'iter':>4s}  {'Python':>12s}  {'C++':>12s}  {'rel_diff%':>10s}")
        for i in list(range(0, 100, 10)) + [99]:
            py_ttt = py_r['ttts'][i]
            cpp_ttt = cpp_r['ttts'][i]
            rd = abs(cpp_ttt - py_ttt) / py_ttt * 100 if py_ttt > 0 else 0
            print(f"  {i:4d}  {py_ttt:12.1f}  {cpp_ttt:12.1f}  {rd:10.2f}%")
