"""Post-refactor DTA benchmark. 3 seeds, max_iter=100."""
import time, warnings, statistics
import numpy as np
warnings.filterwarnings("ignore")
import uxsim
from uxsim.DTAsolvers import SolverDUE, SolverDSO_D2D

SEEDS = [0, 1, 2]
MAX_ITER = 100

def _create_due_world(seed, cpp):
    W = uxsim.World(name="", deltan=10, tmax=4800, duo_update_time=300,
        print_mode=0, save_mode=0, show_mode=0, random_seed=seed, cpp=cpp)
    imax, jmax = 9, 9
    id_center = 4
    nodes = {}
    for i in range(imax):
        for j in range(jmax):
            nodes[i,j] = W.addNode(f"n{(i,j)}", i, j, flow_capacity=1.6)
    for i in range(imax):
        for j in range(jmax):
            ffs = 10
            if i != imax-1:
                if j == id_center: ffs = 20
                W.addLink(f"l{(i,j,i+1,j)}", nodes[i,j], nodes[i+1,j], length=1000, free_flow_speed=ffs)
            ffs = 10
            if i != 0:
                if j == id_center: ffs = 20
                W.addLink(f"l{(i,j,i-1,j)}", nodes[i,j], nodes[i-1,j], length=1000, free_flow_speed=ffs)
            ffs = 10
            if j != jmax-1:
                if i == id_center: ffs = 20
                W.addLink(f"l{(i,j,i,j+1)}", nodes[i,j], nodes[i,j+1], length=1000, free_flow_speed=ffs)
            ffs = 10
            if j != 0:
                if i == id_center: ffs = 20
                W.addLink(f"l{(i,j,i,j-1)}", nodes[i,j], nodes[i,j-1], length=1000, free_flow_speed=ffs)
    df, dd, oi = 0.08, 2400, 3
    for n1 in [(0,j) for j in range(oi, jmax-oi)]:
        for n2 in [(imax-1,j) for j in range(oi, jmax-oi)]:
            W.adddemand(nodes[n1], nodes[n2], 0, dd, df)
        for n2 in [(i,jmax-1) for i in range(oi, imax-oi)]:
            W.adddemand(nodes[n1], nodes[n2], 0, dd, df)
    for n1 in [(i,0) for i in range(oi, imax-oi)]:
        for n2 in [(i,jmax-1) for i in range(oi, imax-oi)]:
            W.adddemand(nodes[n1], nodes[n2], 0, dd, df)
        for n2 in [(imax-1,j) for j in range(oi, jmax-oi)]:
            W.adddemand(nodes[n1], nodes[n2], 0, dd, df)
    return W

out = []
def log(msg=""):
    print(msg, flush=True)
    out.append(msg)

log("=" * 70)
log(f"Post-refactor DTA Benchmark, seeds={SEEDS}, max_iter={MAX_ITER}")
log("=" * 70)

results = {}
for sname, SCls in [("DUE", SolverDUE), ("DSO", SolverDSO_D2D)]:
    for cpp in [False, True]:
        mode = "C++" if cpp else "Python"
        label = f"{sname} {mode}"
        times, ttts = [], []
        for seed in SEEDS:
            func = lambda s=seed, c=cpp: _create_due_world(s, c)
            solver = SCls(func, cpp=cpp)
            t0 = time.perf_counter()
            solver.solve(max_iter=MAX_ITER, print_progress=False)
            el = time.perf_counter() - t0
            ttt = solver.W_sol.analyzer.total_travel_time
            times.append(el)
            ttts.append(ttt)
            log(f"  {label} seed={seed}: {el:.2f}s TTT={ttt:.0f}")
        results[label] = {"times": times, "ttts": ttts}
        log(f"  => median={statistics.median(times):.2f}s std={np.std(times):.2f} TTT_med={statistics.median(ttts):.0f}")
        log("")

log("=" * 70)
log("SUMMARY")
log("=" * 70)
log(f"{'Config':30s} {'median(s)':>10} {'std(s)':>10} {'TTT_med':>12}")
log("-" * 65)
for lb in results:
    r = results[lb]
    log(f"  {lb:28s} {statistics.median(r['times']):10.2f} {np.std(r['times']):10.2f} {statistics.median(r['ttts']):12.0f}")

log("")
log("SPEEDUP (median Python / median C++)")
log("-" * 50)
for sn in ["DUE", "DSO"]:
    py = statistics.median(results[f"{sn} Python"]["times"])
    cp = statistics.median(results[f"{sn} C++"]["times"])
    log(f"  {sn}: {py:.2f}s / {cp:.2f}s = {py/cp:.2f}x")

log("")
log("TTT COMPARISON (median)")
log("-" * 50)
for sn in ["DUE", "DSO"]:
    pt = statistics.median(results[f"{sn} Python"]["ttts"])
    ct = statistics.median(results[f"{sn} C++"]["ttts"])
    d = (ct - pt) / pt * 100
    log(f"  {sn}: Python={pt:.0f}, C++={ct:.0f}, diff={d:+.2f}%")

with open("devlog/bench_refactor.txt", "w") as f:
    f.write("\n".join(out) + "\n")
log("\nSaved to devlog/bench_refactor.txt")
