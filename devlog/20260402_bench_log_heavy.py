"""
Log-heavy benchmark: measures post-simulation log building and analysis.
Exercises _build_all_vehicle_log_caches, pandas conversion, trajectory computation.
"""
import time
import numpy as np
from uxsim import World

DELTAN = 5
TMAX = 4000
IMAX = 7
JMAX = 7
DEMAND_FLOW = 0.03
DEMAND_DURATION = 2000
RANDOM_SEED = 0
N_RUNS = 5


def build_world(cpp=True):
    W = World(
        cpp=cpp, name="", deltan=DELTAN, tmax=TMAX,
        no_cyclic_routing=True, print_mode=0, save_mode=0, show_mode=0,
        random_seed=RANDOM_SEED,
    )
    nodes = {}
    for i in range(IMAX):
        for j in range(JMAX):
            nodes[i, j] = W.addNode(f"n{(i,j)}", i, j)
    for i in range(IMAX):
        for j in range(JMAX):
            if i != IMAX - 1:
                W.addLink(f"l{(i,j,i+1,j)}", nodes[i, j], nodes[i + 1, j],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if i != 0:
                W.addLink(f"l{(i,j,i-1,j)}", nodes[i, j], nodes[i - 1, j],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if j != JMAX - 1:
                W.addLink(f"l{(i,j,i,j+1)}", nodes[i, j], nodes[i, j + 1],
                          length=1000, free_flow_speed=20, jam_density=0.2)
            if j != 0:
                W.addLink(f"l{(i,j,i,j-1)}", nodes[i, j], nodes[i, j - 1],
                          length=1000, free_flow_speed=20, jam_density=0.2)
    for n1 in [(0, j) for j in range(JMAX)]:
        for n2 in [(IMAX - 1, j) for j in range(JMAX)]:
            W.adddemand(nodes[n2], nodes[n1], 0, DEMAND_DURATION, DEMAND_FLOW)
            W.adddemand(nodes[n1], nodes[n2], 0, DEMAND_DURATION, DEMAND_FLOW)
    for n1 in [(i, 0) for i in range(IMAX)]:
        for n2 in [(i, JMAX - 1) for i in range(IMAX)]:
            W.adddemand(nodes[n2], nodes[n1], 0, DEMAND_DURATION, DEMAND_FLOW)
            W.adddemand(nodes[n1], nodes[n2], 0, DEMAND_DURATION, DEMAND_FLOW)
    return W


def run_log_heavy(cpp=True):
    """Run simulation + all log-heavy post-processing."""
    W = build_world(cpp)

    # Phase 1: simulation
    t0 = time.perf_counter()
    W.exec_simulation()
    t_sim = time.perf_counter() - t0

    # Phase 2: basic_analysis (triggers od_analysis)
    t0 = time.perf_counter()
    W.analyzer.basic_analysis()
    t_analysis = time.perf_counter() - t0

    # Phase 3: access all vehicle logs (triggers _build_all_vehicle_log_caches)
    t0 = time.perf_counter()
    for veh in W.VEHICLES.values():
        _ = veh.log_t
        _ = veh.log_x
        _ = veh.log_v
        _ = veh.log_state
        _ = veh.log_link
        _ = veh.log_lane
        _ = veh.log_t_link
    t_logs = time.perf_counter() - t0

    # Phase 4: pandas conversion
    t0 = time.perf_counter()
    W.analyzer.vehicles_to_pandas()
    t_pandas_veh = time.perf_counter() - t0

    t0 = time.perf_counter()
    W.analyzer.od_to_pandas()
    t_pandas_od = time.perf_counter() - t0

    # Phase 5: compute accurate trajectories for all links
    t0 = time.perf_counter()
    W.analyzer.compute_accurate_traj()
    t_traj = time.perf_counter() - t0

    return {
        'sim': t_sim,
        'analysis': t_analysis,
        'logs': t_logs,
        'pandas_veh': t_pandas_veh,
        'pandas_od': t_pandas_od,
        'traj': t_traj,
    }


def main():
    n_veh = None

    # C++ mode x N_RUNS
    print("=" * 70)
    print(f"Log-heavy benchmark: C++ mode x{N_RUNS}")
    print("=" * 70)
    all_times = {k: [] for k in ['sim', 'analysis', 'logs', 'pandas_veh', 'pandas_od', 'traj']}
    totals = []
    for i in range(N_RUNS):
        t = run_log_heavy(cpp=True)
        total = sum(t.values())
        totals.append(total)
        for k, v in t.items():
            all_times[k].append(v)
        print(f"  Run {i+1:2d}: total={total:.3f}s  "
              f"sim={t['sim']:.3f} analysis={t['analysis']:.3f} "
              f"logs={t['logs']:.3f} pandas_v={t['pandas_veh']:.3f} "
              f"pandas_od={t['pandas_od']:.3f} traj={t['traj']:.3f}")

    totals = np.array(totals)
    print(f"\n  Total  median={np.median(totals):.3f}s  std={np.std(totals):.3f}")
    for k in all_times:
        arr = np.array(all_times[k])
        print(f"  {k:12s} median={np.median(arr):.3f}s  std={np.std(arr):.3f}")

    # Python mode x1
    print(f"\n{'=' * 70}")
    print("Log-heavy benchmark: Python mode x1")
    print("=" * 70)
    t = run_log_heavy(cpp=False)
    total = sum(t.values())
    print(f"  total={total:.3f}s  "
          f"sim={t['sim']:.3f} analysis={t['analysis']:.3f} "
          f"logs={t['logs']:.3f} pandas_v={t['pandas_veh']:.3f} "
          f"pandas_od={t['pandas_od']:.3f} traj={t['traj']:.3f}")

    print(f"\n{'=' * 70}")
    print("Summary")
    print("=" * 70)
    print(f"  C++ total median: {np.median(totals):.3f}s")
    print(f"  Python total:     {total:.3f}s")
    print(f"  Speedup:          {total / np.median(totals):.2f}x")


if __name__ == "__main__":
    main()
