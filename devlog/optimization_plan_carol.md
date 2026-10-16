# DTA C++ Performance Optimization Plan (Carol's Analysis)

## Profiling Results (DSO cpp=True, max_iter=20, 9x9 grid)

Total: 8.1s

| Rank | Function | Time | % | Notes |
|------|----------|------|---|-------|
| 1 | `enumerate_k_random_routes` | 2.35s | 29% | One-time at start |
| 2 | `_convert_od_routes_to_ids` | 2.35s | 29% | Called per-iter x20 |
| 3 | `exec_simulation` | 1.81s | 22% | Core simulation |
| 4 | `_setup_analyzer` / `finalize_scenario` | 0.76s | 9% | Per-iter |
| 5 | `findSystemFonts` (matplotlib) | 0.40s | 5% | Per-iter, in Analyzer.__init__ |
| 6 | `_build_all_vehicle_log_caches` | 0.36s | 4% | Per-iter, in simulation_terminated |
| 7 | `link_to_pandas` | 0.26s | 3% | Per-iter |
| 8 | `_convert_cpp_result_to_names` | 0.12s | 1% | Per-iter |
| 9 | `_build_vehicles_enter_log` | 0.09s | 1% | Per-iter, in simulation_terminated |
| 10 | `_register_new_cpp_vehicles` | 0.10s | 1% | Per-iter, in exec_simulation |

## Optimization (A): Skip log caches in DTA intermediate iterations

### Analysis

`simulation_terminated()` (L1180 in uxsim_cpp_wrapper.py) does 3 things:
1. `_build_vehicles_enter_log()` — rebuilds vehicles_enter_log dict for each link
2. `_build_all_vehicle_log_caches()` — batch-fetches log_t/log_x/log_v/... for all vehicles
3. `analyzer.basic_analysis()` — computes TTT, trip stats

In DTA C++ path:
- **route_swap_due/dso reads log_link/log_t directly from C++** — no Python log_cache needed
- **analyzer.basic_analysis() → od_to_pandas() → od_analysis()** — reads `veh.travel_time` and `veh.distance_traveled` which are C++ properties (NOT log_cache)
- **link_to_pandas() → link_analysis_coarse()** — reads `l.cum_departure`, `l.traveltime_actual` etc. which are C++ link properties (NOT log_cache)
- **vehicles_enter_log** — only used by compute_edie_state and visualization, not needed in DTA iteration

**Conclusion: _build_all_vehicle_log_caches() and _build_vehicles_enter_log() can be FULLY SKIPPED in DTA intermediate iterations.**

### Implementation Plan

**File: uxsim_cpp_wrapper.py**

Option 1 (recommended): Add a flag `_skip_log_on_terminate` to CppWorld:
```python
# In CppWorld.__init__:
self._skip_log_on_terminate = False

# In simulation_terminated:
def simulation_terminated(self):
    self.print(" simulation finished")
    self._simulation_done = True
    if not self._skip_log_on_terminate:
        self._build_vehicles_enter_log()
        self._build_all_vehicle_log_caches()
    self.analyzer.basic_analysis()
```

**File: DTAsolvers.py** (Alice's territory — she would add):
```python
# In DTA C++ path, before exec_simulation:
if i != max_iter - 1:
    W._skip_log_on_terminate = True
else:
    W._skip_log_on_terminate = False
```

**Estimated savings: 0.45s per run (0.36s + 0.09s) = ~5.5% of total time**
At max_iter=100: ~2.25s saved (skipping 98 out of 100 iterations)

## Optimization (B): Skip _register_new_cpp_vehicles in DTA

### Analysis

`_register_new_cpp_vehicles()` creates Python CppVehicle proxy objects for every new C++ vehicle. In DTA, the C++ batch route swap reads vehicle data directly from C++ — it doesn't need Python proxy objects for intermediate iterations.

However, `od_analysis()` iterates `W.VEHICLES.values()` and reads `veh.travel_time`, `veh.orig`, `veh.dest`. These are Python proxy attributes that require registration.

**Conclusion: _register_new_cpp_vehicles CANNOT be skipped** because basic_analysis needs it.

BUT: The color generation (`self.rng.random(size=(n_new, 3))`) and unused Python-only attributes could be skipped for DTA. This is minor (~0.01s per iter).

## Optimization (C): link_to_pandas efficiency

### Analysis

`link_to_pandas()` → `link_analysis_coarse()` takes 0.26s/20 iter = 0.013s/iter. 
It iterates all links and reads cum_departure, traveltime_actual from C++.

This is already efficient. Main overhead is:
1. `link_analysis_coarse()` computing numpy operations on traveltime_actual
2. DataFrame construction

**Possible optimization**: Build the DataFrame directly in C++ and return via numpy arrays instead of Python dict iteration. But the savings (~0.013s/iter) don't justify the complexity.

**Conclusion: Not worth optimizing at this stage.**

## Priority Summary

| Optimization | Savings/iter | Effort | Recommended |
|-------------|-------------|--------|-------------|
| (A) Skip log caches | ~0.023s/iter | Low — 5 lines in wrapper, 3 lines in DTAsolvers | YES |
| (B) Skip vehicle registration | Not possible | — | NO |
| (C) link_to_pandas | ~0.013s/iter | High — C++ DataFrame builder | NO |

**Total estimated savings for (A) at max_iter=100: ~2.3s (from ~8.1s baseline for 20 iter)**

## Note on _convert_od_routes_to_ids (2.35s)

This is the #2 bottleneck but it's called only once (L892 in DTAsolvers.py with `if od_route_sets_ids is None` guard). The profiler shows 20 calls which is suspicious — this may be a profiler artifact or the guard isn't working as expected. Alice should verify this.

**UPDATE**: Looking more carefully at the profile, `_convert_od_routes_to_ids` shows `20 calls / 2.35s`. This contradicts the `if od_route_sets_ids is None` guard which should limit it to 1 call. Alice should check if the DSO path also calls this correctly — line 892 in the C++ path has the guard, but maybe it's being called from somewhere else, or the profiler is attributing child time incorrectly.
