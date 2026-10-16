# DTASolvers C++ Integration Plan

## Problem
W(cpp=True) makes exec_simulation 20x faster, but overall DTA solver speedup is only 3x.
The bottleneck is Python loops in solve(): enforce_route×N vehicles, route_swap×N×K cost computations.
These cause O(N*K) Python→C++ transitions per iteration.

## Solution
Move the entire solve() inner loop logic to C++. Keep only the outer iteration loop in Python
(because func_World() is a Python callback for World creation).

## Architecture

### NEW: Separate DTA C++ files (avoid touching traffi.cpp)
- `uxsim/trafficpp/dta_solver.h` — DTA solver C++ header
- `uxsim/trafficpp/dta_solver.cpp` — DTA solver C++ implementation
- These include traffi.h and use World/Vehicle/Link via pointers
- bindings.cpp includes dta_solver.cpp (single TU maintained)
- traffi.h/cpp changes: minimal (only getters if needed)

### C++ Functions (dta_solver.h/cpp)
1. `batch_enforce_routes(World*, mapping)` — all vehicles at once
2. `route_swap_due(World*, route_sets, swap_prob, ...)` — full DUE swap loop
3. `route_swap_dso(World*, route_sets, swap_prob, swap_num, ...)` — full DSO swap loop

### Python Changes (DTAsolvers.py)
- Add `cpp` param to SolverDUE and SolverDSO_D2D
- cpp=True: wrap func_World to inject cpp=True, call C++ batch functions
- cpp=False: existing Python logic unchanged

### Test & Benchmark base
- devlog/benchtestdta.py — 9x9 grid scenario, use max_iter=1 for dev testing
- Compare TTT between Python and C++ modes

## File Ownership
- Bob: dta_solver.h, dta_solver.cpp, bindings.cpp → /tmp/wt-bob/
- Alice: DTAsolvers.py → /tmp/wt-alice/
- Carol: test_cpp_mode.py → /tmp/wt-carol/

## Status
- [x] Investigation complete
- [ ] Phase 1: C++ implementation (Bob) — in progress
- [ ] Phase 1: Python integration (Alice) — in progress
- [x] Phase 1: Test creation (Carol) — 9 tests done, 9x9 grid based
- [ ] Phase 2: Merge & build
- [ ] Phase 3: Test & debug
- [ ] Phase 4: Benchmark
- [ ] Phase 5: PR
