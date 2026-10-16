# Fix: C++ vehicle update order causing diverge test instability

Date: 2026-04-02

## Problem

`test_diverge_lesssaturated_2_1_1` (2-lane link splitting into two 1-lane links) was failing significantly more often in C++ mode (~75%) than in Python mode (~25%).

Both modes share the same simulation logic, the same transfer algorithm, the same car-following model. In deterministic mode (`hard_deterministic_mode=True`), results were perfectly identical. The difference appeared only when randomness was enabled.

## Root cause

### Background: how vehicle transfer works at a diverge node

When vehicles on a 2-lane road reach a fork, they need to be transferred to their respective destination links. The simulator processes one destination link at a time, in a randomly shuffled order. Only the **frontmost vehicle** on the incoming road can be transferred at each attempt.

Consider two vehicles arriving at a fork simultaneously: vehicle A heading left, vehicle B heading right. A is in front (entered the road first).

- If the shuffled order is **[left, right]**: A matches the first attempt (left) and transfers. B becomes the new front and matches the second attempt (right). **2 transfers**.
- If the shuffled order is **[right, left]**: The first attempt (right) finds A at front, but A wants to go left -- no match, skip. The second attempt (left) matches A. **Only 1 transfer**. B is stuck until the next timestep.

This "shuffle mismatch" is an inherent property of the algorithm. It happens in both Python and C++ modes. The expected mismatch rate is 50%.

### The actual bug

In each timestep, the simulator updates all vehicles in a loop. During this loop, vehicles that reach the end of their current road register themselves as candidates for transfer at the next timestep.

- **Python mode**: Always updates vehicles in creation order (vehicle ID order). This is a fixed, predictable order.
- **C++ mode**: Used a performance-optimized list (`active_vehicles`) that gets reordered whenever a vehicle completes its trip. The reordering happens via "swap-and-pop" -- when a vehicle is removed, the last vehicle in the list fills the gap, shuffling the order.

This reordering changed which vehicle registered as a transfer candidate first, which in turn changed the initial arrangement of the shuffled destination list. While each individual shuffle was still fair (50/50), the changing order created a subtle interaction with the random number generator: the sequence of random calls shifted differently depending on which vehicles had recently completed trips.

The result was that in C++ mode, the effective mismatch rate was slightly higher than 50%, causing more vehicles to get stuck at forks. This led to a systematic density increase of about 10% on the upstream road.

### Secondary fix

A hardcoded noise value (`0.01`) in the route search function was also corrected to use the configurable `route_choice_uncertainty` parameter, matching Python's behavior.

## Fix

Changed the C++ vehicle update loop in non-deterministic mode to iterate over the full vehicle list in ID order (matching Python), instead of using the reorder-prone `active_vehicles` list.

## Impact

| Metric | Before fix | After fix | Python reference |
|--------|-----------|-----------|------------------|
| k1 mean | 0.082 | 0.076 | 0.075 |
| k1 std | 0.007 | 0.005 | 0.004 |
| Fail rate | 75% | 29% | 22% |

All 172 C++ mode tests pass.

## Performance benchmark

Benchmark: 11x11 grid network (121 nodes, 440 links), deltan=3, tmax=7200, random_seed=0.
Single-thread (`taskset -c 0`), 10 runs each, median reported.

| | Before fix | After fix | Change |
|---|-----------|-----------|--------|
| Median | 3.484s | 3.005s | **-13.7% (faster)** |
| Std | 0.298s | 0.322s | (comparable) |
| Min | 3.364s | 2.889s | -14.1% |
| Max | 4.197s | 3.862s | -8.0% |

The fix is slightly **faster** than the original. The old `active_vehicles` iteration used swap-and-pop with a while loop (pointer chasing, branch misprediction on removal). The new `vehicles` iteration is a simple sequential scan over a contiguous vector with a state check, which has better cache locality.

## History: how `active_vehicles` was introduced

The `active_vehicles` mechanism was introduced on 2026-04-01 in commit `7a73f36` ("Optimize C++ engine Round 2: active indexing, log reserve, no_cyclic O(1)"), merged to upstream via PR #298.

The optimization targeted the **vehicle update loop**, which at the time accounted for 75.8% of C++ simulation time. The idea was straightforward: instead of iterating all vehicles every timestep (including thousands that have already finished their trip), maintain a separate list of only the "active" (HOME/WAIT/RUN) vehicles. When a vehicle finishes its trip (`end_trip()`), it is removed from this list using O(1) swap-and-pop -- swap the removed element with the last element, then pop the back.

This optimization was sound in principle: skip unnecessary work by not visiting completed vehicles. And it did contribute to a measurable speedup as part of a larger optimization batch (4.97s → 4.43s median, -10.9%).

The problem was an unintended side effect: swap-and-pop changes the **iteration order** of the list. Each time a vehicle completes its trip, the order of remaining vehicles in the list is perturbed. This meant that the C++ vehicle update loop visited vehicles in a different order than Python (which always iterates by vehicle ID), causing the subtle diverge test instability described above.

### Why the simple approach is actually faster

Somewhat counterintuitively, reverting to a simple scan over all vehicles (with a state check to skip finished ones) turned out to be **13.7% faster** than the `active_vehicles` approach. This is likely because:

1. **Cache locality**: `vehicles` is a contiguous vector allocated once. Scanning it sequentially is very cache-friendly. `active_vehicles` suffered from pointer indirection and cache misses due to the swap-and-pop reordering.
2. **Branch prediction**: The state check (`if (state == HOME || WAIT || RUN)`) is highly predictable -- early in the simulation nearly all vehicles are active, and the ratio changes slowly.
3. **No bookkeeping overhead**: Maintaining `active_index`, updating it on swap, and bounds-checking on removal all added per-vehicle overhead that the simple scan avoids.

The `active_vehicles` list is still used for the car-following loop (where the order does not affect results), so it still provides some benefit there.

## Files changed

- `uxsim/trafficpp/traffi.cpp`: vehicle update loop order, noise parameter fix
