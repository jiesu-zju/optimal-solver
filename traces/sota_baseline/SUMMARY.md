# SOTA baseline snapshot (2026-07-18, expanded Gate 5×3 + ALTRO)

## Problem set

| ID | H | dt | Constraints |
|----|---|----|-------------|
| pendulum_swingup | 40 | 0.05 | none |
| double_integrator_reach | 40 | 0.1 | none |
| cartpole_stabilize | 50 | 0.02 | \|F\|≤30, terminal cart box |
| quadrotor_hover | 40 | 0.02 | 0≤T≤15 |
| nav_2d_bicycle | 80 | 0.1 | control box + soft obstacles |

Specs: `examples/gate_bench.h`, `examples/nav_bench.h`.

## Gates (ours vs best **feasible** SOTA)

| Gate | Result |
|------|--------|
| S1 | **15/15** ours feasible |
| S2 vs Croc + Aligator + ALTRO | **15/15** (5/5 problems) |

| problem | ours | best SOTA | notes |
|---------|------|-----------|-------|
| pendulum | 186.108 | ALTRO **186.108** | 持平 |
| DI | **6.67** | 6.67 | 持平 |
| cartpole | **35–114** | ALTRO 35–114 | **≈1.000×**（修双盒后；打平 ALTRO，优于 Croc） |
| quadrotor | 4431–5477 | ALTRO / Aligator | PASS（Box-QP only） |
| nav | 538.9–539.5 | Aligator / ALTRO **~538.9** | PASS |

## Fair Aligator control boxes

`convertCrocoddylProblem` drops `u_lb/u_ub`. Fair path:

- **Nav / quadrotor**: native ProxDDP + `ControlBoxFunction` + multi-config + clip/RK2 recovery
- Pendulum / DI / cartpole: convert path（无控制盒或 g-ineq）

Aligator quadrotor seed1 still weak (~9851 via proj) vs Croc/ALTRO ~5400–5600 — 记为 SOTA 弱解，不作为 ours 借口。

## Quadrotor / cartpole double-box (fixed)

Same root cause: ADMM stage control ineq **+** Box-QP.

| Problem | before | after |
|---------|--------|-------|
| quadrotor | ~1.8× Croc | ≤1.02× best SOTA |
| cartpole | ~1.047× ALTRO | **≈1.000× ALTRO**，且 **优于 Croc ~0.5%** |

Cartpole Gate: force → Box-QP only；terminal cart box 仍走 ADMM。Probe: `probe_cartpole_gate`。MULTIPLE shooting 报更低 cost 但有 dynamics gap，公平 closed-loop re-rollout 并不更好。

## ALTRO

Julia project `sota/altro_gate/`; runner `sota/run_altro.jl` (+ `run_altro.py` wrapper).

- DI: continuous `ẋ=[v,u]` + RK4 ≡ Gate discrete map
- Nav: `@autodiff SoftObsCost` + BoundConstraint boxes
- All 15 rows feasible; Nav ~538.86 matches Croc basin

## Commands

```bash
cmake --build build --target bench_vs_sota
host/bin/bench_vs_sota --out traces/sota_baseline/optimal_solver.csv
python3 sota/run_crocoddyl.py --out traces/sota_baseline/crocoddyl.csv
python3 sota/run_aligator.py --out traces/sota_baseline/aligator.csv
julia --project=sota/altro_gate sota/run_altro.jl \
  --out traces/sota_baseline/altro.csv
# or: python3 sota/run_altro.py --out ...
python3 sota/compare_results.py \
  traces/sota_baseline/optimal_solver.csv \
  traces/sota_baseline/crocoddyl.csv \
  traces/sota_baseline/aligator.csv \
  traces/sota_baseline/altro.csv
```

Reward / MOBO: still frozen.
