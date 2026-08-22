# sota/ — open-source baseline runners (Gate validation, Table II)

These scripts re-run the open-source trajectory-optimization baselines used to
validate the (binary) host solver on the five-problem Gate suite.  They are
independent of the host: only the released problem definitions
(`../benchmarks/benchmark_spec.md`) and the CSV comparison are shared.

## Problem set (5 problems × 3 seeds = 15 rows per solver)

| ID | Notes |
|----|-------|
| `pendulum_swingup` | Unconstrained swing-up |
| `double_integrator_reach` | Discrete LQR reach (ALTRO: continuous DI + RK4 ≡ map) |
| `cartpole_stabilize` | Cart-pole upright; \|F\|≤30, terminal cart box |
| `quadrotor_hover` | 2D quad hover; 0≤T≤15 |
| `nav_2d_bicycle` | Bicycle + soft obstacles + control box |

## Reproduce the baseline side of Table II

```bash
# Crocoddyl (Python)
python3 sota/run_crocoddyl.py --out crocoddyl.csv

# Aligator (Python)
python3 sota/run_aligator.py --out aligator.csv

# ALTRO (Julia; project pinned in sota/altro_gate/Manifest.toml)
julia --project=sota/altro_gate sota/run_altro.jl --out altro.csv
# or the wrapper: python3 sota/run_altro.py --out altro.csv
```

The host side of the table is produced by `../host/bin/bench_vs_sota`
(`--out optimal_solver.csv`), or read directly from
`../traces/sota_baseline/optimal_solver.csv`.

## Compare

```bash
python3 sota/compare_results.py \
  traces/sota_baseline/optimal_solver.csv \
  traces/sota_baseline/crocoddyl.csv \
  traces/sota_baseline/aligator.csv \
  traces/sota_baseline/altro.csv
```

The released baseline outputs live in `../traces/sota_baseline/` and are part
of the automated table verification (`../verify/`).  Note: baseline versions
may drift over time; the released CSVs are the ones reported in the paper.

## Aligator notes

- **Nav / quadrotor**: native ProxDDP + `ControlBoxFunction` (the generic
  convert path drops control bounds).
- Pendulum / DI / cartpole: `convertCrocoddylProblem` (+ g-ineq where needed).

## ALTRO notes

- Project: `altro_gate/` (`Altro`, `TrajectoryOptimization`, `RobotDynamics`, …).
- Instantiate once: `julia --project=altro_gate -e 'using Pkg; Pkg.instantiate()'`.
- Control boxes via `BoundConstraint`; Nav soft obstacles via `@autodiff SoftObsCost`.
