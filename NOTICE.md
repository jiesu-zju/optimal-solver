# Third-party notices

## Open-source baselines (used only as comparison targets in `sota/` and `traces/sota_baseline/`)

- **Crocoddyl**: BSD-3-Clause — https://github.com/loco-3d/crocoddyl
- **Aligator**: BSD-3-Clause — https://github.com/Simple-Robotics/aligator
- **ALTRO** (via TrajectoryOptimization.jl): MIT — https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl
- **Pinocchio** (Crocoddyl dependency): BSD-2-Clause — https://github.com/stack-of-tasks/pinocchio
- **Julia / TrajectoryOptimization.jl** project files under `sota/altro_gate/` are
  the authors' gate configuration; `Manifest.toml` pins the dependency graph used
  for the reported runs.

## Data and documents

- `traces/` and `protocol/` were produced by the authors; no third-party data is
  included.  `traces/sota_baseline/*.csv` are outputs of the open-source solvers
  above (with their own licenses applying to the software, not the data).

## Prebuilt host binaries (`host/bin/`)

- Built by the authors from the private `optimal_solver` source tree (which
  links **Eigen**, MPL-2.0 with exceptions).  The binaries are distributed for
  research reproduction only, under the terms in `host/README.md`.
