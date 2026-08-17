# Thesis one-pager — Feasibility-aware seed budgeting

## Motivation

Constrained iLQR/DDP failures are catastrophic for planning/control. In embodied stacks that end in hierarchical CI-MPC, **warm-start quality under a tight iteration budget** often dominates online reliability—more than another point of asymptotic solver sophistication on easy basins.

## Core idea

1. Maintain a library of initial trajectories (homotopy / heuristic seeds).
2. After each short iLQR batch, read **diagnostics**: feasible cost change \(\Delta J\), constraint violation, improvement rate \(\rho\), and \(\chi\) as pathology.
3. Use **UCB / BO-EI** to allocate the remaining budget to seeds that improve **feasible cost** quickly.
4. Return the feasible-first incumbent.

## Contributions (intended)

- Reward design aligned with the decision objective (feasible cost under budget), with χ as **switch/feature**, not primary reward.
- Open, budget-exact scheduler with Uniform / UCB / BO baselines and full allocation traces.
- Hosted on a Gate-validated constrained iLQR stack (parity evidence separate from the scheduling claim).

## Non-contributions

- “Our iLQR beats all public solvers on every problem.”
- χ as the paper-primary bandit reward (empirically anti-aligned on Nav).
- Legacy 533× / `full_100` numbers.
