# Plan B protocol freeze (code defaults)

Do **not** retune \(\lambda,\eta,c\) or bench costs for table cosmetics without bumping this file.

| Parameter | Value | Where |
|-----------|-------|--------|
| Reward | `BanditRewardMode::FEAS_IMPROVE` | `make_opts` in `probe_seed_budget_suite.cpp` |
| early_success_cost | `+∞` (disabled) | same |
| \(\lambda,\eta\) | PROTOCOL_v1 defaults in scheduler | `BanditOptions` |
| UCB \(c\) | suite default | `make_opts` |
| Multi-seed set | `{42,43,44,45,46}` | `plan_b_freeze/run_freeze.sh` |

## B1 `nav2d_hard`

- K=8, warmup=10, batch=10, B=160
- Seeds: LHS+noise (`generate_seeds_lhs_noise`)
- Binary: `./build/examples/probe_seed_budget_suite --bench nav2d_hard`

## B2 `quadruped`

- Suite: K=8, warmup=2, batch=5, B=80
- Diag: K=4, warm=2, late+=30 (gate in `run_quadruped_diag`)
- Model: planar template + soft step/knee barrier (`planar_quadruped_bench.h`)
- Paper cite: **diag ranking flip**, not suite separation

## B3 `planar5`

- K=8, warmup=3, batch=5, B=80
- Seeds: UP/DOWN homotopy (`planar5_bench.h`)
- Diag: `run_planar5_inversion_diag`

## Reproduce

```bash
cmake --build build --target probe_seed_budget_suite -j$(nproc)
bash experiments/results/plan_b_freeze/run_freeze.sh
```

Manuscript paste tables: `FROZEN_FOR_MANUSCRIPT.md`.
2026-07-20 15:46:10.538923677 +0800 build/examples/probe_seed_budget_suite
