# Plan B — frozen for manuscript
RNG set: `[42, 43, 44, 45, 46]` (n=5). Reward: `FEAS_IMPROVE`. Binary: `probe_seed_budget_suite`.
Reproduce:
```bash
bash experiments/results/plan_b_freeze/run_freeze.sh
```
## Protocol (frozen defaults in code)
| Bench | K | warmup | batch | B | early_success | Role |
|-------|---|--------|-------|---|---------------|------|
| B1 `nav2d_hard` | 8 | 10 | 10 | 160 | off (∞) | Primary discrimination |
| B2 `quadruped` | 8 (diag K=4) | 2 | 5 | 80 | off | Contact-rich ranking inversion (cite **diag**) |
| B3 `planar5` | 8 | 3 | 5 | 80 | off | Second discrimination + clean χ control |

## Table B1 — Nav2D-Hard (policy suite)
Mean ± std over 5 seed_rng. **Feasibility is first-class:** many runs remain slightly infeasible under `feas_eps=1e-3`; compare policies by feasible-first ranking (feas → lower viol → lower cost), not raw cost alone.

> **Supersedes** the 2026-07-19 one-shot story (Uniform 3096 / UCB 2138, both feas). Not reproducible with the current binary. Path `nav_hard_results.csv` now holds the **current** rng=42 snapshot — use multi-seed feasible-first stats for tex.

> **B1 re-freeze (2026-08-04):** Re-ran `nav2d_hard` rng 42–46 after OCR seed-semantics fix (`generate_seeds_lhs_noise` keeps mid-horizon state noise). Numerical tables below match the prior freeze (adaptive feasible-first **5/5**); B2/B3 artifacts unchanged.

| Policy | Cost (mean±std) | Feas rate | Viol mean | Budget OK |
|--------|-----------------|-----------|-----------|----------|
| Uniform | 2002.58 ± 435.61 (all infeas — report viol) | 0% | 0.3058 | yes |
| UCB | 1587.49 ± 852.35 (feas 40%) | 40% | 0.08294 | yes |
| BO-EI | 1587.49 ± 852.35 (feas 40%) | 40% | 0.08294 | yes |
| Greedy-s_r | 1587.49 ± 852.35 (feas 40%) | 40% | 0.08294 | yes |
| Greedy-χ | 1418.72 ± 314.44 (feas 20%) | 20% | 0.2339 | yes |

**Feasible-first head-to-head (Uniform vs UCB; BO/Greedy-s_r match UCB):**

| seed_rng | Uniform feas/cost/viol | UCB feas/cost/viol | adaptive wins |
|----------|------------------------|--------------------|---------------|
| 42 | 0/2611.0/0.656 | 0/1231.2/0.0168 | yes |
| 43 | 0/1576.1/0.208 | 1/1046.0/0 | yes |
| 44 | 0/2275.9/0.106 | 0/1137.0/0.0331 | yes |
| 45 | 0/1894.5/0.0661 | 1/1432.9/0 | yes |
| 46 | 0/1655.4/0.493 | 0/3090.5/0.365 | yes |

Adaptive (UCB/BO/Greedy-s_r) wins feasible-first in **5/5** RNGs.

**rng=42 snapshot (current binary):**

| Policy | cost | feas | viol | best_seed |
|--------|------|------|------|----------|
| Uniform | 2611.00 | 0 | 0.6562 | 5 |
| UCB | 1231.19 | 0 | 0.01683 | 2 |
| BO-EI | 1231.19 | 0 | 0.01683 | 2 |
| Greedy-s_r | 1231.19 | 0 | 0.01683 | 2 |
| Greedy-χ | 1825.20 | 0 | 0.5313 | 2 |

## Table B3 — Planar5 (policy suite)
Mean ± std over 5 seed_rng.

| Policy | Feasible cost | Feas rate | Budget OK |
|--------|---------------|-----------|----------|
| Uniform | 433.92 ± 89.94 | 100% | yes |
| UCB | 277.04 ± 138.36 | 100% | yes |
| BO-EI | 269.70 ± 143.42 | 100% | yes |
| Greedy-s_r | 269.70 ± 143.42 | 100% | yes |
| Greedy-χ | 402.43 ± 50.52 | 100% | yes |

## Table B2 — Quadruped suite (near-tie; do **not** claim separation)
Mean ± std over 5 seed_rng. All policies typically find the gem.

| Policy | Cost | Feas rate |
|--------|------|----------|
| Uniform | 10.47 ± 0.47 | 100% |
| UCB | 10.30 ± 0.80 | 100% |
| BO-EI | 10.30 ± 0.80 | 100% |
| Greedy-s_r | 10.30 ± 0.80 | 100% |
| Greedy-χ | 11.13 ± 0.61 | 100% |

## Table L2a — Quadruped ranking-flip diag (theory witness)
Equal per-seed iLQR budget (warm=2, late+=30), K=4, no bandit.

| seed_rng | best_A@warm | best_A@late | best_B@late | inversion | overall |
|----------|-------------|-------------|-------------|-----------|----------|
| 42 | 22.2 | 22.2 | 7.98 | PASS | PASS |
| 43 | 22.9 | 22.9 | 9.39 | PASS | PASS |
| 44 | 26 | 26 | 7.95 | PASS | PASS |
| 45 | 23.1 | 23.1 | 7.98 | PASS | PASS |
| 46 | 22.1 | 22.1 | 13.1 | PASS | PASS |

Gate pass rate: **5/5**.

## Table L2b — Planar5 ranking-flip diag
Equal per-seed budget (warm=pack.warmup=3, late+=…), K=8.

| seed_rng | best_UP@warm | best_UP@late | best_DOWN@late | inversion | overall |
|----------|--------------|--------------|----------------|-----------|----------|
| 42 | 1.47e+03 | 487 | 161 | PASS | PASS |
| 43 | 1.56e+03 | 555 | 227 | PASS | PASS |
| 44 | 1.47e+03 | 145 | 141 | FAIL | FAIL |
| 45 | 1.23e+03 | 487 | 404 | PASS | PASS |
| 46 | 1.39e+03 | 440 | 330 | PASS | PASS |

Strong gate (`down < 0.92·up`): **4/5**. Weak inversion (`down < up`): **5/5** (rng=44 is weak-only: 141 vs 145).

## Writing guidance (for tex)
- **Main claim (s_r allocation):** B1 feasible-first win rate + Table B3 (all-feasible cost gap).
- **B1 wording:** Prefer “adaptive dominates Uniform under feasible-first ranking (5/5 RNGs)” over citing a single feasible cost pair from the superseded 2026-07-19 CSV.
- **Greedy-χ negative control:** B1 (often worse feas/viol) + B3 (all feasible, mean cost 402 vs BO 270).
- **Lemma 1 empirics:** existing Nav `tab:ranking_flip` + Table L2a (quadruped 5/5) + L2b (planar5 4/5 strong, 5/5 weak).
- **B2 suite:** near-tie footnote only; cite **diag** for theory.
- Do **not** cite Franka Hard / spiral as Plan B primary benches.

## Artifacts
| File | Content |
|------|--------|
| `*_multiseed.csv` | Raw per-(rng,policy) rows |
| `*_summary.csv` | Mean/std aggregates |
| `raw/*` | Per-rng suite CSV + diag logs |
| `PROTOCOL_FREEZE.md` | Parameter lock |
