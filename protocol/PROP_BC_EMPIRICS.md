# Empirics for Prop. B–C (Nav equal-budget arms)

per_arm=80 batch=10

| seed | final_J (feas) |
|------|----------------|
| 0 | 871.257 (yes) |
| 1 | 538.905 (yes) |
| 2 | 538.875 (yes) |

Oracle (min feasible J*): seed **2**
Greedy on s_χ after batch 0: seed **1**
Greedy on s_r after batch 0: seed **0**

| Score | Ranking flip vs oracle? |
|-------|-------------------------|
| s_χ = -log χ | **YES (supports Prop. B+C)** |
| s_r = FEAS_IMPROVE | **YES (supports Prop. B+C)** |

Curves: `ranking_flip_curves.csv`. Theory: `THEORY_RANKING.md`.

---

## Contact-rich witness (B2 quadruped template)

Equal per-seed iLQR budget (warm=2 vs late+=30), no bandit.  
Reproduce: `./build/examples/probe_seed_budget_suite --bench quadruped --diag --K 4`

| seed | cluster | J@warm | J@late (feas) |
|------|---------|--------|---------------|
| 0 | A (over-lift bait) | **22.23** | 22.23 |
| 1 | A | 26.16 | 26.16 |
| 2 | B (mild-clear) | 49.25 | 16.28 |
| 3 | B | 124.4 | **7.98** |

- Early ranking (warm): prefers **A**
- Oracle / late ranking: prefers **B** (\(J^\star\approx 7.98\))
- Supports Prop.\ B/C on a contact-aware constrained iLQR host (soft step + knee barrier basins)
