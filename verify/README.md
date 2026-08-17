# verify/

`verify_paper_tables.py` recomputes every number printed in the manuscript
tables from the raw artifacts under `../traces/` and compares them against
`paper_numbers.json` (the values as published in `../paper/su27a.pdf`).

## Usage

```bash
python3 verify_paper_tables.py            # from anywhere in this repo
python3 verify_paper_tables.py --root /path/to/optimal-solver-release
```

Exit code 0 = all checks pass; 1 = at least one mismatch.

## What is checked

| Paper table | Data source | Check |
|---|---|---|
| Table III (ranking flip, Nav) | `traces/ranking_flip_curves.csv` | oracle J\* per seed, batch-0 `s_χ`/`s_r` scores, greedy picks (s_χ→seed 1, s_r→seed 0) |
| Table IV (quadruped diag) | `traces/plan_b_freeze/raw/quadruped_diag_4X.log` | best-A@warm / best-A@late / best-B@late per RNG 42–46 |
| Table V (B1 Nav2D–Hard) | `traces/plan_b_freeze/nav2d_hard_multiseed.csv` | per-policy mean±std cost, feasibility rate, mean violation; budget accounting on all rows |
| Table VI (B3 Planar5) | `traces/plan_b_freeze/planar5_multiseed.csv` | per-policy mean±std cost, feasibility rate |
| Table VII (Gate validation) | `traces/sota_baseline/*.csv` | 15/15 instances feasible; per-instance cost ratio vs best feasible open-source baseline ≤ threshold per problem |

Tolerances: absolute 0.06 for rounded mean/std/violation values (the JSON holds
the full-precision values from the summary CSVs); per-instance Gate ratios are
computed exactly.

## Extending

To check a fresh re-run (`bash ../run_freeze.sh`), point the script at the new
output tree and compare against the same JSON:

```bash
python3 verify_paper_tables.py --root <fresh-output-dir>
```

(the script expects the same `traces/` layout produced by `run_freeze.sh` +
`sota/` runner outputs).
