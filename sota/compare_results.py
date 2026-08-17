#!/usr/bin/env python3
"""Compare optimal_solver CSV against external SOTA solver CSVs.

Usage:
  python3 scripts/sota/compare_results.py \\
    experiments/results/sota_baseline/optimal_solver.csv \\
    experiments/results/sota_baseline/crocoddyl.csv

Exit code 0 if Gate S2 passes for all shared problems; 1 otherwise.
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path


def load(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def key(row: dict) -> tuple[str, int]:
    return row["problem"], int(row["seed"])


def best_sota(rows: list[dict]) -> dict[tuple[str, int], dict]:
    by_key: dict[tuple[str, int], list[dict]] = defaultdict(list)
    for r in rows:
        if r["solver"] != "optimal_solver":
            by_key[key(r)].append(r)

    best: dict[tuple[str, int], dict] = {}
    for k, group in by_key.items():
        feasible = [g for g in group if g["feasible"] == "1"]
        pool = feasible if feasible else group
        pool.sort(key=lambda g: float(g["final_cost"]))
        best[k] = pool[0]
    return best


def main() -> int:
    if len(sys.argv) < 3:
        print("Usage: compare_results.py ours.csv sota1.csv [sota2.csv ...]")
        return 2

    ours_path = Path(sys.argv[1])
    ours = [r for r in load(ours_path) if r["solver"] == "optimal_solver"]
    sota_rows: list[dict] = []
    for p in sys.argv[2:]:
        sota_rows.extend(load(Path(p)))

    ref = best_sota(sota_rows)
    problems = sorted({k[0] for k in ref})

    print(f"{'problem':<24} {'seed':>4}  {'ours_cost':>12} {'sota_cost':>12}  "
          f"{'ours_feas':>4} {'sota_feas':>4}  S2")
    print("-" * 72)

    s2_hits = 0
    s2_total = 0
    per_problem: dict[str, list[bool]] = defaultdict(list)

    for r in ours:
        k = key(r)
        if k not in ref:
            continue
        s = ref[k]
        ours_cost = float(r["final_cost"])
        sota_cost = float(s["final_cost"])
        ours_feas = r["feasible"] == "1"
        sota_feas = s["feasible"] == "1"

        if sota_feas:
            ok = ours_feas and ours_cost <= 1.05 * sota_cost
        else:
            ok = ours_feas

        s2_total += 1
        s2_hits += int(ok)
        per_problem[k[0]].append(ok)

        print(f"{k[0]:<24} {k[1]:>4}  {ours_cost:12.4f} {sota_cost:12.4f}  "
              f"{int(ours_feas):>4} {int(sota_feas):>4}  {'PASS' if ok else 'FAIL'}")

    print("-" * 72)
    n_problems = len(problems)
    problem_pass_needed = max(1, (2 * n_problems + 2) // 3)
    pair_pass_needed = max(1, (2 * s2_total + 2) // 3)
    print(
        f"Gate S2: {s2_hits}/{s2_total} pairs pass "
        f"(need {pair_pass_needed}/{s2_total} pairs, "
        f"{problem_pass_needed}/{n_problems} problems ≥2/3 seeds)"
    )

    problem_pass = sum(
        1
        for p in problems
        if per_problem.get(p)
        and sum(per_problem[p])
        >= max(1, (2 * len(per_problem[p]) + 2) // 3)
    )
    gate_ok = problem_pass >= problem_pass_needed and s2_hits >= pair_pass_needed
    print(f"Problems meeting 2/3 seed rule: {problem_pass}/{n_problems}")
    return 0 if gate_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
