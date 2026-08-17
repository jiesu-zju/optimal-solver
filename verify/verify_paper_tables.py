#!/usr/bin/env python3
"""Verify that the released traces reproduce the paper's table numbers.

Usage:
    python3 verify_paper_tables.py [--root <release-root>] [--json paper_numbers.json]

The script recomputes every number reported in the manuscript tables from the
raw CSV/log artifacts under <root>/traces and compares against
paper_numbers.json (the values as printed in the manuscript).  Any mismatch
beyond tolerance fails the check (exit code 1).

Tables covered:
  * B1  Nav2D--Hard policy suite  (Table V)      -- nav2d_hard_multiseed.csv
  * B3  Planar5 policy suite      (Table VI)     -- planar5_multiseed.csv
  * B2  Quadruped diag            (Table IV)     -- raw/quadruped_diag_4X.log
  * Ranking flip on Nav           (Table III)    -- ranking_flip_curves.csv
  * Host Gate validation          (Table VII)    -- sota_baseline/*.csv
  * Budget accounting (all suite rows must account exactly)
"""
from __future__ import annotations

import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TOL = 0.06  # absolute tolerance for rounded mean/std/viol values


def mean_std(xs):
    n = len(xs)
    m = sum(xs) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, math.sqrt(var)


def load_csv(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def check(name, ok, detail=""):
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {name} {detail}")
    return ok


def parse_diag_log(path):
    """best_A@warm, best_A@late, best_B@late from a quadruped diag log."""
    text = path.read_text()
    m = re.search(r"best_A@warm=([\d.]+) best_A@late=([\d.]+)", text)
    mb = re.search(r"best_B@late=([\d.]+)", text)
    if not m or not mb:
        return None
    return float(m.group(1)), float(m.group(2)), float(mb.group(1))


def main():
    root = Path(ROOT)
    if "--root" in sys.argv:
        root = Path(sys.argv[sys.argv.index("--root") + 1])
    json_path = root / "verify" / "paper_numbers.json"
    if "--json" in sys.argv:
        json_path = Path(sys.argv[sys.argv.index("--json") + 1])
    exp = json.loads(json_path.read_text())
    traces = root / "traces"
    all_ok = True

    # ------------------------------------------------------------------ B1
    rows = load_csv(traces / "plan_b_freeze" / "nav2d_hard_multiseed.csv")
    agg = defaultdict(list)
    for r in rows:
        agg[r["policy_name"]].append(r)
    for pol, want in exp["B1"].items():
        a = agg.get(pol)
        if not a:
            all_ok = check(f"B1 {pol} present", False)
            continue
        costs = [float(x["final_cost"]) for x in a]
        viols = [float(x["constraint_violation"]) for x in a]
        feas = sum(1 for x in a if int(x["feasible"]))
        m, s = mean_std(costs)
        vm = sum(viols) / len(viols)
        ok = (abs(m - want["cost_mean"]) <= TOL and abs(s - want["cost_std"]) <= TOL
              and feas == want["feas_count"] and abs(vm - want["viol_mean"]) <= TOL)
        all_ok &= check(
            f"B1 {pol} cost={m:.2f}±{s:.2f} feas={feas}/{len(a)} viol={vm:.4f}",
            ok)
    # Budget accounting
    bad = [r for r in rows if int(r["budget_accounted"]) != 1]
    all_ok &= check(f"B1 budget accounting (all {len(rows)} rows exact)", not bad)

    # ------------------------------------------------------------------ B3
    rows = load_csv(traces / "plan_b_freeze" / "planar5_multiseed.csv")
    agg = defaultdict(list)
    for r in rows:
        agg[r["policy_name"]].append(r)
    for pol, want in exp["B3"].items():
        a = agg.get(pol)
        if not a:
            all_ok = check(f"B3 {pol} present", False)
            continue
        costs = [float(x["final_cost"]) for x in a]
        feas = sum(1 for x in a if int(x["feasible"]))
        m, s = mean_std(costs)
        ok = (abs(m - want["cost_mean"]) <= TOL and abs(s - want["cost_std"]) <= TOL
              and feas == want["feas_count"])
        all_ok &= check(f"B3 {pol} cost={m:.2f}±{s:.2f} feas={feas}/{len(a)}", ok)

    # ------------------------------------------------------------------ B2 diag
    diag_want = exp["B2_diag"]
    for rng, want in diag_want.items():
        log = traces / "plan_b_freeze" / "raw" / f"quadruped_diag_{rng}.log"
        if not log.exists():
            all_ok &= check(f"B2 diag rng {rng} log present", False)
            continue
        got = parse_diag_log(log)
        ok = got is not None and all(
            abs(g - w) <= TOL for g, w in zip(got, want))
        all_ok &= check(f"B2 diag rng {rng} A@warm={got[0] if got else '?'} "
                        f"A@late={got[1] if got else '?'} B@late={got[2] if got else '?'}",
                        ok)

    # ------------------------------------------------------- ranking flip
    flip_rows = load_csv(traces / "ranking_flip_curves.csv")
    by_seed = defaultdict(list)
    for r in flip_rows:
        by_seed[int(r["seed"])].append(r)
    for seed, want in exp["ranking_flip"].items():
        rows0 = [r for r in by_seed[int(seed)] if int(r["batch"]) == 0]
        if not rows0:
            all_ok &= check(f"flip seed {seed} batch-0 row", False)
            continue
        r0 = rows0[0]
        # oracle J*: last j_after across batches
        jstar = min(float(r["j_after"]) for r in by_seed[int(seed)])
        s_chi = float(r0["s_chi"])
        s_r = float(r0["r_feas"])
        ok = (abs(jstar - want["oracle_J"]) <= TOL
              and abs(s_chi - want["s_chi_batch0"]) <= TOL
              and abs(s_r - want["s_r_batch0"]) <= 1.0)
        all_ok &= check(f"flip seed {seed} J*={jstar:.3f} s_chi={s_chi:.3f} "
                        f"s_r={s_r:.2f}", ok)
    # greedy picks: highest s_chi -> seed 1; highest s_r (tie -> first-seen) -> seed 0
    s0 = {int(r["seed"]): [x for x in by_seed[int(r["seed"])] if int(x["batch"]) == 0][0]
          for r in flip_rows}
    greedy_chi = max(s0, key=lambda k: float(s0[k]["s_chi"]))
    greedy_sr = max(s0, key=lambda k: float(s0[k]["r_feas"]))
    all_ok &= check(f"flip greedy picks: s_chi->seed {greedy_chi} (want 1), "
                    f"s_r->seed {greedy_sr} (want 0)",
                    greedy_chi == 1 and greedy_sr == 0)

    # ------------------------------------------------------------------ gate
    base = traces / "sota_baseline"
    ours = load_csv(base / "optimal_solver.csv")
    sota_rows = load_csv(base / "crocoddyl.csv") + load_csv(base / "aligator.csv") \
        + load_csv(base / "altro.csv")
    all_ok &= check(f"Gate: ours 15 rows, {sum(int(r['feasible']) for r in ours)} feasible",
                    len(ours) == 15 and all(int(r["feasible"]) == 1 for r in ours))
    best_sota = {}
    for r in sota_rows:
        if int(r["feasible"]) != 1:
            continue
        k = (r["problem"], int(r["seed"]))
        v = float(r["final_cost"])
        if k not in best_sota or v < best_sota[k]:
            best_sota[k] = v
    by_prob = defaultdict(list)
    for r in ours:
        by_prob[r["problem"]].append(float(r["final_cost"]))
    for prob, want in exp["gate"].items():
        costs = by_prob.get(prob)
        if not costs:
            all_ok &= check(f"Gate {prob}: missing data", False)
            continue
        # Per-instance ratio: ours(problem,seed) / best feasible SOTA(problem,seed).
        ratios = []
        for r in ours:
            if r["problem"] != prob:
                continue
            k = (prob, int(r["seed"]))
            if k not in best_sota:
                continue
            ratios.append(float(r["final_cost"]) / best_sota[k])
        max_ratio = max(ratios) if ratios else float("inf")
        ok = (want["feasible"] == 15 and max_ratio <= want["max_ratio"]
              and min(costs) >= want["cost_min"] * (1 - TOL)
              and max(costs) <= want["cost_max"] * (1 + TOL))
        all_ok &= check(f"Gate {prob}: ours [{min(costs):.1f},{max(costs):.1f}] "
                        f"max per-seed ratio vs best SOTA {max_ratio:.4f}", ok)

    print("-" * 60)
    print("RESULT:", "ALL CHECKS PASS" if all_ok else "MISMATCHES FOUND")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
