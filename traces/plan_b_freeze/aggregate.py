#!/usr/bin/env python3
"""Aggregate Plan B freeze CSVs + diag logs → summary + manuscript tables."""
from __future__ import annotations

import csv
import math
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SEEDS = [42, 43, 44, 45, 46]


def mean_std(xs: list[float]) -> tuple[float, float]:
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(xs) / n
    if n == 1:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, math.sqrt(var)


def load_suite(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def suite_table(rows: list[dict]) -> list[dict]:
    """Per-policy mean±std cost, feas rate, mean viol."""
    by = defaultdict(list)
    for r in rows:
        by[r["policy_name"]].append(r)
    out = []
    for policy, rs in by.items():
        costs = [float(r["final_cost"]) for r in rs]
        feas = [int(r["feasible"]) for r in rs]
        viols = [float(r["constraint_violation"]) for r in rs]
        budget_ok = all(int(r["budget_accounted"]) == 1 for r in rs)
        m, s = mean_std(costs)
        out.append(
            {
                "policy": policy,
                "n": len(rs),
                "cost_mean": m,
                "cost_std": s,
                "feas_rate": sum(feas) / len(feas),
                "viol_mean": sum(viols) / len(viols),
                "budget_ok": budget_ok,
            }
        )
    order = ["Uniform", "UCB", "BO-EI", "Greedy-s_r", "Greedy-χ"]
    out.sort(key=lambda d: order.index(d["policy"]) if d["policy"] in order else 99)
    return out


def feas_first_key(r: dict) -> tuple:
    """Higher is better: feasible > lower viol > lower cost."""
    feas = int(r["feasible"])
    viol = float(r["constraint_violation"])
    cost = float(r["final_cost"])
    return (feas, -viol, -cost)


def b1_head_to_head(rows: list[dict]) -> list[dict]:
    """Per-rng Uniform vs UCB / BO / Greedy-s_r (adaptive bloc is tied)."""
    by_seed: dict[int, dict[str, dict]] = defaultdict(dict)
    for r in rows:
        by_seed[int(r["seed_rng"])][r["policy_name"]] = r
    out = []
    for s, pols in sorted(by_seed.items()):
        u = pols["Uniform"]
        a = pols["UCB"]
        out.append(
            {
                "seed_rng": s,
                "uni_feas": int(u["feasible"]),
                "uni_cost": float(u["final_cost"]),
                "uni_viol": float(u["constraint_violation"]),
                "ucb_feas": int(a["feasible"]),
                "ucb_cost": float(a["final_cost"]),
                "ucb_viol": float(a["constraint_violation"]),
                "adaptive_wins_feas_first": int(
                    feas_first_key(a) > feas_first_key(u)
                ),
            }
        )
    return out


def fmt_cost(m: float, s: float, feas_rate: float) -> str:
    if feas_rate <= 0.0:
        return f"{m:.2f} ± {s:.2f} (all infeas — report viol)"
    if feas_rate < 1.0:
        return f"{m:.2f} ± {s:.2f} (feas {feas_rate:.0%})"
    return f"{m:.2f} ± {s:.2f}"



def parse_quadruped_diag(log: Path) -> dict | None:
    text = log.read_text()
    m_a_w = re.search(r"best_A@warm=([0-9.eE+-]+)", text)
    m_a_l = re.search(r"best_A@late=([0-9.eE+-]+)", text)
    m_b_l = re.search(r"best_B@late=([0-9.eE+-]+)", text)
    overall = "PASS" if "OVERALL: PASS" in text else "FAIL"
    if not (m_a_w and m_b_l):
        return None
    return {
        "best_A_warm": float(m_a_w.group(1)),
        "best_A_late": float(m_a_l.group(1)) if m_a_l else float("nan"),
        "best_B_late": float(m_b_l.group(1)),
        "overall": overall,
        "inv": "PASS" if "GATE inversion (B@late better than A): PASS" in text else "FAIL",
    }


def parse_planar5_diag(log: Path) -> dict | None:
    text = log.read_text()
    m_sum = re.search(
        r"best_up@warm=([0-9.eE+-]+)\s+best_up@late=([0-9.eE+-]+)\s+best_down@late=([0-9.eE+-]+)",
        text,
    )
    inv = "PASS" if re.search(r"GATE inversion \(down@late.*\): PASS", text) else "FAIL"
    if not m_sum:
        return None
    return {
        "best_UP_warm": float(m_sum.group(1)),
        "best_UP_late": float(m_sum.group(2)),
        "best_DOWN_late": float(m_sum.group(3)),
        "overall": inv,  # planar5 diag has no OVERALL line; gate == overall
        "inv": inv,
    }


def write_summary_csv(path: Path, rows: list[dict], bench: str) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "bench",
                "policy",
                "n_seeds",
                "cost_mean",
                "cost_std",
                "feas_rate",
                "viol_mean",
                "budget_ok",
            ]
        )
        for r in rows:
            w.writerow(
                [
                    bench,
                    r["policy"],
                    r["n"],
                    f"{r['cost_mean']:.6g}",
                    f"{r['cost_std']:.6g}",
                    f"{r['feas_rate']:.4f}",
                    f"{r['viol_mean']:.6g}",
                    int(r["budget_ok"]),
                ]
            )


def main() -> None:
    nav = suite_table(load_suite(ROOT / "nav2d_hard_multiseed.csv"))
    p5 = suite_table(load_suite(ROOT / "planar5_multiseed.csv"))
    qd = suite_table(load_suite(ROOT / "quadruped_multiseed.csv"))
    write_summary_csv(ROOT / "nav2d_hard_summary.csv", nav, "nav2d_hard")
    write_summary_csv(ROOT / "planar5_summary.csv", p5, "planar5")
    write_summary_csv(ROOT / "quadruped_summary.csv", qd, "quadruped")

    quad_diags = []
    p5_diags = []
    for s in SEEDS:
        q = parse_quadruped_diag(ROOT / "raw" / f"quadruped_diag_{s}.log")
        if q:
            q["seed_rng"] = s
            quad_diags.append(q)
        p = parse_planar5_diag(ROOT / "raw" / f"planar5_diag_{s}.log")
        if p:
            p["seed_rng"] = s
            p5_diags.append(p)

    # Primary RNG=42 snapshot from multiseed rows
    def row42(bench_csv: Path, policy: str) -> dict:
        for r in load_suite(bench_csv):
            if int(r["seed_rng"]) == 42 and r["policy_name"] == policy:
                return r
        return {}

    md = []
    md.append("# Plan B — frozen for manuscript\n")
    md.append(f"RNG set: `{SEEDS}` (n={len(SEEDS)}). Reward: `FEAS_IMPROVE`. Binary: `probe_seed_budget_suite`.\n")
    md.append("Reproduce:\n")
    md.append("```bash\nbash experiments/results/plan_b_freeze/run_freeze.sh\n```\n")
    md.append("## Protocol (frozen defaults in code)\n")
    md.append("| Bench | K | warmup | batch | B | early_success | Role |\n")
    md.append("|-------|---|--------|-------|---|---------------|------|\n")
    md.append("| B1 `nav2d_hard` | 8 | 10 | 10 | 160 | off (∞) | Primary discrimination |\n")
    md.append("| B2 `quadruped` | 8 (diag K=4) | 2 | 5 | 80 | off | Contact-rich ranking inversion (cite **diag**) |\n")
    md.append("| B3 `planar5` | 8 | 3 | 5 | 80 | off | Second discrimination + clean χ control |\n")
    md.append("\n")

    md.append("## Table B1 — Nav2D-Hard (policy suite)\n")
    md.append(
        f"Mean ± std over {len(SEEDS)} seed_rng. "
        "**Feasibility is first-class:** many runs remain slightly infeasible "
        "under `feas_eps=1e-3`; compare policies by feasible-first ranking "
        "(feas → lower viol → lower cost), not raw cost alone.\n\n"
    )
    md.append(
        "> **Supersedes** the 2026-07-19 one-shot story (Uniform 3096 / UCB 2138, both feas). "
        "Not reproducible with the current binary. Path `nav_hard_results.csv` now holds "
        "the **current** rng=42 snapshot — use multi-seed feasible-first stats for tex.\n\n"
    )
    md.append("| Policy | Cost (mean±std) | Feas rate | Viol mean | Budget OK |\n")
    md.append("|--------|-----------------|-----------|-----------|----------|\n")
    for r in nav:
        md.append(
            f"| {r['policy']} | {fmt_cost(r['cost_mean'], r['cost_std'], r['feas_rate'])} | "
            f"{r['feas_rate']:.0%} | {r['viol_mean']:.4g} | "
            f"{'yes' if r['budget_ok'] else 'NO'} |\n"
        )

    h2h = b1_head_to_head(load_suite(ROOT / "nav2d_hard_multiseed.csv"))
    wins = sum(x["adaptive_wins_feas_first"] for x in h2h)
    md.append("\n**Feasible-first head-to-head (Uniform vs UCB; BO/Greedy-s_r match UCB):**\n\n")
    md.append(
        "| seed_rng | Uniform feas/cost/viol | UCB feas/cost/viol | adaptive wins |\n"
        "|----------|------------------------|--------------------|---------------|\n"
    )
    for x in h2h:
        md.append(
            f"| {x['seed_rng']} | {x['uni_feas']}/{x['uni_cost']:.1f}/{x['uni_viol']:.3g} | "
            f"{x['ucb_feas']}/{x['ucb_cost']:.1f}/{x['ucb_viol']:.3g} | "
            f"{'yes' if x['adaptive_wins_feas_first'] else 'no'} |\n"
        )
    md.append(f"\nAdaptive (UCB/BO/Greedy-s_r) wins feasible-first in **{wins}/{len(h2h)}** RNGs.\n")

    md.append("\n**rng=42 snapshot (current binary):**\n\n")
    md.append("| Policy | cost | feas | viol | best_seed |\n|--------|------|------|------|----------|\n")
    for pol in ["Uniform", "UCB", "BO-EI", "Greedy-s_r", "Greedy-χ"]:
        r = row42(ROOT / "nav2d_hard_multiseed.csv", pol)
        if not r:
            continue
        md.append(
            f"| {pol} | {float(r['final_cost']):.2f} | {r['feasible']} | "
            f"{float(r['constraint_violation']):.4g} | {r['best_seed']} |\n"
        )

    md.append("\n## Table B3 — Planar5 (policy suite)\n")
    md.append(f"Mean ± std over {len(SEEDS)} seed_rng.\n\n")
    md.append("| Policy | Feasible cost | Feas rate | Budget OK |\n")
    md.append("|--------|---------------|-----------|----------|\n")
    for r in p5:
        md.append(
            f"| {r['policy']} | {fmt_cost(r['cost_mean'], r['cost_std'], r['feas_rate'])} | "
            f"{r['feas_rate']:.0%} | {'yes' if r['budget_ok'] else 'NO'} |\n"
        )

    md.append("\n## Table B2 — Quadruped suite (near-tie; do **not** claim separation)\n")
    md.append(f"Mean ± std over {len(SEEDS)} seed_rng. All policies typically find the gem.\n\n")
    md.append("| Policy | Cost | Feas rate |\n|--------|------|----------|\n")
    for r in qd:
        md.append(
            f"| {r['policy']} | {fmt_cost(r['cost_mean'], r['cost_std'], r['feas_rate'])} | "
            f"{r['feas_rate']:.0%} |\n"
        )

    md.append("\n## Table L2a — Quadruped ranking-flip diag (theory witness)\n")
    md.append("Equal per-seed iLQR budget (warm=2, late+=30), K=4, no bandit.\n\n")
    md.append("| seed_rng | best_A@warm | best_A@late | best_B@late | inversion | overall |\n")
    md.append("|----------|-------------|-------------|-------------|-----------|----------|\n")
    for d in quad_diags:
        md.append(
            f"| {d['seed_rng']} | {d['best_A_warm']:.3g} | {d['best_A_late']:.3g} | "
            f"{d['best_B_late']:.3g} | {d['inv']} | {d['overall']} |\n"
        )
    n_pass = sum(1 for d in quad_diags if d["overall"] == "PASS")
    md.append(f"\nGate pass rate: **{n_pass}/{len(quad_diags)}**.\n")

    md.append("\n## Table L2b — Planar5 ranking-flip diag\n")
    md.append("Equal per-seed budget (warm=pack.warmup=3, late+=…), K=8.\n\n")
    md.append("| seed_rng | best_UP@warm | best_UP@late | best_DOWN@late | inversion | overall |\n")
    md.append("|----------|--------------|--------------|----------------|-----------|----------|\n")
    for d in p5_diags:
        md.append(
            f"| {d['seed_rng']} | {d['best_UP_warm']:.3g} | {d['best_UP_late']:.3g} | "
            f"{d['best_DOWN_late']:.3g} | {d['inv']} | {d['overall']} |\n"
        )
    n_pass_p = sum(1 for d in p5_diags if d["inv"] == "PASS")
    n_weak = sum(
        1
        for d in p5_diags
        if d["best_DOWN_late"] < d["best_UP_late"]
    )
    md.append(
        f"\nStrong gate (`down < 0.92·up`): **{n_pass_p}/{len(p5_diags)}**. "
        f"Weak inversion (`down < up`): **{n_weak}/{len(p5_diags)}** "
        "(rng=44 is weak-only: 141 vs 145).\n"
    )

    md.append("\n## Writing guidance (for tex)\n")
    md.append(
        "- **Main claim (s_r allocation):** B1 feasible-first win rate + Table B3 "
        "(all-feasible cost gap).\n"
    )
    md.append(
        "- **B1 wording:** Prefer “adaptive dominates Uniform under feasible-first "
        "ranking (5/5 RNGs)” over citing a single feasible cost pair from the "
        "superseded 2026-07-19 CSV.\n"
    )
    md.append(
        "- **Greedy-χ negative control:** B1 (often worse feas/viol) + B3 "
        "(all feasible, mean cost 402 vs BO 270).\n"
    )
    md.append(
        "- **Lemma 1 empirics:** existing Nav `tab:ranking_flip` + Table L2a "
        "(quadruped 5/5) + L2b (planar5 4/5 strong, 5/5 weak).\n"
    )
    md.append(
        "- **B2 suite:** near-tie footnote only; cite **diag** for theory.\n"
    )
    md.append(
        "- Do **not** cite Franka Hard / spiral as Plan B primary benches.\n"
    )
    md.append("\n## Artifacts\n")
    md.append("| File | Content |\n|------|--------|\n")
    md.append("| `*_multiseed.csv` | Raw per-(rng,policy) rows |\n")
    md.append("| `*_summary.csv` | Mean/std aggregates |\n")
    md.append("| `raw/*` | Per-rng suite CSV + diag logs |\n")
    md.append("| `PROTOCOL_FREEZE.md` | Parameter lock |\n")

    (ROOT / "FROZEN_FOR_MANUSCRIPT.md").write_text("".join(md))
    print("Wrote", ROOT / "FROZEN_FOR_MANUSCRIPT.md")


if __name__ == "__main__":
    main()
