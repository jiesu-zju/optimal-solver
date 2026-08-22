# PROTOCOL v1 — Feasibility-aware seed budgeting (Nav)

**Status: FROZEN for Phase-1 probes.**

## Thesis (decision objective)

Under a fixed total iLQR iteration budget \(B\), allocate batches across \(K\) warm-start seeds to maximize the **feasible incumbent cost**, and record how quickly a near-optimal feasible cost is reached.

Kernel Gate S1/S2 (parity vs Crocoddyl/Aligator/ALTRO) establishes a **trusted host**; it is **not** the paper climax.

## Known facts (do not re-litigate)

1. Host: Gate validation 15/15 vs Crocoddyl/Aligator/ALTRO — see `traces/sota_baseline/`.
2. The Greedy-\(\chi\) negative control (pull the arm with the lowest \(\chi\)) is unreliable on Nav: the best-cost arm does not have the smallest \(\chi\).
3. Batch semantics: each pull constructs a **fresh** `ConstrainedILQR` + ADMM; only the **trajectory** warm-starts. Duals / Filter do **not** persist.
4. **Theory of ranking:** `THEORY_RANKING.md` (Prop.\ A–D). Empirics: greedy rules based on \(s_r\) or on \(\chi\) miss the oracle arm on equal-budget Nav arms (`PROP_BC_EMPIRICS.md`).

## Reward (Candidate A) + χ switch (Candidate C)

Per pull of `batch_size` iterations on arm \(i\):

\[
\Delta J = J_{\mathrm{after}} - J_{\mathrm{before}},\quad
v = \max(\mathrm{ineq},\mathrm{eq},\mathrm{dyn}),\quad
\mathrm{viol}_+ = \max(0,\, v-\varepsilon)
\]

\[
\Delta J_{\mathrm{feas}} =
\begin{cases}
\Delta J & \text{if } v \le \varepsilon \\
0 & \text{otherwise}
\end{cases}
\qquad
\rho = \frac{\max(0,\,-\Delta J_{\mathrm{feas}})}{\max(1,\,\mathrm{iters\_used})}
\]

\[
r = \mathrm{clip}\!\left(-\Delta J_{\mathrm{feas}} - \lambda\,\mathrm{viol}_+ + \eta\,\rho,\;[-50,50]\right)
\]

| Constant | Value | Notes |
|----------|-------|-------|
| \(\lambda\) | 10 | fixed a priori |
| \(\eta\) | 0.1 | small speed bonus |
| \(\varepsilon\) | \(10^{-3}\) | = Gate `constraint_tolerance` |
| clip | \([-50,50]\) | numerical |

**χ switch (not in \(r\)):** if after-batch \(\chi > \chi_{\mathrm{cut}}=10^{2}\) or solver throws → arm **terminated**. χ, viol, \(\rho\), \(\Delta J\) are logged and used as **BO features**.

The Greedy-\(\chi\) negative control pulls the arm with the lowest \(\chi\); the
primary reward is \(s_r\) above, and \(\chi\) enters only through the
termination switch, the BO features, and this control rule.

## Phase-1 Nav protocol

| Knob | Value |
|------|-------|
| Problem | `nav_2d_bicycle` (same hooks as `bench_vs_sota`) |
| Arms \(K\) | 3 homotopy seeds (cruise / wait→sprint / +1) |
| Budget \(B\) | 240 |
| Warm-up / batch | 10 / 10 |
| Policies | UNIFORM, UCB(\(c=1\)), BO-EI (GP predicts **\(r\)**) |
| Gate-SOTA near-opt | cost \(\le 1.05\times 538.9 \approx 565.8\) |
| Reward mode | `FEAS_IMPROVE` |
| Early success | stop allocating once feasible incumbent cost ≤ 565.8 |

### Primary metrics

1. Best-of-\(K\) **feasible** cost under the policy.
2. Cumulative iters until first feasible cost \(\le 565.8\) (∞ if never).
3. Allocation: iters per seed; flag if post-hoc best-cost seed gets \(\lt\) warm-up only after warm-up (starvation).

### Phase-1 pass criteria (diagnostic, not significance)

- Budget accounting: \(\sum_i \mathrm{iters}_i = \mathrm{allocated} \le B\).
- UCB/BO must **not** systematically starve the post-hoc best-cost arm.
- Success vs Uniform: best feasible cost **not worse**, **and** near-opt iters **strictly fewer**; **or** best feasible cost **strictly better** at same \(B\).

Phase 1 does **not** claim statistical significance. Phase 2: \(N\ge 5\) RNG replicates.

## Reproduce

```bash
cmake --build build --target probe_seed_budget_nav -j
./build/examples/probe_seed_budget_nav \
  --out-dir output/seed_budget_protocol
```

## Non-contributions / forbidden

- Claiming kernel superiority over Croc/Aligator/ALTRO as the main result.
- Tuning \(\lambda,\eta,c\) on the evaluation seed set before freezing a new PROTOCOL version.
- Pretending duals persist across pulls.
- MOBO/EHVI in v1 main tables.
