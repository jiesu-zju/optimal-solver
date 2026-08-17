# optimal-solver — experiment release for "Feasibility-Aware Seed Budgeting for Constrained iLQR" (RA-L submission)

This repository is the **experiment release** accompanying the manuscript
(currently under review; the preprint will be posted here upon acceptance).
It contains everything needed to **verify** the paper's reported numbers and
to **re-run** the reported experiments end-to-end, while the constrained-iLQR
solver itself ("the host") is distributed only as **prebuilt binaries**
(source is not part of this release).

## What is in this repository

| Path | Contents |
|------|----------|
| `protocol/` | Frozen experimental protocol and theory documents (PROTOCOL_v1, ranking-inversion theory, thesis one-pager, freeze statement) |
| `benchmarks/` | Complete mathematical definitions of all benchmarks (B1 Nav2D–Hard, B2 planar quadruped, B3 Planar5, Gate suite) — implementable from scratch |
| `traces/` | **All raw data behind the paper tables**: per-policy per-RNG suite CSVs, per-batch allocation traces, ranking-flip curves, quadruped diag logs, SOTA-baseline CSVs |
| `verify/` | `verify_paper_tables.py` — recomputes every number in the paper tables from `traces/` and checks it against `paper_numbers.json` (the printed values) |
| `sota/` | Runners for the open-source baselines (Crocoddyl, Aligator, ALTRO) used in the host Gate validation |
| `host/` | Docker image + prebuilt host binaries (probes) + checksums + freeze runner |

## Quick start: verify the paper numbers (no solver needed)

```bash
python3 verify/verify_paper_tables.py
```

Expected output: one `[PASS]` line per table cell and `RESULT: ALL CHECKS PASS`.
The checks cover Tables III–VII of the manuscript (ranking flip, quadruped
diag, B1/B3 policy suites, Gate validation) plus exact budget accounting.

## Quick start: re-run the experiments (host binary / Docker)

The host is provided as prebuilt binaries in `host/bin/` (x86-64 Linux;
see `host/README.md` for provenance and checksums).  Re-run the full frozen
protocol:

```bash
bash run_freeze.sh                # writes output/ (raw CSVs + summaries)
# or, in a container:
docker build -t optimal-solver-host host/
docker run --rm -v "$PWD/output:/opt/optimal-solver/output" optimal-solver-host
```

`run_freeze.sh` reproduces `traces/plan_b_freeze/` exactly (same binary
interface, same RNG ids 42–46, same protocol parameters); the fresh output can
then be fed to `verify/verify_paper_tables.py --root <output-dir>` for an
independent re-check.

The open-source SOTA baselines are re-runnable from `sota/` (Crocoddyl and
Aligator via Python, ALTRO via Julia); see `sota/README.md`.

## What is intentionally *not* released

- The constrained-iLQR solver source (host implementation: iLQR/ADMM/Filter
  stack).  It is distributed only as a prebuilt binary so that the experiments
  remain independently executable.  The host's numerical behaviour is anchored
  publicly by the Gate validation table (Table VII) and by `traces/sota_baseline/`.
- The bandit-scheduler implementation source (it is compiled into the binary
  and is fully specified in the manuscript: Algorithm 1 + Eqs. (2)–(5) +
  Section IV).

## License

- Released code (verify scripts, SOTA runners, protocol/benchmark documents):
  MIT — see `LICENSE`.
- Manuscript: copyright of the authors; the preprint is posted here upon
  acceptance (not included while under review).
- The prebuilt host binaries in `host/bin/` are **not** open source; they are
  provided under the terms in `host/README.md` for research reproduction only.
- Third-party software retains its own licenses — see `NOTICE.md`.

## Reproducibility statement

Every number printed in the manuscript tables is derived from the files under
`traces/` (this is checked automatically by `verify/`).  The traces were
produced by the frozen protocol in `protocol/` with the same binary interface
shipped in `host/bin/`, RNG ids {42, 43, 44, 45, 46}, on Ubuntu 22.04 x86-64
(AMD Ryzen 7 5800X, 32 GB RAM).  Wall-clock figures in the paper are indicative
(machine-specific); iteration budgets and costs are exact.

**RNG portability note.**  Seed generation uses `std::mt19937` with
`std::uniform_real_distribution` / `std::normal_distribution`.  The C++
standard fixes the `mt19937` sequence but *not* the distribution algorithms,
so rebuilding from source with a different standard-library implementation
(libstdc++ / libc++ / MSVC) may produce different seed trajectories and hence
slightly different traces.  The **shipped binaries are the authoritative
implementation** (their rebuild is bit-identical to the binary that produced
the frozen traces); the frozen `traces/` and the verification script remain
the reference in all cases.
