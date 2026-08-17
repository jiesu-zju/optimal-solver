# host/ — prebuilt solver host (binary-only distribution)

The constrained-iLQR solver ("host") that runs the paper's experiments is
distributed here **as prebuilt binaries only**.  The source is not part of
this release.

## Binaries

| Binary | Purpose |
|--------|---------|
| `bin/probe_seed_budget_suite` | Runs the paper's B1/B2/B3 protocol (suites + diagnostics); produces the CSVs/logs in `traces/plan_b_freeze/` |
| `bin/probe_ranking_flip` | Equal-budget ranking-flip probe (Table III) |
| `bin/probe_seed_budget_nav` | Navigation seed-budgeting probe (protocol v1) |
| `bin/bench_vs_sota` | Host-side Gate runner (Table VII) |

All binaries are x86-64 Linux (built on Ubuntu 22.04, glibc ≥ 2.35), statically
linked against the solver core; they require only the system C/C++ runtime and
OpenMP.  `bin/SHA256SUMS` pins the exact artifacts; the checksums are also
recorded in the release notes.

**Provenance:** the binaries were built on 2026-08-16 from the private source
revision that reproduces the frozen traces (re-verified by re-running the
protocol and comparing with `traces/plan_b_freeze/` — see the release notes).
The same binary interface (`--bench`, `--seeds`, `--K`, `--diag`, `--out`) is
used by `../run_freeze.sh`.

## Terms of use (research reproduction)

The binaries are provided for **non-commercial research use**, specifically to
verify and extend the results reported in the accompanying manuscript.  No
rights to the underlying implementation are granted: you may not decompile,
disassemble, or attempt to derive the source, and you may not redistribute the
binaries except as part of this repository or with the authors' written
consent.  Use at your own risk; no warranty is provided.

## Docker

```bash
docker build -t optimal-solver-host .
docker run --rm -v "$PWD/output:/opt/optimal-solver/output" optimal-solver-host
```

The image contains Ubuntu 22.04 + the binaries + `run_freeze.sh` (entrypoint),
which runs the full frozen protocol and writes the raw CSVs/logs and summary
tables into `/opt/optimal-solver/output/`.

## Rebuilding from source (authors only)

`stage_binaries.sh` documents the author-side procedure: build the probe
targets from the private source tree, copy the binaries here, and regenerate
`bin/SHA256SUMS`.  Public users cannot rebuild the binaries (no source); the
checksums + Gate validation table are the trust anchors.
