#!/usr/bin/env bash
# Freeze Plan B suite + diag for manuscript tables.
# Usage: from repo root
#   bash experiments/results/plan_b_freeze/run_freeze.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
BIN="${ROOT}/build/examples/probe_seed_budget_suite"
OUT="${ROOT}/experiments/results/plan_b_freeze"
RAW="${OUT}/raw"
mkdir -p "${RAW}"

if [[ ! -x "${BIN}" ]]; then
  echo "Missing ${BIN}; build probe_seed_budget_suite first." >&2
  exit 1
fi

# Frozen RNG set (manuscript multi-seed).
SEEDS=(42 43 44 45 46)

echo "=== Plan B freeze @ $(date -Iseconds) ==="
echo "binary=${BIN}"

# --- Suites ---
for s in "${SEEDS[@]}"; do
  echo "--- suite nav2d_hard seeds=${s} ---"
  "${BIN}" --bench nav2d_hard --seeds "${s}" --K 8 \
    --out "${RAW}/nav2d_hard_${s}.csv"

  echo "--- suite planar5 seeds=${s} ---"
  "${BIN}" --bench planar5 --seeds "${s}" --K 8 \
    --out "${RAW}/planar5_${s}.csv"

  echo "--- suite quadruped seeds=${s} ---"
  "${BIN}" --bench quadruped --seeds "${s}" --K 8 \
    --out "${RAW}/quadruped_${s}.csv"
done

# --- Diags (theory Layer 2 witnesses); do not abort on gate FAIL ---
diag_status="${OUT}/diag_status.csv"
echo "bench,seed_rng,exit_code" > "${diag_status}"
for s in "${SEEDS[@]}"; do
  echo "--- diag planar5 seeds=${s} ---"
  set +e
  "${BIN}" --bench planar5 --diag --seeds "${s}" --K 8 \
    | tee "${RAW}/planar5_diag_${s}.log"
  ec=${PIPESTATUS[0]}
  set -e
  echo "planar5,${s},${ec}" >> "${diag_status}"

  echo "--- diag quadruped seeds=${s} ---"
  set +e
  "${BIN}" --bench quadruped --diag --seeds "${s}" --K 4 \
    | tee "${RAW}/quadruped_diag_${s}.log"
  ec=${PIPESTATUS[0]}
  set -e
  echo "quadruped,${s},${ec}" >> "${diag_status}"
done

# Concatenate suite CSVs (header once).
{
  head -n1 "${RAW}/nav2d_hard_42.csv"
  for s in "${SEEDS[@]}"; do tail -n +2 "${RAW}/nav2d_hard_${s}.csv"; done
} > "${OUT}/nav2d_hard_multiseed.csv"

{
  head -n1 "${RAW}/planar5_42.csv"
  for s in "${SEEDS[@]}"; do tail -n +2 "${RAW}/planar5_${s}.csv"; done
} > "${OUT}/planar5_multiseed.csv"

{
  head -n1 "${RAW}/quadruped_42.csv"
  for s in "${SEEDS[@]}"; do tail -n +2 "${RAW}/quadruped_${s}.csv"; done
} > "${OUT}/quadruped_multiseed.csv"

python3 "${OUT}/aggregate.py"

echo "=== freeze complete ==="
echo "See ${OUT}/FROZEN_FOR_MANUSCRIPT.md"
