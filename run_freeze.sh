#!/usr/bin/env bash
# Full frozen protocol runner for the released host binaries.
# Usage:
#   bash run_freeze.sh                # writes ./output
#   OUTPUT_DIR=myout bash run_freeze.sh
#
# Mirrors the exact protocol that produced traces/plan_b_freeze/ (RNG 42-46,
# reward FEAS_IMPROVE, warm-up/batch/budget per benchmark, chi_term=100,
# early-success disabled).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${HERE}/host/bin/probe_seed_budget_suite"
OUT="${OUTPUT_DIR:-${HERE}/output}"
RAW="${OUT}/raw"
mkdir -p "${RAW}"

if [[ ! -x "${BIN}" ]]; then
  echo "Missing ${BIN}; see host/README.md" >&2
  exit 1
fi

SEEDS=(42 43 44 45 46)

echo "=== Plan B freeze (release) @ $(date -Iseconds) ==="
echo "binary=${BIN}"

for s in "${SEEDS[@]}"; do
  "${BIN}" --bench nav2d_hard --seeds "${s}" --K 8 --out "${RAW}/nav2d_hard_${s}.csv"
  "${BIN}" --bench planar5   --seeds "${s}" --K 8 --out "${RAW}/planar5_${s}.csv"
  "${BIN}" --bench quadruped --seeds "${s}" --K 8 --out "${RAW}/quadruped_${s}.csv"
done

diag_status="${OUT}/diag_status.csv"
echo "bench,seed_rng,exit_code" > "${diag_status}"
for s in "${SEEDS[@]}"; do
  set +e
  "${BIN}" --bench planar5 --diag --seeds "${s}" --K 8 \
    | tee "${RAW}/planar5_diag_${s}.log"
  echo "planar5,${s},${PIPESTATUS[0]}" >> "${diag_status}"
  "${BIN}" --bench quadruped --diag --seeds "${s}" --K 4 \
    | tee "${RAW}/quadruped_diag_${s}.log"
  echo "quadruped,${s},${PIPESTATUS[0]}" >> "${diag_status}"
  set -e
done

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

echo "=== freeze complete: ${OUT} ==="
echo "Re-check against the paper tables:"
echo "  python3 verify/verify_paper_tables.py --root ${OUT}"
