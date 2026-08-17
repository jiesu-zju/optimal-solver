#!/usr/bin/env bash
# Author-side staging of the prebuilt host binaries into release/host/bin.
#
# The host source tree is PRIVATE; this script documents how the released
# binaries are produced and pinned.  Run it from the private source root:
#
#   bash release/host/stage_binaries.sh <private-source-root>
#
set -euo pipefail

SRC="${1:?usage: stage_binaries.sh <private-source-root>}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="${HERE}/bin"
mkdir -p "${BIN_DIR}"

# 1) Build the probe targets from the private tree.
cmake --build "${SRC}/build" --target \
  probe_seed_budget_suite probe_ranking_flip probe_seed_budget_nav bench_vs_sota \
  --parallel "$(nproc)"

# 2) Copy the binaries (release build).
for t in probe_seed_budget_suite probe_ranking_flip probe_seed_budget_nav bench_vs_sota; do
  cp "${SRC}/build/examples/${t}" "${BIN_DIR}/${t}"
done
chmod 0755 "${BIN_DIR}"/*

# 3) Pin checksums.
(cd "${BIN_DIR}" && sha256sum probe_seed_budget_suite probe_ranking_flip \
  probe_seed_budget_nav bench_vs_sota > SHA256SUMS)
echo "Staged:"
cat "${BIN_DIR}/SHA256SUMS"

# 4) Reproduce the frozen protocol and verify it matches the released traces.
#    (See release/run_freeze.sh and release/verify/verify_paper_tables.py.)
