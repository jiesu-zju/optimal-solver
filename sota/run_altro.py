#!/usr/bin/env python3
"""Wrapper: run ALTRO Gate Julia script and write CSV.

Usage:
  python3 scripts/sota/run_altro.py --out altro.csv
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
JL = Path(__file__).resolve().parent / "run_altro.jl"
PROJ = Path(__file__).resolve().parent / "altro_gate"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="altro.csv")
    ap.add_argument("--julia", default="julia")
    args = ap.parse_args()
    julia = shutil.which(args.julia)
    if not julia:
        print("julia not found on PATH", file=sys.stderr)
        return 1
    if not PROJ.joinpath("Project.toml").exists():
        print(
            f"Julia project missing at {PROJ}. Run:\n"
            f"  julia --project={PROJ} -e 'using Pkg; Pkg.instantiate()'",
            file=sys.stderr,
        )
        return 1
    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    cmd = [julia, f"--project={PROJ}", str(JL), "--out", str(out)]
    print(" ".join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
