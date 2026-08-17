#!/usr/bin/env python3
"""Tune native Aligator Nav for hard control-box satisfaction."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import aligator
import numpy as np
from aligator import SolverProxDDP, constraints, dynamics, manifolds


def _load_ra():
    path = Path(__file__).resolve().parent / "run_aligator.py"
    spec = importlib.util.spec_from_file_location("run_aligator", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def build(mod, mode: str, H: int, dt: float, x0: np.ndarray):
    space = manifolds.VectorSpace(4)
    nu = 2
    umin, umax = mod._NAV_UMIN, mod._NAV_UMAX

    class BikeODE(dynamics.ODEAbstract):
        def __init__(self):
            dynamics.ODEAbstract.__init__(self, space, nu)

        def forward(self, x, u, data):
            th, kap = float(x[2]), float(x[3])
            v, dk = float(u[0]), float(u[1])
            data.xdot[:] = [v * np.cos(th), v * np.sin(th), v * kap, dk]

        def dForward(self, x, u, data):
            th, kap = float(x[2]), float(x[3])
            v = float(u[0])
            data.Jx[:, :] = 0.0
            data.Ju[:, :] = 0.0
            data.Jx[0, 2] = -v * np.sin(th)
            data.Jx[1, 2] = v * np.cos(th)
            data.Jx[2, 3] = v
            data.Ju[0, 0] = np.cos(th)
            data.Ju[1, 0] = np.sin(th)
            data.Ju[2, 0] = kap
            data.Ju[3, 1] = 1.0

    dyn = dynamics.IntegratorRK2(BikeODE(), dt)

    class SoftObs(aligator.CostAbstract):
        def __init__(self):
            aligator.CostAbstract.__init__(self, space, nu)

        def evaluate(self, x, u, data):
            c = 0.0
            for ox, oy, sig, w in mod._NAV_OBS:
                d2 = (float(x[0]) - ox) ** 2 + (float(x[1]) - oy) ** 2
                c += w * math.exp(-d2 / (2 * sig * sig))
            data.value = c

        def computeGradients(self, x, u, data):
            lx = np.zeros(4)
            for ox, oy, sig, w in mod._NAV_OBS:
                dx = float(x[0]) - ox
                dy = float(x[1]) - oy
                d2 = dx * dx + dy * dy
                e = math.exp(-d2 / (2 * sig * sig))
                lx[0] += -w * dx / (sig * sig) * e
                lx[1] += -w * dy / (sig * sig) * e
            data.Lx[:] = lx
            data.Lu[:] = 0.0

        def computeHessians(self, x, u, data):
            Lxx = np.zeros((4, 4))
            for ox, oy, sig, w in mod._NAV_OBS:
                dx = float(x[0]) - ox
                dy = float(x[1]) - oy
                d2 = dx * dx + dy * dy
                e = math.exp(-d2 / (2 * sig * sig))
                s2 = sig * sig
                Lxx[0, 0] += w * e / s2 * ((dx * dx) / s2 - 1.0)
                Lxx[1, 1] += w * e / s2 * ((dy * dy) / s2 - 1.0)
                cross = w * e * dx * dy / (s2 * s2)
                Lxx[0, 1] += cross
                Lxx[1, 0] += cross
            data.Lxx[:, :] = Lxx
            data.Luu[:, :] = 0.0
            data.Lxu[:, :] = 0.0

    rcost = aligator.CostStack(space, nu)
    rcost.addCost(aligator.QuadraticStateCost(space, nu, np.zeros(4), mod._NAV_Q))
    rcost.addCost(aligator.QuadraticControlCost(space, np.zeros(nu), mod._NAV_R))
    rcost.addCost(SoftObs())
    tcost = aligator.CostStack(space, nu)
    tcost.addCost(aligator.QuadraticStateCost(space, nu, mod._NAV_TARGET, mod._NAV_Qf))

    stages = []
    for _ in range(H):
        st = aligator.StageModel(rcost, dyn)
        if mode == "box":
            st.addConstraint(
                aligator.ControlErrorResidual(space.ndx, np.zeros(nu)),
                constraints.BoxConstraint(umin, umax),
            )
        elif mode == "cbf":
            st.addConstraint(
                aligator.ControlBoxFunction(space.ndx, umin, umax),
                constraints.NegativeOrthant(),
            )
        elif mode == "both":
            st.addConstraint(
                aligator.ControlErrorResidual(space.ndx, np.zeros(nu)),
                constraints.BoxConstraint(umin, umax),
            )
            st.addConstraint(
                aligator.ControlBoxFunction(space.ndx, umin, umax),
                constraints.NegativeOrthant(),
            )
        stages.append(st)
    pb = aligator.TrajOptProblem(x0, stages, tcost)
    return pb


def main():
    mod = _load_ra()
    croc = mod._load_croc_module()
    H, dt = 80, 0.1
    xs0, us0 = croc.nav_generate_seeds(0, H, dt)
    x0 = np.asarray(xs0[0], float)

    configs = [
        ("box", 1.0, 100, 150),
        ("cbf", 1.0, 100, 150),
        ("both", 1.0, 100, 150),
        ("cbf", 0.1, 200, 250),
        ("cbf", 0.01, 300, 400),
        ("box", 0.01, 300, 400),
    ]
    for mode, mu, mal, mit in configs:
        pb = build(mod, mode, H, dt, x0)
        print(f"\n=== {mode} mu={mu} mal={mal} mit={mit} dual0={pb.stages[0].num_dual} ===")
        us = [np.clip(np.asarray(u, float), mod._NAV_UMIN, mod._NAV_UMAX).copy() for u in us0]
        xs = [np.asarray(x, float).copy() for x in xs0]
        solver = SolverProxDDP(1e-4, mu, max_iters=mit, verbose=aligator.QUIET)
        solver.max_al_iters = mal
        if hasattr(aligator, "SA_FILTER"):
            solver.sa_strategy = aligator.SA_FILTER
        solver.setup(pb)
        ok = bool(solver.run(pb, xs, us))
        res = solver.results
        us_sol = [np.asarray(u, float) for u in list(res.us)]
        xs_sol = [np.asarray(x, float) for x in list(res.xs)]
        cv = mod._control_box_violation(us_sol, mod._NAV_UMIN, mod._NAV_UMAX)
        cost = mod._nav_cost(xs_sol, us_sol)
        prim = getattr(res, "prim_infeas", None)
        print(
            f"ok={ok} cost={cost:.3f} ctrl_viol={cv:.4e} "
            f"iters={res.num_iters} prim={prim} dual={float(res.dual_infeas):.2e}"
        )


if __name__ == "__main__":
    main()
