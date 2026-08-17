#!/usr/bin/env python3
"""Probe several ways to enforce Nav control boxes under Aligator ProxDDP."""
import importlib.util
from pathlib import Path

import aligator
import crocoddyl
import numpy as np
from aligator import SolverProxDDP, constraints
from aligator.croc import convertCrocoddylProblem


def load_croc():
    path = Path(__file__).resolve().parent / "run_crocoddyl.py"
    spec = importlib.util.spec_from_file_location("run_crocoddyl", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def ctrl_viol(us, umin, umax):
    v = 0.0
    for u in us:
        uu = np.asarray(u, float).reshape(-1)
        v = max(v, float(np.max(np.maximum(0.0, uu - umax))))
        v = max(v, float(np.max(np.maximum(0.0, umin - uu))))
    return v


def eval_cost(running, terminal, xs, us):
    cost = 0.0
    for x, u in zip(xs[:-1], us):
        d = running.createData()
        running.calc(d, x, u)
        cost += float(d.cost)
    td = terminal.createData()
    terminal.calc(td, xs[-1])
    return cost + float(td.cost)


def run_case(name, attach_fn, xs0, us0, running, terminal, croco_pb, mu=0.1):
    prox = convertCrocoddylProblem(croco_pb)
    attach_fn(prox)
    print(f"\n=== {name} dual0={prox.stages[0].num_dual} mu={mu} ===")
    umin = np.asarray(running.u_lb, float)
    umax = np.asarray(running.u_ub, float)
    us_w = [np.clip(np.asarray(u, float), umin, umax).copy() for u in us0]
    xs_w = [np.asarray(xs0[0], float).copy()]
    for u in us_w:
        d = running.createData()
        running.calc(d, xs_w[-1], u)
        xs_w.append(np.asarray(d.xnext, float).copy())

    solver = SolverProxDDP(1e-3, mu, max_iters=150, verbose=aligator.QUIET)
    if hasattr(aligator, "SA_FILTER"):
        solver.sa_strategy = aligator.SA_FILTER
    solver.setup(prox)
    ok = bool(solver.run(prox, xs_w, us_w))
    res = solver.results
    xs = [np.asarray(x, float) for x in res.xs]
    us = [np.asarray(u, float) for u in res.us]
    cost = eval_cost(running, terminal, xs, us)
    cv = ctrl_viol(us, umin, umax)
    print(
        f"ok={ok} cost={cost:.4f} traj_cost={float(res.traj_cost):.4f} "
        f"max|u|={max(float(np.max(np.abs(u))) for u in us):.4f} "
        f"ctrl_viol={cv:.6f} iters={res.num_iters} "
        f"dual_infeas={float(res.dual_infeas):.3e}"
    )
    return cost, cv


def main():
    croc = load_croc()
    H, dt = 80, 0.1
    running = croc.NavAM(dt)
    terminal = croc.NavAM(dt, terminal=True)
    xs0, us0 = croc.nav_generate_seeds(0, H, dt)
    croco_pb = crocoddyl.ShootingProblem(xs0[0].copy(), [running] * H, terminal)
    umin = np.asarray(running.u_lb, float)
    umax = np.asarray(running.u_ub, float)
    ndx, nu = 4, 2

    def attach_none(prox):
        return

    def attach_box_control_error(prox):
        fn = aligator.ControlErrorResidual(ndx, np.zeros(nu))
        box = constraints.BoxConstraint(umin, umax)
        for s in prox.stages:
            s.addConstraint(fn, box)

    def attach_control_box_function(prox):
        cbf = aligator.ControlBoxFunction(ndx, umin, umax)
        for s in prox.stages:
            s.addConstraint(cbf, constraints.NegativeOrthant())

    def attach_both(prox):
        attach_box_control_error(prox)
        # don't double-add if same - skip

    run_case("none", attach_none, xs0, us0, running, terminal, croco_pb, mu=0.1)
    run_case(
        "ControlError+Box",
        attach_box_control_error,
        xs0,
        us0,
        running,
        terminal,
        croco_pb,
        mu=0.1,
    )
    run_case(
        "ControlError+Box mu=1",
        attach_box_control_error,
        xs0,
        us0,
        running,
        terminal,
        croco_pb,
        mu=1.0,
    )
    try:
        run_case(
            "ControlBoxFunction+NegOrthant",
            attach_control_box_function,
            xs0,
            us0,
            running,
            terminal,
            croco_pb,
            mu=0.1,
        )
        run_case(
            "ControlBoxFunction+NegOrthant mu=1",
            attach_control_box_function,
            xs0,
            us0,
            running,
            terminal,
            croco_pb,
            mu=1.0,
        )
    except Exception as e:
        print("ControlBoxFunction path failed:", type(e), e)


if __name__ == "__main__":
    main()
