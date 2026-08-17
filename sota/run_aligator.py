#!/usr/bin/env python3
"""Fair Aligator ProxDDP Gate runner.

Pendulum / DI: convertCrocoddylProblem (no control limits — fair).
Nav: native Aligator (BikeODE + IntegratorRK2 + ControlBoxFunction boxes
     + soft-obstacle cost). convertCrocoddylProblem drops u_lb/u_ub.
     Feasibility uses the same control/kappa/terminal ineq as Gate.

Usage:
  python3 scripts/sota/run_aligator.py \\
    --out aligator.csv
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import time
from pathlib import Path

import numpy as np

TOL = 1e-3

_NAV_Q = np.diag([1.0, 1.0, 0.5, 0.1])
_NAV_R = np.diag([0.1, 0.5])
_NAV_Qf = np.diag([1000.0, 1000.0, 200.0, 50.0])
_NAV_TARGET = np.array([10.0, 0.0, 0.0, 0.0])
_NAV_OBS = (
    (3.5, 1.5, 0.35, 50.0),
    (6.0, -1.2, 0.35, 50.0),
    (8.0, 0.8, 0.35, 50.0),
)
_NAV_UMIN = np.array([-3.0, -2.0])
_NAV_UMAX = np.array([3.0, 2.0])
_NAV_KMAX = 1.5


def _load_croc_module():
    path = Path(__file__).resolve().parent / "run_crocoddyl.py"
    spec = importlib.util.spec_from_file_location("run_crocoddyl", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _require_aligator():
    try:
        import aligator
        from aligator import SolverProxDDP
        from aligator.croc import convertCrocoddylProblem
    except ImportError as e:
        raise SystemExit(
            "aligator Python bindings not found.\n"
            "  conda install -c conda-forge aligator\n"
            f"Import error: {e}"
        ) from e
    return aligator, SolverProxDDP, convertCrocoddylProblem


def _traj_cost_croc(problem, xs, us) -> float:
    cost = 0.0
    for t, (x, u) in enumerate(zip(xs[:-1], us)):
        m = problem.runningModels[t]
        d = m.createData()
        m.calc(d, x, u)
        cost += float(d.cost)
    tm = problem.terminalModel
    td = tm.createData()
    tm.calc(td, xs[-1])
    cost += float(td.cost)
    return cost


def _control_box_violation(us, u_lb, u_ub) -> float:
    viol = 0.0
    for u in us:
        uu = np.asarray(u, dtype=float).reshape(-1)
        viol = max(viol, float(np.max(np.maximum(0.0, uu - u_ub))))
        viol = max(viol, float(np.max(np.maximum(0.0, u_lb - uu))))
    return viol


def _nav_cost(xs, us) -> float:
    """Same scalar as NavAM (discrete; no dt scaling)."""
    cost = 0.0
    for x, u in zip(xs[:-1], us):
        xx = np.asarray(x, float).reshape(-1)
        uu = np.asarray(u, float).reshape(-1)
        c = 0.5 * float(xx @ _NAV_Q @ xx + uu @ _NAV_R @ uu)
        for ox, oy, sig, w in _NAV_OBS:
            d2 = (xx[0] - ox) ** 2 + (xx[1] - oy) ** 2
            c += w * math.exp(-d2 / (2 * sig * sig))
        cost += c
    e = np.asarray(xs[-1], float).reshape(-1) - _NAV_TARGET
    cost += 0.5 * float(e @ _NAV_Qf @ e)
    return cost


def _nav_ineq(xs, us) -> float:
    viol = _control_box_violation(us, _NAV_UMIN, _NAV_UMAX)
    for x in xs:
        xx = np.asarray(x, float).reshape(-1)
        viol = max(viol, max(0.0, abs(xx[3]) - _NAV_KMAX))
    xf = np.asarray(xs[-1], float).reshape(-1)
    viol = max(viol, max(0.0, abs(xf[0] - 10.0) - 0.5))
    viol = max(viol, max(0.0, abs(xf[1]) - 0.3))
    viol = max(viol, max(0.0, abs(xf[2]) - 0.3))
    viol = max(viol, max(0.0, abs(xf[3]) - 0.5))
    return viol


def _solve_prox_converted(croc_problem, xs0, us0, max_iter: int, mu_init: float = 1e-2):
    aligator, SolverProxDDP, convert = _require_aligator()
    prox_pb = convert(croc_problem)
    solver = SolverProxDDP(TOL, mu_init, max_iters=max_iter, verbose=aligator.QUIET)
    if hasattr(aligator, "SA_FILTER"):
        solver.sa_strategy = aligator.SA_FILTER
    solver.setup(prox_pb)
    t0 = time.perf_counter()
    ok = bool(solver.run(prox_pb, xs0, us0))
    ms = (time.perf_counter() - t0) * 1000.0
    res = solver.results
    xs = [np.asarray(x, dtype=float).copy() for x in res.xs]
    us = [np.asarray(u, dtype=float).copy() for u in res.us]
    cost = _traj_cost_croc(croc_problem, xs, us)
    iters = int(getattr(res, "num_iters", max_iter))
    return ok, cost, xs, us, iters, ms


def _build_nav_native(aligator, H: int, dt: float, x0: np.ndarray):
    from aligator import constraints, dynamics, manifolds

    space = manifolds.VectorSpace(4)
    nu = 2

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

    dyn_model = dynamics.IntegratorRK2(BikeODE(), dt)

    class SoftObsCost(aligator.CostAbstract):
        """Exact Gate soft-obstacle term (discrete, no dt factor)."""

        def __init__(self):
            aligator.CostAbstract.__init__(self, space, nu)

        def evaluate(self, x, u, data):
            c = 0.0
            for ox, oy, sig, w in _NAV_OBS:
                d2 = (float(x[0]) - ox) ** 2 + (float(x[1]) - oy) ** 2
                c += w * math.exp(-d2 / (2 * sig * sig))
            data.value = c

        def computeGradients(self, x, u, data):
            lx = np.zeros(4)
            for ox, oy, sig, w in _NAV_OBS:
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
            for ox, oy, sig, w in _NAV_OBS:
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

    # Discrete quadratic stage cost matching NavAM (no dt scaling).
    rcost = aligator.CostStack(space, nu)
    rcost.addCost(aligator.QuadraticStateCost(space, nu, np.zeros(4), _NAV_Q))
    rcost.addCost(aligator.QuadraticControlCost(space, np.zeros(nu), _NAV_R))
    rcost.addCost(SoftObsCost())

    term_cost = aligator.CostStack(space, nu)
    term_cost.addCost(
        aligator.QuadraticStateCost(space, nu, _NAV_TARGET, _NAV_Qf)
    )

    # ControlBoxFunction + NegativeOrthant is the API that ProxDDP actually
    # drives to hard box satisfaction (ControlError+BoxConstraint alone stalls).
    ctrl_box_fn = aligator.ControlBoxFunction(space.ndx, _NAV_UMIN, _NAV_UMAX)

    stages = []
    for _ in range(H):
        stage = aligator.StageModel(rcost, dyn_model)
        stage.addConstraint(ctrl_box_fn, constraints.NegativeOrthant())
        stages.append(stage)

    problem = aligator.TrajOptProblem(x0, stages, term_cost)
    return problem


def _nav_rk2_rollout(x0: np.ndarray, us, dt: float = 0.1):
    """Match BikeODE + IntegratorRK2 used in the native Aligator problem."""
    xs = [np.asarray(x0, float).copy()]
    x = xs[0].copy()
    for u in us:
        uu = np.asarray(u, float).reshape(-1)

        def f(xk, uk):
            th, kap = float(xk[2]), float(xk[3])
            v, dk = float(uk[0]), float(uk[1])
            return np.array([v * np.cos(th), v * np.sin(th), v * kap, dk])

        k1 = f(x, uu)
        k2 = f(x + 0.5 * dt * k1, uu)
        x = x + dt * k2
        xs.append(x.copy())
    return xs


def _solve_prox_once(
    aligator,
    SolverProxDDP,
    problem,
    xs0,
    us0,
    mu_init,
    max_iter,
    max_al_iters,
    sa,
    u_lb,
    u_ub,
):
    us_w = [np.clip(np.asarray(u, float), u_lb, u_ub).copy() for u in us0]
    xs_w = [np.asarray(x, float).copy() for x in xs0]
    solver = SolverProxDDP(1e-4, mu_init, max_iters=max_iter, verbose=aligator.QUIET)
    solver.max_al_iters = max_al_iters
    if sa is not None:
        solver.sa_strategy = sa
    solver.setup(problem)
    t0 = time.perf_counter()
    ok = bool(solver.run(problem, xs_w, us_w))
    ms = (time.perf_counter() - t0) * 1000.0
    res = solver.results
    xs = [np.asarray(x, dtype=float).reshape(-1).copy() for x in list(res.xs)]
    us = [np.asarray(u, dtype=float).reshape(-1).copy() for u in list(res.us)]
    iters = int(getattr(res, "num_iters", max_iter))
    return ok, xs, us, iters, ms


def _solve_nav_once(aligator, SolverProxDDP, problem, xs0, us0, mu_init, max_iter, max_al_iters, sa):
    return _solve_prox_once(
        aligator,
        SolverProxDDP,
        problem,
        xs0,
        us0,
        mu_init,
        max_iter,
        max_al_iters,
        sa,
        _NAV_UMIN,
        _NAV_UMAX,
    )

def _feasible_recovery(xs, us, x0):
    """If ProxDDP leaves a mild box residual, clip u and re-integrate (RK2)."""
    us_c = [np.clip(np.asarray(u, float), _NAV_UMIN, _NAV_UMAX).copy() for u in us]
    xs_c = _nav_rk2_rollout(x0, us_c)
    return xs_c, us_c


def _solve_nav_native(xs0, us0):
    """Fair Nav solve: try a few ProxDDP configs; keep best Gate-feasible cost.

    ProxDDP is AL (soft). When raw traj misses TOL, we accept clip+RK2 re-rollout
    if that trajectory is feasible under the same Gate ineq check — comparable to
    BoxFDDP's hard projection, without dropping the control box from the problem.
    """
    aligator, SolverProxDDP, _ = _require_aligator()
    H = len(us0)
    dt = 0.1
    x0 = np.asarray(xs0[0], float).copy()
    problem = _build_nav_native(aligator, H, dt, x0)

    configs = []
    saf = getattr(aligator, "SA_FILTER", None)
    san = getattr(aligator, "SA_LINESEARCH_NONMONOTONE", None)
    configs.append((1e-2, 400, 300, saf))
    if san is not None:
        configs.append((1e-2, 400, 300, san))
    configs.append((1.0, 200, 200, saf))

    best_feas = None
    best_any = None
    total_ms = 0.0
    total_iters = 0

    for mu, mit, mal, sa in configs:
        ok, xs, us, iters, ms = _solve_nav_once(
            aligator, SolverProxDDP, problem, xs0, us0, mu, mit, mal, sa
        )
        total_ms += ms
        total_iters += iters
        cost = _nav_cost(xs, us)
        ineq = _nav_ineq(xs, us)
        cand = (ok, cost, xs, us, total_iters, total_ms, ineq)
        if best_any is None or cost < best_any[1]:
            best_any = cand
        if ineq <= TOL:
            if best_feas is None or cost < best_feas[1]:
                best_feas = cand
            continue
        # Mild residual → project controls and re-roll with matching dynamics.
        xs_c, us_c = _feasible_recovery(xs, us, x0)
        cost_c = _nav_cost(xs_c, us_c)
        ineq_c = _nav_ineq(xs_c, us_c)
        if ineq_c <= TOL and (best_feas is None or cost_c < best_feas[1]):
            best_feas = (False, cost_c, xs_c, us_c, total_iters, total_ms, ineq_c)

    chosen = best_feas if best_feas is not None else best_any
    assert chosen is not None
    ok, cost, xs, us, iters, ms, _ineq = chosen
    return ok, cost, xs, us, iters, ms


def pendulum(croc, seed: int) -> dict:
    import crocoddyl

    H, dt = 40, 0.05
    problem = crocoddyl.ShootingProblem(
        np.array([math.pi, 0.0]),
        [croc.PendulumAM(dt)] * H,
        croc.PendulumAM(dt, terminal=True),
    )
    xs = [problem.x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    ok, cost, _, _, iters, ms = _solve_prox_converted(problem, xs, us, 80)
    return dict(
        solver="aligator",
        problem="pendulum_swingup",
        seed=seed,
        converged=int(ok),
        feasible=1,
        termination_reason="CONVERGED" if ok else "MAX_ITER",
        final_cost=cost,
        eq_violation=0.0,
        ineq_violation=0.0,
        dynamics_violation=0.0,
        iterations=iters,
        time_ms=ms,
        chi=0.0,
    )


def double_integrator(croc, seed: int) -> dict:
    import crocoddyl

    H, dt = 40, 0.1
    problem = crocoddyl.ShootingProblem(
        np.zeros(2), [croc.DIAM(dt)] * H, croc.DIAM(dt, terminal=True)
    )
    xs = [problem.x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    ok, cost, _, _, iters, ms = _solve_prox_converted(problem, xs, us, 60)
    return dict(
        solver="aligator",
        problem="double_integrator_reach",
        seed=seed,
        converged=int(ok),
        feasible=1,
        termination_reason="CONVERGED" if ok else "MAX_ITER",
        final_cost=cost,
        eq_violation=0.0,
        ineq_violation=0.0,
        dynamics_violation=0.0,
        iterations=iters,
        time_ms=ms,
        chi=0.0,
    )


def _max_g_violation(croc_problem, xs, us) -> float:
    """Delegate to run_crocoddyl helper when available."""
    m = 0.0
    xs_list = [np.asarray(x, dtype=float).reshape(-1) for x in xs]
    us_list = [np.asarray(u, dtype=float).reshape(-1) for u in us]
    tdata = croc_problem.terminalModel.createData()
    croc_problem.terminalModel.calc(tdata, xs_list[-1], None)
    if int(getattr(croc_problem.terminalModel, "ng", 0) or 0) > 0:
        m = max(m, float(np.max(np.asarray(tdata.g, dtype=float))))
    rmodel = croc_problem.runningModels[0]
    rdata = rmodel.createData()
    for x, u in zip(xs_list[:-1], us_list):
        rmodel.calc(rdata, x, u)
        if int(getattr(rmodel, "ng", 0) or 0) > 0:
            m = max(m, float(np.max(np.asarray(rdata.g, dtype=float))))
    return max(0.0, m)


def _row_constrained(
    problem_name: str,
    croc_problem,
    xs0,
    us0,
    max_iter: int,
    ok,
    cost,
    xs,
    us,
    iters,
    ms,
) -> dict:
    ineq = _max_g_violation(croc_problem, xs, us)
    feas = int(ineq <= TOL)
    return dict(
        solver="aligator",
        problem=problem_name,
        seed=0,  # overwritten by caller
        converged=int(ok),
        feasible=feas,
        termination_reason="CONVERGED" if ok else "MAX_ITER",
        final_cost=cost,
        eq_violation=0.0,
        ineq_violation=ineq,
        dynamics_violation=0.0,
        iterations=iters,
        time_ms=ms,
        chi=0.0,
    )


def cartpole_stabilize(croc, seed: int) -> dict:
    import crocoddyl

    H, dt = 50, 0.02
    x0 = croc.cartpole_initial(seed)
    running = croc.CartpoleAM(dt)
    terminal = croc.CartpoleAM(dt, terminal=True)
    problem = crocoddyl.ShootingProblem(x0, [running] * H, terminal)
    xs = [x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    ok, cost, xs_sol, us_sol, iters, ms = _solve_prox_converted(
        problem, xs, us, 100, mu_init=1e-2
    )
    row = _row_constrained(
        "cartpole_stabilize", problem, xs, us, 100, ok, cost, xs_sol, us_sol, iters, ms
    )
    row["seed"] = seed
    return row


_QUAD_Q = np.diag([50.0, 50.0, 1.0, 1.0, 20.0, 0.5])
_QUAD_R = np.eye(2) * 0.01
_QUAD_Qf = np.eye(6) * 100.0
_QUAD_UMIN = np.zeros(2)
_QUAD_UMAX = np.array([15.0, 15.0])
_QUAD_MASS = 0.5
_QUAD_G = 9.81
_QUAD_ARM = 0.15
_QUAD_IYY = 0.002


def _quad_cost(xs, us) -> float:
    cost = 0.0
    for x, u in zip(xs[:-1], us):
        xx = np.asarray(x, float).reshape(-1)
        uu = np.asarray(u, float).reshape(-1)
        cost += 0.5 * float(xx @ _QUAD_Q @ xx + uu @ _QUAD_R @ uu)
    xf = np.asarray(xs[-1], float).reshape(-1)
    cost += 0.5 * float(xf @ _QUAD_Qf @ xf)
    return cost


def _quad_ineq(us) -> float:
    return _control_box_violation(us, _QUAD_UMIN, _QUAD_UMAX)


def _build_quadrotor_native(aligator, H: int, dt: float, x0: np.ndarray):
    """Native ProxDDP quadrotor: RK2 + ControlBoxFunction thrust boxes.

    convertCrocoddylProblem does not enforce u_lb/u_ub; attaching boxes after
    convert is a no-op on dual count. Match Gate cost weights (no dt scaling).
    """
    from aligator import constraints, dynamics, manifolds

    space = manifolds.VectorSpace(6)
    nu = 2

    class QuadODE(dynamics.ODEAbstract):
        def __init__(self):
            dynamics.ODEAbstract.__init__(self, space, nu)

        def forward(self, x, u, data):
            phi, omg = float(x[4]), float(x[5])
            t1, t2 = float(u[0]), float(u[1])
            sp, cp = math.sin(phi), math.cos(phi)
            tot = t1 + t2
            data.xdot[:] = [
                float(x[2]),
                float(x[3]),
                -tot * sp / _QUAD_MASS,
                tot * cp / _QUAD_MASS - _QUAD_G,
                omg,
                _QUAD_ARM * (t1 - t2) / _QUAD_IYY,
            ]

        def dForward(self, x, u, data):
            phi = float(x[4])
            t1, t2 = float(u[0]), float(u[1])
            tot = t1 + t2
            sp, cp = math.sin(phi), math.cos(phi)
            data.Jx[:, :] = 0.0
            data.Ju[:, :] = 0.0
            data.Jx[0, 2] = 1.0
            data.Jx[1, 3] = 1.0
            data.Jx[2, 4] = -tot * cp / _QUAD_MASS
            data.Jx[3, 4] = -tot * sp / _QUAD_MASS
            data.Jx[4, 5] = 1.0
            data.Ju[2, 0] = -sp / _QUAD_MASS
            data.Ju[2, 1] = -sp / _QUAD_MASS
            data.Ju[3, 0] = cp / _QUAD_MASS
            data.Ju[3, 1] = cp / _QUAD_MASS
            data.Ju[5, 0] = _QUAD_ARM / _QUAD_IYY
            data.Ju[5, 1] = -_QUAD_ARM / _QUAD_IYY

    dyn_model = dynamics.IntegratorRK2(QuadODE(), dt)
    rcost = aligator.CostStack(space, nu)
    rcost.addCost(aligator.QuadraticStateCost(space, nu, np.zeros(6), _QUAD_Q))
    rcost.addCost(aligator.QuadraticControlCost(space, np.zeros(nu), _QUAD_R))
    term_cost = aligator.CostStack(space, nu)
    term_cost.addCost(aligator.QuadraticStateCost(space, nu, np.zeros(6), _QUAD_Qf))
    ctrl_box_fn = aligator.ControlBoxFunction(space.ndx, _QUAD_UMIN, _QUAD_UMAX)

    stages = []
    for _ in range(H):
        stage = aligator.StageModel(rcost, dyn_model)
        stage.addConstraint(ctrl_box_fn, constraints.NegativeOrthant())
        stages.append(stage)
    return aligator.TrajOptProblem(x0, stages, term_cost)


def _quad_rk2_rollout(x0: np.ndarray, us, dt: float = 0.02):
    xs = [np.asarray(x0, float).copy()]
    x = xs[0].copy()
    for u in us:
        uu = np.asarray(u, float).reshape(-1)

        def f(xk, uk):
            phi, omg = float(xk[4]), float(xk[5])
            t1, t2 = float(uk[0]), float(uk[1])
            sp, cp = math.sin(phi), math.cos(phi)
            tot = t1 + t2
            return np.array(
                [
                    float(xk[2]),
                    float(xk[3]),
                    -tot * sp / _QUAD_MASS,
                    tot * cp / _QUAD_MASS - _QUAD_G,
                    omg,
                    _QUAD_ARM * (t1 - t2) / _QUAD_IYY,
                ]
            )

        k1 = f(x, uu)
        k2 = f(x + 0.5 * dt * k1, uu)
        x = x + dt * k2
        xs.append(x.copy())
    return xs


def _solve_quadrotor_native(xs0, us0):
    """Fair quadrotor: native ControlBoxFunction + multi-config + clip recovery."""
    aligator, SolverProxDDP, _ = _require_aligator()
    H = len(us0)
    dt = 0.02
    x0 = np.asarray(xs0[0], float).copy()
    problem = _build_quadrotor_native(aligator, H, dt, x0)

    configs = []
    saf = getattr(aligator, "SA_FILTER", None)
    san = getattr(aligator, "SA_LINESEARCH_NONMONOTONE", None)
    configs.append((1e-2, 200, 300, saf))
    if san is not None:
        configs.append((1e-2, 200, 300, san))
    configs.append((1.0, 150, 200, saf))

    best_feas = None
    best_any = None
    total_ms = 0.0
    total_iters = 0

    for mu, mit, mal, sa in configs:
        ok, xs, us, iters, ms = _solve_prox_once(
            aligator,
            SolverProxDDP,
            problem,
            xs0,
            us0,
            mu,
            mit,
            mal,
            sa,
            _QUAD_UMIN,
            _QUAD_UMAX,
        )
        total_ms += ms
        total_iters += iters
        cost = _quad_cost(xs, us)
        ineq = _quad_ineq(us)
        cand = (ok, cost, xs, us, total_iters, total_ms, ineq)
        if best_any is None or cost < best_any[1]:
            best_any = cand
        if ineq <= TOL:
            if best_feas is None or cost < best_feas[1]:
                best_feas = cand
            continue
        us_c = [np.clip(u, _QUAD_UMIN, _QUAD_UMAX).copy() for u in us]
        xs_c = _quad_rk2_rollout(x0, us_c, dt)
        cost_c = _quad_cost(xs_c, us_c)
        ineq_c = _quad_ineq(us_c)
        if ineq_c <= TOL and (best_feas is None or cost_c < best_feas[1]):
            best_feas = (False, cost_c, xs_c, us_c, total_iters, total_ms, ineq_c)

    chosen = best_feas if best_feas is not None else best_any
    assert chosen is not None
    ok, cost, xs, us, iters, ms, _ineq = chosen
    return ok, cost, xs, us, iters, ms


def quadrotor_hover(croc, seed: int) -> dict:
    H, dt = 40, 0.02
    x0 = croc.quadrotor_initial(seed)
    hover = _QUAD_MASS * _QUAD_G / 2.0
    xs = [x0.copy() for _ in range(H + 1)]
    us = [np.array([hover, hover]) for _ in range(H)]
    ok, cost, xs_sol, us_sol, iters, ms = _solve_quadrotor_native(xs, us)
    ineq = _quad_ineq(us_sol)
    feas = int(ineq <= TOL)
    return dict(
        solver="aligator",
        problem="quadrotor_hover",
        seed=seed,
        converged=int(ok),
        feasible=feas,
        termination_reason="CONVERGED" if ok else "MAX_ITER",
        final_cost=cost,
        eq_violation=0.0,
        ineq_violation=ineq,
        dynamics_violation=0.0,
        iterations=iters,
        time_ms=ms,
        chi=0.0,
    )


def nav(croc, seed: int) -> dict:
    xs, us = croc.nav_generate_seeds(seed, 80, 0.1)
    ok, cost, xs_sol, us_sol, iters, ms = _solve_nav_native(xs, us)
    ineq = _nav_ineq(xs_sol, us_sol)
    feas = int(ineq <= TOL)
    return dict(
        solver="aligator",
        problem="nav_2d_bicycle",
        seed=seed,
        converged=int(ok),
        feasible=feas,
        termination_reason="CONVERGED" if ok else "MAX_ITER",
        final_cost=cost,
        eq_violation=0.0,
        ineq_violation=ineq,
        dynamics_violation=0.0,
        iterations=iters,
        time_ms=ms,
        chi=0.0,
    )


FIELDS = [
    "solver",
    "problem",
    "seed",
    "converged",
    "feasible",
    "termination_reason",
    "final_cost",
    "eq_violation",
    "ineq_violation",
    "dynamics_violation",
    "iterations",
    "time_ms",
    "chi",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--out",
        default="aligator.csv",
    )
    args = ap.parse_args()
    _require_aligator()
    croc = _load_croc_module()

    rows = []
    for seed in range(3):
        print(f"pendulum seed={seed}", flush=True)
        rows.append(pendulum(croc, seed))
        print(f"DI seed={seed}", flush=True)
        rows.append(double_integrator(croc, seed))
        print(f"cartpole seed={seed}", flush=True)
        rows.append(cartpole_stabilize(croc, seed))
        print(f"quadrotor seed={seed}", flush=True)
        rows.append(quadrotor_hover(croc, seed))
        print(f"nav seed={seed}", flush=True)
        rows.append(nav(croc, seed))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"Wrote {len(rows)} rows to {out}")
    for r in rows:
        if r["problem"] in ("nav_2d_bicycle", "quadrotor_hover"):
            print(
                f"  {r['problem']} seed={r['seed']} cost={r['final_cost']:.4f} "
                f"feas={r['feasible']} ineq={r['ineq_violation']:.4e} "
                f"iters={r['iterations']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
