#!/usr/bin/env python3
"""Classical single-shooting iLQR reference for Gate S2 smoke comparison.

This is NOT Crocoddyl/ALTRO. It provides a same-problem classical baseline when
those packages are unavailable, so we can still exercise compare_results.py.
Install Crocoddyl for the real SOTA row (see run_crocoddyl.py).
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np

H_PEND = 40
DT_PEND = 0.05
H_DI = 40
DT_DI = 0.1
H_NAV = 80
DT_NAV = 0.1
TOL = 1e-3


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "solver", "problem", "seed", "converged", "feasible", "termination_reason",
        "final_cost", "eq_violation", "ineq_violation", "dynamics_violation",
        "iterations", "time_ms", "chi",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def ilqr_lqr_backward(A, B, q_x, q_u, Q_xx, Q_uu, Q_xu, Vx, Vxx, reg):
    Quu = Q_uu + B.T @ Vxx @ B + reg * np.eye(B.shape[1])
    Qux = Q_xu.T + B.T @ Vxx @ A
    Qu = q_u + B.T @ Vx
    # Regularize if needed
    for _ in range(8):
        try:
            chol = np.linalg.cholesky(Quu)
            break
        except np.linalg.LinAlgError:
            Quu = Quu + (10 ** _) * reg * np.eye(Quu.shape[0])
    else:
        return None
    K = -np.linalg.solve(Quu, Qux)
    k = -np.linalg.solve(Quu, Qu)
    Qx = q_x + A.T @ Vx
    Qxx = Q_xx + A.T @ Vxx @ A
    Vx_new = Qx + K.T @ Quu @ k + K.T @ Qu + Qux.T @ k
    Vxx_new = Qxx + K.T @ Quu @ K + K.T @ Qux + Qux.T @ K
    Vxx_new = 0.5 * (Vxx_new + Vxx_new.T)
    return K, k, Vx_new, Vxx_new


def run_ilqr(rollout_fn, cost_fn, deriv_fn, x0, u, max_iter=80, reg=1e-4):
    t0 = time.perf_counter()
    xs = rollout_fn(x0, u)
    cost = cost_fn(xs, u)
    for it in range(max_iter):
        derivs = deriv_fn(xs, u)
        Vx = derivs["terminal_qx"].copy()
        Vxx = derivs["terminal_Qxx"].copy()
        Ks, ks = [], []
        failed = False
        for k in range(len(u) - 1, -1, -1):
            out = ilqr_lqr_backward(
                derivs["A"][k], derivs["B"][k], derivs["qx"][k], derivs["qu"][k],
                derivs["Qxx"][k], derivs["Quu"][k], derivs["Qxu"][k],
                Vx, Vxx, reg)
            if out is None:
                failed = True
                break
            K, kk, Vx, Vxx = out
            Ks.append(K)
            ks.append(kk)
        if failed:
            reg = min(reg * 10, 1e4)
            continue
        Ks.reverse()
        ks.reverse()
        accepted = False
        step = 1.0
        while step >= 1e-3:
            u_try = []
            x = x0.copy()
            xs_try = [x.copy()]
            for k in range(len(u)):
                du = step * ks[k] + Ks[k] @ (x - xs[k])
                uk = u[k] + du
                u_try.append(uk)
                x = rollout_fn(x, [uk])[1]
                xs_try.append(x.copy())
            c_try = cost_fn(xs_try, u_try)
            if c_try <= cost:
                u = u_try
                xs = xs_try
                cost = c_try
                accepted = True
                reg = max(1e-6, reg * 0.5)
                break
            step *= 0.5
        if not accepted:
            reg = min(reg * 10, 1e4)
        if abs(cost) < 1e-8:
            break
    ms = (time.perf_counter() - t0) * 1000
    return xs, u, cost, it + 1, ms


def pendulum(seed: int) -> dict:
    m, g, l = 1.0, 9.81, 0.5
    I = m * l * l
    H, dt = H_PEND, DT_PEND
    x0 = np.array([math.pi, 0.0])

    def f(x, u):
        th, w = x
        return np.array([w, (m * g * l * math.sin(th) + u[0]) / I])

    def rollout(x, us):
        xs = [np.asarray(x, float).copy()]
        for u in us:
            k1 = f(xs[-1], u)
            k2 = f(xs[-1] + 0.5 * dt * k1, u)
            k3 = f(xs[-1] + 0.5 * dt * k2, u)
            k4 = f(xs[-1] + dt * k3, u)
            xs.append(xs[-1] + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4))
        return xs

    def cost(xs, us):
        c = 0.0
        Q = np.diag([10.0, 0.1])
        R = np.array([[0.01]])
        Qf = np.diag([500.0, 100.0])
        for x, u in zip(xs[:-1], us):
            c += 0.5 * x @ Q @ x + 0.5 * u @ R @ u
        c += 0.5 * xs[-1] @ Qf @ xs[-1]
        return c

    def deriv(xs, us):
        A, B, qx, qu, Qxx, Quu, Qxu = [], [], [], [], [], [], []
        Q = np.diag([10.0, 0.1])
        R = np.array([[0.01]])
        for x, u in zip(xs[:-1], us):
            th = x[0]
            Ac = np.array([[0.0, 1.0], [m * g * l * math.cos(th) / I, 0.0]])
            Bc = np.array([[0.0], [1.0 / I]])
            # RK4 linearization approx via Euler map for reference speed
            A.append(np.eye(2) + dt * Ac)
            B.append(dt * Bc)
            qx.append(Q @ x)
            qu.append(R @ u)
            Qxx.append(Q.copy())
            Quu.append(R.copy())
            Qxu.append(np.zeros((2, 1)))
        Qf = np.diag([500.0, 100.0])
        return {
            "A": A, "B": B, "qx": qx, "qu": qu, "Qxx": Qxx, "Quu": Quu, "Qxu": Qxu,
            "terminal_qx": Qf @ xs[-1], "terminal_Qxx": Qf,
        }

    u = [np.zeros(1) for _ in range(H)]
    xs, u, cost_v, iters, ms = run_ilqr(rollout, cost, deriv, x0, u, max_iter=80)
    return dict(
        solver="classical_ilqr", problem="pendulum_swingup", seed=seed,
        converged=0, feasible=1, termination_reason="MAX_ITER",
        final_cost=cost_v, eq_violation=0, ineq_violation=0, dynamics_violation=0,
        iterations=iters, time_ms=ms, chi=0,
    )


def double_integrator(seed: int) -> dict:
    H, dt = H_DI, DT_DI
    x0 = np.zeros(2)

    def rollout(x, us):
        xs = [np.asarray(x, float).copy()]
        A = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([[0.5 * dt * dt], [dt]])
        for u in us:
            xs.append(A @ xs[-1] + B @ u)
        return xs

    def cost(xs, us):
        c = 0.0
        for x, u in zip(xs[:-1], us):
            c += 0.5 * x @ x + 0.05 * float(u @ u) + 0.5  # match scalar offset loosely
            c += -x[0]  # linear term from q_x = -[1,0]
        xf = xs[-1]
        c += 50.0 * xf @ xf - 100.0 * xf[0] + 50.0
        return c

    def deriv(xs, us):
        A = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([[0.5 * dt * dt], [dt]])
        As, Bs, qx, qu, Qxx, Quu, Qxu = [], [], [], [], [], [], []
        for x, u in zip(xs[:-1], us):
            As.append(A.copy())
            Bs.append(B.copy())
            qx.append(x + np.array([-1.0, 0.0]))
            qu.append(0.1 * u)
            Qxx.append(np.eye(2))
            Quu.append(np.array([[0.1]]))
            Qxu.append(np.zeros((2, 1)))
        Qf = 100 * np.eye(2)
        return {
            "A": As, "B": Bs, "qx": qx, "qu": qu, "Qxx": Qxx, "Quu": Quu, "Qxu": Qxu,
            "terminal_qx": Qf @ xs[-1] + np.array([-100.0, 0.0]),
            "terminal_Qxx": Qf,
        }

    u = [np.zeros(1) for _ in range(H)]
    xs, u, cost_v, iters, ms = run_ilqr(rollout, cost, deriv, x0, u, max_iter=60)
    return dict(
        solver="classical_ilqr", problem="double_integrator_reach", seed=seed,
        converged=0, feasible=1, termination_reason="MAX_ITER",
        final_cost=cost_v, eq_violation=0, ineq_violation=0, dynamics_violation=0,
        iterations=iters, time_ms=ms, chi=0,
    )


def nav(seed: int) -> dict:
    """Near-straight bicycle rollout + light iLQR (box via clip)."""
    H, dt = H_NAV, DT_NAV
    x0 = np.zeros(4)
    target = np.array([10.0, 0.0, 0.0, 0.0])
    vmax, dkm, km = 3.0, 2.0, 1.5

    def f(x, u):
        v, dk = u
        th, kap = x[2], x[3]
        return np.array([v * math.cos(th), v * math.sin(th), v * kap, dk])

    def rollout(x, us):
        xs = [np.asarray(x, float).copy()]
        for u in us:
            uu = np.array([np.clip(u[0], -vmax, vmax), np.clip(u[1], -dkm, dkm)])
            k1 = f(xs[-1], uu)
            k2 = f(xs[-1] + 0.5 * dt * k1, uu)
            k3 = f(xs[-1] + 0.5 * dt * k2, uu)
            k4 = f(xs[-1] + dt * k3, uu)
            xn = xs[-1] + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            xn[3] = np.clip(xn[3], -km, km)
            xs.append(xn)
        return xs

    def stage_cost(x, u):
        Q = np.diag([1.0, 1.0, 0.5, 0.1])
        R = np.diag([0.1, 0.5])
        c = 0.5 * x @ Q @ x + 0.5 * u @ R @ u
        for ox, oy, sig, w in ((3.5, 1.5, 0.35, 50.0), (6.0, -1.2, 0.35, 50.0), (8.0, 0.8, 0.35, 50.0)):
            d2 = (x[0] - ox) ** 2 + (x[1] - oy) ** 2
            c += w * math.exp(-d2 / (2 * sig * sig))
        return c

    def cost(xs, us):
        c = sum(stage_cost(x, u) for x, u in zip(xs[:-1], us))
        e = xs[-1] - target
        Qf = np.diag([1000.0, 1000.0, 200.0, 50.0])
        return c + 0.5 * e @ Qf @ e

    def deriv(xs, us):
        A, B, qx, qu, Qxx, Quu, Qxu = [], [], [], [], [], [], []
        Q = np.diag([1.0, 1.0, 0.5, 0.1])
        R = np.diag([0.1, 0.5])
        for x, u in zip(xs[:-1], us):
            th, kap, v = x[2], x[3], u[0]
            Ac = np.zeros((4, 4))
            Ac[0, 2] = -v * math.sin(th)
            Ac[1, 2] = v * math.cos(th)
            Ac[2, 3] = v
            Bc = np.zeros((4, 2))
            Bc[0, 0] = math.cos(th)
            Bc[1, 0] = math.sin(th)
            Bc[2, 0] = kap
            Bc[3, 1] = 1.0
            A.append(np.eye(4) + dt * Ac)
            B.append(dt * Bc)
            qx.append(Q @ x)
            qu.append(R @ u)
            Qxx.append(Q.copy())
            Quu.append(R.copy())
            Qxu.append(np.zeros((4, 2)))
        e = xs[-1] - target
        Qf = np.diag([1000.0, 1000.0, 200.0, 50.0])
        return {
            "A": A, "B": B, "qx": qx, "qu": qu, "Qxx": Qxx, "Quu": Quu, "Qxu": Qxu,
            "terminal_qx": Qf @ e, "terminal_Qxx": Qf,
        }

    v_nom = 10.0 / (H * dt)
    u = [np.array([v_nom, 0.0]) for _ in range(H)]
    # diversify by seed
    if seed % 2 == 1:
        for k in range(H):
            tau = k / max(1, H - 1)
            u[k][1] = -0.008 * math.sin(2 * math.pi * tau) / dt
    xs, u, cost_v, iters, ms = run_ilqr(rollout, cost, deriv, x0, u, max_iter=80)
    xf = xs[-1]
    term = max(
        xf[0] - 10.5, 9.5 - xf[0], xf[1] - 0.3, -0.3 - xf[1],
        xf[2] - 0.3, -0.3 - xf[2], xf[3] - 0.5, -0.5 - xf[3], 0.0,
    )
    feas = int(term <= TOL)
    return dict(
        solver="classical_ilqr", problem="nav_2d_bicycle", seed=seed,
        converged=0, feasible=feas, termination_reason="MAX_ITER",
        final_cost=cost_v, eq_violation=0, ineq_violation=term, dynamics_violation=0,
        iterations=iters, time_ms=ms, chi=0,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rows = []
    for seed in (0, 1, 2):
        rows.append(pendulum(seed))
        rows.append(double_integrator(seed))
        rows.append(nav(seed))
    write_rows(Path(args.out), rows)
    print(f"Wrote {len(rows)} rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
