#!/usr/bin/env python3
"""Run Crocoddyl FDDP on the same Gate-S2 problem set as bench_vs_sota.

Uses discrete ActionModelAbstract + RK4 (Python bindings of DifferentialAction*
allocate wrong xout/Fx shapes when subclassed).

Usage:
  python3 scripts/sota/run_crocoddyl.py \\
    --out crocoddyl.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np

import crocoddyl

TOL = 1e-3


def _fill(dst, src) -> None:
    """Write into Crocoddyl eigenpy buffers that may be 1-D when a dim is 1."""
    arr = np.asarray(src, dtype=float)
    if dst.ndim == 1:
        dst[:] = arr.reshape(-1)
    else:
        dst[:, :] = arr.reshape(dst.shape)


class _ActionData(crocoddyl.ActionDataAbstract):
    """Re-allocate Jacobians — Python ActionDataAbstract defaults are wrong-sized."""

    def __init__(self, model):
        crocoddyl.ActionDataAbstract.__init__(self, model)
        nx, nu = model.state.nx, model.nu
        object.__setattr__(self, "xnext", np.zeros(nx))
        object.__setattr__(self, "Fx", np.zeros((nx, nx)))
        object.__setattr__(self, "Fu", np.zeros((nx, nu)) if nu > 0 else np.zeros((nx, 0)))
        object.__setattr__(self, "Lx", np.zeros(nx))
        object.__setattr__(self, "Lu", np.zeros(nu))
        object.__setattr__(self, "Lxx", np.zeros((nx, nx)))
        object.__setattr__(self, "Lxu", np.zeros((nx, nu)))
        object.__setattr__(self, "Luu", np.zeros((nu, nu)))
        ng = int(getattr(model, "ng", 0) or 0)
        if ng > 0:
            object.__setattr__(self, "g", np.zeros(ng))
            object.__setattr__(self, "Gx", np.zeros((ng, nx)))
            object.__setattr__(self, "Gu", np.zeros((ng, nu)))


def _rk4(f, x, u, dt):
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


class PendulumAM(crocoddyl.ActionModelAbstract):
    def __init__(self, dt=0.05, terminal=False):
        crocoddyl.ActionModelAbstract.__init__(
            self, crocoddyl.StateVector(2), 0 if terminal else 1
        )
        self.dt = dt
        self.terminal = terminal
        self.m, self.g, self.l = 1.0, 9.81, 0.5
        self.I = self.m * self.l * self.l
        self.Q = np.diag([10.0, 0.1])
        self.R = np.array([[0.01]])
        self.Qf = np.diag([500.0, 100.0])

    def createData(self):
        return _ActionData(self)

    def _f(self, x, u):
        th, w = float(x[0]), float(x[1])
        return np.array(
            [w, (self.m * self.g * self.l * math.sin(th) + float(u[0])) / self.I]
        )

    def calc(self, data, x, u=None):
        if self.terminal or u is None:
            data.xnext[:] = x
            data.cost = 0.5 * float(x @ self.Qf @ x)
            return
        data.xnext[:] = _rk4(self._f, x, u, self.dt)
        data.cost = 0.5 * float(x @ self.Q @ x + u @ self.R @ u)

    def calcDiff(self, data, x, u=None):
        if self.terminal or u is None:
            _fill(data.Fx, np.eye(2))
            data.Lx[:] = self.Qf @ x
            _fill(data.Lxx, self.Qf)
            return
        th = float(x[0])
        Ac = np.array(
            [[0.0, 1.0], [self.m * self.g * self.l * math.cos(th) / self.I, 0.0]]
        )
        Bc = np.array([[0.0], [1.0 / self.I]])
        _fill(data.Fx, np.eye(2) + self.dt * Ac)
        _fill(data.Fu, self.dt * Bc)
        data.Lx[:] = self.Q @ x
        data.Lu[:] = (self.R @ u).ravel()
        _fill(data.Lxx, self.Q)
        _fill(data.Luu, self.R)
        _fill(data.Lxu, np.zeros((2, 1)))


class DIAM(crocoddyl.ActionModelAbstract):
    """Discrete double integrator matching optimal_solver DiscreteMap."""

    def __init__(self, dt=0.1, terminal=False):
        crocoddyl.ActionModelAbstract.__init__(
            self, crocoddyl.StateVector(2), 0 if terminal else 1
        )
        self.dt = dt
        self.terminal = terminal
        self.A = np.array([[1.0, dt], [0.0, 1.0]])
        self.B = np.array([[0.5 * dt * dt], [dt]])
        self.Q = np.eye(2)
        self.R = np.array([[0.1]])
        self.qx = np.array([-1.0, 0.0])
        self.Qf = 100.0 * np.eye(2)
        self.qf = np.array([-100.0, 0.0])
        self.scalar = 0.5
        self.term_scalar = 50.0

    def createData(self):
        return _ActionData(self)

    def calc(self, data, x, u=None):
        if self.terminal or u is None:
            data.xnext[:] = x
            data.cost = (
                0.5 * float(x @ self.Qf @ x) + float(self.qf @ x) + self.term_scalar
            )
            return
        data.xnext[:] = self.A @ x + self.B @ u
        data.cost = (
            0.5 * float(x @ self.Q @ x + u @ self.R @ u)
            + float(self.qx @ x)
            + self.scalar
        )

    def calcDiff(self, data, x, u=None):
        if self.terminal or u is None:
            _fill(data.Fx, np.eye(2))
            data.Lx[:] = self.Qf @ x + self.qf
            _fill(data.Lxx, self.Qf)
            return
        _fill(data.Fx, self.A)
        _fill(data.Fu, self.B)
        data.Lx[:] = self.Q @ x + self.qx
        data.Lu[:] = (self.R @ u).ravel()
        _fill(data.Lxx, self.Q)
        _fill(data.Luu, self.R)
        _fill(data.Lxu, np.zeros((2, 1)))


class NavAM(crocoddyl.ActionModelAbstract):
    def __init__(self, dt=0.1, terminal=False):
        # Running: κ box via g (no post-integration clip). Terminal: goal box.
        ng = 8 if terminal else 2
        crocoddyl.ActionModelAbstract.__init__(
            self, crocoddyl.StateVector(4), 0 if terminal else 2, 1, ng, 0
        )
        self.dt = dt
        self.terminal = terminal
        self.Q = np.diag([1.0, 1.0, 0.5, 0.1])
        self.R = np.diag([0.1, 0.5])
        self.Qf = np.diag([1000.0, 1000.0, 200.0, 50.0])
        self.target = np.array([10.0, 0.0, 0.0, 0.0])
        self.obs = (
            (3.5, 1.5, 0.35, 50.0),
            (6.0, -1.2, 0.35, 50.0),
            (8.0, 0.8, 0.35, 50.0),
        )
        self.vmax, self.dkm, self.km = 3.0, 2.0, 1.5
        if not terminal:
            self.u_lb = np.array([-self.vmax, -self.dkm])
            self.u_ub = np.array([self.vmax, self.dkm])

    def createData(self):
        return _ActionData(self)

    def _f(self, x, u):
        th, kap = float(x[2]), float(x[3])
        v, dk = float(u[0]), float(u[1])
        return np.array([v * math.cos(th), v * math.sin(th), v * kap, dk])

    def calc(self, data, x, u=None):
        if self.terminal or u is None:
            data.xnext[:] = x
            e = x - self.target
            data.cost = 0.5 * float(e @ self.Qf @ e)
            data.g[0] = float(x[0] - 10.5)
            data.g[1] = float(9.5 - x[0])
            data.g[2] = float(x[1] - 0.3)
            data.g[3] = float(-0.3 - x[1])
            data.g[4] = float(x[2] - 0.3)
            data.g[5] = float(-0.3 - x[2])
            data.g[6] = float(x[3] - 0.5)
            data.g[7] = float(-0.5 - x[3])
            return
        data.xnext[:] = _rk4(self._f, x, u, self.dt)
        # κ bounds via g (strict parity: no state clip in dynamics).
        data.g[0] = float(x[3] - self.km)
        data.g[1] = float(-x[3] - self.km)
        c = 0.5 * float(x @ self.Q @ x + u @ self.R @ u)
        for ox, oy, sig, w in self.obs:
            d2 = (float(x[0]) - ox) ** 2 + (float(x[1]) - oy) ** 2
            c += w * math.exp(-d2 / (2 * sig * sig))
        data.cost = c

    def calcDiff(self, data, x, u=None):
        if self.terminal or u is None:
            _fill(data.Fx, np.eye(4))
            e = x - self.target
            data.Lx[:] = self.Qf @ e
            _fill(data.Lxx, self.Qf)
            Gx = np.zeros((8, 4))
            Gx[0, 0] = 1.0
            Gx[1, 0] = -1.0
            Gx[2, 1] = 1.0
            Gx[3, 1] = -1.0
            Gx[4, 2] = 1.0
            Gx[5, 2] = -1.0
            Gx[6, 3] = 1.0
            Gx[7, 3] = -1.0
            _fill(data.Gx, Gx)
            return
        th, kap = float(x[2]), float(x[3])
        v = float(u[0])
        Ac = np.zeros((4, 4))
        Ac[0, 2] = -v * math.sin(th)
        Ac[1, 2] = v * math.cos(th)
        Ac[2, 3] = v
        Bc = np.zeros((4, 2))
        Bc[0, 0] = math.cos(th)
        Bc[1, 0] = math.sin(th)
        Bc[2, 0] = kap
        Bc[3, 1] = 1.0
        _fill(data.Fx, np.eye(4) + self.dt * Ac)
        _fill(data.Fu, self.dt * Bc)
        data.Lx[:] = self.Q @ x
        data.Lu[:] = self.R @ u
        Lxx = self.Q.copy()
        for ox, oy, sig, w in self.obs:
            dx = float(x[0]) - ox
            dy = float(x[1]) - oy
            d2 = dx * dx + dy * dy
            e = math.exp(-d2 / (2 * sig * sig))
            data.Lx[0] += -w * dx / (sig * sig) * e
            data.Lx[1] += -w * dy / (sig * sig) * e
            scale = w * e / (2.0 * sig**4)
            Lxx[0, 0] += scale * dx * dx
            Lxx[0, 1] += scale * dx * dy
            Lxx[1, 0] += scale * dy * dx
            Lxx[1, 1] += scale * dy * dy
        _fill(data.Lxx, Lxx)
        _fill(data.Luu, self.R)
        _fill(data.Lxu, np.zeros((4, 2)))
        Gx = np.zeros((2, 4))
        Gx[0, 3] = 1.0
        Gx[1, 3] = -1.0
        _fill(data.Gx, Gx)


def _solve(problem, xs0, us0, max_iter: int, use_box: bool = False):
    solver = (
        crocoddyl.SolverBoxFDDP(problem) if use_box else crocoddyl.SolverFDDP(problem)
    )
    solver.th_stop = 1e-4
    t0 = time.perf_counter()
    done = bool(solver.solve(xs0, us0, max_iter, False))
    ms = (time.perf_counter() - t0) * 1000.0
    return solver, done, ms


def pendulum(seed: int) -> dict:
    H, dt = 40, 0.05
    problem = crocoddyl.ShootingProblem(
        np.array([math.pi, 0.0]), [PendulumAM(dt)] * H, PendulumAM(dt, terminal=True)
    )
    xs = [problem.x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    solver, done, ms = _solve(problem, xs, us, 80)
    return dict(
        solver="crocoddyl",
        problem="pendulum_swingup",
        seed=seed,
        converged=int(done),
        feasible=1,
        termination_reason="CONVERGED" if done else "MAX_ITER",
        final_cost=float(solver.cost),
        eq_violation=0.0,
        ineq_violation=0.0,
        dynamics_violation=float(getattr(solver, "ffeas", 0.0) or 0.0),
        iterations=int(solver.iter),
        time_ms=ms,
        chi=0.0,
    )


def double_integrator(seed: int) -> dict:
    H, dt = 40, 0.1
    problem = crocoddyl.ShootingProblem(
        np.zeros(2), [DIAM(dt)] * H, DIAM(dt, terminal=True)
    )
    xs = [problem.x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    solver, done, ms = _solve(problem, xs, us, 60)
    return dict(
        solver="crocoddyl",
        problem="double_integrator_reach",
        seed=seed,
        converged=int(done),
        feasible=1,
        termination_reason="CONVERGED" if done else "MAX_ITER",
        final_cost=float(solver.cost),
        eq_violation=0.0,
        ineq_violation=0.0,
        dynamics_violation=float(getattr(solver, "ffeas", 0.0) or 0.0),
        iterations=int(solver.iter),
        time_ms=ms,
        chi=0.0,
    )


def _max_g_violation(problem, xs, us) -> float:
    """Max running/terminal inequality residual (g <= 0 convention)."""
    m = 0.0
    xs_list = [np.asarray(x, dtype=float).reshape(-1) for x in xs]
    us_list = [np.asarray(u, dtype=float).reshape(-1) for u in us]
    tdata = problem.terminalModel.createData()
    problem.terminalModel.calc(tdata, xs_list[-1], None)
    if int(getattr(problem.terminalModel, "ng", 0) or 0) > 0:
        m = max(m, float(np.max(np.asarray(tdata.g, dtype=float))))
    rmodel = problem.runningModels[0]
    rdata = rmodel.createData()
    for x, u in zip(xs_list[:-1], us_list):
        rmodel.calc(rdata, x, u)
        if int(getattr(rmodel, "ng", 0) or 0) > 0:
            m = max(m, float(np.max(np.asarray(rdata.g, dtype=float))))
    return max(0.0, m)


class CartpoleAM(crocoddyl.ActionModelAbstract):
    def __init__(self, dt=0.02, terminal=False):
        ng = 4 if terminal else 2
        crocoddyl.ActionModelAbstract.__init__(
            self, crocoddyl.StateVector(4), 0 if terminal else 1, 1, ng, 0
        )
        self.dt = dt
        self.terminal = terminal
        self.mc, self.mp, self.l, self.g = 1.0, 0.1, 0.5, 9.81
        self.fmax = 30.0
        self.Q = np.diag([5.0, 50.0, 0.0, 0.0])
        self.R = np.array([[0.1]])
        self.Qf = np.eye(4) * 500.0
        if not terminal:
            self.u_lb = np.array([-self.fmax])
            self.u_ub = np.array([self.fmax])

    def createData(self):
        return _ActionData(self)

    def _f(self, x, u):
        th, om = float(x[2]), float(x[3])
        F = float(u[0])
        st, ct = math.sin(th), math.cos(th)
        den = self.mc + self.mp * st * st
        vdot = (F + self.mp * self.l * om * om * st - self.mp * self.g * st * ct) / den
        omegadot = (
            F * ct
            + self.mp * self.l * om * om * st * ct
            - (self.mc + self.mp) * self.g * st
            + self.mp * self.g * st * ct * ct
        ) / (self.l * den)
        return np.array([float(x[1]), vdot, om, omegadot])

    def calc(self, data, x, u=None):
        if self.terminal or u is None:
            data.xnext[:] = x
            data.cost = 0.5 * float(x @ self.Qf @ x)
            data.g[0] = float(x[0] - 0.5)
            data.g[1] = float(-x[0] - 0.5)
            data.g[2] = float(x[1] - 0.5)
            data.g[3] = float(-x[1] - 0.5)
            return
        data.xnext[:] = _rk4(self._f, x, u, self.dt)
        data.cost = 0.5 * float(x @ self.Q @ x + u @ self.R @ u)
        data.g[0] = float(u[0] - self.fmax)
        data.g[1] = float(-u[0] - self.fmax)

    def calcDiff(self, data, x, u=None):
        if self.terminal or u is None:
            _fill(data.Fx, np.eye(4))
            data.Lx[:] = self.Qf @ x
            _fill(data.Lxx, self.Qf)
            Gx = np.zeros((4, 4))
            Gx[0, 0] = 1.0
            Gx[1, 0] = -1.0
            Gx[2, 1] = 1.0
            Gx[3, 1] = -1.0
            _fill(data.Gx, Gx)
            return
        th, om = float(x[2]), float(x[3])
        F = float(u[0])
        st, ct = math.sin(th), math.cos(th)
        den = self.mc + self.mp * st * st
        Ac = np.zeros((4, 4))
        Ac[0, 1] = 1.0
        Ac[2, 3] = 1.0
        Ac[1, 2] = -self.mp * om * om * ct / den
        Ac[1, 3] = 2.0 * self.mp * self.l * om * st / den
        Ac[3, 2] = (
            -(self.mc + self.mp) * self.g * ct
            + self.mp * self.g * (ct * ct - st * st)
            + F * st
            - self.mp * self.l * om * om * (st * st - ct * ct)
        ) / (self.l * den)
        Ac[3, 3] = 2.0 * self.mp * om * st * ct / den
        Bc = np.zeros((4, 1))
        Bc[1, 0] = 1.0 / den
        Bc[3, 0] = ct / (self.l * den)
        _fill(data.Fx, np.eye(4) + self.dt * Ac)
        _fill(data.Fu, self.dt * Bc)
        data.Lx[:] = self.Q @ x
        data.Lu[:] = (self.R @ u).ravel()
        _fill(data.Lxx, self.Q)
        _fill(data.Luu, self.R)
        _fill(data.Lxu, np.zeros((4, 1)))
        Gu = np.zeros((2, 1))
        Gu[0, 0] = 1.0
        Gu[1, 0] = -1.0
        _fill(data.Gu, Gu)


class QuadrotorHoverAM(crocoddyl.ActionModelAbstract):
    def __init__(self, dt=0.02, terminal=False):
        ng = 0 if terminal else 4
        crocoddyl.ActionModelAbstract.__init__(
            self, crocoddyl.StateVector(6), 0 if terminal else 2, 1, ng, 0
        )
        self.dt = dt
        self.terminal = terminal
        self.gacc = 9.81
        self.mass = 0.5
        self.arm = 0.15
        self.Iyy = 0.002
        self.tmax = 15.0
        self.Q = np.diag([50.0, 50.0, 1.0, 1.0, 20.0, 0.5])
        self.R = np.eye(2) * 0.01
        self.Qf = np.eye(6) * 100.0
        if not terminal:
            self.u_lb = np.zeros(2)
            self.u_ub = np.array([self.tmax, self.tmax])

    def createData(self):
        return _ActionData(self)

    def _f(self, x, u):
        phi, omg = float(x[4]), float(x[5])
        T1, T2 = float(u[0]), float(u[1])
        sp, cp = math.sin(phi), math.cos(phi)
        T = T1 + T2
        return np.array(
            [
                float(x[2]),
                float(x[3]),
                -T * sp / self.mass,
                T * cp / self.mass - self.gacc,
                omg,
                self.arm * (T1 - T2) / self.Iyy,
            ]
        )

    def calc(self, data, x, u=None):
        if self.terminal or u is None:
            data.xnext[:] = x
            data.cost = 0.5 * float(x @ self.Qf @ x)
            return
        data.xnext[:] = _rk4(self._f, x, u, self.dt)
        data.cost = 0.5 * float(x @ self.Q @ x + u @ self.R @ u)
        data.g[0] = float(u[0] - self.tmax)
        data.g[1] = float(-u[0])
        data.g[2] = float(u[1] - self.tmax)
        data.g[3] = float(-u[1])

    def calcDiff(self, data, x, u=None):
        if self.terminal or u is None:
            _fill(data.Fx, np.eye(6))
            data.Lx[:] = self.Qf @ x
            _fill(data.Lxx, self.Qf)
            return
        phi = float(x[4])
        T1, T2 = float(u[0]), float(u[1])
        sp, cp = math.sin(phi), math.cos(phi)
        T = T1 + T2
        Ac = np.zeros((6, 6))
        Ac[0, 2] = 1.0
        Ac[1, 3] = 1.0
        Ac[2, 4] = -T * cp / self.mass
        Ac[3, 4] = -T * sp / self.mass
        Ac[4, 5] = 1.0
        Bc = np.zeros((6, 2))
        Bc[2, 0] = -sp / self.mass
        Bc[2, 1] = -sp / self.mass
        Bc[3, 0] = cp / self.mass
        Bc[3, 1] = cp / self.mass
        Bc[5, 0] = self.arm / self.Iyy
        Bc[5, 1] = -self.arm / self.Iyy
        _fill(data.Fx, np.eye(6) + self.dt * Ac)
        _fill(data.Fu, self.dt * Bc)
        data.Lx[:] = self.Q @ x
        data.Lu[:] = self.R @ u
        _fill(data.Lxx, self.Q)
        _fill(data.Luu, self.R)
        _fill(data.Lxu, np.zeros((6, 2)))
        Gu = np.zeros((4, 2))
        Gu[0, 0] = 1.0
        Gu[1, 0] = -1.0
        Gu[2, 1] = 1.0
        Gu[3, 1] = -1.0
        _fill(data.Gu, Gu)


def cartpole_initial(seed: int) -> np.ndarray:
    if seed % 3 == 1:
        return np.array([0.0, 0.0, 0.55, 0.15])
    if seed % 3 == 2:
        return np.array([0.0, 0.0, -0.45, -0.08])
    return np.array([0.0, 0.0, 0.30, 0.0])


def quadrotor_initial(seed: int) -> np.ndarray:
    if seed % 3 == 1:
        return np.array([1.5, 2.5, 0.0, 0.0, -0.15, 0.0])
    if seed % 3 == 2:
        return np.array([2.5, 1.5, 0.1, -0.1, 0.2, 0.05])
    return np.array([2.0, 2.0, 0.0, 0.0, 0.1, 0.0])


def cartpole(seed: int) -> dict:
    H, dt = 50, 0.02
    running = CartpoleAM(dt)
    terminal = CartpoleAM(dt, terminal=True)
    x0 = cartpole_initial(seed)
    problem = crocoddyl.ShootingProblem(x0, [running] * H, terminal)
    xs = [x0.copy() for _ in range(H + 1)]
    us = [np.zeros(1) for _ in range(H)]
    solver, done, ms = _solve(problem, xs, us, 100, use_box=True)
    ineq = _max_g_violation(problem, solver.xs, solver.us)
    return dict(
        solver="crocoddyl",
        problem="cartpole_stabilize",
        seed=seed,
        converged=int(done),
        feasible=int(ineq <= TOL),
        termination_reason="CONVERGED" if done else "MAX_ITER",
        final_cost=float(solver.cost),
        eq_violation=0.0,
        ineq_violation=float(ineq),
        dynamics_violation=float(getattr(solver, "ffeas", 0.0) or 0.0),
        iterations=int(solver.iter),
        time_ms=ms,
        chi=0.0,
    )


def quadrotor_hover(seed: int) -> dict:
    H, dt = 40, 0.02
    running = QuadrotorHoverAM(dt)
    terminal = QuadrotorHoverAM(dt, terminal=True)
    x0 = quadrotor_initial(seed)
    problem = crocoddyl.ShootingProblem(x0, [running] * H, terminal)
    xs = [x0.copy() for _ in range(H + 1)]
    hover = 0.5 * 9.81 * 0.5
    us = [np.array([hover, hover]) for _ in range(H)]
    solver, done, ms = _solve(problem, xs, us, 80, use_box=True)
    ineq = _max_g_violation(problem, solver.xs, solver.us)
    return dict(
        solver="crocoddyl",
        problem="quadrotor_hover",
        seed=seed,
        converged=int(done),
        feasible=int(ineq <= TOL),
        termination_reason="CONVERGED" if done else "MAX_ITER",
        final_cost=float(solver.cost),
        eq_violation=0.0,
        ineq_violation=float(ineq),
        dynamics_violation=float(getattr(solver, "ffeas", 0.0) or 0.0),
        iterations=int(solver.iter),
        time_ms=ms,
        chi=0.0,
    )


def _nav_dynamics_f(x, u):
    th, kap = float(x[2]), float(x[3])
    v, dk = float(u[0]), float(u[1])
    return np.array([v * math.cos(th), v * math.sin(th), v * kap, dk])


def _nav_rk4_rollout(x0, us, dt):
    """Mirror bench_nav::generate_seeds RK4 (no κ clip in dynamics)."""
    H = len(us)
    xs = [np.array(x0, dtype=float, copy=True)]
    for k, u in enumerate(us):
        x = xs[-1]

        def f(xk, uk):
            return _nav_dynamics_f(xk, uk)

        k1 = f(x, u)
        k2 = f(x + 0.5 * dt * k1, u)
        k3 = f(x + 0.5 * dt * k2, u)
        k4 = f(x + dt * k3, u)
        xs.append(x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4))
    return xs, us


def nav_generate_seeds(seed_index: int, H: int = 80, dt: float = 0.1):
    """Same families as examples/nav_bench.h::generate_seeds."""
    target_x, vmax = 10.0, 3.0
    v_cruise = target_x / (H * dt)
    sprint_nom = max(1, int(math.ceil(target_x / (vmax * dt))))
    family = seed_index % 3
    wait_steps = max(0, H - sprint_nom)
    if family == 2:
        wait_steps = min(H - 1, wait_steps + 1)
    us = []
    for k in range(H):
        v = v_cruise
        if family in (1, 2):
            v = 0.0 if k < wait_steps else vmax
        v = min(max(v, 0.0), vmax)
        us.append(np.array([v, 0.0]))
    x0 = np.zeros(4)
    xs, us = _nav_rk4_rollout(x0, us, dt)
    return xs, us


def _nav_max_ineq(xs, terminal_model: NavAM, running_model: NavAM) -> float:
    """Max inequality violation over running κ-g and terminal box."""
    xs_list = [np.array(x, dtype=float).reshape(-1) for x in xs]
    tdata = terminal_model.createData()
    terminal_model.calc(tdata, xs_list[-1], None)
    m = float(np.max(np.asarray(tdata.g, dtype=float)))
    rdata = running_model.createData()
    for x in xs_list[:-1]:
        running_model.calc(rdata, x, np.zeros(2))
        m = max(m, float(np.max(np.asarray(rdata.g, dtype=float))))
    return max(0.0, m)


def nav(seed: int) -> dict:
    H, dt = 80, 0.1
    running = NavAM(dt)
    terminal = NavAM(dt, terminal=True)
    x0 = np.zeros(4)
    problem = crocoddyl.ShootingProblem(x0, [running] * H, terminal)
    xs, us = nav_generate_seeds(seed, H, dt)
    solver, done, ms = _solve(problem, xs, us, 80, use_box=True)
    ineq = _nav_max_ineq(solver.xs, terminal, running)
    return dict(
        solver="crocoddyl",
        problem="nav_2d_bicycle",
        seed=seed,
        converged=int(done),
        feasible=int(ineq <= TOL),
        termination_reason="CONVERGED" if done else "MAX_ITER",
        final_cost=float(solver.cost),
        eq_violation=0.0,
        ineq_violation=float(ineq),
        dynamics_violation=float(getattr(solver, "ffeas", 0.0) or 0.0),
        iterations=int(solver.iter),
        time_ms=ms,
        chi=0.0,
    )


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
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
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rows = []
    for seed in (0, 1, 2):
        print(f"pendulum seed={seed}", flush=True)
        rows.append(pendulum(seed))
        print(f"DI seed={seed}", flush=True)
        rows.append(double_integrator(seed))
        print(f"cartpole seed={seed}", flush=True)
        rows.append(cartpole(seed))
        print(f"quadrotor seed={seed}", flush=True)
        rows.append(quadrotor_hover(seed))
        print(f"nav seed={seed}", flush=True)
        rows.append(nav(seed))
    write_rows(Path(args.out), rows)
    print(f"Wrote {len(rows)} rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
