#!/usr/bin/env julia
# Fair ALTRO Gate runner — same OCP / seeds / CSV as bench_vs_sota.
#
#   julia --project=scripts/sota/altro_gate scripts/sota/run_altro.jl \
#     --out altro.csv
#   python3 sota/run_altro.py --out altro.csv

using Altro
using TrajectoryOptimization
using RobotDynamics
using StaticArrays
using LinearAlgebra
using ForwardDiff
using FiniteDiff

const TO = TrajectoryOptimization
const RD = RobotDynamics
const TOL = 1e-3

# ============================================================================
# Dynamics (Gate parity)
# ============================================================================

RD.@autodiff struct GatePendulum <: RD.ContinuousDynamics end
RD.state_dim(::GatePendulum) = 2
RD.control_dim(::GatePendulum) = 1
function RD.dynamics(::GatePendulum, x, u)
    m, g, l = 1.0, 9.81, 0.5
    I = m * l * l
    return SA[x[2], (m * g * l * sin(x[1]) + u[1]) / I]
end
function RD.dynamics!(::GatePendulum, xdot, x, u)
    xdot .= RD.dynamics(GatePendulum(), x, u)
    return nothing
end

# Continuous DI + RK4 ≡ discrete map x⁺=[x+dt·v+½dt²u, v+dt·u] (Gate DiscreteMap).
RD.@autodiff struct GateDI <: RD.ContinuousDynamics end
RD.state_dim(::GateDI) = 2
RD.control_dim(::GateDI) = 1
function RD.dynamics(::GateDI, x, u)
    return SA[x[2], u[1]]
end
function RD.dynamics!(::GateDI, xdot, x, u)
    xdot .= RD.dynamics(GateDI(), x, u)
    return nothing
end

RD.@autodiff struct GateCartpole <: RD.ContinuousDynamics end
RD.state_dim(::GateCartpole) = 4
RD.control_dim(::GateCartpole) = 1
function RD.dynamics(::GateCartpole, x, u)
    mc, mp, len, grav = 1.0, 0.1, 0.5, 9.81
    th, om, F = x[3], x[4], u[1]
    st, ct = sin(th), cos(th)
    den = mc + mp * st * st
    vdot = (F + mp * len * om * om * st - mp * grav * st * ct) / den
    omegadot = (F * ct + mp * len * om * om * st * ct -
                (mc + mp) * grav * st + mp * grav * st * ct * ct) / (len * den)
    return SA[x[2], vdot, om, omegadot]
end
function RD.dynamics!(::GateCartpole, xdot, x, u)
    xdot .= RD.dynamics(GateCartpole(), x, u)
    return nothing
end

RD.@autodiff struct GateQuadrotor <: RD.ContinuousDynamics end
RD.state_dim(::GateQuadrotor) = 6
RD.control_dim(::GateQuadrotor) = 2
function RD.dynamics(::GateQuadrotor, x, u)
    mass, gacc, arm, Iyy = 0.5, 9.81, 0.15, 0.002
    phi, omg = x[5], x[6]
    T1, T2 = u[1], u[2]
    sp, cp = sin(phi), cos(phi)
    T = T1 + T2
    return SA[x[3], x[4], -T * sp / mass, T * cp / mass - gacc, omg, arm * (T1 - T2) / Iyy]
end
function RD.dynamics!(::GateQuadrotor, xdot, x, u)
    xdot .= RD.dynamics(GateQuadrotor(), x, u)
    return nothing
end

RD.@autodiff struct GateBike <: RD.ContinuousDynamics end
RD.state_dim(::GateBike) = 4
RD.control_dim(::GateBike) = 2
function RD.dynamics(::GateBike, x, u)
    th, kap, v, dk = x[3], x[4], u[1], u[2]
    return SA[v * cos(th), v * sin(th), v * kap, dk]
end
function RD.dynamics!(::GateBike, xdot, x, u)
    xdot .= RD.dynamics(GateBike(), x, u)
    return nothing
end

# Soft-obstacle Nav stage cost (no dt scaling) — matches Gate / Croc soft bumps.
RD.@autodiff struct SoftObsCost{T} <: TO.CostFunction
    Q::SVector{4,T}
    R::SVector{2,T}
end
Base.copy(c::SoftObsCost) = SoftObsCost(c.Q, c.R)
TO.state_dim(::SoftObsCost) = 4
TO.control_dim(::SoftObsCost) = 2
RD.default_diffmethod(::SoftObsCost) = RD.ForwardAD()
const NAV_OBS = ((3.5, 1.5, 0.35, 50.0), (6.0, -1.2, 0.35, 50.0), (8.0, 0.8, 0.35, 50.0))
function RD.evaluate(cost::SoftObsCost, x, u)
    c = 0.5 * (cost.Q[1] * x[1]^2 + cost.Q[2] * x[2]^2 + cost.Q[3] * x[3]^2 +
               cost.Q[4] * x[4]^2 + cost.R[1] * u[1]^2 + cost.R[2] * u[2]^2)
    for (ox, oy, sig, w) in NAV_OBS
        d2 = (x[1] - ox)^2 + (x[2] - oy)^2
        c += w * exp(-d2 / (2 * sig * sig))
    end
    return c
end

# ============================================================================
# CSV / solve helpers
# ============================================================================

function write_csv(path, rows)
    open(path, "w") do io
        println(io, "solver,problem,seed,converged,feasible,termination_reason," *
                    "final_cost,eq_violation,ineq_violation,dynamics_violation," *
                    "iterations,time_ms,chi")
        for r in rows
            println(io, join((
                r.solver, r.problem, r.seed, Int(r.converged), Int(r.feasible),
                r.termination_reason, r.final_cost, r.eq_violation, r.ineq_violation,
                r.dynamics_violation, r.iterations, r.time_ms, r.chi), ","))
        end
    end
end

function make_row(; problem, seed, converged, feasible, reason, cost, ineq, iters, ms)
    return (
        solver = "altro", problem = problem, seed = seed,
        converged = converged, feasible = feasible, termination_reason = reason,
        final_cost = Float64(cost), eq_violation = 0.0, ineq_violation = Float64(ineq),
        dynamics_violation = 0.0, iterations = Int(iters), time_ms = Float64(ms), chi = 0.0,
    )
end

function solve_altro(prob, opts; U0=nothing, X0=nothing)
    U0 !== nothing && initial_controls!(prob, U0)
    X0 !== nothing && initial_states!(prob, X0)
    solver = ALTROSolver(prob, opts)
    t0 = time_ns()
    st = status(solve!(solver))
    ms = (time_ns() - t0) / 1e6
    return solver, st, ms
end

opts(iters) = SolverOptions(
    iterations = iters,
    cost_tolerance = 1e-4,
    constraint_tolerance = TOL,
    penalty_scaling = 10.0,
    penalty_initial = 1.0,
)

ok_status(st) = st == Altro.SOLVE_SUCCEEDED

# ============================================================================
# Problems
# ============================================================================

function pendulum(seed::Int)
    H, dt = 40, 0.05
    N = H + 1
    model = RD.DiscretizedDynamics{RD.RK4}(GatePendulum())
    n, m = RD.dims(model)
    Q = Diagonal(@SVector [10.0, 0.1])
    R = Diagonal(@SVector [0.01])
    Qf = Diagonal(@SVector [500.0, 100.0])
    xf = @SVector zeros(2)
    obj = LQRObjective(Q, R, Qf, xf, N)
    x0 = @SVector [π, 0.0]
    prob = Problem(model, obj, x0, H * dt)
    solver, st, ms = solve_altro(prob, opts(80);
        U0=[zeros(m) for _ = 1:H],
        X0=[copy(Vector(x0)) for _ = 1:N])
    return make_row(problem="pendulum_swingup", seed=seed,
        converged=ok_status(st), feasible=true,
        reason=ok_status(st) ? "CONVERGED" : "MAX_ITER",
        cost=cost(solver), ineq=0.0, iters=iterations(solver), ms=ms)
end

function double_integrator(seed::Int)
    H, dt = 40, 0.1
    N = H + 1
    model = RD.DiscretizedDynamics{RD.RK4}(GateDI())
    n, m = RD.dims(model)
    # Gate scalar ≡ 0.5||x - [1,0]||^2 + 0.5 R u^2 (incl. constant)
    Q = Diagonal(@SVector ones(2))
    R = Diagonal(@SVector [0.1])
    Qf = Diagonal(@SVector [100.0, 100.0])
    xf = @SVector [1.0, 0.0]
    obj = LQRObjective(Q, R, Qf, xf, N)
    x0 = @SVector zeros(2)
    prob = Problem(model, obj, x0, H * dt, xf=xf)
    solver, st, ms = solve_altro(prob, opts(60);
        U0=[zeros(m) for _ = 1:H],
        X0=[zeros(n) for _ = 1:N])
    return make_row(problem="double_integrator_reach", seed=seed,
        converged=ok_status(st), feasible=true,
        reason=ok_status(st) ? "CONVERGED" : "MAX_ITER",
        cost=cost(solver), ineq=0.0, iters=iterations(solver), ms=ms)
end

function cartpole_initial(seed::Int)
    seed % 3 == 1 && return @SVector [0.0, 0.0, 0.55, 0.15]
    seed % 3 == 2 && return @SVector [0.0, 0.0, -0.45, -0.08]
    return @SVector [0.0, 0.0, 0.30, 0.0]
end

function cartpole(seed::Int)
    H, dt = 50, 0.02
    N = H + 1
    model = RD.DiscretizedDynamics{RD.RK4}(GateCartpole())
    n, m = RD.dims(model)
    Q = Diagonal(@SVector [5.0, 50.0, 0.0, 0.0])
    R = Diagonal(@SVector [0.1])
    Qf = Diagonal(@SVector fill(500.0, 4))
    xf = @SVector zeros(4)
    obj = LQRObjective(Q, R, Qf, xf, N)
    conSet = ConstraintList(n, m, N)
    add_constraint!(conSet, BoundConstraint(n, m, u_min=-30.0, u_max=30.0), 1:H)
    add_constraint!(conSet, BoundConstraint(n, m,
        x_min=[-0.5, -0.5, -Inf, -Inf], x_max=[0.5, 0.5, Inf, Inf]), N)
    x0 = cartpole_initial(seed)
    prob = Problem(model, obj, x0, H * dt, constraints=conSet)
    solver, st, ms = solve_altro(prob, opts(100);
        U0=[zeros(m) for _ = 1:H],
        X0=[copy(Vector(x0)) for _ = 1:N])
    ineq = 0.0
    for u in controls(solver)
        ineq = max(ineq, max(0.0, abs(u[1]) - 30.0))
    end
    xf_sol = states(solver)[end]
    ineq = max(ineq, max(0.0, abs(xf_sol[1]) - 0.5), max(0.0, abs(xf_sol[2]) - 0.5))
    return make_row(problem="cartpole_stabilize", seed=seed,
        converged=ok_status(st), feasible=ineq <= TOL,
        reason=ok_status(st) ? "CONVERGED" : "MAX_ITER",
        cost=cost(solver), ineq=ineq, iters=iterations(solver), ms=ms)
end

function quadrotor_initial(seed::Int)
    seed % 3 == 1 && return @SVector [1.5, 2.5, 0.0, 0.0, -0.15, 0.0]
    seed % 3 == 2 && return @SVector [2.5, 1.5, 0.1, -0.1, 0.2, 0.05]
    return @SVector [2.0, 2.0, 0.0, 0.0, 0.1, 0.0]
end

function quadrotor(seed::Int)
    H, dt = 40, 0.02
    N = H + 1
    model = RD.DiscretizedDynamics{RD.RK4}(GateQuadrotor())
    n, m = RD.dims(model)
    Q = Diagonal(@SVector [50.0, 50.0, 1.0, 1.0, 20.0, 0.5])
    R = Diagonal(@SVector [0.01, 0.01])
    Qf = Diagonal(@SVector fill(100.0, 6))
    xf = @SVector zeros(6)
    obj = LQRObjective(Q, R, Qf, xf, N)
    conSet = ConstraintList(n, m, N)
    add_constraint!(conSet, BoundConstraint(n, m, u_min=[0.0, 0.0], u_max=[15.0, 15.0]), 1:H)
    x0 = quadrotor_initial(seed)
    hover = 0.5 * 9.81 * 0.5
    prob = Problem(model, obj, x0, H * dt, constraints=conSet)
    solver, st, ms = solve_altro(prob, opts(150);
        U0=[[hover, hover] for _ = 1:H],
        X0=[copy(Vector(x0)) for _ = 1:N])
    ineq = 0.0
    for u in controls(solver)
        ineq = max(ineq, max(0.0, -u[1]), max(0.0, u[1] - 15.0),
                         max(0.0, -u[2]), max(0.0, u[2] - 15.0))
    end
    return make_row(problem="quadrotor_hover", seed=seed,
        converged=ok_status(st), feasible=ineq <= TOL,
        reason=ok_status(st) ? "CONVERGED" : "MAX_ITER",
        cost=cost(solver), ineq=ineq, iters=iterations(solver), ms=ms)
end

function nav_generate_seeds(seed_index::Int, H::Int=80, dt::Float64=0.1)
    target_x, vmax = 10.0, 3.0
    v_cruise = target_x / (H * dt)
    sprint_nom = max(1, Int(ceil(target_x / (vmax * dt))))
    family = seed_index % 3
    wait_steps = max(0, H - sprint_nom)
    family == 2 && (wait_steps = min(H - 1, wait_steps + 1))
    us = Vector{Vector{Float64}}(undef, H)
    for k in 1:H
        v = v_cruise
        if family in (1, 2)
            v = (k - 1) < wait_steps ? 0.0 : vmax
        end
        us[k] = [clamp(v, 0.0, vmax), 0.0]
    end
    f(x, u) = begin
        th, kap = x[3], x[4]
        v, dk = u[1], u[2]
        [v * cos(th), v * sin(th), v * kap, dk]
    end
    x = zeros(4)
    xs = [copy(x)]
    for u in us
        k1 = f(x, u); k2 = f(x .+ 0.5 .* dt .* k1, u)
        k3 = f(x .+ 0.5 .* dt .* k2, u); k4 = f(x .+ dt .* k3, u)
        x = x .+ (dt / 6) .* (k1 .+ 2 .* k2 .+ 2 .* k3 .+ k4)
        push!(xs, copy(x))
    end
    return xs, us
end

function nav(seed::Int)
    H, dt = 80, 0.1
    N = H + 1
    model = RD.DiscretizedDynamics{RD.RK4}(GateBike())
    n, m = RD.dims(model)
    Qd = @SVector [1.0, 1.0, 0.5, 0.1]
    Rd = @SVector [0.1, 0.5]
    Qf = Diagonal(@SVector [1000.0, 1000.0, 200.0, 50.0])
    xf = @SVector [10.0, 0.0, 0.0, 0.0]
    costs = TO.CostFunction[SoftObsCost{Float64}(Qd, Rd) for _ = 1:H]
    qterm = SVector{4}(-Qf * xf)
    rterm = @SVector zeros(2)
    cterm = 0.5 * (xf' * Qf * xf)
    push!(costs, DiagonalCost(SVector(diag(Qf)), rterm, qterm, rterm, cterm;
                               terminal=true, checks=false))
    obj = Objective(costs)
    conSet = ConstraintList(n, m, N)
    add_constraint!(conSet, BoundConstraint(n, m, u_min=[-3.0, -2.0], u_max=[3.0, 2.0]), 1:H)
    add_constraint!(conSet, BoundConstraint(n, m,
        x_min=[-Inf, -Inf, -Inf, -1.5], x_max=[Inf, Inf, Inf, 1.5]), 1:H)
    add_constraint!(conSet, BoundConstraint(n, m,
        x_min=[9.5, -0.3, -0.3, -0.5], x_max=[10.5, 0.3, 0.3, 0.5]), N)
    xs0, us0 = nav_generate_seeds(seed, H, dt)
    x0 = SVector{4}(xs0[1])
    prob = Problem(model, obj, x0, H * dt, xf=xf, constraints=conSet)
    solver, st, ms = solve_altro(prob, opts(80); U0=us0, X0=xs0)
    ineq = 0.0
    for u in controls(solver)
        ineq = max(ineq, max(0.0, abs(u[1]) - 3.0), max(0.0, abs(u[2]) - 2.0))
    end
    for x in states(solver)
        ineq = max(ineq, max(0.0, abs(x[4]) - 1.5))
    end
    xf_sol = states(solver)[end]
    ineq = max(ineq,
        max(0.0, abs(xf_sol[1] - 10.0) - 0.5),
        max(0.0, abs(xf_sol[2]) - 0.3),
        max(0.0, abs(xf_sol[3]) - 0.3),
        max(0.0, abs(xf_sol[4]) - 0.5))
    return make_row(problem="nav_2d_bicycle", seed=seed,
        converged=ok_status(st), feasible=ineq <= TOL,
        reason=ok_status(st) ? "CONVERGED" : "MAX_ITER",
        cost=cost(solver), ineq=ineq, iters=iterations(solver), ms=ms)
end

function main(args)
    out = "altro.csv"
    i = 1
    while i <= length(args)
        if args[i] == "--out" && i < length(args)
            out = args[i + 1]; i += 2
        else
            i += 1
        end
    end
    rows = Any[]
    for seed in 0:2
        println("pendulum seed=$seed"); flush(stdout); push!(rows, pendulum(seed))
        println("DI seed=$seed"); flush(stdout); push!(rows, double_integrator(seed))
        println("cartpole seed=$seed"); flush(stdout); push!(rows, cartpole(seed))
        println("quadrotor seed=$seed"); flush(stdout); push!(rows, quadrotor(seed))
        println("nav seed=$seed"); flush(stdout); push!(rows, nav(seed))
    end
    mkpath(dirname(out))
    write_csv(out, rows)
    println("Wrote $(length(rows)) rows to $out")
    for r in rows
        if r.problem in ("quadrotor_hover", "nav_2d_bicycle", "cartpole_stabilize")
            println("  $(r.problem) seed=$(r.seed) cost=$(round(r.final_cost, digits=4)) " *
                    "feas=$(Int(r.feasible)) ineq=$(r.ineq_violation) iters=$(r.iterations)")
        end
    end
end

main(ARGS)
