# Benchmark Definitions (paper B1 / B2 / B3 + Gate)

This document gives the complete mathematical definition of every benchmark
reported in the manuscript, so that any trajectory-optimization implementation
can instantiate them independently.  The definitions mirror the code compiled
into the released host binary.  RNG conventions are exact (C++ `std::mt19937`,
`std::uniform_real_distribution<double>(-0.5, 0.5)`, `std::normal_distribution<double>(0,1)`).

Notation: `x_k` state, `u_k` control, horizon `N`, time step `dt`.
All dynamics are integrated with RK4 unless stated otherwise.

---

## B1 — Nav2D–Hard (bicycle navigation with dense soft obstacles)

**State** `x = [x, y, θ, κ] ∈ ℝ⁴`; **control** `u = [v, κ̇] ∈ ℝ²`.
`N = 80`, `dt = 0.1` (RK4).  Start `x₀ = (0,0,0,0)`, target `(10, 0)`.

**Continuous dynamics (bicycle):**

```
ẋ = v cos θ        ẏ = v sin θ        θ̇ = v κ        κ̇ = κ̇
```

**Stage cost** `ℓ_k = ℓ_quad + ℓ_obs`:

```
ℓ_quad = ½ (x'Qx x + u'Qu u),   Qx = diag(1.0, 1.0, 0.5, 0.1),   Qu = diag(0.1, 0.5)
ℓ_obs  = Σ_i W max(0, r_i + m − ‖p − c_i‖)² ,   p = (x,y)
W = 80,  m = 0.35 (margin)
```

Obstacle disks `(c_i, r_i)`:

| i | x | y | r |
|---|-----|------|-----|
| 0 | 3.0 | 1.0 | 1.2 |
| 1 | 5.0 | −1.0 | 1.2 |
| 2 | 7.0 | 1.5 | 1.0 |
| 3 | 4.5 | 0.0 | 0.8 |
| 4 | 6.5 | −0.5 | 0.8 |
| 5 | 8.0 | 0.0 | 1.0 |

**Terminal cost** `ℓ_N = ½ e'Q e`, `Q = diag(1000, 1000, 200, 50)`,
`e = (x−10, y, θ, κ)`.

**Constraints (stage inequalities, g ≤ 0):** `|v| ≤ 3`, `|κ̇| ≤ 2`, `|κ| ≤ 1.5`
(control boxes projected; κ box as inequality).  **Terminal box:**
`|x−10| ≤ 0.5`, `|y| ≤ 0.3`, `|θ| ≤ 0.3`, `|κ| ≤ 0.5`.

**Seeds (K=8), `generate_seeds_lhs_noise`** (the generator name is
historical: the draws are i.i.d. uniform, not Latin-hypercube): RNG = `mt19937(rng)` where `rng`
is the run's RNG id (42…46).  Seeds 0–3 start with heading bias `θ₀ = +0.35 rad`,
seeds 4–7 with `θ₀ = −0.35 rad`.  For each step `k` with `a = k/(N−1)` and
envelope `e = 4a(1−a)`:

```
v  = clamp(v_cruise + 0.5·e·uni·3.0,   0, 3),   v_cruise = 10/(N·dt) ≈ 1.25
κ̇  = clamp(0.5·e·uni·2.0 + bias_k, −2, 2),     bias_k = ±0.4·0.5  for k < N/4
```

then open-loop RK4 rollout; afterwards independent mid-horizon state noise is
added and kept: `x += e'·uni·2.0` on (x,y), `θ += e'·uni·0.5`, `e' = 4a'(1−a')`,
`a' = k/N` (no second rollout).

---

## B2 — Planar quadruped template (contact-rich, soft costs)

**State (14):** `[p_x, p_z, θ, α_f, β_f, α_b, β_b, v_x, v_z, ω, α̇_f, β̇_f, α̇_b, β̇_b]`
(body pose, front/back hip–knee angles, then velocities).
**Control (4):** joint torques `[τ_αf, τ_βf, τ_αb, τ_βb]`.  `N = 60`, `dt = 0.02`, RK4.

**Template dynamics** (floating base + virtual suspension, explicit-RK4 stable;
no stiff contact forces — contact enters as stage cost):

```
body:  v̇_x = (kStride·(τ_αf+τ_αb) − kDrag·v_x)/m_B          m_B=12, kStride=3, kDrag=0.8
       v̇_z = (−kSuspK·(p_z−z₀) − kSuspB·v_z)/m_B            kSuspK=400, kSuspB=40, z₀=0.26
       ω̇   = (−kYawK·θ − kYawB·ω)/I_B                       kYawK=30, kYawB=8, I_B=0.8
joints: α̇̈_f = (τ_αf − kJointDamp·α̇_f)/I_r                  kJointDamp=1.5, I_r=0.05
        β̈_f = (τ_βf − kSpring·(β_f−β_rest) − kJointDamp·β̇_f)/I_r   kSpring=80, β_rest=0.80
        (α_b, β_b symmetric)
```

**Foot kinematics** (links L₀=0.35, L₁=0.25, L₂=0.25):

```
x_f = p_x + L₀cosθ + L₁cos(θ+α_f) + L₂cos(θ+α_f+β_f)
z_f = p_z + L₀sinθ + L₁sin(θ+α_f) + L₂sin(θ+α_f+β_f)      (back foot: −L₀ terms)
```

**Stage cost** = waypoint tracking + joint/velocity regularisation + contact terms:
- waypoint `p_des(s) = (s·0.28, 0.26)`, `s = k/N`: `w_px=12, w_pz=20, w_θ=15`
- joint deviation from `q_n = (α=−1.0, β=0.80, α_b=−1.0, β_b=0.80)`: `w_joint=0.8`
- velocities: `w_vel=0.1`; control: `w_r=5e-4`
- ground penetration `z_foot < 0`: `120·z_foot²`
- mid-step (front foot, `x_f ∈ [0.06, 0.20]`, step height 0.09): `900·(0.09−z_f)²`
- knee barrier `β > 1.45`: `200·(β−1.45)²` (shallow basin — the "bait")

**Terminal cost:** `w=600,300,60` on `(p_x−0.28)², (p_z−0.26)², θ²` + `10·‖vel‖²`.

**Constraints:** knee boxes `0.3 ≤ β_f, β_b ≤ 2.5`; terminal body box
`|p_x−0.28| ≤ 0.08`, `|p_z−0.26| ≤ 0.08`; control box `|u| ≤ 25`.

**Seeds (K=8, diag K=4), two clusters:** A (bait, seeds `< K/2`): over-lift gait,
knee PD gains (kp=55, kd=4), `β_cruise=0.95, β_lift=1.90`, feedforward hip torque
`1.20 + 0.05·norm`; B (gem): `kp=45, kd=6`, `β_cruise=0.72, β_lift=0.88`, FF `1.15+0.04·norm`.
Lift gate `exp(−((s−0.32)/0.14)²)`; knee refs clamped to [0.35, 2.4]; closed-loop
rollout under the PD controller, control box-clamped, RK4; non-finite states reset.

---

## B3 — Planar5 (5-DOF planar arm, UP/DOWN homotopies)

**State** `x = [q, q̇] ∈ ℝ¹⁰`, **control** `u = τ ∈ ℝ⁵` (unit-inertia torques).
`N = 60`, `dt = 0.05`.  **Exact discrete double integrator:**

```
q⁺  = q + q̇·dt + ½ u·dt²
q̇⁺  = q̇ + u·dt
```

Links `L = [0.35, 0.30, 0.25, 0.20, 0.15]` (total reach ≈ 1.25 m); end-effector
`p(q) = Σ L_i·(cos Σq, sin Σq)` with analytic Jacobian.  Start/goal IK postures:
`q₀ = [0.816, −0.555, −1.488, −1.312, −0.711]` (EE at (0.30, 0)),
`q_g = [0.653, −0.130, −0.774, −0.651, −0.357]` (EE at (0.95, 0)).

**Stage cost:** waypoint `p_des(s) = (0.30 + 0.65s, 0)`, `w_pos=40` (through the
analytic Jacobian); velocity `w=0.25`; control `w=0.02`; soft disk obstacle at
`c=(0.60, −0.13)`, `r=0.20`, margin 0.06, `w=900` (Gauss-Newton form);
soft joint barrier `40·max(0, |q_j| − (2.6−0.35))²` (makes the UP arc a shallow basin).

**Terminal cost:** `w_pos=1800` on EE to (0.95, 0), `w_vel=8`.

**Constraints:** joint boxes `|q_j| ≤ 2.6`; control box `|u| ≤ 8`.  No terminal box.

**Seeds (K=8), homotopy UP/DOWN:** seeds `< 4` are UP (bait): joint bump
`amp = 0.95 + 0.08·norm` on sin-arc `q_j += sign·amp·sin(πa)` with `(1.25, 0.95, 0.50)`
weights on joints 2–4, parked near the joint barrier; seeds `≥ 4` are DOWN (gem):
`amp = 1.55 + 0.10·norm`, sign −.  Finite-difference velocities + inverse
double-integrator torques, then re-rollout with control clamping.

---

## Gate suite (host validation, Table II)

| Problem | N | dt | State/Control | Notes |
|---|---|---|---|---|
| pendulum_swingup | 40 | 0.05 | 2/1 | no constraints; ALTRO-parity 186.108 |
| double_integrator_reach | 40 | 0.1 | 4/1 | RK4 discrete map; 6.66568 |
| cartpole_stabilize | 50 | 0.02 | 4/1 | force box \|F\|≤30, terminal cart box \|x\|,\|v\|≤0.5; 3 seeds θ₀ ∈ {0.30, 0.55, −0.45} |
| quadrotor_hover | 40 | 0.02 | 6/2 | thrust box 0≤T≤15; 3 seeds, hover initial guess |
| nav_2d_bicycle | 80 | 0.1 | 4/2 | B1 dynamics w/o hard obstacles; 3 Gaussian soft obs (W=50, σ=0.35) at (3.5,1.5),(6,−1.2),(8,0.8) |

The open-source baseline runners (Crocoddyl / Aligator / ALTRO) are in
[`../sota/`](../sota/README.md); their problem definitions mirror the table
above.  Per-instance (problem × seed) host costs are within 1.02× of the best
feasible baseline cost on all 15/15 instances.

---

## Protocol parameters (all three benchmarks, frozen)

- Reward: `r = clip(−ΔJ_feas − 10·viol₊ + 0.1·ρ, −50, 50)`, `ε = 1e-3`
  (see manuscript Eq. (3) and `../protocol/PROTOCOL_v1.md`).
- χ termination threshold `χ_term = 100` (enabled in code; never triggered in
  any reported run).  Early-success threshold = ∞ (disabled).
- RNG ids `{42, 43, 44, 45, 46}`; warm-up/batch/budget per Table I of the
  manuscript (B1: 10/10/160; B2: 2/5/80; B3: 3/5/80).
- Batch semantics: each pull constructs a fresh solver instance; only the
  trajectory warm-starts (dual variables do not persist).
