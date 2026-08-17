# Theory: Why arm ranking is not fixed (exploration / BO motivation)

**Status:** foundational for PROTOCOL_v1 paper claim.  
**Claim surface:** Propositions A–C below. Legacy “χ-Bandit primary reward” is **not** claimed.

---

## Setup (matches the scheduler)

- Arms \(i=1,\dots,K\): warm-start trajectories.
- At round \(t\), a policy selects arm \(i_t\), runs a **batch** of \(b\) iLQR iterations with trajectory warm-start, observes diagnostics
  \[
  d_i(t)=\bigl(J^{\mathrm{before}},\,J^{\mathrm{after}},\,v,\,\chi,\,b_{\mathrm{used}}\bigr).
  \]
- Instantaneous **score** \(s(d)\) is any scalar used for greedy selection. Two scores of interest:
  - \(s_\chi=-\log\chi\) (legacy; negative control),
  - \(s_r=-\Delta J_{\mathrm{feas}}-\lambda\mathrm{viol}_++\eta\rho\) (PROTOCOL_v1).
- After a fixed total budget (or equal per-arm budget), arm \(i\) has terminal feasible cost \(J_i^\star\) (∞ if never feasible).  
  **Oracle ranking** is by \(J_i^\star\) (feasible-first).

---

## Proposition A (algebraic) — Non-monotonicity of \(\chi\)

**Statement.** There exist closed-loop factors \(\tilde A_k,\tilde A'_k\in\mathbb{R}^{n\times n}\) (\(n\ge 2\)) such that
\[
\|\tilde A'_k\|_2\le\|\tilde A_k\|_2\quad(k=1,2)
\quad\text{but}\quad
\Bigl\|\prod_k\tilde A'_k\Bigr\|_2
\;>\;
\Bigl\|\prod_k\tilde A_k\Bigr\|_2.
\]
Hence \(\chi=\|\Phi\|_2\) need **not** decrease when every factor’s spectral norm decreases.

**Proof (counterexample).** Take
\[
\tilde A_1=\mathrm{diag}(10,0.01),\;
\tilde A_2=\mathrm{diag}(0.01,10),\;
\tilde A'_1=\tilde A'_2=\mathrm{diag}(9.9,0.01).
\]
Then \(\|\tilde A_k\|_2=10\), \(\|\tilde A'_k\|_2=9.9\), but
\[
\|\tilde A_1\tilde A_2\|_2=0.1,\qquad
\|\tilde A'_1\tilde A'_2\|_2=98.01.
\]
\hfill\(\square\)

**Role.** Shows that even a “locally improving” linearization can raise \(\chi\). Therefore **greedy-on-\(\chi\)** is not justified by per-step contraction alone. (This does **not** claim \(s_\chi\) is a good paper reward.)

---

## Proposition B (decision-theoretic) — Instantaneous score ≠ oracle ranking ⇒ greedy fails

**Statement.** Fix any score \(s\) and a finite horizon of pulls. Suppose there exist arms \(i,j\) and a time \(t_0\) with
\[
s\bigl(d_i(t_0)\bigr)\;>\;s\bigl(d_j(t_0)\bigr)
\quad\text{but}\quad
J_i^\star\;>\;J_j^\star
\]
(under the same remaining-budget rule for both arms after \(t_0\)).  
Then the **greedy** policy that always selects \(\arg\max_k s(d_k(t))\) at \(t_0\) (and thereafter on the induced path) is **not** guaranteed to match the oracle that knows \(\{J_k^\star\}\).

**Proof.** At \(t_0\), greedy selects \(i\) over \(j\). The oracle, maximizing terminal feasible quality, prefers \(j\). Any policy that is forced to follow greedy’s choice at \(t_0\) therefore differs from the oracle on this instance. Hence optimality of greedy w.r.t.\ terminal \(J^\star\) cannot hold for all instances whenever such a ranking inversion exists.  
\hfill\(\square\)

**Role.** This is the precise meaning of “arm 与 bandit 没有固定关系”:  
**fixed** would mean “\(\arg\max s(d(\cdot))\) at any \(t\) coincides with \(\arg\min J^\star\)”. Prop.\ B says if that fails even once, greedy is insufficient and **exploration** (UCB bonus / BO uncertainty) is necessary in the worst case.

---

## Proposition C (constructive) — Ranking inversion exists for batch improvement scores

Prop.\ B is vacuous unless inversions exist for the scores we use. We give an **abstract batch model** that mirrors trajectory-warm-start batches (no dual persistence), then point to empirical iLQR instances.

### C1. Abstract two-basin batch model

Consider two arms and batches of size \(b=1\) for clarity. Deterministic cost sequences (feasible always, \(v=0\)):

| pull \(t\) | \(J_A(t)\) | \(J_B(t)\) |
|------------|------------|------------|
| 0 (init)   | 1000       | 1000       |
| after 1    | 200        | 900        |
| after 2    | 180        | 50         |
| after 3+   | 180        | 50         |

PROTOCOL_v1 with \(\lambda,\eta\) arbitrary and \(v=0\):  
\(\Delta J_{\mathrm{feas}}=J_{\mathrm{after}}-J_{\mathrm{before}}\),  
\(r=-\Delta J_{\mathrm{feas}}+\eta\rho\) with \(\rho=\max(0,-\Delta J_{\mathrm{feas}})/b_{\mathrm{used}}\).

At \(t=1\):  
\(\Delta J_A=-800\), \(\Delta J_B=-100\) ⇒ \(r_A>r_B\) (A looks better).  
Terminal: \(J_A^\star=180>J_B^\star=50\).

**Thus** \(s_r(d_A(1))>s_r(d_B(1))\) but \(J_A^\star>J_B^\star\): Prop.\ B applies to \(s_r\).

For \(s_\chi\), any path where arm A has tiny \(\chi\) after pull 1 but stagnates at high \(J\), while B has larger \(\chi\) then reaches low \(J\), yields the same inversion (realized on Nav χ-probe; see Empirics).

\hfill\(\square\) (constructive)

### C2. Why iLQR can realize C1

Constrained iLQR with Filter/ADMM is a **local** method on a nonconvex landscape. Short batches measure **local slope** (cost drop / violation drop), not the identity of the attractor. A seed that drops cost quickly into a shallow basin (large early \(-\Delta J\)) can outrank a seed that descends slowly into a deep basin. Trajectory-warm-start batches without dual persistence further decorrelate early \(r\) from late \(J^\star\). No claim is made that *every* OCP exhibits inversion; Prop.\ C asserts **existence**, which is enough for Prop.\ B.

---

## Proposition D (separability, conditional) — Signal is not pure noise

**Statement (sufficient conditions).** Suppose arms \(i,j\) converge under equal budget to feasible local costs \(J_i^\star<J_j^\star-\gamma\) (\(\gamma>0\)), and after warm-up length \(t_0\) the batch improvements concentrate:
\[
\mathbb{E}\bigl[r_i(t)\bigr]\;\ge\;
\mathbb{E}\bigl[r_j(t)\bigr]+\delta
\quad\text{for all }t\ge t_0
\]
for some \(\delta>0\) (e.g.\ deep basin still yields nonnegative feasible \(\Delta J\) more often, or lower \(\mathrm{viol}_+\)). Then the means of \(s_r\) **separate** the oracle-better arm from the worse one after \(t_0\).

**Proof sketch.** Immediate from the displayed inequality: the better arm’s reward process dominates in expectation, so a bandit that estimates means has a detectable gap \(\delta\).  
\hfill\(\square\)

**Caveat (honesty).** The dominance assumption is **not** automatic for \(s_\chi\) (Nav χ-probe: best-\(J\) arm had worst \(-\log\chi\)). For \(s_r\), dominance is an empirical hypothesis to verify per problem class. Prop.\ D explains when UCB/BO have **something to learn**; Prop.\ B–C explain why they must **explore**.

---

## Link to UCB / BO

| Result | Implies |
|--------|---------|
| Prop.\ A | \(\chi\) itself is a fragile monotone proxy |
| Prop.\ B+C | Instantaneous \(s_r\) or \(s_\chi\) can disagree with \(J^\star\) ⇒ **greedy fails** ⇒ UCB exploration / BO uncertainty is necessary |
| Prop.\ D (when verified) | Mean rewards still carry basin identity ⇒ bandit is not vacuous |

Bayesian optimization is motivated as: maintain a surrogate over **features of \(d_i\)** predicting \(s_r\) (or cumulative return), using posterior uncertainty to explore arms whose early \(r\) looks weak but may become strong—precisely the C1 pattern.

---

## Empirics (accompanying theory)

Artifacts from `./build/examples/probe_ranking_flip` (equal per-arm budget, no bandit):

| Result | Detail |
|--------|--------|
| Oracle \(J^\star\) | seed **2** (538.875) |
| Greedy \(s_\chi\) after batch 0 | seed **1** → **flip** |
| Greedy \(s_r\) after batch 0 | seed **0** → **flip** |
| Curves | `ranking_flip_curves.csv` |
| Summary | `PROP_BC_EMPIRICS.md` |

Algebraic checks: `python3 verify_prop_AC.py` (Prop.\ A + constructive C1).

**Interpretation:** Both scores exhibit ranking inversion on real Nav iLQR ⇒ Prop.\ B applies ⇒ greedy insufficient ⇒ UCB/BO exploration motivated. Separability of mean \(s_r\) on harder libraries remains a follow-up measurement (Prop.\ D).
