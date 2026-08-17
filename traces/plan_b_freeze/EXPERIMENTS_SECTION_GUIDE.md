# Experiments section — paste-ready guide (Plan B freeze)

> **用途：** 写 `su27a.tex` §Experiments 时整段复制。数字与措辞已与当前可复现冻结对齐。  
> **复现：** `bash experiments/results/plan_b_freeze/run_freeze.sh`  
> **禁止粘贴：** 2026-07-19 的 Uniform 3096 / UCB 2138（旧二进制，不可复现）。

---

## 0. 一页地图（先看这个）

| 文稿位置 | 用哪张表 | 主张一句话 |
|----------|----------|------------|
| Setup / protocol | §1 协议表 | 冻结 K/B/warmup；reward=`FEAS_IMPROVE`；early_success 关 |
| 主结果：分配有效 | **T1 B1** + **T2 B3** | adaptive 优于 Uniform |
| 负对照：Greedy-χ | T1 + T2 末行 | χ 贪心不可靠 |
| 理论证人：ranking flip | **T3** Nav equal-budget + **T4** B2 diag（+可选 T5 planar5 diag） | 早分 ≠ 晚 oracle |
| B2 suite | **不要进主表**；脚注即可 | 近乎打平，只说明模板可跑 |

**diag vs suite（四足只记这一句）：**  
- **diag** = 每多种子均等预算、无 bandit → 验证 ranking inversion（理论）。  
- **suite** = 五策抢总预算 → 验证分配策略（主 claim）。B2 只信 diag。

---

## 1. Suggested §Experiments outline（可直接当小节）

```text
5. Experiments
  5.1 Setup and protocol          ← 协议表 + 三件套角色
  5.2 Host note (brief)           ← Gate 一行带过，非 climax
  5.3 Ranking-flip diagnostics    ← T3 + T4（支撑 Lemma / Cor）
  5.4 Seed-budgeting results      ← T1 B1 + T2 B3（主 claim）
  5.5 Negative control (χ-greedy) ← 嵌在 T1/T2 讨论里即可
  5.6 Limitations                 ← B2 suite 打平；B1 多不可行 → feasible-first
```

---

## 2. Protocol（复制进 Setup）

**Reward.** `s_r = FEAS_IMPROVE`（PROTOCOL_v1）；`χ` 仅作终止开关 / BO 特征，不作主 reward。  
**early_success.** 关闭（`+∞`）。  
**RNG.** `{42,43,44,45,46}`（n=5），下表均为 mean±std，除非标明 single-rng。  
**Binary.** `./build/examples/probe_seed_budget_suite`

| ID | System | K | warmup | batch | B | Paper role |
|----|--------|---|--------|-------|---|------------|
| B1 | Nav2D-Hard (bicycle + dense soft obstacles) | 8 | 10 | 10 | 160 | Primary policy discrimination (feasible-first) |
| B2 | Planar quadruped template (soft step / knee barrier) | 8† | 2 | 5 | 80 | Contact-rich ranking inversion (**diag only**) |
| B3 | 5-DOF planar analytic arm (UP/DOWN homotopy) | 8 | 3 | 5 | 80 | Second discrimination + all-feasible χ control |

† Diag uses K=4, warm=2, late+=30, equal per-seed budget, no bandit.

**Suggested prose (EN):**

> We evaluate feasibility-aware seed budgeting on three frozen instances (Table X). All policies share reward \(s_r\) (feasible improvement), identical total iteration budgets, and disabled early-success stopping. UCB, BO–EI, and Greedy-\(s_r\) form the adaptive bloc; Uniform is the equal-allocation baseline; Greedy-\(\chi\) is a negative control.

---

## 3. Tables to paste

### T1 — B1 Nav2D-Hard (suite, n=5)  【主表之一】

**Caption suggestion:**  
*Nav2D-Hard seed budgeting (K=8, B=160). Mean±std over five RNG seeds. Many incumbents remain slightly infeasible at \(\varepsilon=10^{-3}\); we therefore rank by feasible-first order (feasible ≻ lower violation ≻ lower cost). Adaptive policies (UCB / BO–EI / Greedy-\(s_r\)) tie with each other and win this ranking against Uniform on 5/5 seeds.*

| Policy | Cost (mean±std) | Feas. rate | Viol. mean |
|--------|-----------------|------------|------------|
| Uniform | 2002.6 ± 435.6 | 0/5 | 0.306 |
| UCB | 1587.5 ± 852.4 | 2/5 | 0.083 |
| BO–EI | 1587.5 ± 852.4 | 2/5 | 0.083 |
| Greedy-\(s_r\) | 1587.5 ± 852.4 | 2/5 | 0.083 |
| Greedy-\(\chi\) | 1418.7 ± 314.4 | 1/5 | 0.234 |

**Head-to-head (Uniform vs UCB; BO/Greedy-\(s_r\) identical to UCB):**

| RNG | Uniform (feas / cost / viol) | UCB (feas / cost / viol) | Adaptive wins? |
|-----|------------------------------|--------------------------|----------------|
| 42 | 0 / 2611 / 0.656 | 0 / 1231 / 0.017 | yes |
| 43 | 0 / 1576 / 0.208 | **1** / 1046 / 0 | yes |
| 44 | 0 / 2276 / 0.106 | 0 / 1137 / 0.033 | yes |
| 45 | 0 / 1895 / 0.066 | **1** / 1433 / 0 | yes |
| 46 | 0 / 1655 / 0.493 | 0 / 3090 / 0.365 | yes |

**Claim line (copy):**  
> Under feasible-first ranking, the adaptive bloc beats Uniform on **5/5** random seeds.

**Do not write:** “~31% lower feasible cost (3096→2138).”

---

### T2 — B3 Planar5 (suite, n=5, all feasible)  【主表之二】

**Caption suggestion:**  
*Planar5 seed budgeting (K=8, B=80). All runs feasible. Adaptive \(s_r\) policies recover the DOWN homotopy; Uniform often remains on the UP bait; Greedy-\(\chi\) is feasible but worse than UCB/BO.*

| Policy | Cost (mean±std) | Feas. rate |
|--------|-----------------|------------|
| Uniform | 433.9 ± 89.9 | 5/5 |
| UCB | 277.0 ± 138.4 | 5/5 |
| **BO–EI** | **269.7 ± 143.4** | 5/5 |
| **Greedy-\(s_r\)** | **269.7 ± 143.4** | 5/5 |
| Greedy-\(\chi\) | 402.4 ± 50.5 | 5/5 |

**Claim line (copy):**  
> On an all-feasible instance, BO–EI / Greedy-\(s_r\) reduce mean cost by ~38% vs Uniform (270 vs 434); Greedy-\(\chi\) remains worse (402) despite full feasibility—isolating score mis-ranking from infeasibility entanglement.

---

### T3 — Nav equal-budget ranking flip（已有理论表，保留）

*Source: `probe_ranking_flip`, per_arm=80, batch=10. Supports Lemma / Cor on real iLQR.*

| Seed | Oracle \(J^\star\) (feas) | After batch 0: greedy \(s_\chi\) | After batch 0: greedy \(s_r\) |
|------|---------------------------|----------------------------------|------------------------------|
| 0 | 871.26 | — | **selected** |
| 1 | 538.91 | **selected** | — |
| 2 | **538.88 (oracle best)** | — | — |

**Claim line:** both \(s_\chi\) and \(s_r\) invert vs oracle after the first batch.

---

### T4 — B2 Quadruped **diag**（理论证人；主用这张）

**Caption suggestion:**  
*Contact-rich ranking-flip diagnostic on the planar quadruped template (no bandit). Each of K=4 seeds receives warm-up=2 then +30 iLQR iterations. Cluster A (over-lift) looks better after warm-up; cluster B (mild clear) achieves lower terminal cost. Gate pass 5/5 RNGs.*

| RNG | best A @warm | best A @late | best B @late | Gate |
|-----|--------------|--------------|--------------|------|
| 42 | 22.2 | 22.2 | **7.98** | PASS |
| 43 | 22.9 | 22.9 | **9.39** | PASS |
| 44 | 26.0 | 26.0 | **7.95** | PASS |
| 45 | 23.1 | 23.1 | **7.98** | PASS |
| 46 | 22.1 | 22.1 | **13.1** | PASS |

**Claim line:**  
> The same local-improvement incompleteness pattern appears on a contact-aware template: early improvement ranks A above B, while terminal cost ranks B above A.

**Honest model one-liner (must keep):**  
> Planar floating-base template with soft mid-step / knee-barrier stage costs (not stiff hybrid contact / full 3D locomotion).

---

### T5 — B3 Planar5 **diag**（可选，加强 Layer 2）

| RNG | best UP @late | best DOWN @late | Strong gate (`<0.92·UP`) |
|-----|---------------|-----------------|--------------------------|
| 42 | 487 | **161** | PASS |
| 43 | 555 | **227** | PASS |
| 44 | 145 | 141 | FAIL† |
| 45 | 487 | **404** | PASS |
| 46 | 440 | **330** | PASS |

† Still `DOWN < UP` (weak inversion). Strong gate **4/5**; weak **5/5**.

---

### Footnote only — B2 suite（不要当主结果）

| Policy | Cost (mean±std) |
|--------|-----------------|
| Uniform | 10.47 ± 0.47 |
| UCB / BO / Greedy-\(s_r\) | 10.30 ± 0.80 |
| Greedy-\(\chi\) | 11.13 ± 0.61 |

**Footnote text:**  
> On the quadruped *suite*, all policies find the gem basin (near-tie). We therefore cite the quadruped *diagnostic* for ranking inversion, not suite separation.

---

## 4. Suggested result prose（英文段落，可改写后贴）

### 4.1 Ranking flip (after theory / before bandit tables)

> Equal per-arm budgets on Nav homotopy seeds show that both \(s_r\) and \(s_\chi\) can disagree with the oracle ordering after the first batch (Table T3). The same early-vs-late inversion appears on the quadruped template diagnostic (Table T4): the over-lift cluster improves fastest after a short warm-up, yet the mild-clear cluster attains a lower terminal cost once more iterations are granted. These diagnostics instantiate the local-improvement incompleteness argument on real constrained-iLQR trajectories; they do not by themselves prove that any particular bandit policy dominates Uniform.

### 4.2 Main budgeting results

> Under the frozen protocol, adaptive allocation with \(s_r\) improves over Uniform on two independent systems. On Nav2D-Hard, where terminal iterates are often slightly infeasible at \(\varepsilon=10^{-3}\), we compare policies by feasible-first ranking; the adaptive bloc wins on all five RNG seeds (Table T1). On Planar5, every run is feasible: BO–EI and Greedy-\(s_r\) reach mean cost \(269.7\pm143.4\) versus Uniform \(433.9\pm89.9\), while Greedy-\(\chi\) remains worse at \(402.4\pm50.5\) (Table T2)—a clean negative control without feasibility confounding.

### 4.3 Limitations（诚实一段）

> Two limitations are intentional. First, the quadruped *bandit suite* is near-tied because warm-up already surfaces the better basin; the instance is retained as a contact-rich ranking-flip diagnostic, not as a policy-separation benchmark. Second, Nav2D-Hard rarely returns fully feasible incumbents under the frozen tolerance; we therefore report feasible-first outcomes rather than a single feasible-cost percentage gap. Harder seed libraries or relaxed tolerances could further separate UCB from BO–EI; under the present freeze they remain tied.

---

## 5. Checklist before you compile tex

- [ ] 主表只有 **T1 + T2**（分配 claim）
- [ ] 理论证人有 **T3 + T4**（inversion claim）
- [ ] B2 **suite** 最多脚注
- [ ] 未出现 3096 / 2138
- [ ] B2 模型写成 soft-contact **template**
- [ ] Greedy-χ：B1（可行纠缠）+ B3（全可行仍差）都提到
- [ ] Discussion 删掉「还需要更难种子库才能分开 Uniform」的旧 Phase-1 打平叙事（已被 B1/B3 取代）

---

## 6. 一键路径索引

| 文件 | 内容 |
|------|------|
| 本文件 | 文稿指南（你正在看的） |
| `FROZEN_FOR_MANUSCRIPT.md` | 数字源表 |
| `PROTOCOL_FREEZE.md` | 参数锁 |
| `nav2d_hard_head_to_head.csv` | B1 逐 RNG |
| `*_multiseed.csv` / `*_summary.csv` | 原始聚合 |
| `../PLAN_B_BENCHMARK_ROLES.md` | 角色总表 |
