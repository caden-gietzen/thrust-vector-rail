# Project Metrics

> **Purpose:** This document defines the quantitative acceptance criteria for each stage of
> the project. It is not enough to *show something works* — each stage
> must hit defensible metrics before the next stage is allowed to begin. Each stage's **exit
> metric becomes the next stage's entry assumption**.

---

## 0. Requirements flowdown (top-down)

The per-stage metrics in Sections 1–5 are *allocated* from a single top-level requirement.

### The chain

```
L0  hardware tracking |e| ≤ 5 mm peak               ← the only number chosen outright
 ↑ earns
L1  sim tracking ≤ 2.5 mm   (degradation factor 2×) ← Stage 5 requirement
 ↑ earns
L2  split the 2.5 mm sim budget (RSS):
      controller / disturbance ≤ 2.0 mm             ← Stage 4 controller requirement
      estimator-induced        ≤ 1.5 mm             ← Stage 4 estimator requirement
 ↑ earns
L3  model fidelity good enough to design the
    bandwidth L2 needs (small ν-gap past ω_c)       ← Stage 3 requirement
 ↑ earns
L4  stabilizer keeps the rig alive and excites
    enough to reach L3 fidelity                     ← Stage 2 requirement
 ↑ earns
L5  open-loop CIs tight enough that one robust
    controller covers the whole uncertainty set     ← Stage 1 requirement
```

> **L0 is the chosen requirement: peak position tracking error $\leq 5$ mm on hardware over
> the operating envelope.** The $2\times$ sim-to-hardware degradation factor is a documented placeholder to be confirmed and updated once Stage 5 measures the real degradation.)

### The error budget

| Level | Allocation | Source / rationale |
|-------|-----------|--------------------|
| **L0** hardware peak error | $\leq 5$ mm | The qualification requirement |
| **L1** sim peak error | $\leq 2.5$ mm | $= 5\ \text{mm} / k_\text{hw}$ with degradation factor $k_\text{hw} = 2$ (placeholder, → Stage 5) |
| **L2a** controller / disturbance | $\leq 2.0$ mm | RSS with L2b → $\sqrt{2.0^2 + 1.5^2} = 2.5$ mm |
| **L2b** estimator-induced | $\leq 1.5$ mm | Feeds L2a through the LQG loop; NEES/NIS consistency is a separate pass/fail gate |
| **L3** model uncertainty | $\delta_\nu$ small out past required $\omega_c$ | Achievable bandwidth is capped where multiplicative uncertainty reaches 0 dB |
| **L4** excitation SNR + robust stability | sufficient to reach L3 over the L5 set | Estimate variance $\propto 1/(\text{SNR}\cdot N)$ |
| **L5** open-loop parameter CIs | tight enough for one robust controller to cover the set | Set by L4 stabilizability, *and* by the friction-feedforward demand below |

### The feasibility catch

The L2a controller allocation sets a **required bandwidth**. For a position loop with integral
action, the peak excursion to a force disturbance is approximately $e_\text{peak} \approx F_d /
(m\,\omega_c^2)$. Rejecting the $\mu_c \approx 1$ N Coulomb friction to the 2.0 mm allocation
would require

$$
\omega_c \gtrsim \sqrt{\frac{F_d}{m\,e_\text{allow}}}
= \sqrt{\frac{1\ \text{N}}{0.4536\ \text{kg}\cdot 0.002\ \text{m}}}
\approx 33\ \text{rad/s} \approx 5.3\ \text{Hz}.
$$

But the servo delay $L_\theta = 28.8$ ms caps the achievable bandwidth at $\omega_c \lesssim
0.3/L_\theta \approx 10\ \text{rad/s} \approx 1.7$ Hz. At that ceiling, pure feedback yields
$e_\text{peak} \approx 1/(0.4536\cdot 10^2) \approx 22$ mm — far over 5 mm.

**Conclusion — an architectural requirement, derived not assumed:** the 5 mm spec is
infeasible with feedback alone on this actuator. **Model-based friction feedforward is
mandatory**, and friction must be cancelled to $\sim$90% so the residual disturbance the
feedback sees is $\sim$0.1 N ($\Rightarrow \sim$2 mm). That demand flows straight back to
Stage 1: the **friction-ID accuracy requirement (L5) is set by the feedforward cancellation
the tracking spec demands**, not by an arbitrary CI width. This is the chain working as
intended — the top-level number reaches all the way down and tells you how good your friction
identification has to be, and why.

> These coefficients ($e_\text{peak} \approx F_d/(m\omega_c^2)$, $\omega_c L \lesssim 0.3$) are
> order-of-magnitude rules of thumb for sizing the budget; confirm the exact excursions in
> simulation once the controller exists.

---

## The one honesty metric per stage

Every stage has a single metric that *qualifies* the stage:

| Stage | The honesty metric | Why it's the one that counts |
|-------|--------------------|------------------------------|
| 1 — Open-loop ID | Residual whiteness on **validation** data | "Works" = train fit; trustworthy = validation residuals carry no dynamics |
| 2 — Crude stabilizer | **Worst-case-over-set** margins | Nominal margins are meaningless under model uncertainty |
| 3 — Closed-loop ID | $\nu$-gap inside the stabilizer's robustness ball | A fit % does not prove the model is *control-relevant* |
| 4 — Aggressive control + estimation | Filter consistency (NEES / NIS in-bounds) | An accurate but overconfident filter is **not** validated |
| 5 — Sim vs. real | Monte Carlo envelope coverage + re-ID residual | "It tracked on hardware" is not the same as "I predicted reality" |

---

## 1. Open-loop actuator identification (servo, thrust, friction)

**Goal:** a trustworthy truth model with quantified uncertainty.

**Gating metric:** residual whiteness and input-decorrelation on **validation** (held-out)
data. If residuals are white and uncorrelated with the input, the dynamics are captured.

| Metric | Target | Why it is the right metric |
|--------|--------|----------------------------|
| Validation fit (VAF or NRMSE) | VAF $\geq 90\%$ / NRMSE-fit $\geq 80\%$ on held-out data | "Show it works" = train fit; trustworthy = *validation* fit |
| Train–validation fit gap | $\leq 5$–$10$ pts | Overfit detector — a large gap means noise was memorized |
| Residual autocorrelation | within 95% confidence band | No unmodeled *dynamics* left in the residual |
| Residual–input cross-correlation | within 95% band | Input no longer explains the residual → model order sufficient |
| FRF coherence $\gamma^2$ | $\geq 0.9$ over identified band | **Defines the frequency band you may trust** — handed to Stages 2 and 4 |
| Parameter CIs ($K$, $\tau$, $L$) | relative SE $\leq 10$–$20\%$ | **This *is* the Stage 2 uncertainty set** — not optional |
| Cross-seed / cross-run parameter spread | within the CIs above | Repeatability vs. one lucky fit |
| Friction: Coulomb/viscous split + asymmetry | each with CI; directional bound | Friction is a worst-case *disturbance* → need the **bounding envelope**, not a point estimate |

**Exit → Entry:** the parameter CIs and the friction envelope *become* the Stage 2
uncertainty set. A CI too wide to design against is a Stage 1 failure, not a Stage 2 problem.

---

## 2. Crude robust stabilizer (bootstrap for closed-loop ID)

**Goal:** safe closed-loop operation over the *entire* uncertainty set, with enough authority
to inject persistent excitation. This controller's job is **safe excitation, not tracking
performance.**

**Gating metric:** worst-case robustness margins over every Monte Carlo sample of
the uncertainty set — **not** the nominal model.

| Metric | Target | Why |
|--------|--------|-----|
| Gain / phase margin (worst-case over set) | GM $\geq 6$ dB, PM $\geq 30$–$45^\circ$ at 100% of set samples | Stability against the model you don't know exactly |
| Sensitivity peak $\lVert S \rVert_\infty$ ($M_s$) | $\leq 2$ (6 dB) across set | Single-number robustness; bounds disturbance amplification |
| Complementary peak $\lVert T \rVert_\infty$ ($M_t$) | $\leq 1.3$ across set | Bounds noise amplification and ringing |
| Delay margin | $\geq 2 L_\theta \approx 58$ ms | $L_\theta$ is itself uncertain — pad it |
| Closed-loop crossover | $\leq \sim 1$ Hz ($\ll 1/L_\theta$) | Conservative band per servo identification recommendation |
| Friction limit-cycle amplitude | no sustained limit cycle; any transient stick-slip bounded $< 5$ mm | Coulomb + asymmetry as input disturbance → check for stick-slip. Loose bound — this stage is about *safe* operation for data collection, not precision |
| Servo command headroom under excitation | stays inside saturation with margin while injecting the ID signal | Excitation that saturates biases the Stage 3 data |
| **Set coverage** | **100% of MC / vertex cases stable and meeting margins** | The actual pass criterion |
| Achievable excitation SNR under the stabilizer | sufficient for persistent excitation in the target band | The *purpose* metric — can good closed-loop data actually be collected? |

**Exit → Entry:** the robustness ball this controller tolerates (its $M_s$ / $\nu$-gap radius)
defines the target that the Stage 3 identified model must land inside.

---

## 3. Closed-loop identification / control-relevant model

**Goal:** a better model in the bandwidth that matters for the aggressive controller, free of
closed-loop correlation bias.

**Gating metric:** the $\nu$-gap $\delta_\nu$ between the closed-loop-identified model and the
model the stabilizer assumes, measured against the stabilizer's stability margin $b(P,C)$.
Pass = the identified plant sits inside the stabilizer's robustness ball:

$$
\delta_\nu(\hat{P}, P_\text{design}) < b(P_\text{design}, C)
$$


| Metric | Target | Why |
|--------|--------|-----|
| $\nu$-gap $\delta_\nu$ vs. $b(P,C)$ | $\delta_\nu < b(P,C)$ | Confirms the bootstrap controller still stabilizes the *better* model — closes the chicken-and-egg loop |
| Control-relevant fit (bandwidth-weighted) | high in crossover region | A model accurate *where the next controller has authority*, not at DC |
| Closed-loop validity (residual $\perp$ reference) | within band | Guards against closed-loop correlation bias (two-stage / IV / known-controller method) |
| Order selection (AIC / BIC or fit-vs-order knee) | clear knee | Avoid over-ordering |
| Uncertainty reduction vs. Stage 1 | CIs / model-set shrink in the control band | **The metric that proves closed-loop ID was worth doing** |

**Exit → Entry:** the residual model uncertainty here sets how aggressive Stage 4 is *allowed*
to be (the robust performance budget).

---

## 4. Aggressive tracking controller + estimator (simulation)

**Goal:** high-performance tracking and state reconstruction against the earned model. Keep
the controller and estimator metrics separate, then run a combined LQG check.

**Gating metric (controller):** tracking error vs. the ~2 mm precision spec.
**Gating metric (estimator):** filter consistency (NEES / NIS in-bounds). An accurate but
overconfident filter is not validated.

### 4.1 Controller (gain-scheduled LQR / tracking)

These are **simulation** targets, sized from the L2a controller allocation (2.0 mm peak) so
that the L1 sim budget (2.5 mm peak) is met before hardware.

| Metric | Target | Source |
|--------|--------|--------|
| Peak position tracking error | $\leq 2.0$ mm | L2a allocation |
| RMS tracking error on reference profile | $\leq 1.0$ mm | $\approx$ peak/2 for the reference profile |
| Steady-state position error | $\leq 0.5$ mm | Integral action; floor is encoder quantization $\approx 0.015$ mm |
| Overshoot | $\leq 10\%$ | Well-damped aggressive response |
| Settling time (5% band) | $\leq 1.0$ s | Consistent with $\omega_c \sim 10$ rad/s |
| Margins across scheduling points | GM / PM hold at every gain-schedule node (pole-migration plot) | Robustness |
| Control saturation duty cycle | low / bounded; no sustained saturation | Authority |

### 4.2 Estimator (EKF)

| Metric | Target | Why |
|--------|--------|-----|
| Position estimate RMS error $\hat{p}$ vs. truth | $\leq 0.5$ mm | Sized from the L2b estimator allocation (1.5 mm peak) so it does not blow the controller budget when fed through the LQG loop |
| Velocity / $\theta$ / $T$ estimate RMS error | within sim sensitivity budget (set once the loop exists) | These propagate to position error through the loop; size them in sim by sensitivity, not in isolation |
| **NEES** (normalized estimation error squared) | inside $\chi^2$ 95% bounds (4 states) | **Covariance honesty** — the most-skipped metric |
| **NIS** (normalized innovation squared) | inside $\chi^2$ bounds | Validates $Q$ / $R$ tuning without truth |
| Innovation whiteness | autocorrelation within band | No unmodeled dynamics left in the filter |
| Convergence time from a bad initial condition | bounded | Filter robustness |
| Gating (Mahalanobis / NIS) false-reject rate | $\leq$ chosen $\alpha$ | Outlier handling is real, not cosmetic |

### 4.3 Combined (LQG / separation principle)

| Metric | Target |
|--------|--------|
| LQG vs. full-state-LQR tracking degradation | quantified, bounded |
| Separation-principle check | margins / poles hold, or document where nonlinearity + scheduling break it |

**Exit → Entry:** the predicted sim performance envelope and predicted margins become the
hypotheses that Stage 5 tries to falsify on hardware.

---

## 5. Simulation vs. real comparison

**Goal:** a *quantified* sim-to-hardware gap — not a demonstration that it works on hardware.

**Gating metric:** the hardware response falls inside the Monte-Carlo-predicted envelope. A
miss is not a failure of the document — the size and direction of the miss is the finding.

| Metric | Target / meaning |
|--------|------------------|
| Sim-vs-hardware trajectory match (NRMSE / peak err, same reference + same controller) | high match → model earned; the **prediction-error** number |
| Hardware tracking error $\div$ sim tracking error | degradation ratio — honest measure of what reality costs |
| Monte Carlo envelope coverage | % of hardware runs inside the predicted bounds (high target; misses are the interesting data) |
| Margin retention | no oscillation onset / instability that sim said was safe |
| Estimator consistency on real data | NIS in-bounds on hardware (innovation-only — no truth available) |
| Post-mismatch re-ID residual | after feeding the discrepancy back into the model, does the gap shrink? |

---

## Metric definitions

For reference, the less-common metrics above:

- **VAF** (variance accounted for): $\text{VAF} = \left(1 - \dfrac{\operatorname{var}(y - \hat{y})}{\operatorname{var}(y)}\right) \times 100\%$.
- **NRMSE fit:** $\text{fit} = \left(1 - \dfrac{\lVert y - \hat{y}\rVert}{\lVert y - \bar{y}\rVert}\right) \times 100\%$ (MATLAB `compare` convention).
- **Coherence** $\gamma^2(\omega) = \dfrac{|S_{uy}(\omega)|^2}{S_{uu}(\omega)\,S_{yy}(\omega)}$; $1$ = fully linear/noise-free at that frequency.
- **Sensitivity** $S = (1 + PC)^{-1}$, **complementary sensitivity** $T = PC(1+PC)^{-1}$; $\lVert\cdot\rVert_\infty$ is the peak over frequency.
- **$\nu$-gap** $\delta_\nu(P_1, P_2) \in [0,1]$: a frequency-weighted distance between plants that is meaningful for feedback; **$b(P,C)$** is the generalized stability margin. Robust stability holds when $\delta_\nu < b(P,C)$.
- **NEES** $\epsilon_k = (x_k - \hat{x}_k)^\top P_k^{-1}(x_k - \hat{x}_k)$; for $n$ states the 95% acceptance interval comes from the $\chi^2_n$ distribution (time-averaged over a Monte Carlo ensemble).
- **NIS** $\nu_k^\top S_k^{-1} \nu_k$ where $\nu_k$ is the innovation and $S_k$ its covariance; same $\chi^2$ test, usable on hardware because it needs no truth.
