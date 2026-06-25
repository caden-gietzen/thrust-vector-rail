# Objectives, Requirements, and Constraints

> This document is the top-level specification for the project. The quantitative numbers below
> are not chosen in isolation — they are the top of the requirements flowdown formalized in
> [docs/project_metrics.md](project_metrics.md). Read that document for the per-stage allocation
> and the "earn the next stage" derivation; read this one for *what* the system must achieve and
> the hard limits it must achieve it within.

---

## 1. Project Objective

Precisely control the position of a thrust-vectored cart on a one-dimensional friction-dominated rail
using an indirectly-actuated propulsion force $F_\text{rail} = T\sin\theta$, achieving
**position tracking to within 5 mm peak error on hardware** over the operating envelope.

The physical control problem is deliberately *indirect*: rail acceleration is produced by
steering a thrust vector, not by a direct linear actuator. This preserves the aerial-vehicle
control concept — translational acceleration generated through the magnitude and direction of a
propulsion force — in a tabletop, single-degree-of-freedom system. See
[docs/system_architecture.md](system_architecture.md) for the design rationale.

## 2. Project Purpose

Beyond the physical objective, the rail is a **testbed for the full controls / estimation / V&V
stack**, exercised end to end against a model that was learned from hardware rather than assumed.
The guiding principle:

> You cannot estimate or control what you cannot model, and you cannot trust a model you have not
> validated against the real thing.

The project is built to around one narrative: build a trustworthy
simulator from hardware-identified parameters, design estimation and control against it, then try
to break the whole stack with realistic sensor degradation, model mismatch, and Monte Carlo
stress before trusting it on hardware.

## 3. Vehicle (Product) Functional Requirements

What the rail vehicle must *do* — capabilities of the physical system, verifiable on hardware.

1. The vehicle shall regulate cart position by commanding the thrust-vector angle $\theta$, with
   thrust held at a fixed feedforward operating point during the bootstrap stage.
2. The vehicle shall operate in closed loop under a stabilizing controller without exceeding rail
   travel and end-stop limits.
3. The vehicle shall reconstruct its full state $[p, v, \theta, T]$ from position-only
   measurement.
4. The vehicle shall command thrust and thrust-vector angle within their physical ranges (see
   [Constraints](#6-constraints)) under real-time control on the embedded controller.
5. The vehicle shall log position, actuator commands, and controller outputs at a fixed sample
   rate for offline analysis.

## 4. Program (Methodology) Requirements

What the *engineering project* must do to earn trust in the vehicle above. Each is a stage gate
that earns the next (see [docs/project_metrics.md](project_metrics.md)).

1. The project shall identify the actuator and friction subsystems from hardware excitation data,
   producing parameterized models with quantified uncertainty (servo, thrust — **done**;
   friction — underway).
2. The project shall assemble those subsystem models into a single nonlinear truth model
   (see [docs/rough_truth_model.md](rough_truth_model.md)).
3. The project shall design a crude, robust bootstrap controller against the truth-model
   uncertainty set, so the rail can be run safely in closed loop.
4. The project shall, under that stabilizer, collect closed-loop identification data sufficient to
   earn a control-relevant model.
5. The project shall design an aggressive gain-scheduled tracking controller and an EKF state
   estimator against that model in simulation.
6. The project shall validate the stack by comparison of simulation prediction against hardware
   response, quantifying the sim-to-hardware gap.

## 5. Performance Requirements

How *well* it must do it. These are the top of the [error budget](project_metrics.md#0-requirements-flowdown-top-down);
the full per-stage allocation lives in [docs/project_metrics.md](project_metrics.md).

| ID | Requirement | Value | Notes |
|----|-------------|-------|-------|
| P1 | Hardware peak position tracking error (L0) | $\leq 5$ mm | The qualification requirement |
| P2 | Hardware RMS position tracking error | $\leq 2$ mm | $\approx$ peak/2.5 for the reference profile |
| P3 | Sim-to-hardware degradation factor | $\leq 2\times$ | Placeholder; confirmed at Stage 5 |
| P4 | Simulation peak tracking error (L1) | $\leq 2.5$ mm | $= $ P1 $/$ P3 |
| P5 | Estimator covariance consistency | NEES / NIS within $\chi^2$ 95% bounds | Filter honesty, not just accuracy |
| P6 | Worst-case robustness over uncertainty set | GM $\geq 6$ dB, PM $\geq 30$–$45^\circ$, $\lVert S\rVert_\infty \leq 2$ | At 100% of set samples, not nominal |
| P7 | Closed-loop ID model fidelity | $\nu$-gap $\delta_\nu < b(P,C)$ of the stabilizer | Identified plant inside the robustness ball |

**5 mm peak (P1) is the budgeted requirement** the rest of the stack is sized to.

## 6. Constraints

Hard physical, electrical, computational, and safety limits the design must live within.

### Actuator and authority
1. **Servo delay limits achievable bandwidth.** The servo is first-order-plus-delay with
   $L_\theta = 28.8$ ms, capping closed-loop bandwidth at $\omega_c \lesssim 0.3/L_\theta \approx
   1.7$ Hz. This is a hard ceiling, not a tuning choice.
2. **Friction feedforward is therefore mandatory.** Rejecting $\mu_c \approx 1$ N Coulomb friction
   to the tracking spec by feedback alone would require $\sim 5.3$ Hz of bandwidth — above the
   ceiling in (1). Model-based friction feedforward ($\sim 90\%$ cancellation) is a *derived*
   architectural requirement; see [the feasibility catch](project_metrics.md#0-requirements-flowdown-top-down).
3. **Servo vectoring range:** $\approx \pm 90^\circ$ over 450–2450 µs.
4. **Thrust range:** 0.23–4.17 N over 1075–1950 µs; thrust dynamics are FODT
   ($\tau_T = 78.1$ ms, $L_T = 25.2$ ms).
5. **Crude-stabilizer authority is servo-only.** During the bootstrap stage thrust is held at a
   fixed feedforward operating point, so position is regulated through $\theta$ alone with the
   local thrust gain fixed.

### Sensing
6. **Position-only measurement.** The only direct measurement is cart position from a quadrature
   encoder; velocity, $\theta$, and $T$ must be estimated. Conversion constant
   $C_m = 64{,}810.4$ counts/m, quantization floor $\approx 0.015$ mm.
7. The encoder introduces realistic sensing limits — finite resolution, missed counts, noise, and
   sampling constraints — which the estimator and V&V stages must explicitly model.

### Physical / electrical
8. **Vehicle mass** $m = 0.4536$ kg with substantial uncertainty $\sigma_m = 0.136$ kg
   ($\sim 30\%$); mass must be weighed before each campaign and treated as an uncertainty axis.
9. Finite rail length and physical end-stops bound usable travel.
10. Battery voltage variation perturbs the thrust map over a run.

### Computational / safety
11. Control runs in real time on a Raspberry Pi Pico RP2040 at a fixed sample rate; encoder
    decoding is offloaded to the PIO peripheral.
12. The system shall remain compact enough to operate as a tabletop testbed.
13. Exposed-propeller operation imposes safety limits on standalone and closed-loop runs.

## 7. Evaluation Metrics

How methods are compared. The complete, per-stage metric set with gating criteria and definitions
is [docs/project_metrics.md](project_metrics.md). At the project level, the discriminating
comparisons are:

- **Gain-scheduled vs. fixed-gain control** across the operating envelope — the model-mismatch
  Monte Carlo (Stage 5) is what makes this comparison substantive.
- **Full-state LQR vs. LQG** — quantify the performance cost of estimation and whether the
  separation principle holds under nonlinearity + scheduling.
- **EKF consistency** — NEES / NIS, innovation whiteness, gating false-reject rate.
- **Identification quality** — validation-data residual whiteness and parameter-CI width
  (Stage 1); $\nu$-gap reduction (Stage 3).

## 8. Success Criteria

What counts as a successful milestone. Each is the gating metric of its stage in
[docs/project_metrics.md](project_metrics.md):

1. **Stage 1** — open-loop models pass validation-data residual whiteness with parameter CIs tight
   enough to define a stabilizable uncertainty set.
2. **Stage 2** — the crude stabilizer holds GM/PM and $\lVert S\rVert_\infty$ margins at 100% of
   the uncertainty set and permits persistent excitation without saturation.
3. **Stage 3** — the closed-loop-identified model lands inside the stabilizer's robustness ball
   ($\delta_\nu < b(P,C)$) with reduced uncertainty in the control band.
4. **Stage 4** — the aggressive controller meets the sim tracking budget (P4) and the EKF is
   consistent (P5).
5. **Stage 5 (project success)** — hardware tracking meets P1 ($\leq 5$ mm peak), the response
   falls inside the Monte-Carlo-predicted envelope, and the sim-to-hardware gap is *quantified*
   and fed back into the model.
