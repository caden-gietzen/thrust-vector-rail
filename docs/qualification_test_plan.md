# Qualification Test Plan

> **Status:** Defines the concrete qualification maneuvers, the disturbance-injection method, and the pass/fail scoring that demonstrate the top-level requirements in [requirements.md](requirements.md). This is the V&V-facing companion to that document: [requirements.md](requirements.md) says *what* the system shall do; this file says *how* it is demonstrated and *what artifact* proves it. The per-stage acceptance budget these maneuvers are scored against lives in [project_metrics.md](project_metrics.md).

## 1. Purpose and scope

The final rail is qualified against a suite of **four maneuvers**. Each isolates a different job of the plant/controller, so a pass or failure points at a specific mechanism rather than a blended score:

| Maneuver | What it exercises | Requirement(s) |
|---|---|---|
| **M1** — minimum-jerk slew | tracking of a smooth commanded profile | [R4](requirements.md#2-tracking-performance), [R5](requirements.md#2-tracking-performance), [R3](requirements.md#1-operating-envelope) |
| **M2** — 0.5 Hz sine | sustained sinusoidal tracking | [R4](requirements.md#2-tracking-performance), [R5](requirements.md#2-tracking-performance), [R1](requirements.md#1-operating-envelope), [R2](requirements.md#1-operating-envelope) |
| **M3** — center-hold disturbance rejection | feedback rejection of a force step | [R13](requirements.md#5-disturbance-rejection), [R6](requirements.md#2-tracking-performance) |
| **M4** — multisine closed-loop ID | model validation + closed-loop bandwidth | [R14](requirements.md#6-model-validation) |

**Shared tracking spec (M1, M2):** peak $\lvert e_p\rvert \le 15$ mm, RMS $e_{p,\text{RMS}} \le 7$ mm ([R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance)). Sized by the harder maneuver.

## 2. Common setup and scoring

- **Vehicle:** mass weighed before each campaign (nominal $0.5$ kg, design range $0.45$–$0.75$ kg per [hardware_selection.md](hardware_selection.md#2-requirements)).
- **Rail frame:** homed with [`lib/encoder_home.py`](../firmware/pico_micropython/lib/encoder_home.py); position zero $=$ rail center; travel bounded by the measured $316.5$ mm end-stop separation.
- **Encoder scale:** $64\,810.4$ counts/m ([encoder_calibration/results.md](../experiments/encoder_calibration/results.md)); quantization floor $\approx 0.015$ mm.
- **Logging ([R11](requirements.md#4-sensing-and-logging-requirements)):** $p$, $p_\text{ref}$, servo command, thrust command, controller output $\theta_\text{cmd}$, and timestamps, at the control/log rate, sufficient to recompute every metric offline ([R12](requirements.md#4-sensing-and-logging-requirements)).
- **Error convention:** $e_p(t) = p(t) - p_\text{ref}(t)$. **Peak** $= \max_t \lvert e_p\rvert$ over the scoring window; **RMS** $= \sqrt{\frac{1}{N}\sum e_p^2}$ over the scoring window. Each maneuver defines its own scoring window below (transients before the window are excluded).
- **Pass discipline:** a maneuver passes only if every listed criterion is met; results are reported as a per-maneuver card plus the consolidated report in [Section 7](#7-consolidated-qualification-report).

## 3. M1 — Minimum-jerk slew

**Definition.** A rest-to-rest minimum-jerk trajectory from $-100$ mm to $+100$ mm (a $\Delta p = 200$ mm move) over duration $T = 2.5$ s:

$$
p_\text{ref}(t) = p_0 + \Delta p\left(10\tau^3 - 15\tau^4 + 6\tau^5\right), \qquad \tau = t/T,
$$

with zero velocity, acceleration, and jerk at both ends. Peak kinematics: $v_\text{peak} = 0.15$ m/s and $a_\text{peak} = 0.185$ m/s² (inertial force $\approx 0.09$ N; actual control authority required includes friction and disturbance rejection). The maneuver is deliberately gentle on authority; it probes tracking of a realistic band-limited command, not saturation.

**Scoring window.** Start of motion through $1$ s after nominal arrival.

**Pass criteria.**
- peak $\lvert e_p\rvert \le 15$ mm and RMS $\le 7$ mm over the window ([R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance));
- settle to $\le 3$ mm within $1$ s of nominal arrival;
- no end-stop contact ([R3](requirements.md#1-operating-envelope)).

**Deliverables.** Reference-vs-actual position plot, $e_p(t)$ time series, pass/fail card. Run in both directions ($-100 \to +100$ and $+100 \to -100$).

## 4. M2 — Sinusoidal tracking

**Definition.** $p_\text{ref}(t) = A\sin(2\pi f t)$ with $A = 100$ mm (amplitude; $200$ mm peak-to-peak) and $f = 0.5$ Hz, run for at least $5$ cycles. Span $200$ mm on the $316.5$ mm rail leaves $\approx 58$ mm/end margin. Peak acceleration $a_\text{peak} = 0.99$ m/s² (inertial force $\approx 0.49$ N; actual control authority required includes friction and disturbance rejection).

**Scoring window.** After the first full cycle (exclude the start-up transient), over the remaining steady-state cycles.

**Pass criteria.** peak $\lvert e_p\rvert \le 15$ mm and RMS $\le 7$ mm over the steady-state window ([R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance)); no end-stop contact ([R3](requirements.md#1-operating-envelope)).

**Deliverables.** Reference-vs-actual position plot, $e_p(t)$ time series, per-cycle peak/RMS table, pass/fail card.

## 5. M3 — Center-hold disturbance rejection

**Definition.** Regulate $p = 0$ (rail center). After the controller settles, inject a **known input-referred force step** $F_d = 1.0$ N, hold for $\approx 5$ s, then remove it. Repeat for **both** signs of $F_d$.

**Injection method.** This rail has no external force actuator, so the disturbance is produced by the vectoring actuator itself and hidden from the controller: a bias is added to the commanded thrust-vector angle that the control law does not account for, sized so the resulting rail-axis force equals $F_d$. The required vector-angle bias $\Delta\theta_d = \arcsin(F_d/T)$ scales with the design operating thrust. In simulation this is added directly as a rail-axis force step at the plant input; on hardware it is the equivalent un-modeled command bias. Because $F_d$ is commanded, it is exactly known and repeatable, which is what makes the excursion scoreable against the $e_\text{peak} \approx F_d/(m\omega_c^2)$ budget in [project_metrics.md](project_metrics.md).

**Metrics (per sign).** peak excursion from center; recovery time to re-enter $\pm 3$ mm; steady-state error while the disturbance is held (tests integral action); and behavior on disturbance removal.

**Pass criteria ([R13](requirements.md#5-disturbance-rejection)).** peak excursion $\le 15$ mm; recover to $\le 3$ mm within $2$ s; steady-state $\le 1$ mm under the held disturbance; no sustained oscillation, in **both** directions (asymmetry between signs is expected from friction and is reported, not failed, unless a sign breaches spec).

**Companion check ([R6](requirements.md#2-tracking-performance)).** An initial-condition release: start the cart at an offset (up to $\pm 100$ mm) with the loop closed and confirm it recovers to the hold point without sustained oscillation.

**Deliverables.** Disturbance-recovery plot (excursion + settle) per sign, a table of peak / recovery-time / steady-state for both signs, IC-release recovery plot, pass/fail card.

## 6. M4 — Multisine closed-loop identification

**Definition.** A pseudo-random periodic signal (PRPS) multisine reference about center, small amplitude ($\approx \pm 25$ mm, to stay in a near-linear regime), exciting a band from $\approx 0.05$ Hz up to $\approx 2.5$ Hz, one period $\approx 40$ s, repeated over multiple seeds. This reuses the existing reference tooling ([`tvr_stabilizer/make_prps_reference.m`](../tvr_stabilizer/make_prps_reference.m), `ref.type = 5`).

**Analysis.**
- Estimate the closed-loop frequency response (reference → measured position) and coherence $\gamma^2(\omega)$; the coherent band ($\gamma^2 \ge 0.9$) defines where the estimate is trustworthy.
- Report the $-3$ dB closed-loop bandwidth.
- Validate the design model: compute the $\nu$-gap $\delta_\nu(\hat{P}, P_\text{design})$ against the stabilizer's generalized stability margin $b(P,C)$.

**Pass criteria ([R14](requirements.md#6-model-validation)).** $\delta_\nu(\hat{P}, P_\text{design}) < b(P,C)$ (the identified plant sits inside the stabilizer's robustness ball); coherent band covers the reported closed-loop bandwidth. This is not a tracking-error test.

**Deliverables.** Closed-loop Bode plot + coherence with the design-model overlay, the $\nu$-gap number, the identified $-3$ dB bandwidth, and the seed-to-seed spread.

## 7. Consolidated qualification report

A single Markdown report summarizes all four maneuvers against spec — the traceable artifact that closes the requirements loop:

- one pass/fail line per maneuver with the measured peak/RMS (or excursion/recovery, or $\nu$-gap/bandwidth) next to the requirement value and R-ID;
- links to the per-maneuver plots;
- the operating conditions (mass, thrust setpoint, controller version, date, dataset);
- an explicit note of any margin held or any maneuver that only just passed, so the sim-to-hardware degradation budget (L1 in [project_metrics.md](project_metrics.md)) can be checked.

## 8. Spec summary

| Maneuver | Reference | Scoring window | Pass criteria | Requirement |
|---|---|---|---|---|
| M1 | min-jerk $-100 \to +100$ mm, $T=2.5$ s | move + 1 s settle | peak $\le 15$ mm, RMS $\le 7$ mm, settle $\le 3$ mm/1 s, no end-stop | [R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance)/[R3](requirements.md#1-operating-envelope) |
| M2 | $100\sin(2\pi\cdot 0.5\,t)$ mm, $\ge 5$ cycles | after cycle 1 | peak $\le 15$ mm, RMS $\le 7$ mm, no end-stop | [R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance) |
| M3 | hold $p=0$; $F_d = \pm 1.0$ N step | disturbance on → removal | peak $\le 15$ mm, recover $\le 3$ mm in $2$ s, ss $\le 1$ mm, both signs | [R13](requirements.md#5-disturbance-rejection)/[R6](requirements.md#2-tracking-performance) |
| M4 | PRPS multisine $\pm 25$ mm, $0.05$–$2.5$ Hz | coherent band | $\delta_\nu < b(P,C)$; report BW | [R14](requirements.md#6-model-validation) |

**Open parameters** (defaults above; revisit if the design or hardware changes): M1 duration $T = 2.5$ s, M3 disturbance magnitude $F_d = 1.0$ N, M4 excitation amplitude and band. These are sizing defaults, not requirements; they are tuned in feasibility and simulation before the hardware run.
