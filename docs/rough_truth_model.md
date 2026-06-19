# Rough Truth Model

> **Status:** This is the **rough truth model** — the plant assembled from open-loop,
> hardware-identified actuator and friction parameters. It is the model we design the
> first **crude stabilizer** against (a robust hand-tuned PID), so that we can safely run
> **closed-loop system identification** and *earn* a refined model. It is deliberately
> labelled "rough," not "earned": every parameter below comes from open-loop excitation of
> an individual subsystem, none from the coupled closed-loop system. See
> [Why this is "rough" and not "earned"](#why-this-is-rough-and-not-earned).

This document consolidates the conclusions of the completed open-loop identification work
into a single plant definition. It is the bridge between Phase 1 identification and the
crude-stabilizer design that bootstraps closed-loop identification.

Source identification reports:
- Servo — [experiments/servo_identification/results.md](../experiments/servo_identification/results.md)
- Thrust — [experiments/thrust_identification/results.md](../experiments/thrust_identification/results.md)
- Friction — [experiments/friction_identification/results.md](../experiments/friction_identification/results.md)
- Encoder — [experiments/encoder_calibration/results.md](../experiments/encoder_calibration/results.md)

---

## 1. State, inputs, and outputs

### State vector

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $p$ | cart position along the rail | m |
| $v$ | cart velocity along the rail | m/s |
| $\theta$ | thrust-vector (servo) angle | rad |
| $T$ | motor thrust magnitude | N |

### Inputs

| Symbol | Description | Units |
|--------|-------------|-------|
| $u_\theta$ | commanded servo PWM (delta from neutral) | µs |
| $u_T$ | commanded ESC PWM | µs |

### Measured output

Only position is directly measured (quadrature encoder); velocity is estimated. This is
the sensing reality the closed-loop controller and, later, the estimator must live with.

$$
y = p = \frac{N_\text{enc}}{C_m}, \qquad C_m = 64{,}810.4 \ \text{counts/m}
$$

---

## 2. Equations of motion (nonlinear truth model)

$$
\dot{p} = v
$$

$$
\dot{v} = \frac{T}{m}\sin\theta \; - \; \frac{\mu_c}{m}\operatorname{sign}(v)
$$

$$
\dot{\theta} = -\frac{1}{\tau_\theta}\,\theta \; + \; \frac{K_\theta}{\tau_\theta}\,u_\theta\!\left(t - L_\theta\right)
$$

$$
\dot{T} = -\frac{1}{\tau_T}\,T \; + \; \frac{K_T}{\tau_T}\,u_T\!\left(t - L_T\right)
$$

The rail-direction coupling is the geometric nonlinearity that gives this testbed its value:

$$
F_\text{rail} = T\sin\theta
$$

Both actuators are modelled as **first-order-plus-delay (FODT)** states rather than static
maps — the servo and thrust lags are large enough relative to any usable rail bandwidth that
treating them as instantaneous would over-promise achievable performance (servo decision
documented in [results.md §9](../experiments/servo_identification/results.md)).

---

## 3. Parameter table

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Vehicle mass | $m$ | 0.4536 kg (1 lb nominal), $\sigma_m = 0.136$ kg | measured; weigh before each campaign |
| Servo DC gain | $K_\theta$ | 0.001556 rad/µs | [servo](../experiments/servo_identification/results.md) |
| Servo time constant | $\tau_\theta$ | 24.4 ms | [servo](../experiments/servo_identification/results.md) |
| Servo delay | $L_\theta$ | 28.8 ms | [servo](../experiments/servo_identification/results.md) |
| Servo neutral command | $u_{\theta,0}$ | 1431 µs | [servo](../experiments/servo_identification/results.md) |
| Thrust DC gain (global) | $K_T$ | 0.00414 N/µs | [thrust](../experiments/thrust_identification/results.md) |
| Thrust time constant | $\tau_T$ | 78.1 ms | [thrust](../experiments/thrust_identification/results.md) |
| Thrust delay | $L_T$ | 25.2 ms | [thrust](../experiments/thrust_identification/results.md) |
| Coulomb friction | $\mu_c$ | ≈ 1.0 N (least-settled; see [§4.3](#43-friction)) | [friction](../experiments/friction_identification/results.md) |
| Viscous friction | $b$ | ≈ 2.3 N·s/m (provisional) | [friction](../experiments/friction_identification/results.md) |
| Encoder constant | $C_m$ | 64,810.4 counts/m | [encoder](../experiments/encoder_calibration/results.md) |
| Gravity / weight | $W = mg$ | 4.45 N | derived |

---

## 4. Subsystem models

### 4.1 Servo (thrust-vector angle)

First-order-plus-delay, identified by PRPS frequency-domain fitting and validated against
held-out data (worst-case ≤ 0.5 dB magnitude, ≤ 2.6° phase):

$$
G_\theta(s) = \frac{\theta(s)}{u_\theta(s)} = \frac{0.001556}{1 + 0.0244\,s}\,e^{-0.0288\,s} \quad [\text{rad}/\mu\text{s}]
$$

Static command-to-angle map (for command ↔ angle conversion):

$$
\theta_\text{deg} = -0.091092\,(u_\theta - 1431) \quad\Longleftrightarrow\quad u_\theta = 1431 - \frac{\theta_\text{deg}}{0.091092}
$$

| Property | Value | Note |
|---|---|---|
| Neutral command | 1431 µs | zero angle |
| Usable range | 450–2450 µs | ≈ ±90° vectoring |
| Static gain | −0.00159 rad/µs | agrees with dynamic $K_\theta$ within 2% |
| Hysteresis | ~2° mean, ~4° max | do not over-interpret sub-4° angle differences |
| Phase lag at 1 Hz | ~19° | combined $\tau_\theta + L_\theta = 53$ ms |

The 28.8 ms transport delay is the binding constraint: it cannot be loop-shaped away and
sets a hard ceiling on closed-loop bandwidth (see [§6](#6-control-design-implications)).

### 4.2 Thrust (ESC → force)

**Dynamic** — global first-order-plus-delay over the full usable range (1100–1950 µs):

$$
G_T(s) = \frac{\Delta F(s)}{u_T(s)} = \frac{0.00414}{1 + 0.0781\,s}\,e^{-0.0252\,s} \quad [\text{N}/\mu\text{s}]
$$

**Static** — degree-4 polynomial in the centered/scaled command
$\hat{u} = (u_T - 1525.0)/256.2$:

$$
F_{ss}(\hat{u}) = 1.828\times10^{-5}\,\hat{u}^4 + 0.053686\,\hat{u}^3 + 0.17600\,\hat{u}^2 + 1.06699\,\hat{u} + 1.74620 \quad [\text{N}]
$$

| Property | Value | Note |
|---|---|---|
| Dead zone | $u_T < 1075$ µs | no measurable thrust |
| Saturation | $u_T > 1950$ µs | propeller-limited plateau |
| Usable range | 1075–1950 µs → 0.23–4.17 N | |
| Two-regime gain | 0.00371 N/µs (1075–1650) vs 0.00654 N/µs (1650–1950) | 76% gain jump at ~1650 µs |
| Dominant bandwidth | 2.04 Hz | faster than the rail loop |

The static map is **nonlinear** (gain changes between two near-linear regimes). For the
crude PID, hold thrust near a fixed feedforward operating point so the local thrust gain is
well-defined and the loop closes only over the servo/position channel.

### 4.3 Friction

> **Least-settled of the three identifications.** Friction data is still in `candidate/`
> (not promoted to `accepted/`), the model selection is unsettled, and two analysis passes
> disagree. Treat the numbers here as a **provisional bound**, not a finalized parameter.
> Full status in [experiments/friction_identification/results.md](../experiments/friction_identification/results.md).

Residual-based identification (predict rail force from the servo + thrust models, attribute
the acceleration mismatch to friction):

$$
f_\text{friction}(v) = -\,b\,v \; - \; \mu_c\operatorname{sign}(v)
$$

The **latest** (multi-angle, 2026-05-29) analysis selected **viscous + Coulomb** with
$b \approx 2.3$ N·s/m, $\mu_c \approx 1.0$ N (pooled RMSE 0.44 N). An **earlier** single-angle
pass had suggested **Coulomb-only**, $\mu_c \approx 0.82$ N. Both passes agree on the two
things that matter for a robust stabilizer:

1. **Coulomb friction is large** — $\mu_c$ is order ~1 N.
2. **Directional asymmetry is real** — 29–42% depending on the dataset.

**Authority context:** at $\theta = 20°$ the maximum rail force is
$F_{\text{rail,max}} = 4.17\sin 20° = 1.43$ N against $W = 4.45$ N — a thrust-to-weight of
**0.32**. With $\mu_c \approx 1$ N, the Coulomb force alone is ~70% of the maximum available
rail force. This is a **low-authority, friction-dominated** system: the PID will need its
integrator (and likely a friction feedforward) just to hold position, and
breakaway/stiction will dominate small-signal behavior near zero velocity. Design the crude
stabilizer against the **high** friction estimate ($\mu_c \approx 1$ N, with asymmetry) so it
stays robust if friction is worse than the symmetric nominal.

---

## 5. Linearization for control design

Linearize the EOM about an operating point $(\theta^\ast, T^\ast)$ with $v^\ast \neq 0$ so
the Coulomb term is locally constant. With state $\mathbf{x} = [p, v, \theta, T]^\top$ and
the actuator commands as inputs, the Jacobian is

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & \dfrac{T^\ast}{m}\cos\theta^\ast & \dfrac{\sin\theta^\ast}{m} \\
0 & 0 & -\dfrac{1}{\tau_\theta} & 0 \\
0 & 0 & 0 & -\dfrac{1}{\tau_T}
\end{bmatrix},
\qquad
B = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
\dfrac{K_\theta}{\tau_\theta} & 0 \\
0 & \dfrac{K_T}{\tau_T}
\end{bmatrix}
$$

with input delays $L_\theta, L_T$ handled by Padé approximation. The operating-point
dependence enters through the velocity row: servo authority scales as $(T^\ast/m)\cos\theta^\ast$
and thrust-to-rail coupling as $\sin\theta^\ast/m$. Near the nominal hover-equivalent
($\theta^\ast \approx 0$), the $\sin\theta^\ast/m$ term vanishes and servo authority is
maximal — the natural design point for the first stabilizer.

> The earlier gain-scheduled LQR exploration (now removed) used this same family across a
> grid of $(\theta^\ast, T^\ast)$. That is **Phase 2** work. The crude stabilizer should
> use a **single** conservative linearization and lean on robustness, not scheduling.

---

## 6. Control-design implications

| Constraint | Value | Consequence |
|---|---|---|
| Servo combined lag $\tau_\theta + L_\theta$ | 53 ms | hard bandwidth ceiling |
| Servo phase lag at 1 Hz | ~19° | leaves ~26° margin at a 45° PM target |
| Recommended initial rail bandwidth | **≤ 1 Hz** | conservative; do not target > 2 Hz |
| Thrust lag at ≤ 0.3 Hz loop | < 3° phase | thrust ≈ static gain + delay at low BW |
| Thrust-to-weight at 20° | 0.32 | low authority; saturation reachable |
| Friction / max rail force | $\mu_c / 1.43 \approx 0.7$ | dominant; needs integral action + likely FF |

**Design target for the crude stabilizer:** a robust hand-tuned PID commanding servo angle
to regulate position, thrust held at a fixed feedforward operating point. Bandwidth
**0.1–0.3 Hz** (well inside the servo phase-lag budget), generous phase margin, with the
explicit goal of being *un-fussy and hard to destabilize* across model error — not
performant. Its only job is to make the rail safe to run in closed loop so we can collect
closed-loop ID data.

---

## Why this is "rough" and not "earned"

- Every model here was identified **open-loop**, one subsystem at a time. None captures the
  **coupled** closed-loop behavior (servo–thrust–cart interaction under feedback).
- Friction is the **least-settled** subsystem: two passes disagree on structure
  (Coulomb-only vs viscous+Coulomb) and magnitude ($\mu_c \approx 0.82$–1.0 N), data is still
  in `candidate/`, and a ~30–42% directional asymmetry is collapsed to a symmetric nominal.
  Stiction/breakaway near $v = 0$ is bracketed, not modelled.
- The thrust static map is **nonlinear** and voltage-dependent; a single global gain hides
  a 76% regime-to-regime gain change and battery-sag drift.
- Servo dynamics are **mildly amplitude-dependent** ($\tau_\theta$ grows from 15 ms to 34 ms
  with excursion); the global model pools this away.

These are exactly the discrepancies closed-loop ID is meant to surface and the refined
"earned" model is meant to capture. **Do not trust this model past the point of getting a
safe stabilizer onto the rail.**
