# Friction Identification Procedure

## Objective

Identify a low-order friction model for the rail vehicle that captures the dominant resistive force opposing cart motion:

$$
F_{friction}(\dot{x}) \approx b\dot{x} + \mu\,\mathrm{sign}(\dot{x})
$$

where $b$ is a viscous drag coefficient (N·s/m) and $\mu$ is a Coulomb (constant-magnitude) friction force (N).

The approach is **residual-based**: during open-loop rail tests with known commands, the net rail-direction force is predicted using the identified servo and thrust dynamic models. Friction is inferred as the mismatch between that predicted force and the acceleration observed from the encoder:

$$
F_{friction}(t) = F_{thrust}(t)\sin\!\bigl(\theta(t)\bigr) - M\,\ddot{x}_{measured}(t)
$$

Stiction is identified from the same campaign: the lowest ESC command level that produces motion sets an upper bound on the breakaway force, and the highest command that produces no motion sets a lower bound.

---

## Prerequisites

| Prerequisite | Parameters | Reference |
|---|---|---|
| Servo dynamic model | $K_{servo} = 0.001556$ rad/µs, $\tau_{servo} = 24.4$ ms, $L_{servo} = 28.8$ ms, $u_{neutral} = 1431$ µs | [`experiments/servo_identification/results.md`](../servo_identification/results.md) |
| Thrust dynamic model | $K_T = 0.00414$ N/µs, $\tau_T = 78.1$ ms, $L_T = 25.2$ ms, dead zone $u < 1075$ µs | [`experiments/thrust_identification/results.md`](../thrust_identification/results.md) |
| Encoder calibration | $C_m = 64{,}810.4$ counts/m (8.02% above nominal; do **not** use nominal 60,000) | [`experiments/encoder_calibration/results.md`](../encoder_calibration/results.md) |
| Vehicle mass | $M = 0.4536$ kg (1 lb nominal), $\sigma_M = 0.136$ kg (±0.3 lb) | Weigh before each test campaign |

### Servo model — plugged-in form

$$
G_{servo}(s) = \frac{0.001556}{1 + 0.0244\,s}\,e^{-0.0288\,s} \quad [\text{rad/µs}]
$$

Static relationship (for angle-to-command conversion):

$$
\theta_{deg} = -0.091092\,(u - 1431) \implies u = 1431 - \frac{\theta_{deg}}{0.091092}
$$

Servo commands for the ±20° test angle:

| Angle | Servo command |
|---:|---:|
| +20° | ≈ 1211 µs |
| −20° | ≈ 1651 µs |

### Thrust model — plugged-in form

$$
G_{thrust}(s) = \frac{0.00414}{1 + 0.0781\,s}\,e^{-0.0252\,s} \quad [\text{N/µs}]
$$

- Motor dead zone: $u < 1075$ µs → $F_{thrust} \approx 0$
- Static range: 0.23–4.17 N over 1075–1950 µs

---

## Equation of Motion

$$
M\ddot{x} = F_{thrust}(t)\sin\!\bigl(\theta(t)\bigr) - F_{friction}(\dot{x})
$$

Rearranging for the residual:

$$
F_{friction}(t) = F_{thrust}(t)\cdot\sin\!\bigl(\theta(t)\bigr) - M\,\hat{\ddot{x}}(t)
$$

Everything except $F_{friction}$ is either measured (encoder position) or predicted from known dynamic models, making friction the residual.

---

## Vehicle Mass

Nominal mass: **M = 0.4536 kg (1.00 lb)**, uncertainty σ = 0.136 kg (±0.3 lb).

- Weight: $W = Mg = 4.45$ N (range 3.11–5.78 N over ±0.3 lb)
- Weigh the complete assembly — cart, motor, ESC, servo, propeller, battery, all mount hardware — on a calibrated scale before each test campaign.
- Mass uncertainty propagates directly into $b$ and $\mu$: a ±30% mass range produces ±30% coefficient range. Weigh to ±5% or better to keep coefficient uncertainty meaningful.

---

## Physics Analysis — Thrust Authority at 20°

At the test angle of 20°: $\sin(20°) = 0.342$.

| ESC (µs) | $F_{thrust}$ (N) | $F_{rail} = F_{thrust}\sin 20°$ (N) | $a$ if no friction (m/s²) |
|---:|---:|---:|---:|
| 1100 | 0.09 | 0.03 | 0.07 |
| 1150 | 0.28 | 0.10 | 0.21 |
| 1200 | 0.46 | 0.16 | 0.35 |
| 1250 | 0.65 | 0.22 | 0.49 |
| 1300 | 0.84 | 0.29 | 0.63 |
| 1400 | 1.21 | 0.41 | 0.91 |
| 1500 | 1.58 | 0.54 | 1.19 |
| 1600 | 1.95 | 0.67 | 1.47 |

*(Thrust values from static thrust map at representative ESC levels; actual force from dynamic model during analysis.)*

**Maximum rail force (at 1950 µs):** $F_{rail,max} = 4.17 \times 0.342 = 1.43$ N

**Maximum rail acceleration (zero friction):** $a_{max} = 1.43 / 0.4536 = 3.15$ m/s² ≈ 0.32 g

**Thrust-to-weight ratio at 20°:** $F_{rail,max} / W = 1.43 / 4.45 = 0.32$

This is a low-authority system — the motor produces at most 0.32× the vehicle weight as rail force at 20°. Friction must be modest for the cart to move at intermediate thrust levels.

**Stiction threshold estimate:** If Coulomb friction is in the 0.3–0.7 N range typical for belt-and-rail systems, the breakaway ESC level is approximately 1300–1500 µs. Runs below this level are expected to produce no motion (stiction halts) and directly bracket the breakaway force.

---

## Test Design

### Overview

The friction and stiction campaign is a single orchestrated sweep: the servo is fixed at a given angle (±20°), the cart is placed at the appropriate end stop, a constant ESC command is applied, and the run continues until the cart reaches the far end stop or remains motionless for 10 seconds. This is repeated at 8 ESC command levels in each direction for 16 total runs.

### End-stop placement rule

| Servo angle | Cart starting position | Reason |
|---:|---|---|
| +20° (→ 1211 µs) | Low end stop (minimum encoder count) | Thrust pushes cart toward high count |
| −20° (→ 1651 µs) | High end stop (maximum encoder count) | Thrust pushes cart toward low count |

The firmware prompts the user with the correct end stop before each run.

### ESC sweep levels

8 levels per direction: 1100, 1150, 1200, 1250, 1300, 1400, 1500, 1600 µs. Fine steps at the low end (1100–1250, step 50) where the stiction boundary is expected. Coarser steps above 1250 µs.

### Run phases

| Phase | ESC | Servo | Duration |
|---|---|---|---|
| `user_wait` | safe (1000 µs) | neutral | 10 s user positioning window |
| `servo_settle` | safe (1000 µs) | target angle | 1 s settle time |
| `run` | ESC command | target angle | Until halt or 90 s timeout |
| `stiction_halt` | safe (1000 µs) | target angle | Final samples when stiction halt declared |
| `endstop_halt` | safe (1000 µs) | target angle | Final samples when far-end halt declared |
| `timeout_end` | safe (1000 µs) | target angle | Samples after 90 s hard stop |

### Halt detection

During the `run` phase, a ring buffer of the last 500 encoder samples (10 s at 50 Hz) is maintained. When the range (max − min) of that buffer falls below 3 counts, motion has stopped:

- If total displacement from run start is < 15 counts → classified as **stiction_halt** (never overcame static friction)
- Otherwise → classified as **endstop_halt** (reached far end stop)

### Relevant files

| Item | Path |
|---|---|
| Firmware script | [`firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.py`](../../firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.py) |
| Orchestration config | [`firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.orchestrate.json`](../../firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.orchestrate.json) |
| MATLAB analysis | [`analysis/system_identification/friction_identification/friction_sweep_log/analyze_friction_sweep.m`](../../analysis/system_identification/friction_identification/friction_sweep_log/analyze_friction_sweep.m) |

### Running the campaign

```sh
# Full 16-segment campaign
python tools/run_pico_and_pull.py firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.py

# Single run without orchestration (for verification)
python tools/run_pico_and_pull.py firmware/pico_micropython/system_identification/friction_identification/friction_sweep_log.py --no-orchestrate
```

Data lands in `data/raw/system_identification/friction_identification/friction_sweep_log/candidate/`.

---

## Data Processing

### Step 1 — Convert encoder counts to position

$$
x(t) = \frac{N_{enc}(t)}{C_m}, \quad C_m = 64{,}810.4 \text{ counts/m}
$$

Do not use the nominal 60,000 counts/m; the 8% error accumulates to ~25 mm over the full rail.

### Step 2 — Filter and differentiate

Apply a Savitzky-Golay filter (order 3, window 0.2 s) to the position series before differentiating:

$$
\hat{\dot{x}}(t) = \frac{d}{dt}\bigl[x_{filtered}(t)\bigr], \qquad
\hat{\ddot{x}}(t) = \frac{d}{dt}\bigl[\hat{\dot{x}}(t)\bigr]
$$

Widen the window (up to 0.5 s) if the acceleration residual is noise-dominated. Trim the first and last 0.5 s of each run and exclude samples where $|\hat{\dot{x}}| < 0.01$ m/s (near-zero velocity gives poor friction estimates).

### Step 3 — Predict servo angle via lsim

```matlab
[n_pad, d_pad] = pade(L_servo, 3);
G_servo = tf(K_servo, [tau_servo, 1]) * tf(n_pad, d_pad);
theta_rad = lsim(G_servo, servo_us_col - u_neutral, t_s);
```

### Step 4 — Predict thrust via lsim

```matlab
[n_pad, d_pad] = pade(L_T, 3);
G_thrust = tf(K_T, [tau_T, 1]) * tf(n_pad, d_pad);
u_T_col  = max(0, esc_us_col - u_T_min);   % zero below dead zone
F_thrust = max(0, lsim(G_thrust, u_T_col, t_s));
```

### Step 5 — Compute friction residual

$$
\hat{F}_{friction}(t) = \hat{F}_{thrust}(t)\cdot\sin\!\bigl(\hat{\theta}(t)\bigr) - M\,\hat{\ddot{x}}(t)
$$

Only use `run`-phase samples from runs that produced motion (i.e., not `stiction_halt` runs).

### Step 6 — Stiction boundary

For each direction, identify:
- $u_{stiction,max}$: highest ESC level whose run was classified `stiction_halt`
- $u_{motion,min}$: lowest ESC level whose run produced motion

The breakaway rail force is bounded between the static thrust predictions at these two command levels:

$$
F_{stiction} \approx F_{thrust}(u_{motion,min}) \cdot \sin(20°)
$$

---

## Friction Model Candidates

Let $v = \hat{\dot{x}}$ and $\tilde{F} = \hat{F}_{friction}$.

### Viscous only

$$
\tilde{F} = b\,v \quad \text{(1 parameter)}
$$

### Coulomb only

$$
\tilde{F} = \mu\,\mathrm{sign}(v) \quad \text{(1 parameter)}
$$

### Viscous + Coulomb

$$
\tilde{F} = b\,v + \mu\,\mathrm{sign}(v) \quad \text{(2 parameters, linear regression)}
$$

Fit by OLS: $[v, \mathrm{sign}(v)] \backslash \tilde{F}$.

---

## Model Selection

Fit all three candidates on the pooled $(v, \tilde{F})$ data from all motion runs. Apply a **10% simplicity tolerance**: accept the viscous+Coulomb model only if its RMSE is more than 10% lower than the best single-parameter model; otherwise prefer the simpler model.

```
if RMSE(viscous+coulomb) < 0.90 × min(RMSE(viscous), RMSE(coulomb)):
    selected = viscous+coulomb
elif RMSE(viscous) ≤ RMSE(coulomb):
    selected = viscous
else:
    selected = coulomb
```

Check that $b \geq 0$ and $\mu \geq 0$ for physical plausibility. Report RMSE, coefficient values, and 95% confidence intervals from the mass uncertainty sweep (M ± σ_M).

---

## Key Design Question

If $\mu / F_{rail,max}$ is large (> 0.1 relative to the 1.43 N maximum rail force), friction is significant enough to warrant explicit feed-forward compensation in the controller. If friction is small relative to available authority, the integrator in a PID controller will reject it adequately.

---

## Expected Output

1. 16-run raw CSV dataset in `data/raw/system_identification/friction_identification/friction_sweep_log/candidate/`
2. Stiction boundary: ESC range that brackets breakaway force, and implied stiction force bounds
3. $\hat{F}_{friction}$ vs $\hat{\dot{x}}$ scatter plot across all motion runs
4. Fitted model coefficients: $b$ (N·s/m), $\mu$ (N) with mass-uncertainty bands
5. Model selection decision (viscous / Coulomb / viscous+Coulomb)
6. Decision on whether explicit friction compensation is needed for the rail controller
7. Results documented in `results.md`

---

## Current Status

The campaign has been **run and analyzed** (multi-angle dataset, 2026-05-29); see
[results.md](results.md) for the current best estimate and open questions. Firmware,
orchestration, and analysis scripts are complete.

Friction is **identified but not finalized** — the candidate runs have not been triaged into
`accepted/`, and the model selection (Coulomb-only vs. viscous+Coulomb) is still open. The
remaining steps to lock it in are listed in
[results.md → Open items before friction is "earned"](results.md#open-items-before-friction-is-earned):

1. Triage the 2026-05-29 candidate runs into `accepted/` / `rejected/`.
2. Settle the friction model structure and grouping.
3. Deconfound servo direction from motion direction in the asymmetry estimate.
4. Decide on a direction-dependent model vs. a robust symmetric bound for the controller.
5. Quantify stiction/breakaway explicitly.
