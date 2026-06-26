# Servo Actuator Identification Procedure

## Objective

Identify a low-order dynamic model for the servo actuator mapping commanded servo input to measured physical servo angle:

$$
u_{servo} \rightarrow \theta
$$

where $u_{servo}$ is the commanded PWM pulse width (µs) sent to the servo and $\theta$ is the physical servo angle inferred from encoder motion.

The deliverable is a control-relevant truth-model subsystem **with quantified uncertainty**: a first-order-plus-delay transfer function, the frequency band over which it is trustworthy, and parameter confidence intervals. Those CIs *are* the Stage 2 uncertainty set — see [project_metrics.md §1](../../docs/project_metrics.md#1-open-loop-actuator-identification-servo-thrust-friction). The model also decides whether the servo can be treated as an instantaneous command-to-angle map or must enter the rail model as an explicit actuator state.

---

## Identification Strategy: PRPS-primary, step-as-recon-and-validity

The servo is identified with two excitation types doing **two different jobs**.

### PRPS is the identification workhorse

Targeted pseudo-random phase-sum (PRPS) multisine excitation,

$$
u(t) = u_0 + \sum_{k=1}^{N} A_k \sin(2\pi f_k t + \phi_k),
$$

is the **sole source of the model parameters $K, \tau, L$ and of every acceptance metric** in [project_metrics.md §1](../../docs/project_metrics.md#1-open-loop-actuator-identification-servo-thrust-friction). It concentrates excitation power at deliberately chosen frequencies across the band being identified, so coherence and SNR are highest exactly where the model is fit. Critically, only PRPS produces the two **gating** quantities:

- **FRF coherence $\gamma^2(\omega)$** — defines the frequency band that may be trusted and handed to Stages 2 and 4.
- **Parameter confidence intervals** on $K, \tau, L$ (via bootstrap resampling) — the Stage 2 uncertainty set.

PRBS was considered and rejected: a binary sequence spreads energy across a band with no control over which frequencies receive it, encouraging fit in regions where the actuator has already lost authority.

### Step / large-signal tests: recon and validity

The step response runs **first** and does two jobs — neither of which is estimating $K, \tau, L$.

**Recon — locate the band.** A targeted multisine cannot be designed without first knowing roughly where the dynamics live. A step gives an order-of-magnitude corner frequency $f_c \approx 1/(2\pi\tau)$ and the rough delay scale, which is all that is needed to center the PRPS experiment. A coarse $\tau$ from a rise time is fine for *pointing the experiment*; it is not fine for the reported parameter.

**Validity — bound the nonlinearities.** Step and large-reversal tests expose the effects a linear FRF structurally cannot reveal: slew / rate limiting (reversal steps slower than center-out steps), deadband and backlash near center, and large-reversal directional asymmetry. Their output is an **operating-envelope of validity** for $G(s)$ — the command-excursion and rate limits within which the linear model holds — not transfer-function parameters.

The reported $\tau, L$ never come from a step: a rise time conflates delay, lag, and slew, $t_r \approx L + \tau + (\text{slew})$, so $t_r \approx 2.2\tau$ inflates $\tau$. Step recon points the PRPS experiment; PRPS produces the numbers.

---

## Hardware Setup

Servo driven by 50 Hz PWM from the Raspberry Pi Pico (GP15); motion measured through a GT2 belt/pulley transmission by the quadrature encoder. Encoder counts are converted to angle by the calibration below.

---

## Procedure Steps

### 1. Encoder count-to-angle calibration

Establish $N_{enc} \rightarrow \theta$ experimentally rather than from nominal geometry, to absorb pulley, belt-tension, and backlash error. A half-revolution sweep with repeated up/down passes checks repeatability and counts/rev.

### 2. Static command-to-angle map

Sweep commanded PWM across the control-relevant range, letting the servo settle at each point, and fit

$$
\theta_{ss} \approx K_\theta (u_{servo} - u_0).
$$

Identifies static gain $K_\theta$, neutral command $u_0$, usable range, deadband, and up/down hysteresis. The static gain is an **independent cross-check** on the PRPS DC gain $K$.

### 3. Step-response recon and validity

Run center-out steps (small and moderate) and full cross-center reversals. Two outputs, neither of which is a fitted parameter:

- **Recon:** an order-of-magnitude corner frequency $f_c \approx 1/(2\pi\tau)$ and rough delay scale, used only to center the PRPS band in step 4.
- **Validity envelope:** slew / rate behavior, deadband, saturation, and large-reversal asymmetry as **envelope bounds** on where the linear model holds.

### 4. PRPS frequency-domain identification (primary)

Center the excitation band on the corner located in step 3 — span roughly a decade below to a half-decade above it, log-spaced — and push the upper edge until coherence degrades or motion approaches the encoder noise floor. That coherence falloff *is* the measured edge of the servo's usable dynamic range; it is not assumed in advance. (Sampling rate sets only the hard Nyquist ceiling; the band is set by where the actuator's own dynamics and SNR die, not by Nyquist.)

Apply PRPS at **multiple amplitudes** (to expose amplitude-dependent dynamics) and **multiple phase seeds** (to separate repeatability from one lucky fit), reserving held-out seeds per amplitude for validation. From each run estimate the empirical FRF and coherence; fit candidate low-order models; select the lowest-order model that captures the dominant behavior. Bootstrap the training FRFs to obtain parameter CIs.

---

## Model Candidates

Kept intentionally low order; the selected model is the simplest that captures the dominant servo behavior affecting rail control.

| Model | Form | Use when |
|---|---|---|
| Static map | $\theta \approx f(u_{servo})$ | Servo bandwidth $\gg$ rail bandwidth; lag and delay negligible |
| First-order | $\dfrac{K}{1 + \tau s}$ | Dominant lag, negligible delay |
| First-order + delay | $\dfrac{K}{1 + \tau s}\,e^{-Ls}$ | Dominant lag **and** meaningful transport delay |
| Second-order (+ delay) | $\dfrac{K\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$ | Overshoot/resonance the first-order form cannot capture — only if data clearly justifies it |

Selection criteria: validation accuracy over the usable band, ability to capture delay and lag, physical interpretability, and parsimony. A model that fits high-frequency behavior outside the usable band is not preferred over a simpler one that captures the control-relevant dynamics.

---

## Key Design Question

Can the servo be treated as an instantaneous command-to-angle map, or must its dynamics enter the rail model explicitly? If servo bandwidth $\gg$ rail bandwidth and delay is negligible, the static approximation is acceptable. Otherwise the servo enters the plant as an actuator state,

$$
\dot{\theta} = -\frac{1}{\tau}\theta + \frac{K}{\tau}\,u_{servo}(t - L).
$$

---

## Expected Output

1. Encoder count-to-angle calibration.
2. Static command-to-angle map: gain, neutral, usable range, hysteresis.
3. PRPS FRF and coherence across amplitudes; selected low-order model.
4. Parameter values **with confidence intervals** for $K, \tau, L$ (bootstrap cloud), and the derived **bandwidth confidence interval / Bode envelope**.
5. Step / large-signal validity envelope: slew rate, deadband, saturation, reversal asymmetry.
6. Decision on static vs. explicit actuator dynamics, and initial rail-controller bandwidth recommendation.

---

## Uncertainty Quantification

A single best-fit model is not a deliverable on its own — Stage 2 needs the *uncertainty set*, not a point. Uncertainty is quantified by **Monte-Carlo-style bootstrap resampling of the PRPS dataset**: repeatedly draw synthetic training sets, refit the same model family, and accumulate the resulting parameter distribution. Full mechanics, units, and outputs are in [bootstrap_uncertainty.md](bootstrap_uncertainty.md); the essentials:

1. **Resample units with replacement.** The unit is one complete PRPS period (`period_index`), so draws never cut through an excitation cycle. Sampling is **stratified by amplitude** so each synthetic set preserves the original amplitude mix.
2. **Refit, don't re-select.** Each draw refits the already-selected model family — model *structure* is fixed; only the *parameters* move.
3. **Score on fixed held-out data.** Every draw is validated against the same reserved seeds, and unphysical/failed fits are flagged and excluded from summary percentiles.
4. **Keep the full correlated cloud.** The output is the joint $(K, \tau, L)$ sample cloud, not three independent intervals — downstream Monte Carlo must sweep whole sample rows to preserve the $K$–$\tau$–$L$ correlation.

**Parameter CIs** (metric #6 below) are read directly off this cloud as percentile intervals.

**Bandwidth confidence interval / cloud.** Because actuator bandwidth is a deterministic function of the parameters, propagating each cloud sample through the bandwidth expression yields a **distribution of bandwidth estimates** — a CI on usable bandwidth rather than a single number. Each $(K, \tau, L)$ draw maps to a −3 dB magnitude bandwidth and a phase-limited (~45°) bandwidth; the spread across the cloud is reported as the bandwidth confidence interval and visualized as a **Bode envelope** (the band swept by the cloud's frequency responses).

---

## Validation and Acceptance Criteria

This section binds the procedure to [project_metrics.md §1](../../docs/project_metrics.md#1-open-loop-actuator-identification-servo-thrust-friction). The servo model is **not accepted** until every metric below is reported and met. Friction-specific rows of §1 do not apply to the servo; the servo's analogue is the amplitude-dependence check.

**Gating metric (the one that qualifies the stage):** residual whiteness and input-decorrelation on **held-out** PRPS data. A good training fit shows the model *can* fit; white, input-uncorrelated validation residuals show the dynamics are actually *captured*.

| # | Metric (§1) | Target | How it is constructed | Data required |
|---|---|---|---|---|
| 1 | Validation fit (VAF / NRMSE) | VAF $\ge 90\%$ / NRMSE-fit $\ge 80\%$ | `lsim` the selected model on each held-out PRPS seed; compare to measured $\theta$ | Held-out PRPS seeds (≥1 per amplitude), not used in fitting |
| 2 | Train–validation fit gap | $\le 5$–$10$ pts | Difference between training-file fit and held-out fit (overfit detector) | Training + held-out PRPS seeds |
| 3 | Residual autocorrelation | within 95% confidence band | ACF of the time-domain residual $e = \theta - \hat\theta$ from the `lsim` prediction on held-out data | Held-out PRPS time series |
| 4 | Residual–input cross-correlation | within 95% band | Cross-correlation of $e$ with $u_{servo}$ on held-out data → confirms model order sufficient | Held-out PRPS time series + command log |
| 5 | FRF coherence $\gamma^2$ | $\ge 0.9$ over identified band | Per-frequency coherence from the PRPS spectral estimate; **defines the trustworthy band** handed downstream | All PRPS runs (per amplitude) |
| 6 | Parameter CIs on $K, \tau, L$ | relative SE $\le 10$–$20\%$ | Bootstrap resample of training FRFs, refit the FOPD model, take the parameter cloud (preserves $K$–$\tau$–$L$ correlation) | All PRPS training files |
| 7 | Cross-seed / cross-run spread | within the CIs of #6 | Compare per-seed and per-amplitude point fits against the bootstrap CIs (repeatability vs. one lucky fit) | ≥4 seeds × multiple amplitudes |
| 8 | Amplitude-dependence (servo analogue of the friction row) | bound and document, not a point estimate | Report $\tau$ vs. amplitude trend and the cross-amplitude generalization error; fold into the model's validity envelope | PRPS at ≥3 amplitudes |

**Exit → Entry.** The bootstrap parameter cloud from metric #6 and the coherence band from metric #5 are the deliverables consumed by Stage 2. A CI too wide for a single robust stabilizer to cover the set is a **Stage 1 failure**, not a Stage 2 problem — it means the identification must be tightened (more averaging, higher SNR, narrower band), not worked around downstream.

**Out of scope.** Step / large-signal results (slew, deadband, saturation, asymmetry) are reported as validity bounds, not graded against the table above. They qualify *where* the accepted model may be used, not *whether* it is accepted.

---

## Current Status

Open-loop servo identification **complete and accepted**; all eight [project_metrics.md §1](../../docs/project_metrics.md#1-open-loop-actuator-identification-servo-thrust-friction) metrics met and consolidated in [results.md](results.md). Residual whiteness (#3–#4) is assessed via the frequency-domain residual (the multisine-appropriate form); a time-domain ACF/CCF diagnostic is available in [analyze_servo_prps_frequency_fit.m](../../analysis/system_identification/servo_identification/servo_prps_log/analyze_servo_prps_frequency_fit.m) but over-rejects on PRPS line-spectrum data and is not the metric of record.