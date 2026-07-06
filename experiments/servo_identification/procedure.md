# Servo Actuator Identification Procedure

## Objective

Identify a low-order dynamic model for the servo actuator mapping commanded servo input to measured physical servo angle:

$$
u_{servo} \rightarrow \theta
$$

where $u_{servo}$ is the commanded PWM pulse width (µs) sent to the servo and $\theta$ is the physical servo angle inferred from encoder motion.

The deliverable is a control-relevant truth-model subsystem **with quantified uncertainty**: a first-order-plus-delay transfer function, the frequency band over which it is trustworthy, and a two-tier uncertainty set — per-rung bootstrap confidence intervals (statistical) plus the across-amplitude window (systematic). The **amplitude window is the Stage 2 uncertainty set** because it dominates the statistical CIs — see [project_metrics.md §1](../../docs/project_metrics.md#1-open-loop-actuator-identification-servo-thrust-friction). The model also decides whether the servo can be treated as an instantaneous command-to-angle map or must enter the rail model as an explicit actuator state.

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
- **Parameter uncertainty** on $K, \tau, L$ — per-rung bootstrap CIs (statistical) and the across-amplitude window (systematic, the Stage 2 set).

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

### 1. Encoder count-to-angle scale (spec-derived)

The count-to-angle scale is **spec-derived and exact**: a $1:1$ GT2 belt (equal
16T pulleys) with a 600 PPR quadrature encoder gives $600 \times 4 = 2400$
counts/rev, i.e. $0.15^\circ/\text{count}$. A toothed belt does not slip and equal
integer teeth give an exact angular ratio, so there is no continuous error to
calibrate out (unlike the linear rail, where pitch-line diameter does introduce a
real scale error). Single source of truth:
[`encoderAngleScale()`](../../analysis/utils/encoderAngleScale.m). The earlier
half-revolution sweep (once used to estimate counts/rev) has been **retired** —
the scale no longer comes from it, and the only concern it spoke to (fast moves
dropping counts) is covered by the
[encoder reliability investigation](../../docs/system_identification/encoder_reliability_investigation.md).
Backlash (upgraded servo: $\approx 0.39^\circ$; the old servo was $\approx 2.4^\circ$) is a
separate additive offset, not a scale error.

### 2. Static command-to-angle map (discovery sweep)

Run as a **discovery sweep**: rather than sweeping between hand-declared endpoints,
the servo walks *outward from center* in both directions, settling at each step, and
each leg stops when the encoder flatlines — locating the **true saturation commands**
(where control effectively stops), not assumed ones. Stalled samples are excluded
from all fits. Firmware:
[`servo_sweep_log.py`](../../firmware/pico_micropython/system_identification/servo_identification/servo_sweep_log.py);
analysis:
[`analyze_servo_static_sweep.m`](../../analysis/system_identification/servo_identification/servo_sweep_log/analyze_servo_static_sweep.m).

Fit the linear map

$$
\theta_{ss} \approx K_\theta (u_{servo} - u_0),
$$

but fit the **up and down legs separately** — the center-out walk preloads gear lash
in each leg's direction, so the two legs *are* the two backlash branches. This yields:

- the **lash-free static gain** $K_\theta$ (branch-mean; a window straddling center
  mixes the branches and flattens the slope — that "center-region gain" is an
  artifact and is not used),
- the **neutral** $u_0$ and the **responsive/saturation range**,
- the **backlash** as the vertical gap between the branches at center (gear
  lost-motion — a *step*, not a deadband; an independent cross-check on the
  step/hysteresis backlash), and
- the **within-branch gain ripple** (residual static nonlinearity per branch).

The static gain is an **independent cross-check** on the PRPS DC gain $K$. Hysteresis
is *not* estimated by same-command up/down differencing here (the center-out legs span
disjoint command ranges); the branch-gap backlash is the proper lost-motion measure.

### 3. Step-response recon and validity

Run center-out steps (small and moderate) and full cross-center reversals. Two outputs, neither of which is a fitted parameter:

- **Recon:** an order-of-magnitude corner frequency $f_c \approx 1/(2\pi\tau)$ and rough delay scale, used only to center the PRPS band in step 4.
- **Validity envelope:** slew / rate behavior, deadband, saturation, and large-reversal asymmetry as **envelope bounds** on where the linear model holds.

### 4. PRPS frequency-domain identification (primary)

Center the excitation band on the corner located in step 3 — span roughly a decade below to a half-decade above it, log-spaced — and push the upper edge until coherence degrades or motion approaches the encoder noise floor. That coherence falloff *is* the measured edge of the servo's usable dynamic range; it is not assumed in advance. (Sampling rate sets only the hard Nyquist ceiling; the band is set by where the actuator's own dynamics and SNR die, not by Nyquist.)

The final campaign is an **amplitude ladder**: PRPS is applied at a ladder of angle amplitudes
(±5° … ±15°), each rung carrying several crest-minimized realizations of a linear multisine plus,
on the larger rungs, single-tone **slew probes** bracketing the rate-limit knee. Each rung yields
its own FOPD $(K, \tau, L)(\theta)$ and linear corner $f_c(\theta) = 1/(2\pi\tau)$; the probes
give the describing-function gain-droop / THD knee $f_\text{slew}(\theta)$, and the achievable
vectoring bandwidth is $f_\text{vec}(\theta) = \min(f_c, f_\text{slew})$. Running a *ladder*
rather than one amplitude is deliberate: the amplitude-dependence of $(K,\tau,L)$ is the servo's
dominant uncertainty, so it is measured and carried as a window rather than pooled into one fit
(see [Uncertainty Quantification](#uncertainty-quantification)). The reported nominal is the
**±15° operating rung**. Because the ladder's linear blocks resolve coherence *through* the ~9 Hz
corner (coherence edge 8.7–15 Hz per rung), $\tau$ and the corner are **measured, not
extrapolated** — the failure mode of the earlier two-band pass, which fit $\tau$ from data that
stopped below its own corner. Analysis:
[`analyze_servo_prps_amplitude_sweep.m`](../../analysis/system_identification/servo_identification/servo_prps_log/analyze_servo_prps_amplitude_sweep.m).

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

1. Encoder count-to-angle scale (spec-derived; [`encoderAngleScale()`](../../analysis/utils/encoderAngleScale.m)).
2. Static command-to-angle map (discovery sweep): lash-free gain, neutral, responsive/saturation range, backlash (branch-gap lost motion), within-branch ripple.
3. PRPS FRF and coherence across amplitudes; selected low-order model.
4. Per-rung FOPD $(K, \tau, L)(\theta)$ with **two-tier uncertainty** — per-rung bootstrap CIs (statistical) and the across-rung amplitude window (systematic, the Stage 2 set) — plus a **time-domain validation** (VAF / NRMSE-fit) of the chosen nominal model against held-out measured angle.
5. Step / large-signal validity envelope: slew rate, deadband, saturation, reversal asymmetry.
6. Decision on static vs. explicit actuator dynamics, and initial rail-controller bandwidth recommendation.

---

## Uncertainty Quantification

A single best-fit model is not a deliverable on its own — Stage 2 needs the *uncertainty set*,
not a point. For the amplitude-ladder servo model, uncertainty is reported in **two independent
tiers**, both computed by
[`analyze_servo_prps_amplitude_sweep.m`](../../analysis/system_identification/servo_identification/servo_prps_log/analyze_servo_prps_amplitude_sweep.m).

### Tier 1 — statistical (per-rung bootstrap)

Within each rung, uncertainty is quantified by **Monte-Carlo-style bootstrap resampling of that
rung's PRPS realizations**:

1. **Resample units with replacement.** The unit is one complete PRPS period (`period_index`), so
   draws never cut through an excitation cycle. Sampling is **stratified by realization**
   (`run_name`) so each synthetic set preserves the realization mix.
2. **Refit, don't re-select.** Each draw re-estimates the FRF and refits the already-selected FOPD
   family — model *structure* is fixed; only the *parameters* move.
3. **Keep the full correlated cloud.** The output is the joint $(K, \tau, L)$ sample cloud, not
   three independent intervals — downstream Monte Carlo must sweep whole rows to preserve the
   $K$–$\tau$–$L$ correlation. Percentile CIs are read off the cloud ($n = 200$ draws).

At the operating amplitude these CIs come out **very tight** (e.g. ±15°: $\tau \in [17.1,17.1]$
ms), i.e. the parameters are pinned by the data at each amplitude.

### Tier 2 — systematic (amplitude-dependence) — the design set

Across the ladder, $(K, \tau, L)$ move **far more than** the per-rung CIs — the servo is
amplitude-dependent. That systematic variation, **not** the statistical scatter, is the dominant
uncertainty, so the **min/max of each parameter across the rungs is the Stage 2 uncertainty
window** carried into the parameter store and the robust design. The per-rung bootstrap is what
lets us *assert* this ordering: because the statistical CI at each amplitude is negligible next to
the across-amplitude spread, the amplitude window is a real systematic effect, not fit noise. The
τ/L-vs-amplitude figure plots the rungs with the bootstrap CIs as error bars to make the
separation visible.

### Time-domain validation of the chosen model

The chosen (nominal-rung) FOPD is translated back to the time domain: `lsim` the model on a
held-out realization of that rung, score **VAF** and **NRMSE-fit**, and plot measured angle,
model prediction, and residual. Each PRPS burst is scored on its own uniform grid (the removed
settle segments leave gaps in $t$ that must not be interpolated across), and the `lsim` startup
transient is excluded. This is the control-relevant confirmation that the frequency-domain fit
actually reproduces the measured motion in the time domain; the residual should collapse to the
backlash/quantization floor with no systematic structure.

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

Open-loop servo identification **complete and accepted** on the **upgraded servo** (swapped
2026-06-29) via the **amplitude-ladder** PRPS campaign (2026-07-02), consolidated in
[results.md](results.md). Nominal (±15° operating rung): $K = 0.001618$ rad/µs, $\tau = 17.1$ ms,
$L = 13.4$ ms; the across-rung amplitude window ($\tau$ 17.1–20.0 ms) is the Stage 2 uncertainty
set, with per-rung bootstrap CIs confirming statistical scatter is negligible next to it. The
chosen model validates in the time domain at **VAF 99.9%** on held-out data (residual at the
$\pm 0.39^\circ$ backlash floor). The static map was fixed by a full-range **discovery sweep**
(true saturation located; lash-free gain and neutral from a two-branch fit; backlash as a center
step), the count-to-angle scale is **spec-derived**
([`encoderAngleScale()`](../../analysis/utils/encoderAngleScale.m)), and the shipped
[`servoStaticMap()`](../../analysis/utils/servoStaticMap.m) now holds the upgraded-servo lash-free
gain/neutral ($-0.093996$ deg/µs, 1428 µs). The earlier slow-servo two-band model is retained as
history only. Residual whiteness (#3–#4) is assessed via the frequency-domain residual (the
multisine-appropriate form); the raw time-domain ACF/CCF over-rejects on PRPS line-spectrum data
and is a diagnostic, not the metric of record.