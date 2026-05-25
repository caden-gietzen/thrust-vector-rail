# Thrust Identification Procedure

## Objective

Identify a low-order dynamic model for the thrust actuator mapping from ESC command to measured thrust force:

$$
u_{PWM} \rightarrow F_{thrust}
$$

where:

- $u_{PWM}$ is the PWM command sent to the ESC (µs)
- $F_{thrust}$ is the net thrust force measured by the load cell (N)

The purpose of this procedure is to **identify the thrust actuator's dynamic behavior before initial rail-controller design**. Unlike the servo, the thrust actuator is nonlinear: steady-state thrust scales roughly with the square of motor speed, and motor speed is only approximately linear in PWM command. The identification must therefore characterize both the static nonlinearity and the dominant dynamic lag.

The goal is control-relevant identification. The identified model will be used to:

- set the initial rail-controller bandwidth
- determine whether thrust dynamics must be included explicitly in the control model or can be approximated as a static map
- inform PD/PID gain selection via simple pole placement for an initial stabilizing controller, before moving to LQR

The final model should be simple enough to support initial controller design while still capturing the dominant thrust behavior that constrains closed-loop performance.

---

## Control-Relevant Identification Philosophy

The thrust identification procedure follows the same general framework used for the servo:

1. Begin with a static map.
2. Use broadband excitation to characterize dominant dynamics.
3. Refine the model over the frequency range that will constrain closed-loop performance.

The question of which broadband excitation signal to use is important, and the answer differs from the servo case. For the servo, the initial plan included step response testing before moving to PRPS. For thrust, PRPS is used directly without a step response step.

### Why PRPS is preferred from the start:

PRPS (pseudo-random phase-sum / periodic multisine) excites a user-selected set of sinusoidal frequencies simultaneously, with controlled amplitude and random phases. This gives several advantages over PRBS for thrust identification:

- **Controlled frequency content**: excitation can be concentrated in the 0.05–3 Hz range where rail-control authority is most critical.
- **Controlled amplitude**: the signal is normalized to a peak amplitude around a chosen center command, so the test stays within a selected PWM operating region.
- **Periodicity**: the signal repeats with a known period, allowing coherent averaging across multiple periods to reduce noise and improve FRF estimates.
- **Crest factor**: because the signal is a sum of sinusoids with randomized phases, the peak-to-RMS ratio is lower than PRBS, which reduces the risk of driving into ESC hard limits during testing.

This is a deliberate choice, not an oversight. The firmware (`thrust_prps_daq_voltage.py`) is written specifically for PRPS excitation and reflects this decision.

---

## Hardware Setup

The thrust identification setup uses:

- motor and propeller
- ESC (electronic speed controller) driven from Raspberry Pi Pico PWM output (GP13, GP14)
- HX711 load cell amplifier with attached load cell (GP20 DAT, GP21 SCK)
- Pixhawk TELEM2 MAVLink output connected to Pico UART0 (GP0 TX, GP1 RX) for battery voltage and current telemetry
- LiPo battery with known capacity

ESC commands are applied through 50 Hz PWM. The resulting thrust force is measured by the load cell and converted from HX711 raw counts to Newtons using a calibrated scale factor.

Battery voltage and current are logged at every sample via MAVLink. This is essential because motor speed — and therefore thrust — varies with battery voltage under load. All analysis should inspect battery voltage trends across each run to detect sag that may bias the FRF estimate.

---

## Load Cell Calibration

The load cell output must be converted from HX711 raw counts to Newtons.

The scale factor `SCALE_G_PER_COUNT` is already configured in the firmware. The procedure should verify this calibration before collecting identification data:

1. With the motor off and no additional load, confirm that the tared reading is approximately zero.
2. Apply a known reference mass to the load cell. Record the tared count.
3. Compute the measured `SCALE_G_PER_COUNT` as `known_mass_g / tared_count`.
4. Compare to the firmware value and update if the deviation exceeds ±2%.

The load cell calibration should also be checked for:

- repeatability over multiple tare readings
- linearity across the expected force range
- direction: confirm that the `FORCE_SIGN` convention in the firmware matches the physical mounting direction (thrust away from rail produces a positive force reading)

---

## Static Thrust Map

The first identification step is a **static map from PWM command to steady-state thrust force**.

With the motor at thermal equilibrium (after a brief warm-up run), apply a sequence of fixed PWM commands across the operating range. Allow the motor to settle at each command level before recording the mean load cell reading.

The static map identifies:

- minimum command to produce measurable thrust (motor start threshold)
- full usable PWM range: `[PWM_HARD_MIN_US, PWM_HARD_MAX_US]` = [1100, 1950] µs
- static thrust-to-command relationship (expected: roughly quadratic in motor speed, approximately quadratic-to-cubic in PWM offset from minimum)
- any dead zones near the low end of the command range
- asymmetry or hysteresis between increasing and decreasing command sweeps

The static map can be written as:

$$
F_{ss} = f(u_{PWM})
$$

For controller design, a linearized approximation around a nominal operating point $u_0$ is used:

$$
F_{ss} \approx K_T (u_{PWM} - u_0)
$$

where:

- $F_{ss}$ is the steady-state thrust force
- $K_T$ is the static thrust-gain at the operating point (N/µs)
- $u_0$ is the center command for the operating point

$K_T$ will vary significantly across the PWM range because of the underlying nonlinearity. The value relevant for controller design is $K_T$ evaluated at the nominal hover or operating point, not the globally averaged gain.

This static map defines the operating point and amplitude bounds for all subsequent dynamic tests.

---

## Justification for Skipping Step-Response Tests

For the servo identification, step-response tests preceded PRPS testing. The step tests were used to estimate rough time constants and delay before designing the PRPS frequency plan.

For thrust identification, step-response tests are not used as an intermediate stage. The reasons are:

1. **Step responses for motors are highly nonlinear.** A step command produces a transient that depends strongly on the initial motor speed, battery voltage at that instant, and propeller load. The response shape changes significantly across the operating range. A single step test gives a time constant that is difficult to generalize to other operating conditions.

2. **The PRPS signal itself provides delay and time-constant information.** The low-frequency bins of the PRPS FRF estimate encode the steady-state gain and low-frequency phase behavior. The phase roll-off and magnitude slope across the excited frequency range directly reveal the dominant lag and effective delay. No separate step test is needed to initialize the model candidate.

The static map provides the operating-point information that step responses would otherwise supply. PRPS provides the frequency-domain characterization needed for model fitting.

---

## PRPS Frequency-Domain Identification

### Firmware Configuration

The firmware `thrust_prps_daq_voltage.py` is organized around `TEST_RUNS`, each defined by a center PWM command and a peak-to-center amplitude:

```python
"TEST_RUNS": [
    ("local_1100_1350", 1225, 125),
]
```

This means: center command = 1225 µs, amplitude = ±125 µs, so the command sweeps between approximately 1100 and 1350 µs.

Key parameters to set before each test campaign:

| Parameter | Description | Typical value |
|---|---|---|
| `CENTER_PWM` | Operating point command (µs) | Chosen from static map |
| `AMPLITUDE_PWM` | Peak excitation amplitude (µs) | 10–25% of usable range |
| `AUTO_FREQ_MIN_HZ` | Lowest excited frequency | 0.05 Hz |
| `AUTO_FREQ_MAX_HZ` | Highest excited frequency | 3.0 Hz |
| `AUTO_NUM_FREQS` | Number of frequency bins | 20–25 |
| `PRPS_PERIOD_S` | Period of one PRPS cycle | 40 s |
| `NUM_PERIODS_PER_RUN` | Periods per acquisition | 3 |
| `COMMAND_UPDATE_DT_MS` | Sample interval | 20 ms |

The PRPS period controls frequency resolution: `f0 = 1 / PRPS_PERIOD_S`. All excited frequencies are snapped to integer multiples of `f0`. A 40 s period gives `f0 = 0.025 Hz`, providing adequate resolution across the 0.05–3 Hz range.

Three periods per run allows coherent averaging to reduce variance in the FRF estimate. The orchestrator can run multiple acquisition sets with different random phase realizations (different PRPS seeds) to further improve statistical quality.

### Excitation Range Selection

The PRPS frequency range should be designed to span from the quasi-static regime through the region where motor dynamics cause meaningful phase lag and magnitude attenuation.

Initial guideline: excite from **0.05 Hz to 3 Hz** with approximately **25 log-spaced bins**.

Upper-end guidance: extend the excitation until one or more of these conditions is observed:

- measured thrust magnitude drops by approximately 3 dB relative to the low-frequency gain
- phase lag approaches approximately 45° or more
- coherence function drops below approximately 0.8–0.9
- the thrust signal becomes small relative to load cell noise

These are not hard cutoffs. They mark the region where the motor begins to lose useful dynamic authority for rail control.

### Operating Point Selection

The static map determines which center command to use. For a given vehicle weight and desired hover thrust, the operating point is the PWM value that produces approximately half the maximum thrust. PRPS identification should be performed at this operating point because $K_T$ and the dynamic lag both vary with motor speed.

If the static map reveals that the thrust-command relationship is strongly nonlinear, consider running PRPS tests at two or three operating points to characterize how $K_T$ and the lag change with operating condition. For an initial stabilizing controller, a single operating-point identification is sufficient.

### Battery Voltage Monitoring

Battery voltage must be logged and inspected for every run. Motor thrust depends on battery voltage: at a fixed PWM command, thrust decreases as the battery sags under load. This introduces a slowly-varying drift in the measured thrust that biases the FRF estimate at low frequencies.

Checks after each run:

- Inspect `battery_voltage_V` across the run. A drop of more than 0.3–0.5 V over a single 2-minute run is a warning sign of excessive voltage sag.
- Inspect `battery_remaining_pct` at start and end of each acquisition set.
- If voltage sag is large, detrend the thrust signal before computing the FRF, or discard the run and recharge.

### Data Segments

Each PRPS run produces the following labeled segments in the CSV:

| Segment label | Description |
|---|---|
| `baseline_pre` | Motor at center PWM, no excitation; establishes pre-run thrust baseline |
| `settle` | Brief settle at center before PRPS begins |
| `prps` | Active PRPS excitation (identified by `phase == "prps"`) |
| `baseline_post` | Motor returns to center PWM after PRPS ends |

The `prps` segment is used for FRF estimation. The baseline segments are used to check for thermal drift and tare consistency.

---

## Model Candidates

Candidate models should be kept intentionally low order. The identification maps from PWM command deviation to thrust force deviation at the operating point.

Let $\tilde{u} = u_{PWM} - u_0$ and $\tilde{F} = F_{thrust} - F_0$ denote deviations from the operating point.

### Static Gain Only

If thrust dynamics are fast relative to the rail control bandwidth:

$$
\tilde{F}(t) \approx K_T \tilde{u}(t)
$$

This model is acceptable only if motor lag and delay are negligible compared to the desired rail control bandwidth.

### First-Order Model

If there is a dominant lag but delay is small:

$$
\frac{\tilde{F}(s)}{\tilde{u}(s)} = \frac{K_T}{\tau_T s + 1}
$$

where $\tau_T$ is the dominant motor/aerodynamic time constant.

### First-Order Plus Delay Model

If there is both dominant lag and meaningful delay from ESC processing, motor ramp-up, or sampling effects:

$$
\frac{\tilde{F}(s)}{\tilde{u}(s)} = \frac{K_T e^{-L_T s}}{\tau_T s + 1}
$$

where $L_T$ is the effective delay.

This is the expected model structure based on known brushless motor dynamics. The delay term absorbs ESC command processing lag and any sampling-induced delay from the 20 ms command update interval.

### Second-Order Model

If the FRF shows a resonance or behavior that cannot be captured with a first-order lag — for example, if propeller inertia and motor electrical dynamics interact noticeably at the upper end of the excitation range — a second-order model may be warranted:

$$
\frac{\tilde{F}(s)}{\tilde{u}(s)} = \frac{K_T \omega_n^2}{s^2 + 2\zeta_T \omega_n s + \omega_n^2}
$$

A second-order model should only be used if the data clearly justifies the additional complexity.

---

## Model Selection Criteria

Candidate models are compared based on:

- prediction accuracy on validation data
- ability to capture dominant lag and delay
- accuracy over the frequency range relevant to rail control (0.05–2 Hz)
- physical interpretability of parameters
- model order (prefer lower)
- usefulness for pole-placement controller design

The preferred model is the **lowest-order model that captures the dominant thrust behavior affecting rail control**.

---

## From Identified Model to Initial Controller

The identified thrust model directly informs the initial stabilizing controller. The procedure is:

### 1. Linearized rail dynamics

The rail is a second-order translational system. With thrust as the primary actuation and ignoring coupling to thrust angle (servo) for initial design:

$$
M \ddot{x} = F_{thrust}(t) - F_{friction}(t)
$$

where $M$ is the total rail vehicle mass and $F_{friction}$ is the friction force (characterized separately). In Laplace form, ignoring friction for the initial gain selection:

$$
\frac{X(s)}{F_{thrust}(s)} = \frac{1}{M s^2}
$$

### 2. Series plant model

The full open-loop plant from PWM command to rail position is:

$$
P(s) = G_{thrust}(s) \cdot \frac{1}{M s^2}
$$

where $G_{thrust}(s)$ is the identified thrust transfer function (e.g., first-order plus delay).

### 3. Pole placement for PD/PID

For an initial PD controller $C(s) = K_p + K_d s$, the closed-loop characteristic equation places poles at desired locations. With a double integrator plant, the dominant effect of proportional-derivative action is to add a zero at $s = -K_p/K_d$, providing damping.

Target pole locations should be chosen to be:

- slower than the servo bandwidth (identified: ~1 Hz; rail bandwidth should be substantially lower, ≤ 0.2–0.5 Hz initially)
- slow enough that thrust dynamics ($1/\tau_T$) can be treated as approximately instantaneous at the control frequency
- slow enough to provide reasonable gain margin above the delay $L_T$

If the identified thrust lag $\tau_T$ is much smaller than the desired closed-loop time constant, the static gain approximation $G_{thrust}(s) \approx K_T$ is acceptable for initial pole placement. If $\tau_T$ is comparable to the rail time constant, the lag must be included in the plant model used for pole placement.

The delay $L_T$ constrains the maximum achievable gain: as a rule of thumb, the closed-loop bandwidth should not exceed $1/(3 L_T)$ to $1/(5 L_T)$.

### 4. Integral action

Adding integral action (PID) eliminates steady-state position error. For pole placement with PID, place the integrator pole at a frequency at least 3–5× slower than the proportional poles to avoid winding up. With a known friction model, friction-compensating feed-forward can reduce the required integral action.

---

## Key Design Question

The main design question for thrust identification is whether thrust dynamics can be treated as an approximately static command-to-force map for the initial rail controller, or whether the motor lag and delay must be included explicitly.

If the thrust lag $\tau_T$ is small relative to the desired rail control time constant, the static approximation is acceptable for initial PD/PID design. The delay $L_T$ must always be accounted for as a gain-margin constraint, even if the lag is neglected.

If $\tau_T$ is large enough to significantly affect closed-loop performance at the desired control bandwidth, the full first-order-plus-delay model should be included in the plant used for pole placement.

This determination is made after examining the identified Bode plot and comparing $\tau_T$ and $L_T$ to the target rail control bandwidth.

---

## Expected Output

The final thrust identification results should include:

1. Verified load cell calibration (`SCALE_G_PER_COUNT`, `FORCE_SIGN`)
2. Static thrust map across the full PWM range
3. Linearized static thrust gain $K_T$ at the nominal operating point (N/µs)
4. PRPS frequency-response estimate (FRF magnitude and phase vs. frequency)
5. Coherence function across the excitation band
6. Battery voltage and current profiles for each run
7. Candidate model comparison (static, first-order, first-order-plus-delay)
8. Final selected low-order thrust model with parameters $K_T$, $\tau_T$, $L_T$
9. Decision on whether thrust dynamics must be included explicitly in the rail control plant
10. Initial guidance for PD/PID pole placement: suggested bandwidth, gain constraints from delay

---

## Current Status

Initial planning stage. Static thrust map and PRPS identification have not yet been run.

Next steps:

1. Verify load cell calibration.
2. Run static PWM sweep to produce the thrust map and identify the nominal operating point.
3. Configure `thrust_prps_daq_voltage.py` with the operating-point center command and appropriate amplitude.
4. Collect PRPS identification data (multiple acquisition sets, varying phase seeds).
5. Estimate FRF and coherence from collected data.
6. Fit and validate low-order thrust models.
7. Select the simplest model that supports initial pole-placement controller design.
8. Document identified parameters in `results.md`.
