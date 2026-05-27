# Servo Actuator Identification Procedure

## Objective

Identify a low-order dynamic model for the servo actuator mapping from commanded servo input to measured physical servo angle:

$$
\theta_{cmd} \rightarrow \theta
$$

where:

- $\theta_{cmd}$ is the commanded servo angle, or the equivalent pulse-width modulation (PWM) command sent from the Raspberry Pi Pico.
- $\theta$ is the measured physical servo angle inferred from encoder motion.

The purpose of this procedure is to **identify the servo actuator's usable dynamic range before final rail-controller design**. Since the controller bandwidth has not yet been selected, these open-loop tests are used to determine the frequency range over which the servo can reliably track commanded angle with acceptable gain, phase lag, delay, repeatability, and coherence.

This makes the identification **control-relevant without assuming the final controller bandwidth in advance**. The resulting actuator model will be used to inform the initial rail-controller bandwidth target and determine whether the servo can be treated as an instantaneous command-to-angle mapping or must be included explicitly as an actuator dynamic.

The goal is to characterize the servo behavior that matters for rail control, including:

- static command-to-angle mapping
- dominant actuator lag
- effective input/output delay
- usable actuator bandwidth
- saturation limits
- rate limits or slew-rate behavior
- repeatability and hysteresis
- whether servo dynamics must be included explicitly in the rail control model

The final model should be simple enough to support controller design while still capturing the dominant actuator behavior that constrains closed-loop performance.

---

## Control-Relevant Identification Philosophy

The servo identification procedure follows the same general structure used for [thrust identification](../thrust_identification/procedure.md):

1. Begin with a static map.
2. Use simple transient tests to understand dominant behavior.
3. Apply broadband excitation.
4. Refine the excitation method toward frequencies that are most relevant for control.

The initial idea was to use step-response tests and pseudo-random binary sequence (PRBS) excitation. However, **PRBS excitation provides limited direct control over which frequencies are excited and with what strength**.

After observing this limitation during actuator and [thrust identification](../thrust_identification/procedure.md), the procedure shifted toward explicitly exciting selected frequencies that reveal the actuator's usable dynamic range. Since these open-loop actuator tests precede final controller design, the purpose is not to assume a rail-controller bandwidth in advance. Instead, the purpose is to **identify the frequency range over which the servo can still track commanded angle reliably enough to support closed-loop control**.

This actuator bandwidth estimate can then be used to inform the initial rail-controller bandwidth target.

For that reason, the preferred dynamic identification method is not generic broadband PRBS testing. **Instead, the preferred approach is targeted frequency excitation using pseudo-random phase-sum (PRPS) signals**.

This makes the identification more useful for controller design because the model is fit and validated over the frequency range where the actuator dynamics actually affect the closed-loop rail system, rather than chasing high-frequency fit quality in regions that are not control-relevant or are already dominated by actuator lag, delay, and attenuation.

---

## Hardware Setup

The servo identification setup uses:

- servo actuator
- quadrature encoder
- Raspberry Pi Pico
- GT2 belt and pulley transmission
- mechanical linkage between servo motion and encoder measurement

Servo commands are applied through PWM from the Raspberry Pi Pico. The resulting servo motion is measured using the encoder and converted from encoder counts to physical angle.

---

## Measurement Calibration

The encoder measurement must be converted from counts to physical servo angle.

Possible conversion methods include:

- encoder counts per revolution
- measured angular span over a known motion range
- belt/pulley geometry using the GT2 belt and 20-tooth pulley
- experimental calibration between commanded servo motion and measured encoder counts

Because pulley geometry, belt tension, backlash, and mechanical coupling may introduce small errors, the preferred method is **experimental calibration rather than relying only on nominal geometry**.

The calibration procedure should establish a mapping:

$$
N_{enc} \rightarrow \theta
$$

where:

- $N_{enc}$ is encoder count displacement
- $\theta$ is physical servo angle

The resulting calibration should be checked for:

- linearity
- repeatability
- direction-dependent differences
- backlash or deadband
- usable angular range

---

## Static Command-to-Angle Identification

The first identification step is a **static map from command to measured steady-state angle**.

For a sequence of commanded PWM values or commanded angles, the servo is allowed to settle, and the final encoder-derived angle is recorded.

The static map identifies:

- command-to-angle gain
- neutral or center command
- angular saturation limits
- deadband near center
- asymmetry between positive and negative deflection
- hysteresis between increasing and decreasing command sweeps

The static mapping can be written as:

$$
\theta_{ss} = f(\theta_{cmd})
$$

or, if approximately linear over the usable range:

$$
\theta_{ss} \approx K_{\theta}(\theta_{cmd} - \theta_{0})
$$

where:

- $\theta_{ss}$ is the measured steady-state servo angle
- $\theta_{cmd}$ is the commanded servo input
- $K_{\theta}$ is the static command-to-angle gain
- $\theta_{0}$ is the command corresponding to zero or neutral angle

This static map defines the usable operating range for the dynamic tests.

---

## Step-Response Tests

Step commands are used as the first dynamic test because they provide a direct way to observe the dominant actuator behavior.

Step-response tests are used to estimate:

- effective delay
- rise time
- settling time
- dominant time constant
- overshoot
- saturation behavior
- rate limiting or slew-rate behavior

A candidate low-order model is:

$$
\frac{\theta(s)}{\theta_{cmd}(s)} =
\frac{K e^{-Ls}}{\tau s + 1}
$$

where:

- $K$ is the steady-state gain from command to angle
- $L$ is the effective delay
- $\tau$ is the dominant time constant
- $s$ is the Laplace-domain complex frequency variable

The step tests are not expected to fully define the final model by themselves. Their main purpose is to **reveal the rough actuator time scale**, expose nonlinear effects, and guide the design of later frequency-excitation tests.

Important observations from step testing should include:

- whether the servo behaves approximately like a first-order lag
- whether delay is significant relative to the desired control-loop period
- whether motion is rate-limited for large steps
- whether small steps behave differently from large steps

---

## Initial PRBS Testing

A pseudo-random binary sequence (PRBS) was initially considered for dynamic servo identification.

PRBS excitation is useful because it can excite a broad range of frequencies in a single experiment. However, for this project, PRBS was found to have an important limitation: **it provides limited direct control over the exact frequency content being excited**.

For actuator modeling, this matters because the objective is not simply to identify the servo over the widest possible frequency range. The objective is to identify the servo over the frequency range that will matter for rail control.

Therefore, PRBS is useful as an exploratory test, but it is not the preferred final identification method.

PRBS testing may still be used to estimate:

- approximate actuator bandwidth
- rough delay effects
- whether the actuator behaves like a low-order system
- whether there are unexpected resonances or nonlinear effects

However, final model fitting should prioritize the actuator's usable dynamic range.

---

## Targeted Frequency Excitation

After the limitations of PRBS excitation were identified, the procedure shifted toward explicitly exciting selected frequencies.

The preferred test signal is a pseudo-random phase-sum (PRPS) or multi-sine command of the form:

$$
u(t) = u_{0} + \sum_{k=1}^{N} A_k \sin(2\pi f_k t + \phi_k)
$$

where:

- $u(t)$ is the commanded servo input
- $u_{0}$ is the center command
- $A_k$ is the amplitude of the $k^{\text{th}}$ frequency component
- $f_k$ is the $k^{\text{th}}$ excitation frequency
- $\phi_k$ is a randomized phase
- $N$ is the number of excited frequencies

This approach gives direct control over the frequencies used for identification.

Because these tests precede final controller design, the selected excitation frequencies should first be used to **identify the servo actuator's usable dynamic range**. The frequency range should begin where the servo behaves approximately like a static command-to-angle map and extend through the region where magnitude roll-off, phase lag, delay, loss of coherence, rate limiting, or saturation make the actuator less useful for closed-loop control.

The goal is **not** to chase high-frequency model fit quality beyond the actuator's useful range. The goal is to determine the range over which commanded servo angle is tracked reliably enough that the actuator can support rail control.

This identified actuator range will later inform the initial controller bandwidth and determine whether the servo can be modeled as an instantaneous input or must be included as an explicit actuator dynamic.

As an initial practical guideline, the excitation range should extend at least through the first frequency region where one or more of the following occurs:

- measured magnitude drops by approximately 3 dB from the low-frequency gain
- measured phase lag approaches approximately 45 degrees
- coherence becomes too low for a reliable frequency-response estimate
- measured servo motion becomes small relative to encoder noise
- the servo exhibits clear rate limiting, saturation, or nonlinear behavior
- repeated trials no longer produce consistent output behavior

These thresholds are not treated as hard pass/fail rules. They are used as practical markers for identifying where the servo begins to lose useful dynamic authority.

Targeted frequency excitation is used to estimate:

- frequency response
- actuator bandwidth
- phase lag
- effective delay
- gain roll-off
- model fit quality over the actuator's usable dynamic range

---

## Model Candidates

Candidate models should be kept intentionally low order.

### Static Map Only

If the servo bandwidth is much higher than the rail control bandwidth, the actuator may be approximated as instantaneous:

$$
\theta(t) \approx f(\theta_{cmd}(t))
$$

This is the simplest possible model and is acceptable only if servo delay and lag are negligible relative to the rail control loop.

### First-Order Model

If the servo has a dominant lag but delay is small:

$$
\frac{\theta(s)}{\theta_{cmd}(s)} =
\frac{K}{\tau s + 1}
$$

### First-Order Plus Delay Model

If the servo has both dominant lag and meaningful delay:

$$
\frac{\theta(s)}{\theta_{cmd}(s)} =
\frac{K e^{-Ls}}{\tau s + 1}
$$

### Second-Order Model

If the servo response shows overshoot, resonance, or behavior that cannot be captured by a first-order lag:

$$
\frac{\theta(s)}{\theta_{cmd}(s)} =
\frac{K \omega_n^2}
{s^2 + 2\zeta \omega_n s + \omega_n^2}
$$

where:

- $\omega_n$ is the natural frequency
- $\zeta$ is the damping ratio

A second-order model should only be used if the data clearly justifies the added complexity.

---

## Model Selection Criteria

Candidate models are compared based on:

- prediction accuracy
- ability to capture delay and lag
- accuracy over the actuator's usable dynamic range
- physical interpretability
- low model order
- usefulness for controller design
- validation performance on separate data

The preferred model is the **lowest-order model that captures the dominant servo behavior affecting rail control**.

A model that fits high-frequency behavior but adds unnecessary complexity outside the actuator's usable dynamic range should not be preferred over a simpler model that captures the dynamics that will realistically constrain rail-controller design.

---

## Key Design Question

The main design question is whether the servo can eventually be treated as an instantaneous command-to-angle mapping for the rail controller, or whether servo dynamics must be included explicitly in the rail model.

Because final controller design has not yet been completed, these tests first identify the actuator's usable dynamic range. The resulting bandwidth, delay, phase lag, and rate-limit information will then be used to choose an initial rail-controller bandwidth and decide whether a static actuator approximation is acceptable.

If the servo bandwidth is much higher than the rail control bandwidth, then a static command-to-angle approximation may be acceptable.

If servo lag, delay, saturation, or rate limits are significant relative to the rail control loop, then the servo should be modeled as an actuator dynamic state.

In that case, the rail model should include servo dynamics such as:

$$
\dot{\theta} =
\frac{1}{\tau}
\left(
K\theta_{cmd} - \theta
\right)
$$

or, with delay handled separately:

$$
\theta(s) =
\frac{K e^{-Ls}}{\tau s + 1}\theta_{cmd}(s)
$$

---

## Expected Output

The final servo identification results should include:

1. Static command-to-angle map
2. Usable command range
3. Saturation limits
4. Hysteresis or repeatability assessment
5. Step-response plots
6. Estimated rise time, settling time, delay, and time constant
7. Frequency-response estimate from targeted excitation
8. Candidate model comparison
9. Final selected low-order actuator model
10. Decision on whether servo dynamics must be included in the rail control model

---

## Current Status

Initial planning stage.

Next steps:

1. Define the encoder count-to-angle calibration procedure.
2. Collect static command-to-angle sweep data.
3. Collect step-response data over small and moderate command changes.
4. Use step-response results to estimate rough actuator time scale.
5. Design targeted PRPS or multi-sine excitation to identify the servo's usable dynamic range before selecting the initial rail-controller bandwidth.
6. Fit and validate low-order servo models.
7. Decide whether the servo can be modeled as static or must be included explicitly in the rail control model.