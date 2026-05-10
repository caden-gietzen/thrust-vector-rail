# Objective
Identify a low-order dynamic model for the servo actuator mapping from commanded servo signal to measured servo angle:

$$\theta_{cmd} \to \theta$$
where $\theta_{cmd}$ is the commanded servo angle or equivalent pulse-width modulation (PWM) command, and  $\theta$ is the measured physical servo angle.

The goal is to characterize:
- static command-to-servo-angle mapping
- dominant lag
- effective delay
- saturation limits
- rate limits or slew-rate behavior
- repeatability and hysteresis
- whether explicit servo input dynamics must be included in the rail control model

The final model should be simple enough to support controller design while still capturing the dominant behavior that affects closed-loop performance.
# Methodology
Servo commands will be applied through the Raspberry Pi Pico using pulse-width modulation. The resulting servo motion will be measured using the encoder and converted from counts to physical angle.

The measurement conversion will be based on one or more of the following:

- encoder counts per revolution
- measured angular span over a known motion range
- belt/pulley geometry using the GT2 belt and 20-tooth pulley
- experimental calibration between commanded motion and measured encoder counts

Because pulley geometry and belt coupling may introduce small errors, the preferred approach is to experimentally calibrate the count-to-angle relationship rather than relying only on nominal pulley geometry or specified counts per revolution.

Two types of input tests will be used:

### 1. Step-response tests

Step commands will be used to estimate:

- time delay
- rise time
- settling time
- dominant time constant
- overshoot, if present
- saturation behavior
- rate limiting

These tests are useful for identifying a simple first-order or first-order-plus-delay model.

Candidate model:
$$
\frac{\theta(s)}{\theta_{cmd}(s)} = \frac{K e^{-Ls}}{\tau s + 1}
$$
where:
- $K$ is the steady-state gain from command to angle
- $L$ is the effective time delay
- $\tau$ is the dominant time constant
- $s$ is the Laplace-domain complex frequency variable

### 2. PRBS frequency-excitation tests

A pseudo-random binary sequence (PRBS) will be used to excite the servo over a range of frequencies. This will help estimate the actuator bandwidth and validate whether the step-response model captures the relevant dynamics.

The pseudo-random binary sequence input will be designed with controlled amplitude, switching period, and duration so that the servo is excited without repeatedly driving into hard saturation.

RBS testing will be used to estimate:

- frequency response
- bandwidth
- delay effects
- model fit quality across the frequency range relevant to the control loop
## Hardware

The servo identification setup uses:

- servo actuator
- quadrature encoder
- Raspberry Pi Pico
- GT2 belt and pulley transmission
- mechanical linkage between servo motion and encoder measurement
## Model Selection Criteria

Candidate servo models will be compared based on:

- prediction accuracy
- ability to capture delay and lag
- physical interpretability
- low model order
- usefulness for controller design
- performance in the expected control-loop frequency range

The preferred model is the lowest-order model that captures the dominant servo behavior affecting rail control.

## Key Question

The main design question is whether the servo can be treated as an instantaneous command-to-angle mapping at the chosen control-loop rate, or whether the servo dynamics must be included explicitly in the controller design.

If the servo bandwidth is much higher than the rail control bandwidth, a static command-to-angle approximation may be acceptable.

If the servo lag, delay, or rate limits are significant relative to the rail control loop, the servo should be modeled as an additional actuator dynamic state.

## Current Status

Initial planning stage. Next steps are to define the encoder count-to-angle calibration procedure, collect step-response data, and then design the PRBS excitation signal.