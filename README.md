# Thrust-Vector Rail Controls Testbed

## Purpose

This project is a one-dimensional controls testbed for modeling, identifying, and controlling a thrust-vectoring rail vehicle.

The goal is to stabilize a cart at a commanded position along a linear rail using a thrust-producing motor/propeller and a servo-driven thrust-vectoring mechanism. The system is designed as a practical platform for control-oriented system identification, model validation, feedback control, state estimation, and controller comparison.

The broader purpose is to build an engineering workflow that connects:

```text
physical testbed → experimental data → dynamic model → validation → controller design
```

## Why This Project Exists
Many controls projects stop at simulation. This project is intended to close the loop between theory and hardware by forcing the controller design process to deal with real constraints:

- actuator bandwidth and delay
- thrust limits
- servo angle limits
- rail travel limits
- encoder resolution
- sensor noise
- sampling rate limitations
- friction and mechanical resistance
- battery voltage variation
- repeatability across trials

The testbed provides a controlled physical system for evaluating how well different modeling and control methods perform on real hardware.

## System Overview

The system consists of:

- a cart constrained to move along a linear rail
- a motor and propeller that produce thrust
- a servo mechanism that vectors the thrust direction
- a quadrature encoder for position measurement
- load-cell instrumentation for thrust measurement
- a microcontroller for embedded control and data logging
- MATLAB/Python scripts for system identification and validation

## Control Objective

The initial objective is to regulate the cart position to a commanded setpoint without contacting the rail end-stops.

Initial performance targets:

- recover from initial displacements of approximately ±150 mm
- settle within ±5 mm of the commanded position
- maintain steady-state position error below 5 mm after settling
- avoid sustained oscillations
- respect motor pulse-width modulation limits
- respect servo angle limits
- log time-stamped position, velocity estimate, actuator commands, and controller output

Detailed requirements and constraints are documented in:

```
docs/objectives_requirements_constraints.md
```

## Nominal Dynamic Model

The nominal plant model describes the cart as a 1D system driven by a thrust-vectoring actuator pair. This is the working model used for system-identification planning and initial controller design — it is not yet fully validated.

### State vector

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $p$ | cart position along the rail | m |
| $v$ | cart velocity along the rail | m/s |
| $\theta$ | servo / thrust-vector angle | rad |
| $T$ | total motor thrust magnitude | N |

### Inputs

| Symbol | Description |
|--------|-------------|
| $u_\theta$ | commanded servo angle or servo PWM, depending on modeling layer |
| $u_T$ | commanded thrust or ESC PWM, depending on modeling layer |

### Equations of motion

**Cart kinematics:**

$$
\dot{p} = v
$$

**Cart dynamics** (Newton's second law along the rail axis):

$$
\dot{v} = \frac{T}{m}\sin(\theta) + \frac{f_\text{friction}(p,\, v,\, \theta,\, T)}{m}
$$

**Servo angle dynamics** (identified first-order lag with transport delay):

$$
\dot{\theta} = \frac{1}{\tau_\theta}\bigl(\theta_\text{cmd}(t - L_\theta) - \theta\bigr)
$$

**Thrust dynamics** (structure to be determined from thrust identification):

$$
\dot{T} = \frac{1}{\tau_T}\bigl(T_\text{cmd}(t - L_T) - T\bigr) \quad \text{(TBD — parameters under identification)}
$$

| Parameter | Value | Source |
|-----------|-------|--------|
| $m$ | cart mass | to be measured |
| $\tau_\theta$ | 24.4 ms | [PRPS servo identification](experiments/servo_identification/results.md) |
| $L_\theta$ | 28.8 ms | [PRPS servo identification](experiments/servo_identification/results.md) |
| $\tau_T$, $L_T$ | — | thrust identification pending |
| $f_\text{friction}$ | — | friction identification pending |

### Current modeling assumptions

- Motion is modeled along one rail axis only.
- Servo angle and motor thrust are included as actuator states, not static mappings.
- Friction is not ignored — it enters as an additive force term $f_\text{friction}$ to be identified from rail experiments.
- The nominal model retains $\sin(\theta)$ to preserve nonlinear geometry; small-angle linearization may be applied later for LQR or frequency-domain design.
- Thrust dynamics are not finalized and will be updated after thrust identification is validated.

### Why this matters

This model connects the hardware testbed to flight-controls-style engineering: actuator dynamics, friction/disturbance modeling, frequency-domain system identification, model validation against experimental data, and model-based controller design — all on real hardware with real constraints.

## Engineering Workflow

The project is organized around a repeatable controls-analysis workflow:

```
1. Design and instrument the testbed
2. Collect experimental input/output data
3. Process and organize accepted/rejected datasets
4. Identify static and dynamic actuator models
5. Validate models against separate datasets
6. Select simplified models for controller design
7. Implement and compare controllers on hardware
8. Document assumptions, limitations, and results
```

## Current Technical Focus

The project is in the actuator characterization phase, progressing toward closed-loop controller implementation.

**Completed:**

- encoder count-to-angle calibration (2400 counts/rev, <1% endpoint error)
- static PWM-to-angle mapping for the servo actuator (neutral ~1431 µs, gain −0.00159 rad/µs)
- servo step-response analysis (~3 Hz bandwidth, no overshoot, mild amplitude dependence)
- servo PRPS frequency-domain identification over 0.15–3.05 Hz (four amplitudes, training and validation datasets)
- servo transfer-function model selection and validation

The selected servo model is a **first-order lag with transport delay**:

$$
G(s) = \frac{0.001556}{1 + 0.0244\,s}\,e^{-0.0288\,s} \quad [\text{rad}/\mu\text{s}]
$$

Validation errors on held-out data: ≤ 0.5 dB magnitude, ≤ 2.6° phase. The servo cannot be approximated as an instantaneous actuator — the combined 53 ms lag constrains any rail controller to an initial bandwidth of ≤ 1 Hz.

**In progress / next:**

- friction and mechanical resistance identification
- thrust actuator dynamic characterization
- closed-loop rail-controller implementation and comparison

## Candidate Control Methods

The project is intended to compare multiple modeling and control approaches, including:

- open-loop system identification
- closed-loop system identification
- local linearization
- feedback linearization
- proportional-derivative control
- proportional-integral-derivative control
- linear-quadratic regulator control
- model predictive control
- filtered encoder-based state estimation
- model-based observers

Controller performance will be evaluated using:

- settling time
- overshoot
- steady-state error
- control effort
- actuator saturation
- robustness to sensor noise
- repeatability across trials
- sensitivity to operating region

## Repository Structure

```
firmware/     Embedded control and data-acquisition code
analysis/     MATLAB and Python scripts for identification and validation
data/         Experimental datasets
docs/         Engineering notes, requirements, and reports
plots/        Generated figures and model-comparison outputs
```

## Tools and Technologies

- MATLAB
- Python
- Git/GitHub
- Raspberry Pi Pico
- quadrature encoder feedback
- load-cell force measurement
- pulse-width modulation motor control
- servo actuation
- system identification
- frequency-response analysis
- transfer-function modeling
- feedback control

## Relevance

This project is intended to build practical experience in the same engineering pattern used in larger flight-controls and robotics workflows:

```
model the system → test the system → compare model vs. data → refine assumptions → design controller
```

Although the testbed is one-dimensional, it supports development of skills relevant to flight-controls analysis, actuator modeling, experimental validation, simulation-supported controller design, and Guidance, Navigation, and Control (GNC) workflows.

## Current Status

Servo actuator identification is complete with a validated first-order-plus-delay model. Friction and thrust identification are the next steps before closed-loop controller design. Detailed results and model parameters are documented in `experiments/servo_identification/results.md`.