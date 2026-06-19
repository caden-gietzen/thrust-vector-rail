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

Detailed requirements and constraints are documented in [docs/objectives_requirements_constrains.md](docs/objectives_requirements_constrains.md).

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
| $m$ | 0.4536 kg (1 lb nominal) | measured |
| $\tau_\theta$ | 24.4 ms | [servo identification](experiments/servo_identification/results.md) |
| $L_\theta$ | 28.8 ms | [servo identification](experiments/servo_identification/results.md) |
| $\tau_T$ | 78.1 ms | [thrust identification](experiments/thrust_identification/results.md) |
| $L_T$ | 25.2 ms | [thrust identification](experiments/thrust_identification/results.md) |
| $\mu_c$ | ≈ 1.0 N (provisional; $b \approx 2.3$ N·s/m viscous) | [friction identification](experiments/friction_identification/results.md) — not yet finalized; ~30% directional asymmetry observed |

### Current modeling assumptions

- Motion is modeled along one rail axis only.
- Servo angle and motor thrust are included as actuator states, not static mappings.
- Friction is carried as a provisional bound ($\mu_c \approx 1$ N, with a viscous term $b \approx 2.3$ N·s/m) for initial controller design; a ~30% directional asymmetry has been observed. Friction is the least-settled identification and is not yet finalized — see [experiments/friction_identification/results.md](experiments/friction_identification/results.md).
- The nominal model retains $\sin(\theta)$ to preserve nonlinear geometry; the controller design uses local linearization at chosen operating points.

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

Open-loop actuator and friction identification is complete (servo, thrust, friction —
friction still being finalized). The conclusions are consolidated into a single
hardware-identified plant: [docs/rough_truth_model.md](docs/rough_truth_model.md).

The project is now at the **closed-loop ID bootstrap** step of Phase 1: design a **crude,
robust hand-tuned PID stabilizer** against the rough truth model so the rail can be run
safely in closed loop, then collect closed-loop ID data to *earn* a refined model. Heavier
control design (gain-scheduled LQR, EKF, LQG, Monte Carlo V&V) is deliberately deferred to
later phases of the project.

### Completed Identification

**Encoder calibration:**  64 810 counts/m (measured; nominal 60 000); <1% endpoint error. Details: [experiments/encoder_calibration/results.md](experiments/encoder_calibration/results.md).

**Servo actuator** — first-order lag with transport delay:

$$
G_\theta(s) = \frac{0.001556}{1 + 0.0244\,s}\,e^{-0.0288\,s} \quad [\text{rad}/\mu\text{s}]
$$

Validation errors ≤ 0.5 dB magnitude, ≤ 2.6° phase. The combined 53 ms lag (τ + L) constrains initial rail-controller bandwidth to ≤ 1 Hz. Details: [experiments/servo_identification/results.md](experiments/servo_identification/results.md).

**Thrust actuator** — static map and PRPS dynamic identification over the full usable range (1075–1950 µs → 0.23–4.17 N):

$$
G_T(s) = \frac{0.00414}{1 + 0.0781\,s}\,e^{-0.0252\,s} \quad [\text{N}/\mu\text{s}]
$$

The static map is nonlinear; a degree-4 polynomial captures it with RMSE = 0.023 N. Dynamic gain increases 91% from low to high operating point; delay is nearly constant at ≈25 ms across all regions. Details: [experiments/thrust_identification/results.md](experiments/thrust_identification/results.md).

**Friction identification** — residual-based method using the servo and thrust models applied to fixed-angle, stepped-ESC runs. The latest (multi-angle) pass gives a viscous + Coulomb fit with $b \approx 2.3$ N·s/m, $\mu_c \approx 1.0$ N:

$$
f_\text{friction}(v) = -\,b\,v \; - \; \mu_c\operatorname{sign}(v)
$$

A ~30% directional asymmetry was observed. Friction is **identified but not finalized** — the data is still in `candidate/` and the model selection is open. See [experiments/friction_identification/results.md](experiments/friction_identification/results.md) for the current estimate, the discrepancy with an earlier Coulomb-only pass, and the open items.

### Next step — crude stabilizer for closed-loop ID

The immediate task is **not** a high-performance controller. It is a **robust, hand-tuned PID** position controller that commands servo angle (thrust held at a fixed feedforward operating point), designed in simulation against the [rough truth model](docs/rough_truth_model.md). Its only job is to be hard to destabilize across model error so the rail can be driven safely in closed loop — the prerequisite for collecting closed-loop identification data.

Target: bandwidth 0.1–0.3 Hz (well inside the servo's ~19°-at-1-Hz phase-lag budget), generous phase margin, integral action plus likely friction feedforward to overcome the low thrust-to-weight (0.32 at 20°) and significant friction. Design rationale and constraints are in [docs/rough_truth_model.md](docs/rough_truth_model.md).

**Deferred to later phases:** gain-scheduled LQR (Phase 2), EKF estimation (Phase 3), LQG (Phase 4), and Monte Carlo / SIL / HIL V&V (Phase 5).

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
firmware/                    Embedded control and data-acquisition code (MicroPython, RP2040)
analysis/
  system_identification/     MATLAB identification scripts (servo, thrust, friction)
  hardware_validation/       MATLAB validation scripts (encoder, load cell)
data/                        Experimental datasets (candidate/accepted/rejected/diagnostics)
docs/                        Engineering notes, requirements, design docs, rough_truth_model.md
experiments/                 Per-subsystem procedure, model selection, and results documents
plots/                       Generated figures and model-comparison outputs
tools/                       Python orchestration scripts (run_pico_and_pull.py)
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

| Phase | Status |
|-------|--------|
| Encoder calibration | **Complete** — 64 810 counts/m |
| Servo identification | **Complete** — FOPD model, K = 0.001556 rad/µs, τ = 24.4 ms, L = 28.8 ms |
| Thrust static map | **Complete** — degree-4 polynomial, 1075–1950 µs → 0.23–4.17 N |
| Thrust dynamic identification | **Complete** — FOPD model, K = 0.00414 N/µs, τ = 78.1 ms, L = 25.2 ms |
| Friction identification | **Identified, not finalized** — viscous+Coulomb, μ_c ≈ 1.0 N, b ≈ 2.3 N·s/m; ~30% asymmetry; data in `candidate/` |
| Rough truth model | **Complete** — consolidated plant in [docs/rough_truth_model.md](docs/rough_truth_model.md) |
| Crude PID stabilizer (simulation) | **Next** — robust hand-tuned PID for closed-loop ID bootstrap |
| Closed-loop identification | Pending crude stabilizer |
| Gain-scheduled LQR / EKF / LQG / V&V | Deferred — later project phases |

Detailed results and model parameters are in the [experiments/](experiments/) directory. The consolidated plant for controller design is [docs/rough_truth_model.md](docs/rough_truth_model.md).