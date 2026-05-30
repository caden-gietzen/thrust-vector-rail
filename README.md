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
| $\mu_c$ | 0.8158 N | [friction identification](experiments/friction_identification/procedure.md) — Coulomb-only model; 42% directional asymmetry observed |

### Current modeling assumptions

- Motion is modeled along one rail axis only.
- Servo angle and motor thrust are included as actuator states, not static mappings.
- Friction is modeled as a symmetric Coulomb term $f_\text{friction} = -\mu_c \operatorname{sign}(v)$ for initial controller design. A 42% directional asymmetry has been observed and is flagged as a refinement candidate.
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

The project has completed actuator characterization and is now in the controller design and simulation phase.

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

**Friction identification** — residual-based method using servo and thrust models applied to fixed-angle, stepped-ESC runs. Selected model is Coulomb-only (viscous term not significant):

$$
f_\text{friction}(v) = -\mu_c\operatorname{sign}(v), \quad \mu_c = 0.8158\,\text{N}
$$

RMSE = 0.24 N. A 42% directional asymmetry was observed (positive rail direction: $\mu = 0.56$ N; negative: $\mu = 0.86$ N) and is flagged for future refinement. See [experiments/friction_identification/procedure.md](experiments/friction_identification/procedure.md) for the full analysis method.

### Controller Design (current phase)

Controller design proceeds in simulation before hardware deployment. Two control architectures are in development:

**LQR gain scheduling** — three simulation tiers of increasing scheduling complexity ([docs/gain_scheduling.md](docs/gain_scheduling.md)):

| Tier | Strategy | Scheduling variable(s) | Simulink model |
|------|----------|------------------------|----------------|
| 1 | Fixed-gain LQR | none — single linearization at $\theta^* = 0$, $T^* = 2.574$ N | [fixed_lqr.slx](analysis/control_design/gain_scheduling/fixed_lqr.slx) |
| 2 | Angle-scheduled LQR | $\theta^*$ grid over $[-60°, 60°]$ at nominal thrust | [theta_sched_lqr.slx](analysis/control_design/gain_scheduling/theta_sched_lqr.slx) |
| 3 | Pair-scheduled LQR | $(\theta^*, T^*)$ grid — 25 × 7 = 175 gain matrices | [pair_sched_lqr.slx](analysis/control_design/gain_scheduling/pair_sched_lqr.slx) |

The linearization structure is: $A$ is constant (independent of operating point); all operating-point dependence enters through $B(\theta^*, T^*)$. Tier 2 captures servo-authority variation with angle; Tier 3 additionally captures thrust-level scaling of $B_\theta = (T/m)\cos\theta$. Gain matrices are precomputed by [trim_analysis.m](analysis/control_design/gain_scheduling/trim_analysis.m) and saved as `.mat` files; [build_model.m](analysis/control_design/gain_scheduling/build_model.m) constructs the matching Simulink models programmatically.

**PID position controller** — SISO loop that commands servo angle to regulate rail position, with thrust held at the nominal feedforward value (2.574 N). The Simulink model [position_pid.slx](analysis/control_design/pid_design/position_pid.slx) is built by [build_pid_model.m](analysis/control_design/pid_design/build_pid_model.m) and is tunable via MATLAB's PID Tuner app (target bandwidth 0.3–0.5 Hz). Serves as a performance baseline for the scheduled LQR variants.

Both designs use the same nonlinear plant equations of motion (mass $m = 0.4536$ kg, Coulomb friction $\mu_c = 0.8158$ N, identified servo and thrust dynamics).

**Next steps:** hardware deployment of controllers; closed-loop performance comparison; directional friction refinement if position errors are asymmetric.

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
  control_design/
    gain_scheduling/         LQR trim analysis, gain tables, Simulink models (Tiers 1–3)
    pid_design/              PID position controller and Simulink model
data/                        Experimental datasets (candidate/accepted/rejected/diagnostics)
docs/                        Engineering notes, requirements, and design documentation
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
| Friction identification | **Complete** — Coulomb-only, μ_c = 0.8158 N; 42% directional asymmetry flagged |
| LQR gain scheduling (simulation) | **In progress** — Tiers 1–3 precomputed; Simulink models built |
| PID controller (simulation) | **In progress** — Simulink model built; awaiting PID Tuner session |
| Hardware controller deployment | Pending simulation validation |
| Directional friction refinement | Deferred — revisit if asymmetric position errors emerge |

Detailed results and model parameters are in the [experiments/](experiments/) directory. Controller design documentation is in [docs/gain_scheduling.md](docs/gain_scheduling.md).