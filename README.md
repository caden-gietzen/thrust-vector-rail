# Thrust Vector Rail

## Objective

The objective of this project is to design, model, and control a one-dimensional thrust-vectoring rail vehicle that can stabilize itself at a commanded position along a linear rail.

The system uses a motor-driven thrust source and a thrust-vectoring mechanism to control the motion of a cart constrained to one translational degree of freedom. The control objective is to regulate the cart position to a desired setpoint and eventually track commanded trajectories while respecting physical constraints such as actuator bandwidth, rail length, sensor noise, encoder resolution, thrust limits, and servo angle limits.

This project is structured as an engineering testbed for system identification, model validation, feedback linearization, state estimation, and controller comparison. These methods are evaluated according to how well they satisfy measurable closed-loop performance requirements on the physical system.

## System Overview

The system consists of:

- A cart constrained to move along a linear rail.
- A thrust-producing motor and propeller.
- A thrust-vectoring servo mechanism.
- A quadrature encoder for position measurement.
- A microcontroller for real-time control.
- Data logging and analysis scripts for model identification and controller evaluation.

## Initial Performance Targets

The initial control objective is to stabilize the rail vehicle from multiple starting positions without contacting the rail end-stops.

Initial target requirements include:

- Stabilization from an initial displacement of ±150 mm from the commanded setpoint.
- Settling within ±5 mm of the commanded position.
- Steady-state position error below 5 mm after settling.
- No sustained oscillations.
- Actuator commands constrained within allowable motor PWM and servo angle limits.
- Time-stamped logging of position, velocity estimate, commanded thrust, commanded servo angle, and controller output.

Detailed requirements, constraints, and success criteria are defined in `docs/objectives_requirements_constraints.md`.

## System Constraints

The system is constrained by:

- One-dimensional rail travel length.
- Maximum allowable cart displacement before mechanical end-stop contact.
- Motor thrust limits.
- Servo angle limits.
- Servo bandwidth and delay.
- Electronic speed controller response lag.
- Encoder resolution and possible missed counts.
- Sensor sampling rate and controller loop timing.
- Battery voltage variation.
- Mechanical friction and belt/rail resistance.
- Computation limits of the embedded controller.

## Method Evaluation

Different modeling and control methods will be evaluated by their ability to satisfy the project requirements. Candidate methods include:

- Open-loop and closed-loop system identification.
- Local linearization and feedback linearization.
- Proportional-derivative control.
- Proportional-integral-derivative control.
- Linear-quadratic regulator control.
- Model predictive control.
- State estimation using filtered encoder measurements and model-based observers.

Each method will be compared using consistent performance metrics, including settling time, overshoot, steady-state error, control effort, robustness to sensor noise, and repeatability across trials.

## Repository Structure

- `firmware` — Microcontroller and embedded control code.
- `analysis` — MATLAB and Python scripts for system identification and data analysis.
- `data` — Experimental data.
- `docs` — Engineering notes and documentation.
- `plots` — Generated figures.

## Current Status

Initial repository setup in progress.