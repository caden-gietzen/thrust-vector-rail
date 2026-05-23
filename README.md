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

The current phase focuses on actuator characterization and control-oriented model identification.

Work completed or in progress includes:

- static thrust mapping across motor command ranges
- dynamic thrust identification using excitation signals
- frequency-response analysis
- transfer-function model fitting
- model comparison using training and validation datasets
- actuator bandwidth and delay estimation
- structured data organization for repeatable analysis
- preparation for closed-loop controller implementation

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

The testbed hardware, data-collection workflow, and initial actuator identification process are in progress. Current work is focused on finalizing validated actuator models and preparing the system for closed-loop controller comparison.