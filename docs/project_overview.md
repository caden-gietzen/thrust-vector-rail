# Project Overview & Objectives

> **Status:** Stage-0 baseline — the top of the requirements flowdown. It states *what* the system is, *why* it exists, and the hard limits it lives within, then hands off to [`requirements.md`](requirements.md) for the quantified requirements. This is a deliberately lean baseline meant to be marked up and grown; detailed numbers live in the downstream documents it links to (single source of truth), not here.

## 1. Objective

Precisely control the position of a thrust-vectored cart on a one-dimensional, friction-dominated rail, where rail-axis force is produced *indirectly* by steering a thrust vector:

$$
F_\text{rail} = T\sin\theta
$$

The control problem is deliberately **indirect** — acceleration comes from the magnitude $T$ and direction $\theta$ of a propulsion force, not from a direct linear actuator. This preserves the aerial-vehicle control concept in a tabletop, single-degree-of-freedom system. The quantified target is **hardware position tracking within $15$ mm peak / $7$ mm RMS** over the operating envelope ([R4](requirements.md#2-tracking-performance)/[R5](requirements.md#2-tracking-performance)); the design is then pushed to beat it. See [`system_architecture.md`](system_architecture.md) for the design rationale.

## 2. Purpose & guiding narrative

Beyond moving a cart, the rail is a **testbed for the full controls / estimation / V&V stack**, exercised end to end against a model *learned from hardware* rather than assumed. The guiding principle:

> You cannot estimate or control what you cannot model, and you cannot trust a model you have not validated against the real thing.

The one-line narrative the whole project is built to support:

> Build a trustworthy simulator from hardware-identified parameters, design estimation and control against it, then try to break the whole stack with realistic sensor degradation, model mismatch, and Monte Carlo stress before trusting the full closed-loop stack on hardware.

Hardware is a **bookend, not a finale**: early hardware characterizes the rail (identification only); the middle phases are all in simulation against the earned model; late hardware hands the full stack control authority over the real rig.

## 3. Scope

- **In scope:** 1-DOF translational position control via thrust vectoring; system identification of the servo, thrust, and friction subsystems; a crude bootstrap stabilizer; closed-loop identification; gain-scheduled control, state estimation (EKF), and a Monte Carlo → SIL → HIL V&V campaign.
- **Bootstrap simplification (current phase):** position is regulated through the vector angle $\theta$ alone, with **thrust held at a fixed feedforward operating point**. Full thrust control and gain scheduling are deferred (see the phase plan in [`CLAUDE.md`](../CLAUDE.md) and the metric flowdown in [`project_metrics.md`](project_metrics.md)).
- **Out of scope:** free-flight, multi-DOF attitude control, and motor mixing — the rail intentionally reduces the drone problem to one translational axis.

## 4. Constraints

The hard limits the design must live within. Numbers are maintained in the linked source documents; this is the orienting list.

- **Indirect, bidirectional actuation** — rail force is $T\sin\theta$, so both signs of force require both signs of $\theta$ ([R7](requirements.md#3-actuation-requirements), [`actuator_modeling_approach.md`](actuator_modeling_approach.md)).
- **Position-only measurement** — the only direct sensor is cart position (quadrature encoder); velocity, $\theta$, and $T$ must be estimated ([R10](requirements.md#4-sensing-and-logging-requirements), [`encoder_calibration/results.md`](../experiments/encoder_calibration/results.md)).
- **Friction-dominated plant** — significant Coulomb friction with directional asymmetry is a disturbance the drive must overcome; it is characterized before actuator selection (see [`component_specification.md §1`](component_specification.md#1-philosophy-over-bound-first-identify-second)).
- **Actuator bandwidth set by servo delay** — the achievable vectoring bandwidth is limited primarily by servo delay, not raw speed ([`component_specification.md §4.2`](component_specification.md#42-servo-vectoring-actuator)).
- **Finite travel** — physical end-stops bound usable rail travel; the operating envelope must fit inside it with margin ([R1](requirements.md#1-operating-envelope)/[R3](requirements.md#1-operating-envelope)).
- **Voltage-dependent thrust** — battery sag perturbs the thrust map over a run, so a low-sag supply is required for repeatability ([`component_specification.md §4.4`](component_specification.md#44-battery--power-supply)).
- **Mass uncertainty** — design mass range $0.45$–$0.75$ kg, weighed before each campaign and treated as an uncertainty axis ([`qualification_test_plan.md §2`](qualification_test_plan.md#2-common-setup-and-scoring)).
- **Real-time embedded control** — the loop runs on a Raspberry Pi Pico (RP2040) at a fixed sample rate, with encoder decode offloaded to the PIO peripheral ([`component_specification.md §4.7`](component_specification.md#47-mcu-raspberry-pi-pico)).
- **Exposed-propeller safety** — standalone and closed-loop runs are bounded by propeller-safety limits.

## 5. The narrative flowdown

This document is the entry point. Each stage earns the next; read in order:

| Stage | Document | Role |
|---|---|---|
| **0** | **this document** | project objective, purpose, scope, constraints |
| 1 | [`requirements.md`](requirements.md) | quantified a-priori requirements (R1–R14) |
| 2 | [`plant_model_structure.md`](plant_model_structure.md), [`actuator_modeling_approach.md`](actuator_modeling_approach.md) | plant structure ($F=T\sin\theta$, FODT actuators) |
| 3 | [`qualification_test_plan.md`](qualification_test_plan.md) | qualification maneuvers M1–M4 and scoring |
| 4 | [`feasibility.md`](feasibility.md) | requirements → actuator criteria (the method) |
| 5 | [`component_specification.md`](component_specification.md) | required spec per component (the buy decision) |
| 6 | [`hardware_selection.md`](hardware_selection.md) | selection criteria and rationale |
| ↓ | [`rough_truth_model.md`](rough_truth_model.md), [`project_metrics.md`](project_metrics.md) | consolidated plant + per-stage error budget |

Downstream of hardware selection: system identification, controller and estimator design in simulation, and the Monte Carlo → SIL → HIL validation campaign — sequenced in the [Strategic Roadmap](../CLAUDE.md).
