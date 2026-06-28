# Thrust-Vector Rail Controls Testbed

This project is a one-dimensional controls testbed for modeling, identifying, and controlling a thrust-vectoring rail vehicle. A cart moves along a linear rail while a motor produces thrust and a servo-driven mechanism redirects that thrust to create rail-direction force.

The project is built around a staged engineering workflow:

```text
requirements -> plant structure -> feasibility -> hardware selection -> system identification -> simulation -> controller design -> hardware validation
```

The current documentation pass is intentionally back at the beginning of that flow. The goal is to make the day-zero assumptions, plant structure, and requirements clean before redoing feasibility and hardware-selection analysis.

## Core Model

The minimum plant abstraction is:

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

where $p$ is rail position, $v$ is rail velocity, $\theta$ is thrust-vector angle, and $T$ is thrust magnitude. The rail-direction force is:

$$
F_x = T\sin\theta
$$

The corresponding rail dynamics are:

$$
\dot{p} = v
$$

$$
m\dot{v} = T\sin\theta - F_f(v) + d
$$

where $F_f(v)$ represents friction and $d$ represents bounded unmodeled disturbance. The commanded vector angle and thrust are not assumed instantaneous; they enter through identified actuator dynamics:

$$
\theta(s) = G_\theta(s)u_\theta(s)
$$

$$
T(s) = G_T(s)u_T(s)
$$

with static thrust mapping retained where needed. Around $\theta = 0$, the force projection reduces to $F_x \approx T_0\theta$, so lateral-force bandwidth can be approximated as the vectoring-actuation bandwidth for feasibility and hardware-selection purposes. For larger sweeps, simulation keeps the nonlinear $T\sin\theta$ projection.

This structure is the core reason the system is useful: translational motion is one-dimensional, but actuation remains indirect, nonlinear, bandwidth-limited, friction-limited, and constrained by real hardware.

## Documentation Flow

Read the project narrative in this order:

1. [docs/plant_model_structure.md](docs/plant_model_structure.md): step-zero state, inputs, output, and force coupling.
2. [docs/requirements.md](docs/requirements.md): a priori operating envelope and performance requirements.
3. [docs/actuator_modeling_approach.md](docs/actuator_modeling_approach.md): how lateral-force authority and bandwidth are inferred from thrust and vectoring dynamics.
4. [docs/hardware_selection.md](docs/hardware_selection.md): feasibility and hardware-selection analysis. This is being realigned to the updated requirements and assumptions.
5. [docs/system_identification.md](docs/system_identification.md): identification workflow for the subsystem models.
6. [docs/rough_truth_model.md](docs/rough_truth_model.md): current assembled plant model from identified subsystem data.
7. [docs/crude_stabilizer_design.md](docs/crude_stabilizer_design.md): bootstrap stabilizer design for closed-loop identification.

## Current Status

The project is revisiting the top of the narrative before pushing deeper into controller work:

- step-zero plant structure has been formalized;
- a priori requirements have replaced the old combined objectives/requirements/constraints document;
- actuator bandwidth is treated as inferred from vectoring bandwidth under the constant-thrust, quasi-static projection assumption;
- feasibility and hardware selection are expected to be redone against the updated requirements with explicit margin;
- identified subsystem results remain available under [experiments/](experiments/) and [docs/rough_truth_model.md](docs/rough_truth_model.md), while [docs/pre_overhaul_baseline.md](docs/pre_overhaul_baseline.md) preserves the old values for before/after comparison.

## Engineering Plan

The intended project progression is:

1. Define the required operating envelope and tracking performance.
2. Use the plant structure to derive actuator authority and bandwidth needs.
3. Select or revise hardware using stricter feasibility criteria than the bare requirements.
4. Identify servo, thrust, friction, and sensor behavior on hardware.
5. Assemble a simulation model with uncertainty bounds.
6. Design a conservative stabilizer to collect closed-loop identification data.
7. Use the earned model for higher-performance control, estimation, Monte Carlo validation, SIL/HIL, and final hardware deployment.

## Repository Map

```text
firmware/                    MicroPython data-acquisition and control code
analysis/                    MATLAB/Python identification and feasibility analysis
data/                        Experimental datasets
docs/                        Requirements, model structure, design rationale, and analysis notes
experiments/                 Per-subsystem procedures and results
plots/                       Generated analysis figures
tools/                       Orchestration utilities
tvr_stabilizer/              Simulation and stabilizer design work
```

## Guiding Principle

You cannot estimate or control what you cannot model, and you cannot trust a model you have not validated against the real thing.
