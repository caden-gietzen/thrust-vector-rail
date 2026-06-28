# Plant Model Structure

> **Status:** Step-zero model definition. This document defines the minimum plant abstraction for the thrust-vector rail before feasibility, hardware selection, system identification, or controller design.

This rail is a one-dimensional thrust-vectoring controls testbed. The cart translates along a linear rail, but the actuator does not push directly along that rail. A motor produces thrust magnitude, a servo-driven mechanism redirects that thrust, and the rail-direction force is the horizontal component of the thrust vector. This structure is the reason the project is useful as a controls testbed: it keeps the translational motion simple while preserving actuator lag, nonlinear force projection, saturation, friction, sensing limits, and model mismatch.

## 1. Minimal state

Use the state vector:

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

where:

- $p$ is cart position along the rail,
- $v$ is cart velocity along the rail,
- $\theta$ is thrust-vector angle, and
- $T$ is motor thrust magnitude.

The first two states describe the rail motion. The last two states describe the actuator itself. Including $\theta$ and $T$ as states, rather than treating the actuator as instantaneous, is the core modeling choice that keeps the simulator honest.

## 2. Inputs and measured output

The physical command inputs are:

$$
\mathbf{u} = \begin{bmatrix} u_\theta \\ u_T \end{bmatrix}
$$

where $u_\theta$ is the servo or vectoring command and $u_T$ is the motor or ESC command.

The directly measured output is cart position:

$$
y = p
$$

Velocity is estimated from position or reconstructed by an observer. Thrust-vector angle and thrust magnitude can be characterized during subsystem tests, but they are not the primary closed-loop rail measurements in the current architecture.

## 3. Core force coupling

The defining actuator relationship is:

$$
F_x = T\sin\theta
$$

where $F_x$ is the rail-direction force. This is the coupling that connects actuator modeling to feasibility and control. Motor command changes available thrust magnitude. Servo command changes force direction. Rail acceleration depends on both.

For small perturbations around $\theta = 0$ at approximately constant thrust $T_0$:

$$
F_x \approx T_0\theta
$$

This small-signal approximation explains why vectoring bandwidth can be used as the first estimate of lateral-force actuation bandwidth. The nonlinear relationship $F_x = T\sin\theta$ should still be retained in simulation for finite-angle motion.

## 4. Nominal structural dynamics

The minimal continuous-time structure is:

$$
\dot{p} = v
$$

$$
\dot{v} = \frac{1}{m}\left(T\sin\theta - F_\text{friction}(v, p, t)\right)
$$

$$
\theta = G_\theta(s)u_\theta
$$

$$
T = G_T(s)u_T
$$

The actuator models $G_\theta(s)$ and $G_T(s)$ are placeholders at this stage. They should be interpreted structurally as delayed, bandwidth-limited actuator responses, not as finalized identified transfer functions. The identified numerical versions belong in [rough_truth_model.md](rough_truth_model.md).

## 5. Design meaning

This structure separates the early design problem into three questions:

- **Authority:** can the motor produce enough thrust, after the $\sin\theta$ projection, to generate the required rail force?
- **Bandwidth:** can the vectoring mechanism change $\theta$ fast enough for the required lateral-force bandwidth?
- **Observability and estimation:** can the controller stabilize the rail when only $p$ is directly measured during normal operation?

The first two questions feed [actuator_modeling_approach.md](actuator_modeling_approach.md) and [hardware_selection.md](hardware_selection.md). The third feeds later estimator and closed-loop validation work.

## 6. Natural document progression

This document is the structural starting point. The intended narrative flow is:

1. [plant_model_structure.md](plant_model_structure.md): define the state, inputs, measured output, and force coupling.
2. [actuator_modeling_approach.md](actuator_modeling_approach.md): explain how lateral-force authority and bandwidth are inferred from thrust and vectoring dynamics.
3. [requirements.md](requirements.md): define the a priori operating envelope and performance requirements.
4. [hardware_selection.md](hardware_selection.md): translate requirements into motor authority and vectoring bandwidth needs.
5. [system_identification.md](system_identification.md): identify the subsystem models needed by the simulator.
6. [rough_truth_model.md](rough_truth_model.md): assemble identified parameters into the first usable plant model.
7. [crude_stabilizer_design.md](crude_stabilizer_design.md): design the conservative controller used to bootstrap closed-loop identification.

## 7. Bottom line

The plant is a one-dimensional translational system driven by a two-state actuator:

$$
\mathbf{x} = \begin{bmatrix} p \\ v \\ \theta \\ T \end{bmatrix}
$$

with rail force:

$$
F_x = T\sin\theta
$$

That single modeling choice drives the rest of the project. Motor selection is primarily an authority problem, vectoring selection is primarily a bandwidth problem, and the simulator must preserve both actuator lag and nonlinear force projection before any controller result can be trusted.
