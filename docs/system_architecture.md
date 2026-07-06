# System Architecture

## Design Motivation

The goal of this project is to create a small, repeatable, and physically meaningful controls testbed related to unmanned aerial vehicle dynamics without immediately attempting full free-flight control.

A full drone introduces many coupled problems at once, including six-degree-of-freedom motion, attitude stabilization, translational control, sensor fusion, motor mixing, battery variation, and safety risk. To make the problem more manageable, this project constrains the vehicle to one translational degree of freedom along a low-friction rail.

This reduced system preserves several important features of drone control, including thrust-based actuation, actuator lag, nonlinear force direction through thrust vectoring, sensor feedback, limited actuator authority, and real-time embedded control. The rail therefore provides a practical platform for studying system identification, feedback control, state estimation, and linearization methods on real hardware.

---

## Hardware Overview

The system consists of a rail-constrained cart actuated by a brushless drone motor and servo-driven thrust-vectoring mechanism. A Raspberry Pi Pico RP2040 runs the embedded control logic, while a quadrature encoder provides position feedback through a pulley and belt system.

The main hardware subsystems are:

- Embedded controller: Raspberry Pi Pico RP2040.
- Propulsion: brushless drone motor and electronic speed controller.
- Vectoring mechanism: servo-driven thrust-vector mount.
- Mechanical guide: low-friction linear rail.
- Sensing: quadrature encoder coupled to the rail motion through a pulley and belt.
- Data interface: logged position, estimated velocity, actuator commands, and controller outputs for analysis.


---

## Hardware Selection Rationale

The hardware was selected to create a minimal closed-loop control platform with real actuation, real sensing, and repeatable experimental conditions.

### Raspberry Pi Pico RP2040

The Raspberry Pi Pico RP2040 was selected as the embedded controller because it is inexpensive, lightweight, supports real-time microcontroller operation, and provides access to Programmable Input/Output (PIO) peripherals. This makes it suitable for reading encoder signals, generating actuator commands, and running control logic at a controlled sampling rate.

The PIO subsystem is especially useful for this project because it allows timing-sensitive quadrature encoder decoding to run independently of the main control loop. This lets the main firmware focus on state estimation, control computation, actuator command generation, and data logging while the encoder count is maintained in the background.

### Drone Motor and Electronic Speed Controller

A brushless drone motor and electronic speed controller were selected as the thrust-generation subsystem. This design choice satisfies the need for electronically controllable thrust while keeping the system compact enough for a tabletop rail testbed.

Using a drone propulsion system makes the actuation physically relevant to small unmanned aerial vehicle applications. It also allows the project to study motor command-to-thrust behavior, electronic speed controller response, actuator lag, thrust limits, and closed-loop control using a real propulsion source.

Constraining the propulsion system to a one-dimensional rail reduces the complexity and safety risk compared to free-flight testing, while still preserving important control challenges associated with thrust-based actuation.

### Thrust-Vectoring Mechanism

A servo-driven thrust-vectoring mechanism was selected instead of direct linear actuation to preserve a physically meaningful nonlinear control problem.

Direct actuation, such as a belt-driven motor, wheel-driven cart, linear actuator, or opposing fixed-thrust motors, would make the rail vehicle easier to control. For example, two fixed motors could be mounted in opposite directions along the rail so that one motor produces positive acceleration and the other produces negative acceleration. This would create a more direct relationship between actuator command and rail force.

However, this approach would remove the thrust-vectoring geometry that makes the system useful as a nonlinear controls testbed. With opposing fixed motors, the system would primarily become a one-dimensional force-control problem. By contrast, thrust vectoring requires the controller to account for nonlinear input geometry, actuator saturation, servo bandwidth limits, and coupling between the propulsion and vectoring subsystems.

This design choice makes the rail system more relevant to unmanned aerial vehicle control than a directly actuated cart. Although the rail constrains the vehicle to one translational degree of freedom, the system preserves the important aerial-vehicle control concept that translational acceleration is generated indirectly through the magnitude and direction of a propulsion force. The servo was characterized as a first-order-plus-delay actuator (upgraded servo, ±15° rung: $\tau = 17.1$ ms, $L = 13.4$ ms); see [experiments/servo_identification/results.md](../experiments/servo_identification/results.md) for the full identification.

### Low-Friction Rail

The rail constrains the vehicle to one translational degree of freedom while reducing unwanted frictional effects. This simplifies the dynamics enough to make modeling and controller comparison practical while still preserving real mechanical effects such as friction, end-stop limits, vibration, and imperfect alignment.

### Quadrature Encoder

A quadrature encoder was selected as the primary position sensor because the system requires direct measurement of cart position for closed-loop control. The encoder provides a high-resolution and cost-effective way to measure linear displacement along the rail when coupled to the cart through a pulley and belt system.

Position measurement is the minimum required sensing capability for stabilization, tracking, velocity estimation, system identification, and performance evaluation. The encoder also introduces realistic sensing limitations, including finite resolution, missed counts, noise, and sampling constraints. The measured conversion constant is 64.810 counts/mm (8% above nominal); see [experiments/encoder_calibration/results.md](../experiments/encoder_calibration/results.md) for the calibration procedure and results.

---

## Sensing Architecture

The sensing architecture is centered around encoder-based position measurement. The encoder provides the measured cart position used by the controller and by offline analysis scripts.

From the measured position signal, velocity can be estimated using numerical differentiation, filtering, or model-based observers. Acceleration may also be inferred indirectly, but direct acceleration estimates from position data are expected to be noise-sensitive.

Using an encoder instead of dedicated velocity or acceleration sensors makes the platform useful for evaluating state estimation methods. Different filters and observers can be compared based on how well they reconstruct velocity, reject measurement noise, handle missed encoder counts, and improve closed-loop control performance.

A future extension of the sensing architecture could include integrating an Inertial Measurement Unit (IMU). This would allow the project to explore sensor fusion between encoder-based position measurements and inertial measurements, making the testbed more representative of unmanned aerial vehicle state estimation problems where multiple noisy sensor sources must be fused to estimate the system state.

---

## Actuation Architecture

The actuation architecture consists of two coupled subsystems: thrust generation and thrust vectoring.

The brushless motor and electronic speed controller generate the thrust magnitude. The servo-driven thrust-vectoring mechanism redirects that thrust to produce a horizontal force component along the rail. The effective rail force depends on both the motor thrust and the thrust-vector angle.

At a simplified level, the rail-direction force can be modeled as:

$$F_{rail} = T \sin(\theta)$$

where:
- $T$ is the motor thrust magnitude and
- $\theta$ is the thrust-vector angle relative to the vertical axis.

This actuation structure creates a nonlinear relationship between actuator commands and rail acceleration. Motor commands affect thrust magnitude through the electronic speed controller and motor dynamics, while servo commands affect the thrust direction through the vectoring mechanism. Both subsystems have physical limits, bandwidth constraints, and response delays that must be considered in modeling and control design.

---

## Control Architecture

The control architecture is organized around a real-time embedded control loop running on the Raspberry Pi Pico RP2040.

At each control step, the controller reads the latest encoder count, converts it to cart position, estimates velocity, computes a control command, and sends actuator commands to the motor and servo. Logged data can then be used for system identification, model validation, controller comparison, and estimator evaluation.

A simplified control-loop structure is:

> encoder measurement $\to$ position conversion $\to$ velocity/state estimation $\to$ control law $\to$ motor and servo commands $\to$ rail vehicle dynamics $\to$ updated encoder measurement

The control architecture is intended to support multiple control methods, including proportional-derivative (PD) control, proportional-integral-derivative (PID) control, linear-quadratic regulator (LQR) control, model predictive control (MPC), local linearization, and feedback linearization.

---

## What the Hardware Enables

This hardware configuration enables:
- Repeatable closed-loop stabilization experiments.
- Direct measurement of position along the rail.
- Velocity estimation from encoder position data.
- Identification of motor thrust dynamics.
- Identification of servo angle dynamics.
- Evaluation of actuator lag, saturation, and bandwidth limits.
- Testing of PD, PID, LQR, and MPC controllers.
- Evaluation of local linearization and feedback linearization approaches.
- Investigation of estimator performance under encoder noise and missed counts.
- Quantitative comparison of controllers using settling time, overshoot, steady-state error, control effort, robustness, and repeatability.

---
## Current Limitations

The current system is limited by rail friction, finite rail length, missed encoder counts, actuator delay, battery voltage variation, imperfect mechanical alignment, and exposed propeller safety considerations.
