# Requirements

> **Status:** A priori top-level requirements. These are chosen before hardware-specific feasibility, subsystem identification, or controller design. They define what the final rail system should achieve. The requirements are deliberately **loose** (underpromise/overdeliver): they define an achievable qualification bar, and the design is then pushed to beat it. The concrete test cases that demonstrate these requirements are the qualification maneuver suite in [qualification_test_plan.md](qualification_test_plan.md).

This document is the top of the requirements flowdown. It states only the quantifiable requirements that downstream documents must trace back to; the maneuver definitions, disturbance-injection method, and pass/fail scoring live in [qualification_test_plan.md](qualification_test_plan.md).

## 1. Operating Envelope

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R1 | The final rail system shall track bounded one-dimensional position references within a usable travel envelope. | $\lvert p_\text{ref}\rvert \le 100$ mm | Reference amplitude about the rail centerline. A $\pm 100$ mm span uses 200 mm of the measured 316.5 mm end-stop separation ([encoder_calibration/results.md](../experiments/encoder_calibration/results.md)), leaving 116.5 mm of margin; tracking excursion must fit in that margin (R3). |
| R2 | The final rail system shall track sinusoidal references within the qualification envelope. | $f_\text{ref} = 0.5$ Hz | Qualification (maneuver M2) is demonstrated at $0.5$ Hz. |
| R3 | The final rail system shall remain inside the usable rail travel during qualification runs. | no end-stop contact | Feasibility and controller design must preserve travel margin. |

## 2. Tracking Performance

These apply to the tracking maneuvers **M1** (minimum-jerk slew) and **M2** (0.5 Hz sine) in [qualification_test_plan.md](qualification_test_plan.md). Disturbance rejection and model validation are separate requirements ([R13](#5-disturbance-rejection), [R14](#6-model-validation)).

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R4 | Hardware peak tracking error shall remain below the qualification limit over the operating envelope. | $\lvert e_p\rvert_\text{peak} \le 15$ mm | Primary final-performance requirement. |
| R5 | Hardware RMS tracking error shall remain below the qualification limit over the operating envelope. | $e_{p,\text{RMS}} \le 7$ mm | Secondary performance requirement for sustained tracking quality. |
| R6 | After a bounded initial displacement, the rail shall recover to the commanded operating region without sustained oscillation. | $\lvert p(0)-p_\text{cmd}\rvert \le 100$ mm | Stabilization requirement for the same position envelope. |

## 3. Actuation Requirements

| ID | Requirement | Value | Notes |
|---|---|---|---|
| R7 | The actuator system shall provide bidirectional rail force through thrust-vectoring. | both signs of $F_x$ | The force mechanism is $F_x = T\sin\theta$; see [plant_model_structure.md](plant_model_structure.md). |
| R8 | The actuator system shall provide enough rail-force authority to track the operating envelope without saturating in nominal qualification conditions. | derived in feasibility | [hardware_selection.md](hardware_selection.md) converts R1-R6 into thrust and vectoring criteria. |
| R9 | The actuator system shall provide enough lateral-force bandwidth to track the operating envelope without relying on operation at the edge of vectoring bandwidth. | derived in feasibility | [actuator_modeling_approach.md](actuator_modeling_approach.md) justifies using vectoring bandwidth as the first lateral-force bandwidth estimate. |

## 4. Sensing and Logging Requirements

| ID | Requirement | Value | Notes |
|---|---|---|---|
| R10 | The system shall directly measure cart position for feedback and performance scoring. | $y = p$ | Additional state estimates may be reconstructed from this measurement. |
| R11 | The system shall log the signals needed to evaluate tracking, actuator usage, and model validation. | $p$, $p_\text{ref}$, actuator commands, controller output, timestamps | Exact log format belongs in firmware and analysis procedures. |
| R12 | Logged data shall be sufficient to reproduce qualification metrics offline. | R4-R6, R13-R14 computable from logs | Prevents subjective pass/fail claims. |

## 5. Disturbance Rejection

Exercised by qualification maneuver **M3** (center-hold under an injected force step) in [qualification_test_plan.md](qualification_test_plan.md).

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R13 | While holding a fixed position, the final rail system shall reject a bounded input-referred force-step disturbance without sustained oscillation, and return to the hold point. | peak excursion $\le 15$ mm; recover to $\le 3$ mm within $2$ s; steady-state $\le 1$ mm | Tested in **both** directions to expose friction asymmetry. The disturbance is injected as a known software force step (see [qualification_test_plan.md](qualification_test_plan.md)); steady-state rejection depends on integral action. |

## 6. Model Validation

Exercised by qualification maneuver **M4** (multisine closed-loop identification) in [qualification_test_plan.md](qualification_test_plan.md).

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R14 | The closed-loop system shall be identified from data, and the model used for control design shall be validated against the identified frequency response over the coherent band. | $\nu$-gap $\delta_\nu(\hat{P}, P_\text{design}) < b(P,C)$; report $-3$ dB closed-loop bandwidth | Confirms the design model is control-relevant. |

## 8. Traceability

| Requirement source | Downstream use |
|---|---|
| R1-R3 | Define the operating envelope for [hardware_selection.md](hardware_selection.md), the qualification maneuvers (M1-M4), and controller validation. |
| R4-R6 | Define the tracking maneuvers (M1, M2) and the top-level tracking error budget in [project_metrics.md](project_metrics.md). |
| R7-R9 | Flow into actuator authority and bandwidth analysis in [actuator_modeling_approach.md](actuator_modeling_approach.md) and [hardware_selection.md](hardware_selection.md). |
| R10-R12 | Flow into logging, scoring, estimator design, and validation procedures. |
| R13 | Defines the disturbance-rejection maneuver (M3) and the feedback crossover the error budget in [project_metrics.md](project_metrics.md) must earn. |
| R14 | Defines the closed-loop identification maneuver (M4) and traces to the Stage 3 exit metric in [project_metrics.md](project_metrics.md). |
