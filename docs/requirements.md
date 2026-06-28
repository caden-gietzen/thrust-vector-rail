# Requirements

> **Status:** A priori top-level requirements. These are chosen before hardware-specific feasibility, subsystem identification, or controller design. They define what the final rail system should achieve; downstream feasibility and hardware-selection criteria may be stricter to provide design margin.

This document is the top of the requirements flowdown. This file only states the quantifiable requirements that downstream documents must trace back to.

## 1. Operating Envelope

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R1 | The final rail system shall track bounded one-dimensional position references within a usable travel envelope. | $\lvert p_\text{ref}\rvert \le 150$ mm | Reference amplitude about the rail centerline. |
| R2 | The final rail system shall track sinusoidal references up to the qualification frequency. | $f_\text{ref} \le 1$ Hz | Defines the top-level motion envelope for feasibility. |
| R3 | The final rail system shall support the design vehicle mass envelope. | $0.45 \le m \le 0.75$ kg | A priori design range used for sizing, not an identified plant conclusion. |
| R4 | The final rail system shall remain inside the usable rail travel during qualification runs. | no end-stop contact | Feasibility and controller design must preserve travel margin. |

## 2. Tracking Performance

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R5 | Hardware peak tracking error shall remain below the qualification limit over the operating envelope. | $\lvert e_p\rvert_\text{peak} \le 10$ mm | Primary final-performance requirement. |
| R6 | Hardware RMS tracking error shall remain below the qualification limit over the operating envelope. | $e_{p,\text{RMS}} \le 5$ mm | Secondary performance requirement for sustained tracking quality. |
| R7 | After a bounded initial displacement, the rail shall recover to the commanded operating region without sustained oscillation. | $\lvert p(0)-p_\text{cmd}\rvert \le 150$ mm | Stabilization requirement for the same position envelope. |

## 3. Actuation Requirements

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R8 | The actuator system shall provide bidirectional rail force through thrust-vectoring. | both signs of $F_x$ | The force mechanism is $F_x = T\sin\theta$; see [plant_model_structure.md](plant_model_structure.md). |
| R9 | The actuator system shall provide enough rail-force authority to track the operating envelope without saturating in nominal qualification conditions. | derived in feasibility | [hardware_selection.md](hardware_selection.md) converts R1-R7 into stricter thrust and vectoring criteria with margin. |
| R10 | The actuator system shall provide enough lateral-force bandwidth to track the operating envelope without relying on operation at the edge of vectoring bandwidth. | derived in feasibility | [actuator_modeling_approach.md](actuator_modeling_approach.md) justifies using vectoring bandwidth as the first lateral-force bandwidth estimate. |

## 4. Sensing and Logging Requirements

| ID | Requirement | Value | Notes |
|---|---|---:|---|
| R11 | The system shall directly measure cart position for feedback and performance scoring. | $y = p$ | Additional state estimates may be reconstructed from this measurement. |
| R12 | The system shall log the signals needed to evaluate tracking, actuator usage, and model validation. | $p$, $p_\text{ref}$, actuator commands, controller output, timestamps | Exact log format belongs in firmware and analysis procedures. |
| R13 | Logged data shall be sufficient to reproduce qualification metrics offline. | R5-R7 computable from logs | Prevents subjective pass/fail claims. |

## 5. Feasibility Flowdown

These requirements are intentionally looser than hardware-selection criteria. The requirements say what the final system must do; feasibility asks whether candidate hardware can do it with breathing room.

Downstream feasibility shall therefore:

1. Convert R1-R7 into required rail force, thrust magnitude, vectoring angle, and vectoring bandwidth.
2. Apply explicit design margin rather than accepting hardware that only barely equals the requirement.
3. State which requirement each feasibility criterion traces back to.
4. Keep hardware constraints and identified plant limitations out of this document; those belong in feasibility, subsystem identification, [rough_truth_model.md](rough_truth_model.md), and controller-design documents.

## 6. Traceability

| Requirement source | Downstream use |
|---|---|
| R1-R4 | Define the operating envelope for [hardware_selection.md](hardware_selection.md) and controller validation. |
| R5-R7 | Define the final hardware tracking target and the top-level error budget in [project_metrics.md](project_metrics.md). |
| R8-R10 | Flow into actuator authority and bandwidth analysis in [actuator_modeling_approach.md](actuator_modeling_approach.md) and [hardware_selection.md](hardware_selection.md). |
| R11-R13 | Flow into logging, scoring, estimator design, and validation procedures. |
