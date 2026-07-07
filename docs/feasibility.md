# Feasibility Procedure

> **Current workflow:** The active requirements-first feasibility analysis is [`analyze_requirement_feasibility.m`](../analysis/feasibility/analyze_requirement_feasibility.m) (servo bandwidth/delay screen and requirement force flowdown) plus [`analyze_propulsion_sizing.m`](../analysis/feasibility/analyze_propulsion_sizing.m) (propulsion-chain and compute sizing). Their consolidated output — the purchase-facing required spec for every component — is the **[component specification sheet](component_specification.md)**. The older hardware-margin heuristic has been retired; [`analyze_hardware_margin.m`](../analysis/feasibility/analyze_hardware_margin.m) is now a thin wrapper that calls the active script.

## Current Requirements-First Result

The active analysis uses only [`requirements.md`](requirements.md), [`qualification_test_plan.md`](qualification_test_plan.md), and explicit design assumptions. It does **not** use measured servo dynamics or measured thrust as inputs; those are later confirmation steps. Friction is the one exception: it is a **disturbance** to be characterized on the rail mechanism *before* actuator selection (see [component_specification.md §1](component_specification.md#1-philosophy-over-bound-first-identify-second)), not a spec of a purchased part.

Maneuver split:

| Maneuver | Requirement trace | Feasibility role |
|---|---|---|
| M1 minimum-jerk slew | [R4/R5/R3](requirements.md#2-tracking-performance) | feedforward command rendering through $G_\text{act}(s)$ |
| M2 0.5 Hz sine | [R4/R5/R1/R2/R3](requirements.md#2-tracking-performance) | feedforward command rendering through $G_\text{act}(s)$ |
| M3 force-step rejection | [R13/R6](requirements.md#5-disturbance-rejection) | feedback crossover support |
| M4 closed-loop ID | [R14](requirements.md#6-model-validation) | validation band context |

The purchase-facing targets, and the reason for each, are consolidated in the [component specification sheet](component_specification.md). In brief:

- **Vectoring actuator (servo):** delay $\le 25$ ms and bandwidth $\ge 5$ Hz (hard, M3-driven — M1/M3 need no faster); $\sim 10$ Hz bandwidth is a conservative target (M2 open-loop feedforward screen + margin, relaxed by feedback); slew $\ge 900\,^\circ$/s. See [component_specification.md §4.2](component_specification.md#42-servo-vectoring-actuator).
- **Thrust authority:** governing rail-axis force over-bound $F_{x,\max} = 2.30$ N (M3 hold: reject the $1$ N step + break the $1.30$ N worst-direction friction). Hard floor $0.33$ kgf at $\theta_\max = 45^\circ$; recommended $\ge 0.95$ kgf so the required deflection stays small (see below).

Why thrust matters even when the authority floor is easy:

$$
\theta_\text{req}=\sin^{-1}\left(\frac{F_x}{T}\right)
$$

More thrust shrinks the required vector angle, and smaller vector-angle amplitudes allow higher usable vectoring bandwidth before the servo hits slew or amplitude limits.

The M3 crossover is derived directly from [R13](requirements.md#5-disturbance-rejection):

$$
e_\text{peak}\approx \frac{F_d}{m\omega_c^2}
$$

Using $F_d=1.0$ N, $m=0.45$ kg, and $e_\text{peak}=15$ mm gives $f_c=1.94$ Hz. Candidate actuators are then screened at that crossover with:

- $\lvert G_\text{act}(j2\pi f_c)\rvert\ge -3$ dB
- actuator phase lag $\le45^\circ$

For $G_\text{act}(s)=\frac{1}{\tau s+1}e^{-sT_d}$, this gives the delay bound:

$$
T_d \le \frac{45^\circ-\tan^{-1}(f_c/f_a)}{360 f_c}
$$

The $10$ Hz / $25$ ms class clears the current screen:

- M1 FFT feedforward rendering: $6.86$ mm peak error
- M2 sine feedforward rendering: $12.83$ mm peak error
- M3 support at $1.94$ Hz: $-0.16$ dB gain and $28.4^\circ$ lag

The candidate rows couple bandwidth and delay, hiding which is binding. Decoupling them shows the **hard** driver is delay $\le \sim 25$ ms with bandwidth $\ge \sim 5$ Hz (that alone clears M1 and M3); the remaining push to $\sim 10$ Hz comes only from M2's *open-loop* feedforward-rendering screen, which feedback relaxes in closed loop. So $\sim 10$ Hz is a conservative target, not a maneuver requirement — full argument in [component_specification.md §4.2](component_specification.md#42-servo-vectoring-actuator).

Generated outputs:

- [`feasibility_maneuver_force_angle_flowdown.png`](../plots/feasibility/feasibility_maneuver_force_angle_flowdown.png)
- [`feasibility_actuator_bandwidth_delay_map.png`](../plots/feasibility/feasibility_actuator_bandwidth_delay_map.png)
- [`feasibility_thrust_deflection_slew_trade.png`](../plots/feasibility/feasibility_thrust_deflection_slew_trade.png)

Reproduce:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); run('c:/dev/thrust-vector-rail/analysis/feasibility/analyze_requirement_feasibility.m')"
```

## Thrust-Angle-Bandwidth Trade (PRPS planning)

For servo PRPS planning, use [`analyze_thrust_angle_bandwidth_trade.m`](../analysis/feasibility/analyze_thrust_angle_bandwidth_trade.m) — a different job from the buy decision above: given a candidate thrust magnitude, what servo angle amplitude must be identified, and what bandwidth must be verified at that amplitude?

$$
\theta = \sin^{-1}\left(\frac{F_x}{T}\right)
$$

It plots three force cases (heavy-mass feedforward, light-mass feedback disturbance, heavy-mass simultaneous) and overlays the required vectoring bandwidth, so the readout is $f_{\text{vec,measured}}(\theta_{\text{PRPS}}) \ge f_{\text{vec,req}}$.

The follow-on servo PRPS campaign therefore measures amplitude-dependent vectoring bandwidth over the deflections that matter for hardware selection. The active sidecar is [`servo_prps_log.orchestrate.json`](../firmware/pico_micropython/system_identification/servo_identification/servo_prps_log.orchestrate.json), generated by [`design_servo_prps_excitation.py`](../tools/design_servo_prps_excitation.py), with rungs at $5^\circ$, $7^\circ$, $9^\circ$, $11^\circ$, $13^\circ$, and $15^\circ$. These correspond to peak command amplitudes of roughly $53$, $74$, $96$, $117$, $138$, and $160~\mu\text{s}$ using the current static map. The small-amplitude rungs push the linear PRPS block to the $15~\text{Hz}$ trusted coherence ceiling so the $\sim 9~\text{Hz}$ corner is visible with margin; the $11^\circ$, $13^\circ$, and $15^\circ$ rungs add single-tone probes around the estimated slew knee.

Outputs:

- [`thrust_angle_bandwidth_trade.png`](../plots/feasibility/thrust_angle_bandwidth_trade.png)
- [`bandwidth_required_vs_angle_amplitude.png`](../plots/feasibility/bandwidth_required_vs_angle_amplitude.png)
- [`thrust_angle_bandwidth_trade.csv`](../data/processed/feasibility/thrust_angle_bandwidth_trade.csv)

Reproduce:

```powershell
matlab -batch "openProject('c:/dev/thrust-vector-rail/thrust-vector-rail.prj'); run('c:/dev/thrust-vector-rail/analysis/feasibility/analyze_thrust_angle_bandwidth_trade.m')"
```

To regenerate the active servo PRPS ladder sidecar:

```powershell
.venv\Scripts\python.exe tools/design_servo_prps_excitation.py --amplitude-ladder --write-orchestrate firmware/pico_micropython/system_identification/servo_identification/servo_prps_log.orchestrate.json
```
