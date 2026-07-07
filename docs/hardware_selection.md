# Vectoring Hardware Selection & Overhaul Requirements

> **Current status:** Hardware selection is consolidated into the **[component specification sheet](component_specification.md)** — the purchase-facing required spec for every component, with each number traced to a requirement. This file retains the actuation-side selection *criteria* and the bandwidth/delay rationale that feed that sheet; the requirements-first feasibility method behind them is in [`feasibility.md`](feasibility.md) and [`analyze_requirement_feasibility.m`](../analysis/feasibility/analyze_requirement_feasibility.m). The older friction-ID-based margin heuristic has been retired (see git history).

## Current Selection Criteria

The current selection criteria trace directly to [requirements.md](requirements.md) and [qualification_test_plan.md](qualification_test_plan.md), before knowing what thrust or servo dynamics the selected hardware will actually produce.

| Criterion | Target | Trace |
|---|---:|---|
| Vectoring actuator delay | $\le 25$ ms | M3 phase-lag support at $f_c=1.94$ Hz (the **hard** driver — delay-dominated) |
| Vectoring actuator bandwidth | $\ge 5$ Hz hard; $\sim 10$ Hz target | M3 crossover needs only $\ge 5$ Hz (M1/M3 no more); $\sim 10$ Hz is a conservative margin from M2's open-loop feedforward screen — see [component_specification.md §4.2](component_specification.md#42-servo-vectoring-actuator) |
| Vectoring actuator slew | $\ge 900\,^\circ$/s | keeps the worst-case deflection from slew-limiting the $\sim 10$ Hz vectoring target |
| Thrust authority (governing) | $\ge 0.95$ kgf; hard floor $0.33$ kgf | keeps the required vector deflection small — see [component_specification.md §4.1](component_specification.md#41-motor--prop-thrust) |

The requirement-only rail-axis force envelope from the qualification maneuvers (before friction) is:

- M1 minimum-jerk slew: $0.139$ N peak inertial force at $m=0.75$ kg
- M2 0.5 Hz sine: $0.740$ N peak inertial force at $m=0.75$ kg
- M3 force-step rejection: $1.000$ N counter-force

The **authoritative** thrust authority number adds the friction disturbance the motor must physically overcome. The governing rail-axis force is the M3 hold case — reject the $1.0$ N step *and* break the $1.30$ N worst-direction friction at once:

$$
F_{x,\max} = F_\text{inj} + F_\text{fric} = 1.0 + 1.30 = 2.30\ \text{N}
$$

giving a hard thrust floor $T_\min = 2.30/\sin 45^\circ = 3.25$ N $= 0.33$ kgf, and a recommended $\ge 0.95$ kgf so the required deflection stays small. The full derivation, the friction over-bound, and the per-component sizing are in the [component specification sheet](component_specification.md).

More thrust reduces the required vector angle:

$$
\theta_\text{req}=\sin^{-1}\left(\frac{F_x}{T}\right)
$$

and smaller commanded deflections let the servo achieve higher usable vectoring bandwidth before slew or amplitude limits bind:

$$
f_\text{slew}=\frac{\dot{\theta}_\max}{2\pi\theta_\text{req}}
$$

## Current Bandwidth/Delay Rationale

M1 and M2 are feedforward-dominated: the reference trajectory is known, so the feasibility screen asks whether the actuator can render the inverse-dynamics vector-angle command.

- M1 is evaluated by FFT-rendering the actual $200$ mm, $2.5$ s minimum-jerk $\theta_\text{ff}$ command through $G_\text{act}(j\omega)$.
- M2 is evaluated as a single-frequency feedforward command at $0.5$ Hz.

M3 is feedback-dominated. Its required crossover comes from [R13](requirements.md#5-disturbance-rejection):

$$
e_\text{peak}\approx\frac{F_d}{m\omega_c^2}
$$

Using $F_d=1.0$ N, $m=0.45$ kg, and $e_\text{peak}=15$ mm gives:

$$
f_c=1.94\ \text{Hz}
$$

For a first-order-plus-delay vectoring actuator,

$$
G_\text{act}(s)=\frac{1}{\tau s+1}e^{-sT_d}
$$

the actuator must leave enough gain and phase headroom at $f_c$:

$$
\left|G_\text{act}(j2\pi f_c)\right|\ge -3\ \text{dB}
$$

$$
\tan^{-1}\left(\frac{f_c}{f_a}\right)+360f_cT_d\le45^\circ
$$

which implies:

$$
T_d \le \frac{45^\circ-\tan^{-1}(f_c/f_a)}{360 f_c}
$$

The $10$ Hz / $25$ ms actuator class clears the screen:

| Case | Result |
|---|---:|
| M1 FFT feedforward rendering | $6.86$ mm peak error |
| M2 sine feedforward rendering | $12.83$ mm peak error |
| M3 gain at $1.94$ Hz | $-0.16$ dB |
| M3 phase lag at $1.94$ Hz | $28.4^\circ$ |

The candidate rows couple bandwidth and delay together, which hides which one is binding. Decoupling them (hold delay at $25$ ms, vary bandwidth) shows the **hard** driver is delay $\le \sim 25$ ms with bandwidth $\ge \sim 5$ Hz — that alone satisfies M1 and M3. The remaining push to $\sim 10$ Hz comes only from M2's *open-loop* feedforward-rendering screen, which feedback relaxes in closed loop, so $\sim 10$ Hz is a conservative target rather than a maneuver requirement (full argument in [component_specification.md §4.2](component_specification.md#42-servo-vectoring-actuator)).

Passing the feasibility screen is not final validation; it is the justification for hardware selection. The selected hardware must still be confirmed by identification before closed-loop qualification. The per-component required specs — motor, prop, ESC, battery, servo, encoder, load cell, and MCU — and the reason each is needed are in the [component specification sheet](component_specification.md).
